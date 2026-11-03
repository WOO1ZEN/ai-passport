// main/ble_client.c —— 设备详情页的"连接并只读信息/状态/版本"GATT 客户端。
//
// 设计要点:
//   * 只有一个 worker 任务和一个消息队列。NimBLE host 回调只做两件事:把事件
//     (连接/断开/MTU/发现完成/写完成)或 notify 字节拷进队列,绝不在 host 任务里
//     解析协议、更不在 host 任务里暂停/恢复扫描;
//   * 一次只允许一个读取会话;s_session busy 时新请求直接忽略(不排队覆盖);
//   * 会话开始前若扫描处于 SCANNING,由 worker 调 ble_scan_pause();无论成功失败,
//     会话结束都由 worker 调 ble_scan_resume() 恢复;
//   * 所有发球机机型共用同一组 GATT UUID(service ...fe00 / write ...ff01 /
//     notify ...ff02 / CCCD 0x2902),差异只在应用层线格式:
//       - NOVA            : Bare([result][opcode][plen LE][payload]),需要
//                           0x07/0x08 登录后再读 0x01/0x02;
//       - OMNI / HALO / PACE: P1 Framed(41 14 / 45 54 头 + Modbus CRC16,
//                           同样 0x07/0x08 登录后读 0x01/0x02);
//       - AURA            : P2 Frame_ID(41 14 + Frame_ID 00 00 + CRC16,
//                           不需要登录,直接读 0x01/0x02);
//   * SEEKER 是另一套协议:没有固定 UUID,按"唯一 UART 候选"发现后走
//     "22 字节分包头 + JSON"。协议细节全部在 nova_proto / framed_proto /
//     aura_proto / seeker_proto,本文件只负责 GATT 时序与状态机;
//   * 日志只记录型号/opcode/长度/句柄,不打印私钥、token 或整包 payload。
#include "ble_client.h"

#include "aura_proto.h"
#include "ble_scan.h"
#include "framed_proto.h"
#include "nova_proto.h"
#include "seeker_proto.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "host/ble_att.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "os/os_mbuf.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>

static const char *TAG = "ble_client";

#define CLI_TASK_STACK            8192
#define CLI_TASK_PRIO             5
#define CLI_MSG_QUEUE_DEPTH       16
// 单个 notify 的最大长度:MTU 256 时 ATT 载荷最多 253 字节。
#define CLI_RX_MAX                256
// 单次读取会话总时限(连接 + 发现 + 认证 + 信息 + 状态)。
#define CLI_SESSION_CAP_MS        45000   // 先 4s 稳定等待,慢速 0x07/0x08 也要留余量
#define CLI_CONNECT_TIMEOUT_MS    10000
#define CLI_DISCOVERY_TIMEOUT_MS  5000
// 每型号单条命令的超时(测试/真机参考值):PACE 的 0x07 登录最慢用 15s。
#define CLI_NOVA_CMD_TIMEOUT_MS   5000
#define CLI_OMNI_CMD_TIMEOUT_MS   8000
#define CLI_PACE_CMD_TIMEOUT_MS   15000
#define CLI_AURA_CMD_TIMEOUT_MS   8000
#define CLI_SEEKER_REQ_TIMEOUT_MS 8000   // SEEKER 单次请求(含补包)
#define CLI_WRITE_TIMEOUT_MS      2000   // with-response 写的完成等待
#define CLI_MTU_TIMEOUT_MS        2000
#define CLI_MTU_REQUEST           247    // SEEKER 请求的 MTU
#define CLI_MTU_MIN               64     // NOVA 只读帧很小, >=64 即可
// SEEKER 真机单包 = 22B 头 + 128B 正文 = 150B,留出 ATT 头余量后要求 >=158。
#define CLI_SEEKER_MTU_MIN        158
#define CLI_NOVA_STABLE_MS        4000   // Android SDK 在订阅前的稳定等待
#define CLI_OMNI_STABLE_MS        4000   // OMNI/HALO 同 NOVA
#define CLI_PACE_STABLE_MS        2000   // PACE 的 connectionStableMs 更短
#define CLI_AURA_STABLE_MS        2000   // AURA P2:规格 >=1000ms,Lead 要求 2000
#define CLI_SEEKER_DISC_STABLE_MS 500    // SEEKER:服务发现后、订阅前的稳定等待
#define CLI_SEEKER_STABLE_MS      300    // SEEKER:MTU 之后留一点稳定时间
#define CLI_DISCONNECT_WAIT_MS    1500
// 陈旧扫描条目的判定:超过 8s 没有新广播,或当前根本没在扫描(例如列表来自
// 已结束的 20s 扫描),连接前先重新扫描并确认设备仍能出现。
#define CLI_MAX_DEV_AGE_MS        8000
#define CLI_RESCAN_WAIT_MS        5000
#define CLI_RESCAN_POLL_MS        100
// 连接失败后最多重试一次,重试前等 0.5s 让射频/对端恢复。
#define CLI_CONNECT_RETRY_DELAY_MS 500
// 0x3E = HCI "Connection Failed to be Established"(NimBLE 返回码 = 512+0x3E)。
#define CLI_RC_CONN_ESTABLISH_3E  574
#define CLI_POLL_MS               100    // 取消/超时的轮询粒度
#define CLI_REPAIR_ROUNDS         3      // SEEKER 补包最多 3 轮
#define CLI_REPAIR_IDLE_MS        1000   // 有分片但闲置 1s 才发补包请求
#define CLI_MAX_SVCS              12
#define CLI_MAX_CHRS              16
#define CLI_CCCD_UUID16           0x2902
// 主动断开时给对端的 HCI reason(remote user terminated connection)。
#define CLI_HCI_REASON_USER_TERM  0x13

// ---- 消息队列里的事件 ----
enum {
    CLI_EVT_CONNECT = 1,
    CLI_EVT_DISCONNECT,
    CLI_EVT_MTU,
    CLI_EVT_WRITE_DONE,
    CLI_EVT_DISC_SVC_DONE,
    CLI_EVT_DISC_CHR_DONE,
    CLI_EVT_DISC_DSC_DONE,
    CLI_EVT_NOTIFY,
};

typedef struct {
    uint8_t kind;
    int status;
    uint16_t conn_handle;
    uint16_t attr_handle;
    uint16_t value;   // MTU 值
    uint16_t len;
    uint8_t data[CLI_RX_MAX];
} client_msg_t;

// ---- 快照与单飞控制 ----
static bool s_started;
static SemaphoreHandle_t s_info_mutex;
static ble_client_info_t s_info;
static bool s_busy;
static volatile bool s_cancel;
static QueueHandle_t s_req_queue;   // 容量 1,元素 ble_scan_dev_t
static QueueHandle_t s_msg_queue;   // NimBLE 回调 -> worker 的事件/数据
static TaskHandle_t s_task;

// ---- 本次会话的时序状态(只在 worker 任务里读写) ----
static device_model_t s_model;
// 会话结束后是否要恢复扫描:true 表示会话前是 SCANNING;FINISHED/PAUSED 会
// 在会话后保持暂停(不把新一轮 20s 扫描留在后台)。
static bool s_scan_resume_after;
static bool s_paused_by_client;
static int64_t s_deadline_us;
static bool s_failed;
static int s_fail_err;
static char s_fail_detail[64];
static uint16_t s_conn = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_mtu = 23;
static volatile bool s_rx_overflow;
// 球机写类型:-1 未确定,0 = no-response,1 = with-response。只用于日志/缓存。
static int s_write_path = -1;
// 当前型号的单条命令超时(毫秒),由会话开始时按型号设置。
static uint32_t s_cmd_timeout_ms = CLI_NOVA_CMD_TIMEOUT_MS;

// ---- 球机线格式:NOVA=Bare,OMNI/HALO/PACE=P1 Framed,AURA=P2 Frame_ID ----
typedef enum {
    WIRE_BARE = 0,
    WIRE_FRAMED,
    WIRE_P2,
} wire_mode_t;

static wire_mode_t s_wire = WIRE_BARE;

static wire_mode_t wire_mode_for_model(device_model_t model)
{
    switch (model) {
    case DEVICE_MODEL_OMNI:
    case DEVICE_MODEL_HALO:
    case DEVICE_MODEL_PACE:
        return WIRE_FRAMED;
    case DEVICE_MODEL_AURA:
        return WIRE_P2;
    default:
        return WIRE_BARE;
    }
}

// 按当前型号的线格式组请求帧。统一用最大的 P2/AURA 请求缓冲区,
// 三套 builder 都自带容量检查。
static size_t wire_build_request(uint8_t opcode, const void *payload,
                                 size_t payload_len, uint8_t *out, size_t cap)
{
    switch (s_wire) {
    case WIRE_FRAMED:
        return framed_build_request(opcode, payload, payload_len, out, cap);
    case WIRE_P2:
        return aura_build_request(opcode, payload, payload_len, out, cap);
    default:
        return nova_build_request(opcode, payload, payload_len, out, cap);
    }
}

static size_t wire_extract_responses(uint8_t *buf, size_t *buf_len,
                                     nova_response_t *out, size_t max_out)
{
    switch (s_wire) {
    case WIRE_FRAMED:
        return framed_extract_responses(buf, buf_len, out, max_out);
    case WIRE_P2:
        return aura_extract_responses(buf, buf_len, out, max_out);
    default:
        return nova_extract_responses(buf, buf_len, out, max_out);
    }
}

// ---- 球机发现结果(四种机型共用同一组 UUID) ----
static uint16_t s_ball_svc_start;
static uint16_t s_ball_svc_end;
static uint16_t s_ball_write_handle;
static uint16_t s_ball_notify_handle;
static uint16_t s_ball_cccd_handle;
static bool s_ball_svc_found;
// 球机接收累积缓冲(半帧/粘包由所选线格式的 extractor 处理)。
// 2 帧上限:取三种线格式里最大的 AURA 响应帧,避免粘包时溢出。
static uint8_t s_ball_rx[2 * AURA_MAX_RESPONSE];
static size_t s_ball_rx_len;

// ---- SEEKER 发现结果 ----
typedef struct {
    uint16_t start_handle;
    uint16_t end_handle;
    char uuid[BLE_UUID_STR_LEN];
} cli_svc_t;

typedef struct {
    uint16_t val_handle;
    uint16_t svc_index;
    uint16_t cccd_handle;
    uint8_t props;
    bool cccd_found;
} cli_chr_t;

static cli_svc_t s_svcs[CLI_MAX_SVCS];
static size_t s_svc_count;
static cli_chr_t s_chrs[CLI_MAX_CHRS];
static size_t s_chr_count;
static size_t s_dsc_scan_index;  // 当前正在发现描述符的 s_chrs 下标
static uint16_t s_uart_write_handle;
static uint16_t s_uart_notify_handle;
static uint16_t s_uart_cccd_handle;
static uint8_t s_uart_write_props;
static uint8_t s_uart_notify_props;
// 服务/特征数量超出静态上限时置位:候选规则无法可靠判断,直接失败而不是猜。
static bool s_disc_overflow;

// ---- SEEKER 分片重组与 JSON(协议模块负责细节) ----
static seeker_assembler_t s_seeker_asm;
static uint8_t s_seeker_body[SEEKER_MAX_BODY];
static size_t s_seeker_body_len;
static seeker_feed_result_t s_seeker_feed;
static char s_token[64];
static bool s_has_token;

// ---------------------------------------------------------------------------
// 时间与错误记录
// ---------------------------------------------------------------------------

static int64_t now_us(void)
{
    return esp_timer_get_time();
}

// 距离会话总时限还剩多少毫秒;已超时返回 0。
static uint32_t remaining_ms(void)
{
    const int64_t left = s_deadline_us - now_us();
    if (left <= 0) return 0;
    const uint64_t ms = (uint64_t)(left / 1000);
    return ms > UINT32_MAX ? UINT32_MAX : (uint32_t)ms;
}

// 单步超时不能超过会话剩余时间。
static uint32_t step_timeout(uint32_t want_ms)
{
    const uint32_t left = remaining_ms();
    if (left == 0) return 0;
    return want_ms < left ? want_ms : left;
}

static esp_err_t session_failf(int err, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

static esp_err_t session_failf(int err, const char *fmt, ...)
{
    s_failed = true;
    s_fail_err = err;
    va_list ap;
    va_start(ap, fmt);
    (void)vsnprintf(s_fail_detail, sizeof(s_fail_detail), fmt, ap);
    va_end(ap);
    return ESP_FAIL;
}

// 可取消的延时:分片轮询,避免长等待期间对用户的"取消"无响应。
static esp_err_t delay_capped(uint32_t ms)
{
    const int64_t deadline = now_us() + (int64_t)ms * 1000;
    while (now_us() < deadline) {
        if (s_cancel) return ESP_ERR_INVALID_STATE;
        if (remaining_ms() == 0) return ESP_ERR_TIMEOUT;
        uint32_t slice = CLI_POLL_MS;
        const int64_t left = deadline - now_us();
        if (left > 0 && (uint64_t)(left / 1000) < slice) slice = (uint32_t)(left / 1000) + 1;
        vTaskDelay(pdMS_TO_TICKS(slice));
    }
    return ESP_OK;
}

// 在扫描快照里找同 addr+type、且在 baseline 之后又收到过广播的条目。
// 只认"刷新开始之后的新广播":否则 SCANNING 期间 8~15s 的旧条目会被原样接受,
// 又回到 connect status 13 / 0x3E 的老问题。
static bool scan_snapshot_has(const ble_scan_dev_t *want, uint32_t baseline_ms,
                              ble_scan_dev_t *out)
{
    ble_scan_dev_t list[BLE_SCAN_TABLE_CAP];
    const size_t count = ble_scan_snapshot(list, BLE_SCAN_TABLE_CAP);
    for (size_t i = 0; i < count; i++) {
        if (list[i].addr_type == want->addr_type &&
            memcmp(list[i].addr, want->addr, sizeof(want->addr)) == 0) {
            // 无符号回绕安全的"比 baseline 新":差值为正才算新广播。
            if ((int32_t)(list[i].last_seen_ms - baseline_ms) <= 0) continue;
            *out = list[i];
            return true;
        }
    }
    return false;
}

// 刷新扫描后等待目标重新出现(最多 CLI_RESCAN_WAIT_MS,每 100ms 一次,且不超过
// 会话剩余预算);返回 false 表示设备已经不在附近/地址已轮换/被取消。
static bool wait_for_fresh_entry(const ble_scan_dev_t *want, uint32_t baseline_ms,
                                 ble_scan_dev_t *out)
{
    int64_t deadline = now_us() + (int64_t)CLI_RESCAN_WAIT_MS * 1000;
    if (s_deadline_us != 0 && s_deadline_us < deadline) deadline = s_deadline_us;
    for (;;) {
        if (scan_snapshot_has(want, baseline_ms, out)) return true;
        if (s_cancel) return false;
        if (now_us() >= deadline) return false;
        vTaskDelay(pdMS_TO_TICKS(CLI_RESCAN_POLL_MS));
    }
}

// ---------------------------------------------------------------------------
// 快照(UI 轮询用,全部在 s_info_mutex 内更新)
// ---------------------------------------------------------------------------

// 有界拷贝:字段长度由协议决定,比源缓冲区短,直接 memcpy 截断避免 snprintf
// 的 format-truncation 告警(本项目把该类告警当错误)。
static void copy_str(char *dst, size_t cap, const char *src)
{
    if (dst == NULL || cap == 0) return;
    if (src == NULL) {
        dst[0] = '\0';
        return;
    }
    size_t n = strlen(src);
    if (n >= cap) n = cap - 1;
    memcpy(dst, src, n);
    dst[n] = '\0';
}

static void info_lock(void)
{
    if (s_info_mutex != NULL) (void)xSemaphoreTake(s_info_mutex, portMAX_DELAY);
}

static void info_unlock(void)
{
    if (s_info_mutex != NULL) (void)xSemaphoreGive(s_info_mutex);
}

static void info_init(const ble_scan_dev_t *dev, device_model_t model)
{
    info_lock();
    memset(&s_info, 0, sizeof(s_info));
    s_info.state = BLE_CLIENT_CONNECTING;
    s_info.model = model;
    if (dev != NULL) {
        snprintf(s_info.device_name, sizeof(s_info.device_name), "%s", dev->name);
    }
    info_unlock();
}

static void info_set_state(ble_client_state_t state)
{
    info_lock();
    s_info.state = state;
    info_unlock();
}

static void info_set_connected(bool connected)
{
    info_lock();
    s_info.connected = connected;
    info_unlock();
}

static void info_terminal(ble_client_state_t state, int err, const char *detail)
{
    info_lock();
    s_info.state = state;
    s_info.connected = false;
    s_info.last_error = err;
    if (detail != NULL) {
        snprintf(s_info.detail, sizeof(s_info.detail), "%s", detail);
    } else {
        s_info.detail[0] = '\0';
    }
    info_unlock();
}

static void info_set_nova_info(const nova_device_info_t *info)
{
    info_lock();
    copy_str(s_info.sn, sizeof(s_info.sn), info->sn);
    copy_str(s_info.firmware_version, sizeof(s_info.firmware_version),
             info->firmware_version);
    copy_str(s_info.hardware_version, sizeof(s_info.hardware_version),
             info->hardware_version);
    info_unlock();
}

static void info_set_nova_status(uint8_t level, uint16_t code, bool has_battery,
                                 uint8_t battery)
{
    info_lock();
    s_info.status_valid = true;
    s_info.nova_level = level;
    s_info.nova_code = code;
    if (has_battery) {
        s_info.battery_valid = true;
        s_info.battery = battery;
    }
    info_unlock();
}

static void info_set_seeker_info(const seeker_device_info_t *info)
{
    info_lock();
    copy_str(s_info.sn, sizeof(s_info.sn), info->sn);
    // SEEKER: firmware_version = APK 版本; hardware_version = 系统固件版本。
    copy_str(s_info.firmware_version, sizeof(s_info.firmware_version),
             info->firmware_version);
    copy_str(s_info.hardware_version, sizeof(s_info.hardware_version),
             info->system_version);
    info_unlock();
}

static void info_set_seeker_status(const char *hawkeye)
{
    info_lock();
    s_info.status_valid = true;
    snprintf(s_info.seeker_state, sizeof(s_info.seeker_state), "%s", hawkeye);
    info_unlock();
}

// 会话结束即擦除敏感/原始响应数据:token、SEEKER 正文、NOVA 半帧与重组 arena。
// 不在等待下一次请求时才清,避免失败会话的残留数据长期留在内存里。
static void session_wipe_sensitive(void)
{
    memset(s_token, 0, sizeof(s_token));
    s_has_token = false;
    memset(s_seeker_body, 0, sizeof(s_seeker_body));
    s_seeker_body_len = 0;
    seeker_assembler_reset(&s_seeker_asm);
    s_seeker_feed = SEEKER_FEED_NONE;
    memset(s_ball_rx, 0, sizeof(s_ball_rx));
    s_ball_rx_len = 0;
}

// 终端状态的一行验收日志:只含型号/SN/版本/状态,不含私钥、token 或 payload。
static void log_terminal_summary(ble_client_state_t state)
{
    ble_client_info_t info;
    info_lock();
    info = s_info;
    info_unlock();

    const int battery = info.battery_valid ? (int)info.battery : 255;
    switch (state) {
    case BLE_CLIENT_DONE:
        ESP_LOGI(TAG,
                 "read ok: model=%s sn=%s fw=%s hw=%s level=%u code=%u seeker=%s battery=%d",
                 device_model_name(info.model), info.sn, info.firmware_version,
                 info.hardware_version, (unsigned)info.nova_level,
                 (unsigned)info.nova_code,
                 info.seeker_state[0] != '\0' ? info.seeker_state : "-", battery);
        break;
    case BLE_CLIENT_UNSUPPORTED:
        ESP_LOGI(TAG, "read unsupported: model=%s", device_model_name(info.model));
        break;
    default:
        ESP_LOGW(TAG, "read failed: model=%s detail=%s last_error=%d",
                 device_model_name(info.model), info.detail, info.last_error);
        break;
    }
}

// ---------------------------------------------------------------------------
// 事件队列:回调只拷贝,worker 消费
// ---------------------------------------------------------------------------

static void drain_queue(void)
{
    if (s_msg_queue == NULL) return;
    client_msg_t msg;
    while (xQueueReceive(s_msg_queue, &msg, 0) == pdTRUE) {
        // 丢弃上一次会话的残留事件。
    }
}

static void queue_msg(const client_msg_t *msg)
{
    if (s_msg_queue == NULL) return;
    if (xQueueSend(s_msg_queue, msg, 0) != pdTRUE) {
        // 队列满说明主循环没有及时消费;标记溢出让 worker 尽快失败并重连,
        // 而不是悄悄丢掉半帧导致协议解析错位。
        s_rx_overflow = true;
    }
}

// 等待一个控制事件;NOTIFY 由 worker 自己按协议路由,不在这里丢。
static esp_err_t pump_msg(client_msg_t *out, uint32_t timeout_ms)
{
    const int64_t deadline = now_us() + (int64_t)timeout_ms * 1000;

    for (;;) {
        if (s_cancel) return ESP_ERR_INVALID_STATE;
        const int64_t left = deadline - now_us();
        if (left <= 0) return ESP_ERR_TIMEOUT;
        uint32_t slice = (uint32_t)(left / 1000);
        if (slice == 0) slice = 1;
        if (slice > CLI_POLL_MS) slice = CLI_POLL_MS;

        if (xQueueReceive(s_msg_queue, out, pdMS_TO_TICKS(slice)) != pdTRUE) {
            continue;
        }
        if (s_cancel) return ESP_ERR_INVALID_STATE;

        switch (out->kind) {
        case CLI_EVT_DISCONNECT:
            // 对端主动断开:会话立即失败,不再等 MTU/响应。
            s_conn = BLE_HS_CONN_HANDLE_NONE;
            return session_failf(out->status, "disconnected");
        case CLI_EVT_MTU:
            // MTU 事件可能在等别的事件时到达(例如对端先发起交换),统一在这里
            // 记下最佳已知值,避免 exchange_mtu() 误判成默认 23。
            if (out->conn_handle == s_conn && out->value > s_mtu) {
                s_mtu = out->value;
            }
            return ESP_OK;
        default:
            return ESP_OK;
        }
    }
}

// 一直等到指定类型的事件(期间的 NOTIFY/其它事件交给对应处理)。
static esp_err_t wait_kind(uint8_t kind, client_msg_t *out, uint32_t timeout_ms)
{
    const int64_t deadline = now_us() + (int64_t)timeout_ms * 1000;

    for (;;) {
        const int64_t left = deadline - now_us();
        if (left <= 0) return ESP_ERR_TIMEOUT;
        esp_err_t err = pump_msg(out, (uint32_t)(left / 1000) + 1);
        if (err != ESP_OK) return err;
        if (out->kind == CLI_EVT_NOTIFY) {
            // 提前到达的业务数据先存好,真正等待响应的步骤会再取。设备可能在
            // with-response 写的完成回调之前就回 notify,所以这里必须按线格式
            // 缓存,而不是丢弃。
            if (s_model == DEVICE_MODEL_SEEKER) {
                if (out->conn_handle == s_conn && out->attr_handle == s_uart_notify_handle) {
                    size_t body_len = 0;
                    s_seeker_feed = seeker_assembler_feed(
                        &s_seeker_asm, out->data, out->len, s_seeker_body,
                        sizeof(s_seeker_body), &body_len);
                    if (s_seeker_feed == SEEKER_FEED_COMPLETE) {
                        s_seeker_body_len = body_len;
                    }
                }
            } else {
                // NOVA / OMNI / HALO / PACE / AURA 都是球机:共用同一 notify 句柄,
                // 先累积到 s_ball_rx,等 ball_transact 里用对应 extractor 解析。
                if (out->conn_handle == s_conn && out->attr_handle == s_ball_notify_handle) {
                    if (s_ball_rx_len + out->len > sizeof(s_ball_rx)) {
                        s_rx_overflow = true;
                    } else {
                        memcpy(s_ball_rx + s_ball_rx_len, out->data, out->len);
                        s_ball_rx_len += out->len;
                    }
                }
            }
            continue;
        }
        if (out->kind == kind) return ESP_OK;
        // 其它迟到事件(例如上一阶段的 MTU)直接忽略。
    }
}

// 主动断开后等 GAP 的 DISCONNECT 事件,确保连接句柄已经回收。
static void wait_disconnect(uint32_t timeout_ms)
{
    const int64_t deadline = now_us() + (int64_t)timeout_ms * 1000;
    for (;;) {
        const int64_t left = deadline - now_us();
        if (left <= 0) return;
        client_msg_t msg;
        if (xQueueReceive(s_msg_queue, &msg, pdMS_TO_TICKS((left / 1000) + 1)) != pdTRUE) {
            return;
        }
        if (msg.kind == CLI_EVT_DISCONNECT) return;
    }
}

// ---------------------------------------------------------------------------
// NimBLE 回调:只把事件/字节拷进队列,不做协议解析
// ---------------------------------------------------------------------------

static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    client_msg_t msg;
    memset(&msg, 0, sizeof(msg));

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        msg.kind = CLI_EVT_CONNECT;
        msg.status = event->connect.status;
        msg.conn_handle = event->connect.conn_handle;
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        msg.kind = CLI_EVT_DISCONNECT;
        msg.status = event->disconnect.reason;
        msg.conn_handle = event->disconnect.conn.conn_handle;
        break;
    case BLE_GAP_EVENT_MTU:
        if (event->mtu.channel_id != BLE_L2CAP_CID_ATT) return 0;
        msg.kind = CLI_EVT_MTU;
        msg.conn_handle = event->mtu.conn_handle;
        msg.value = event->mtu.value;
        break;
    case BLE_GAP_EVENT_NOTIFY_RX: {
        if (event->notify_rx.conn_handle != s_conn) return 0;
        if (event->notify_rx.om == NULL) {
            s_rx_overflow = true;
            return 0;
        }
        const uint16_t len = (uint16_t)OS_MBUF_PKTLEN(event->notify_rx.om);
        if (len == 0 || len > CLI_RX_MAX) {
            s_rx_overflow = true;
            return 0;
        }
        msg.kind = CLI_EVT_NOTIFY;
        msg.conn_handle = event->notify_rx.conn_handle;
        msg.attr_handle = event->notify_rx.attr_handle;
        msg.len = len;
        if (os_mbuf_copydata(event->notify_rx.om, 0, len, msg.data) != 0) {
            s_rx_overflow = true;
            return 0;
        }
        break;
    }
    default:
        return 0;
    }
    queue_msg(&msg);
    return 0;
}

// ---- NOVA 发现回调 ----

static int ball_svc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                       const struct ble_gatt_svc *service, void *arg)
{
    (void)conn_handle;
    (void)arg;

    client_msg_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.kind = CLI_EVT_DISC_SVC_DONE;

    if (error->status == BLE_HS_EDONE) {
        msg.status = s_ball_svc_found ? 0 : 1;
        queue_msg(&msg);
        return 0;
    }
    if (error->status != 0) {
        msg.status = error->status;
        queue_msg(&msg);
        return 0;
    }
    if (service == NULL) return 0;

    char uuid[BLE_UUID_STR_LEN];
    if (ble_uuid_to_str(&service->uuid.u, uuid) != NULL &&
        strcasecmp(uuid, NOVA_SERVICE_UUID) == 0) {
        s_ball_svc_start = service->start_handle;
        s_ball_svc_end = service->end_handle;
        s_ball_svc_found = true;
    }
    return 0;
}

static int ball_chr_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                       const struct ble_gatt_chr *chr, void *arg)
{
    (void)conn_handle;
    (void)arg;

    client_msg_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.kind = CLI_EVT_DISC_CHR_DONE;

    if (error->status == BLE_HS_EDONE) {
        queue_msg(&msg);
        return 0;
    }
    if (error->status != 0) {
        msg.status = error->status;
        queue_msg(&msg);
        return 0;
    }
    if (chr == NULL) return 0;

    char uuid[BLE_UUID_STR_LEN];
    if (ble_uuid_to_str(&chr->uuid.u, uuid) == NULL) return 0;
    if (strcasecmp(uuid, NOVA_WRITE_UUID) == 0) {
        s_ball_write_handle = chr->val_handle;
    } else if (strcasecmp(uuid, NOVA_NOTIFY_UUID) == 0) {
        s_ball_notify_handle = chr->val_handle;
    }
    return 0;
}

static int ball_dsc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                       uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc,
                       void *arg)
{
    (void)conn_handle;
    (void)arg;

    client_msg_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.kind = CLI_EVT_DISC_DSC_DONE;

    if (error->status == BLE_HS_EDONE) {
        queue_msg(&msg);
        return 0;
    }
    if (error->status != 0) {
        msg.status = error->status;
        queue_msg(&msg);
        return 0;
    }
    if (dsc == NULL || chr_val_handle != s_ball_notify_handle) return 0;
    if (ble_uuid_u16(&dsc->uuid.u) == CLI_CCCD_UUID16) {
        s_ball_cccd_handle = dsc->handle;
    }
    return 0;
}

// ---- SEEKER 发现回调(全部服务 + 全部特征 + 候选 notify 的 CCCD) ----

static int seeker_svc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                         const struct ble_gatt_svc *service, void *arg)
{
    (void)conn_handle;
    (void)arg;

    client_msg_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.kind = CLI_EVT_DISC_SVC_DONE;

    if (error->status == BLE_HS_EDONE) {
        queue_msg(&msg);
        return 0;
    }
    if (error->status != 0) {
        msg.status = error->status;
        queue_msg(&msg);
        return 0;
    }
    if (service == NULL) return 0;
    if (s_svc_count >= CLI_MAX_SVCS) {
        s_disc_overflow = true;
        return 0;
    }

    cli_svc_t *svc = &s_svcs[s_svc_count];
    memset(svc, 0, sizeof(*svc));
    svc->start_handle = service->start_handle;
    svc->end_handle = service->end_handle;
    if (ble_uuid_to_str(&service->uuid.u, svc->uuid) == NULL) {
        svc->uuid[0] = '\0';
    }
    s_svc_count++;
    return 0;
}

static int seeker_chr_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                         const struct ble_gatt_chr *chr, void *arg)
{
    (void)conn_handle;
    (void)arg;

    client_msg_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.kind = CLI_EVT_DISC_CHR_DONE;

    if (error->status == BLE_HS_EDONE) {
        queue_msg(&msg);
        return 0;
    }
    if (error->status != 0) {
        msg.status = error->status;
        queue_msg(&msg);
        return 0;
    }
    if (chr == NULL) return 0;
    if (s_chr_count >= CLI_MAX_CHRS) {
        s_disc_overflow = true;
        return 0;
    }

    // 把特征归到拥有它的服务里:SEEKER 的候选规则要求 write/notify 同属一个服务。
    int svc_index = -1;
    for (size_t i = 0; i < s_svc_count; i++) {
        if (chr->def_handle >= s_svcs[i].start_handle &&
            chr->def_handle <= s_svcs[i].end_handle) {
            svc_index = (int)i;
            break;
        }
    }
    if (svc_index < 0) return 0;

    cli_chr_t *rec = &s_chrs[s_chr_count++];
    memset(rec, 0, sizeof(*rec));
    rec->val_handle = chr->val_handle;
    rec->svc_index = (uint16_t)svc_index;
    rec->props = chr->properties;
    return 0;
}

static int seeker_dsc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                         uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc,
                         void *arg)
{
    (void)conn_handle;
    (void)arg;

    client_msg_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.kind = CLI_EVT_DISC_DSC_DONE;

    if (error->status == BLE_HS_EDONE) {
        queue_msg(&msg);
        return 0;
    }
    if (error->status != 0) {
        msg.status = error->status;
        queue_msg(&msg);
        return 0;
    }
    if (dsc == NULL || s_dsc_scan_index >= s_chr_count) return 0;
    cli_chr_t *rec = &s_chrs[s_dsc_scan_index];
    if (chr_val_handle != rec->val_handle) return 0;
    if (ble_uuid_u16(&dsc->uuid.u) == CLI_CCCD_UUID16) {
        rec->cccd_handle = dsc->handle;
        rec->cccd_found = true;
    }
    return 0;
}

// ---- ATT 写完成回调(GATT 过程都经过这里回到 worker) ----

static int write_done_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                         struct ble_gatt_attr *attr, void *arg)
{
    (void)attr;
    (void)arg;
    client_msg_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.kind = CLI_EVT_WRITE_DONE;
    msg.status = error->status;
    msg.conn_handle = conn_handle;
    queue_msg(&msg);
    return 0;
}

// ---------------------------------------------------------------------------
// 连接 / MTU / 订阅
// ---------------------------------------------------------------------------

// 取消尚未完成的连接,并回收可能迟到的 CONNECT(否则会留下孤儿连接占住
// MAX_CONNECTIONS=1)。只在 worker 任务里调用。
static void cancel_connect_and_reap(void)
{
    (void)ble_gap_conn_cancel();
    const int64_t deadline = now_us() + 300000;  // 最多等 300ms
    for (;;) {
        const int64_t left = deadline - now_us();
        if (left <= 0) return;
        client_msg_t msg;
        if (xQueueReceive(s_msg_queue, &msg, pdMS_TO_TICKS((left / 1000) + 1)) != pdTRUE) {
            return;
        }
        if (msg.kind == CLI_EVT_CONNECT && msg.status == 0) {
            (void)ble_gap_terminate(msg.conn_handle, CLI_HCI_REASON_USER_TERM);
            wait_disconnect(300);
            return;
        }
        if (msg.kind == CLI_EVT_DISCONNECT) return;
    }
}

// 连接失败统一出口:信息页只显示约 26 个 ASCII 字符,所以 detail 用短字符串;
// 完整诊断(addr/type/conn/尝试次数/状态码)只进 ESP_LOG。
static esp_err_t connect_fail(int status, const char *addr, unsigned type,
                              unsigned conn, int attempts)
{
    ESP_LOGW(TAG, "connect failed status=%d addr=%s type=%u conn=%u attempts=%d",
             status, addr, type, conn, attempts);
    if (status == ESP_ERR_TIMEOUT || status == BLE_HS_ETIMEOUT) {
        return session_failf(ESP_ERR_TIMEOUT, "connect timeout");
    }
    if (status == CLI_RC_CONN_ESTABLISH_3E) {
        return session_failf(status, "connect failed 0x3E");
    }
    return session_failf(status, "connect rejected");
}

// 只对"没连上"(超时/0x3E)重试;对端拒绝等错误不重试。
static bool connect_retryable(int status)
{
    return status == BLE_HS_ETIMEOUT || status == CLI_RC_CONN_ESTABLISH_3E;
}

// 连接策略:同一对端最多尝试 2 次(第一次 + 重试一次),每次 10s 上限;
// 只对 BLE_HS_ETIMEOUT(13) 与 0x3E(574) 重试,重试前等 500ms。
static esp_err_t cli_connect(const ble_scan_dev_t *dev)
{
    uint8_t own_addr_type = 0;
    int rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) return session_failf(rc, "connect failed");

    struct ble_gap_conn_params params;
    memset(&params, 0, sizeof(params));
    // NimBLE 会把 scan_itvl/scan_window 原样写进 HCI LE Create Connection 命令。
    // 这两个字段留 0 会被控制器判为 "Invalid HCI Command Parameters"
    // (0x212 = 530,真机上表现为 "connect 530"),所以必须填合法值:
    // 0x0060 = 96 × 0.625ms = 60ms 扫描间隔,0x0030 = 30ms 扫描窗口。
    params.scan_itvl = 0x0060;
    params.scan_window = 0x0030;
    // 15..30ms 间隔、无 latency、4s supervision,兼顾只读交互的响应与功耗。
    params.itvl_min = 12;
    params.itvl_max = 24;
    params.latency = 0;
    params.supervision_timeout = 400;
    params.min_ce_len = 0;
    params.max_ce_len = 0;

    ble_addr_t peer;
    memset(&peer, 0, sizeof(peer));
    peer.type = dev->addr_type;
    memcpy(peer.val, dev->addr, sizeof(peer.val));

    char addr[18];
    ble_scan_format_addr(dev->addr, addr);
    const unsigned peer_type = (unsigned)peer.type;
    const unsigned peer_conn = dev->connectable ? 1u : 0u;
    int last_status = 0;

    for (int attempt = 1; attempt <= 2; attempt++) {
        if (attempt > 1) {
            // 扫描此时已暂停,保持暂停;等 0.5s 再重试(可被取消)。
            const esp_err_t derr = delay_capped(CLI_CONNECT_RETRY_DELAY_MS);
            if (derr == ESP_ERR_INVALID_STATE) return derr;
            if (derr != ESP_OK) return session_failf(ESP_ERR_TIMEOUT, "connect timeout");
        }

        const uint32_t timeout = step_timeout(CLI_CONNECT_TIMEOUT_MS);
        if (timeout == 0) return session_failf(ESP_ERR_TIMEOUT, "connect timeout");

        s_conn = BLE_HS_CONN_HANDLE_NONE;
        rc = ble_gap_connect(own_addr_type, &peer, (int32_t)timeout, &params,
                             gap_event_cb, NULL);
        if (rc != 0) {
            last_status = rc;
            if (attempt == 1 && connect_retryable(rc)) {
                ESP_LOGW(TAG, "connect retry after status %d (attempt %d)", rc, attempt);
                continue;
            }
            return connect_fail(rc, addr, peer_type, peer_conn, attempt);
        }

        const int64_t deadline = now_us() + (int64_t)timeout * 1000;
        for (;;) {
            const int64_t left = deadline - now_us();
            if (left <= 0) {
                cancel_connect_and_reap();
                last_status = ESP_ERR_TIMEOUT;
                break;
            }
            client_msg_t msg;
            esp_err_t err = pump_msg(&msg, (uint32_t)(left / 1000) + 1);
            if (err == ESP_ERR_TIMEOUT) {
                cancel_connect_and_reap();
                last_status = ESP_ERR_TIMEOUT;
                break;
            }
            if (err != ESP_OK) {
                cancel_connect_and_reap();
                if (err == ESP_ERR_INVALID_STATE) return err;  // 用户取消
                // 对端在连接建立前就断开:reason 在 s_fail_err。
                last_status = s_fail_err;
                break;
            }
            if (msg.kind == CLI_EVT_NOTIFY) continue;  // 连接建立前不应有业务数据
            if (msg.kind != CLI_EVT_CONNECT) continue;
            if (msg.status != 0) {
                last_status = msg.status;
                break;
            }
            s_conn = msg.conn_handle;
            s_mtu = 23;
            info_set_connected(true);
            ESP_LOGI(TAG, "connected, model=%s attempt=%d",
                     device_model_name(s_model), attempt);
            return ESP_OK;
        }

        if (attempt == 1 && connect_retryable(last_status)) {
            ESP_LOGW(TAG, "connect retry after status %d (addr=%s type=%u conn=%u)",
                     last_status, addr, peer_type, peer_conn);
            continue;
        }
        return connect_fail(last_status, addr, peer_type, peer_conn, attempt);
    }

    // 循环最多两轮,理论不可达;保留统一失败出口。
    return connect_fail(last_status, addr, peer_type, peer_conn, 2);
}

// 协商 MTU。对端可能先发起过交换(ble_gattc_exchange_mtu 返回 EALREADY),
// 也可能 MTU 事件在等别的事件时已经到达;所以这里不把 s_mtu 重置为 23,
// 而是始终保留"最佳已知值",失败时再读 ble_att_mtu()/等事件兜底。
static esp_err_t exchange_mtu(void)
{
    uint16_t known = ble_att_mtu(s_conn);
    if (known > s_mtu) s_mtu = known;

    const int rc = ble_gattc_exchange_mtu(s_conn, NULL, NULL);
    if (rc == 0) {
        const uint32_t timeout = step_timeout(CLI_MTU_TIMEOUT_MS);
        if (timeout == 0) return session_failf(ESP_ERR_TIMEOUT, "session timeout");
        client_msg_t msg;
        esp_err_t err = wait_kind(CLI_EVT_MTU, &msg, timeout);
        if (err == ESP_OK) {
            if (msg.value > s_mtu) s_mtu = msg.value;
        } else if (err != ESP_ERR_TIMEOUT) {
            return err;
        }
        return ESP_OK;
    }

    ESP_LOGI(TAG, "exchange mtu rc=%d, keep known=%u", rc, (unsigned)s_mtu);
    known = ble_att_mtu(s_conn);
    if (known > s_mtu) s_mtu = known;
    if (s_mtu >= CLI_MTU_MIN) return ESP_OK;

    // 仍没有可用值:再等最多 2s,看对端是否自己发起了 MTU 交换。
    const uint32_t timeout = step_timeout(CLI_MTU_TIMEOUT_MS);
    if (timeout != 0) {
        client_msg_t msg;
        esp_err_t err = wait_kind(CLI_EVT_MTU, &msg, timeout);
        if (err == ESP_OK) {
            if (msg.value > s_mtu) s_mtu = msg.value;
        } else if (err != ESP_ERR_TIMEOUT) {
            return err;
        }
    }
    return ESP_OK;
}

// MTU 下限检查:floor 以下直接失败并给出具体协议标签。
static esp_err_t require_mtu(uint16_t floor, const char *label)
{
    if (s_mtu >= floor) return ESP_OK;
    if (label == NULL) {
        return session_failf(ESP_FAIL, "mtu %u < %u", (unsigned)s_mtu,
                             (unsigned)floor);
    }
    return session_failf(ESP_FAIL, "mtu %u < %u (%s)", (unsigned)s_mtu,
                         (unsigned)floor, label);
}

// 订阅 CCCD。球机(ff02)按规格恒按 NOTIFY 处理,写 01 00(不探测 INDICATE 属性);
// SEEKER 则会根据特征属性在 01 00 / 02 00 之间选择。
static esp_err_t write_cccd(uint16_t handle, bool notify)
{
    uint8_t value[2] = { notify ? 1 : 2, 0 };
    int rc = ble_gattc_write_flat(s_conn, handle, value, sizeof(value),
                                  write_done_cb, NULL);
    if (rc != 0) return session_failf(rc, "subscribe %d", rc);

    const uint32_t timeout = step_timeout(CLI_WRITE_TIMEOUT_MS);
    if (timeout == 0) return session_failf(ESP_ERR_TIMEOUT, "session timeout");

    client_msg_t msg;
    esp_err_t err = wait_kind(CLI_EVT_WRITE_DONE, &msg, timeout);
    if (err == ESP_ERR_TIMEOUT) return session_failf(ESP_ERR_TIMEOUT, "subscribe timeout");
    if (err != ESP_OK) return err;
    if (msg.status != 0) return session_failf(ESP_FAIL, "subscribe status %d", msg.status);
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// 球机(NOVA/OMNI/HALO/PACE/AURA):发现、收发与认证
// ---------------------------------------------------------------------------

static esp_err_t wait_discovery(uint8_t kind, int *status)
{
    const uint32_t timeout = step_timeout(CLI_DISCOVERY_TIMEOUT_MS);
    if (timeout == 0) return session_failf(ESP_ERR_TIMEOUT, "session timeout");

    client_msg_t msg;
    esp_err_t err = wait_kind(kind, &msg, timeout);
    if (err == ESP_ERR_TIMEOUT) return session_failf(ESP_ERR_TIMEOUT, "discovery timeout");
    if (err != ESP_OK) return err;
    *status = msg.status;
    return ESP_OK;
}

static esp_err_t ball_discover(void)
{
    ble_uuid_any_t service_uuid;
    if (ble_uuid_from_str(&service_uuid, NOVA_SERVICE_UUID) != 0) {
        return session_failf(ESP_FAIL, "uuid parse");
    }

    s_ball_svc_found = false;
    s_ball_svc_start = 0;
    s_ball_svc_end = 0;
    s_ball_write_handle = 0;
    s_ball_notify_handle = 0;
    s_ball_cccd_handle = 0;

    int rc = ble_gattc_disc_svc_by_uuid(s_conn, &service_uuid.u, ball_svc_cb, NULL);
    if (rc != 0) return session_failf(rc, "disc svc %d", rc);
    int status = 0;
    esp_err_t err = wait_discovery(CLI_EVT_DISC_SVC_DONE, &status);
    if (err != ESP_OK) return err;
    if (status != 0 || !s_ball_svc_found) return session_failf(ESP_FAIL, "service not found");

    rc = ble_gattc_disc_all_chrs(s_conn, s_ball_svc_start, s_ball_svc_end,
                                 ball_chr_cb, NULL);
    if (rc != 0) return session_failf(rc, "disc chr %d", rc);
    err = wait_discovery(CLI_EVT_DISC_CHR_DONE, &status);
    if (err != ESP_OK) return err;
    if (status != 0) return session_failf(ESP_FAIL, "chr discovery %d", status);
    if (s_ball_write_handle == 0 || s_ball_notify_handle == 0) {
        return session_failf(ESP_FAIL, "characteristics not found");
    }

    // notify 值句柄已是服务最后一个属性时没有 CCCD 空间,NimBLE 会直接回
    // EINVAL;这里先判范围,给出明确的 "cccd not found" 而不是底层错误码。
    if (s_ball_notify_handle >= s_ball_svc_end) {
        return session_failf(ESP_FAIL, "cccd not found");
    }
    rc = ble_gattc_disc_all_dscs(s_conn, s_ball_notify_handle, s_ball_svc_end,
                                 ball_dsc_cb, NULL);
    if (rc != 0) return session_failf(rc, "disc dsc %d", rc);
    err = wait_discovery(CLI_EVT_DISC_DSC_DONE, &status);
    if (err != ESP_OK) return err;
    if (status != 0) return session_failf(ESP_FAIL, "dsc discovery %d", status);
    if (s_ball_cccd_handle == 0) return session_failf(ESP_FAIL, "cccd not found");
    return ESP_OK;
}

// 球机发送:优先 write-with-response;若该特征只支持 no-response,设备会回
// ATT 错误,此时只回退一次到 write-no-response(单飞请求,不会放大重试)。
static esp_err_t ball_send_frame(const uint8_t *frame, size_t len)
{
    // 本会话里如果已经确认该特征只吃 no-response(s_write_path==0),后续请求
    // 直接走 no-response,不再为每条命令付一次失败的 with-response。
    if (s_write_path != 0) {
        int rc = ble_gattc_write_flat(s_conn, s_ball_write_handle, frame,
                                      (uint16_t)len, write_done_cb, NULL);
        if (rc == 0) {
            const uint32_t timeout = step_timeout(CLI_WRITE_TIMEOUT_MS);
            if (timeout == 0) return session_failf(ESP_ERR_TIMEOUT, "session timeout");

            client_msg_t msg;
            esp_err_t err = wait_kind(CLI_EVT_WRITE_DONE, &msg, timeout);
            if (err == ESP_OK && msg.status == 0) {
                if (s_write_path != 1) {
                    ESP_LOGI(TAG, "%s write path: with-response", device_model_name(s_model));
                    s_write_path = 1;
                }
                return ESP_OK;
            }
            if (err == ESP_ERR_TIMEOUT) {
                // 写已发出但没等到完成回调:不能盲目再发一次(可能重复指令),
                // 直接判失败,由上层断开,避免放大风险。
                return session_failf(ESP_ERR_TIMEOUT, "write timeout");
            }
            if (err != ESP_OK) return err;
            // 设备返回 ATT 错误:该特征可能只接受 no-response,回退一次。
        }
    }

    const int rc = ble_gattc_write_no_rsp_flat(s_conn, s_ball_write_handle, frame,
                                               (uint16_t)len);
    if (rc != 0) return session_failf(rc, "gatt write %d", rc);
    if (s_write_path != 0) {
        ESP_LOGW(TAG, "%s write path: no-response (fallback)", device_model_name(s_model));
        s_write_path = 0;
    }
    return ESP_OK;
}

// 从 NOVA 累积缓冲取一条与 want 匹配的响应;认证失败帧(0x08 result!=0)即使
// 用户正在等别的 opcode 也要立刻结束等待。
static bool ball_take_match(uint8_t want, nova_response_t *out, bool *auth_fail)
{
    if (s_ball_rx_len == 0) return false;

    nova_response_t frames[4];
    const size_t count = wire_extract_responses(s_ball_rx, &s_ball_rx_len, frames, 4);
    for (size_t i = 0; i < count; i++) {
        const nova_response_t *frame = &frames[i];
        // Bare / P1 Framed 的 0x08 失败都是 result!=0 + 空 payload:立即报认证失败。
        // AURA 不发认证帧,这个特判对它没有副作用。
        const bool af = (frame->opcode == NOVA_OP_ACCESS_VERIFY &&
                         frame->result != 0 && frame->payload_len == 0);
        if (frame->opcode == want || af) {
            *out = *frame;
            *auth_fail = af;
            return true;
        }
    }
    return false;
}

static esp_err_t ball_transact(uint8_t opcode, const uint8_t *payload,
                               size_t payload_len, nova_response_t *resp)
{
    // 三套线格式里 AURA P2 的请求帧最大(2+2+2+1+2+payload+2)。
    uint8_t frame[AURA_MAX_REQUEST];
    const size_t frame_len = wire_build_request(opcode, payload, payload_len,
                                                frame, sizeof(frame));
    if (frame_len == 0) return session_failf(ESP_FAIL, "request build");

    // 新请求开始:清掉上一请求可能残留的半帧,避免 opcode 匹配到迟到数据。
    s_ball_rx_len = 0;
    s_rx_overflow = false;

    esp_err_t err = ball_send_frame(frame, frame_len);
    if (err != ESP_OK) return err;

    const uint32_t timeout = step_timeout(s_cmd_timeout_ms);
    if (timeout == 0) return session_failf(ESP_ERR_TIMEOUT, "session timeout");
    const int64_t deadline = now_us() + (int64_t)timeout * 1000;

    for (;;) {
        if (s_rx_overflow) return session_failf(ESP_FAIL, "rx overflow");

        nova_response_t match;
        bool auth_fail = false;
        if (ball_take_match(opcode, &match, &auth_fail)) {
            if (match.result != 0) {
                if (auth_fail) return session_failf(ESP_FAIL, "auth rejected");
                return session_failf(ESP_FAIL, "result %u", (unsigned)match.result);
            }
            if (resp != NULL) *resp = match;
            return ESP_OK;
        }

        if (s_cancel) return ESP_ERR_INVALID_STATE;
        const int64_t left = deadline - now_us();
        if (left <= 0) return session_failf(ESP_ERR_TIMEOUT, "response timeout");

        client_msg_t msg;
        err = pump_msg(&msg, (uint32_t)(left / 1000) + 1);
        if (err == ESP_ERR_TIMEOUT) continue;
        if (err != ESP_OK) return err;
        if (msg.kind == CLI_EVT_NOTIFY) {
            if (msg.conn_handle != s_conn || msg.attr_handle != s_ball_notify_handle) continue;
            if (s_ball_rx_len + msg.len > sizeof(s_ball_rx)) {
                s_rx_overflow = true;
                continue;
            }
            memcpy(s_ball_rx + s_ball_rx_len, msg.data, msg.len);
            s_ball_rx_len += msg.len;
        }
    }
}

// 球机通用会话:发现 -> 稳定等待 -> 订阅 -> MTU -> (可选登录) -> 0x01 -> 0x02。
// wire 模式由 s_wire 决定:Bare(NOVA)与 P1 Framed 需要 0x07/0x08 登录,
// AURA(P2)不需要登录,直接读信息。
static esp_err_t ball_session(uint32_t stable_ms, bool require_auth)
{
    esp_err_t err = ball_discover();
    if (err != ESP_OK) return err;

    // 连接稳定等待(不同机型时长不同,见 CLI_*_STABLE_MS)。
    uint32_t stable = stable_ms;
    if (stable > remaining_ms()) stable = remaining_ms();
    err = delay_capped(stable);
    if (err == ESP_ERR_INVALID_STATE) return err;
    if (err != ESP_OK) return session_failf(ESP_ERR_TIMEOUT, "session timeout");

    info_set_state(BLE_CLIENT_SUBSCRIBING);
    err = write_cccd(s_ball_cccd_handle, true);
    if (err != ESP_OK) return err;

    err = exchange_mtu();
    if (err != ESP_OK) return err;
    err = require_mtu(CLI_MTU_MIN, NULL);
    if (err != ESP_OK) return err;
    ESP_LOGI(TAG, "%s mtu=%u", device_model_name(s_model), (unsigned)s_mtu);

    nova_response_t resp;
    if (require_auth) {
        // 需要 0x07/0x08 登录的是 NOVA / OMNI / HALO / PACE;AURA 免登录。
        info_set_state(BLE_CLIENT_AUTHENTICATING);
        err = ball_transact(NOVA_OP_ACCESS_INFO, NULL, 0, &resp);
        if (err != ESP_OK) return err;

        nova_access_info_t access;
        if (!nova_parse_access_info(&resp, &access)) {
            return session_failf(ESP_FAIL, "access payload");
        }

        char key[33];
        if (!nova_make_private_key((const uint8_t *)access.sn, access.sn_len,
                                   (const uint8_t *)access.code, access.code_len, key)) {
            return session_failf(ESP_FAIL, "auth key");
        }
        err = ball_transact(NOVA_OP_ACCESS_VERIFY, (const uint8_t *)key, 32, &resp);
        memset(key, 0, sizeof(key));
        if (err != ESP_OK) return err;
    }

    info_set_state(BLE_CLIENT_READING);
    err = ball_transact(NOVA_OP_DEVICE_INFO, NULL, 0, &resp);
    if (err != ESP_OK) return err;
    nova_device_info_t info;
    // AURA 的 0x01 允许末尾填充,用 aura_parse_device_info();其它球机要求
    // 长度严格自洽,继续用 nova_parse_device_info()。
    const bool info_ok = (s_model == DEVICE_MODEL_AURA)
                             ? aura_parse_device_info(&resp, &info)
                             : nova_parse_device_info(&resp, &info);
    if (!info_ok) {
        return session_failf(ESP_FAIL, "info payload");
    }
    info_set_nova_info(&info);
    ESP_LOGI(TAG, "%s info: sn=%s fw=%s hw=%s", device_model_name(s_model),
             info.sn, info.firmware_version, info.hardware_version);

    err = ball_transact(NOVA_OP_STATUS, NULL, 0, &resp);
    if (err != ESP_OK) return err;
    nova_state_t state = NOVA_STATE_UNKNOWN;
    uint16_t code = 0;
    if (!nova_parse_status(&resp, &state, &code)) {
        return session_failf(ESP_FAIL, "status payload");
    }
    // ble_client_info 里保存原始 level(UI 自己映射文案),code 为 u16 LE。
    const uint8_t level = resp.payload_len >= 1 ? resp.payload[0] : (uint8_t)state;
    // 电量只在 NOVA Bare 的 0x02 里允许取第 4 字节;P1/P2 多出来的字节是填充/
    // 噪声,不能当电量展示(电量的权威来源是 0x11,本应用未读)。
    const bool has_battery = (s_wire == WIRE_BARE && resp.payload_len >= 4);
    const uint8_t battery = has_battery ? resp.payload[3] : 0;
    info_set_nova_status(level, code, has_battery, battery);
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// SEEKER:唯一 UART 候选发现、订阅、JSON 请求与补包
// ---------------------------------------------------------------------------

static uint16_t chr_service_end(uint16_t svc_index)
{
    if (svc_index < s_svc_count) return s_svcs[svc_index].end_handle;
    return 0xFFFF;
}

static esp_err_t seeker_select_uart(void)
{
    size_t candidates = 0;
    int sel_write = -1;
    int sel_notify = -1;

    for (size_t s = 0; s < s_svc_count; s++) {
        for (size_t w = 0; w < s_chr_count; w++) {
            if (s_chrs[w].svc_index != s) continue;
            if ((s_chrs[w].props & (BLE_GATT_CHR_PROP_WRITE |
                                    BLE_GATT_CHR_PROP_WRITE_NO_RSP)) == 0) continue;
            for (size_t n = 0; n < s_chr_count; n++) {
                if (s_chrs[n].svc_index != s) continue;
                if ((s_chrs[n].props & (BLE_GATT_CHR_PROP_NOTIFY |
                                        BLE_GATT_CHR_PROP_INDICATE)) == 0) continue;
                if (!s_chrs[n].cccd_found) continue;
                candidates++;
                sel_write = (int)w;
                sel_notify = (int)n;
            }
        }
    }

    // 诊断信息(不含数据内容):所有服务/特征与候选数量,便于现场定位。
    for (size_t i = 0; i < s_svc_count; i++) {
        ESP_LOGI(TAG, "svc[%u] %s [%u..%u]", (unsigned)i, s_svcs[i].uuid,
                 (unsigned)s_svcs[i].start_handle, (unsigned)s_svcs[i].end_handle);
    }
    for (size_t i = 0; i < s_chr_count; i++) {
        ESP_LOGI(TAG, "chr[%u] val=%u props=0x%02x svc=%u cccd=%u", (unsigned)i,
                 (unsigned)s_chrs[i].val_handle, (unsigned)s_chrs[i].props,
                 (unsigned)s_chrs[i].svc_index, (unsigned)s_chrs[i].cccd_handle);
    }

    // 不是唯一候选就拒绝猜测:多写一个特征可能把控制指令发给错误对象。
    if (candidates != 1) {
        return session_failf(ESP_FAIL, "uart candidates %u", (unsigned)candidates);
    }

    s_uart_write_handle = s_chrs[sel_write].val_handle;
    s_uart_write_props = s_chrs[sel_write].props;
    s_uart_notify_handle = s_chrs[sel_notify].val_handle;
    s_uart_notify_props = s_chrs[sel_notify].props;
    s_uart_cccd_handle = s_chrs[sel_notify].cccd_handle;
    ESP_LOGI(TAG, "SEEKER uart write=%u notify=%u cccd=%u",
             (unsigned)s_uart_write_handle, (unsigned)s_uart_notify_handle,
             (unsigned)s_uart_cccd_handle);
    return ESP_OK;
}

static esp_err_t seeker_discover(void)
{
    s_svc_count = 0;
    s_chr_count = 0;
    s_dsc_scan_index = 0;
    s_disc_overflow = false;

    int rc = ble_gattc_disc_all_svcs(s_conn, seeker_svc_cb, NULL);
    if (rc != 0) return session_failf(rc, "disc svc %d", rc);
    int status = 0;
    esp_err_t err = wait_discovery(CLI_EVT_DISC_SVC_DONE, &status);
    if (err != ESP_OK) return err;
    if (status != 0) return session_failf(ESP_FAIL, "svc discovery %d", status);
    if (s_svc_count == 0) return session_failf(ESP_FAIL, "no services");

    rc = ble_gattc_disc_all_chrs(s_conn, 0x0001, 0xFFFF, seeker_chr_cb, NULL);
    if (rc != 0) return session_failf(rc, "disc chr %d", rc);
    err = wait_discovery(CLI_EVT_DISC_CHR_DONE, &status);
    if (err != ESP_OK) return err;
    if (status != 0) return session_failf(ESP_FAIL, "chr discovery %d", status);

    // 候选 notify 特征逐个找 CCCD(0x2902);没有 CCCD 的 notify 不能订阅。
    // NimBLE 对 start_handle >= end_handle(或空范围)直接返回 BLE_HS_EINVAL,
    // 因此先跳过"值句柄已是服务最后一个属性、没有 CCCD 空间"的候选;
    // 万一仍返回 EINVAL,只跳过该候选,不能因此废掉整个会话。
    for (size_t i = 0; i < s_chr_count; i++) {
        if ((s_chrs[i].props & (BLE_GATT_CHR_PROP_NOTIFY |
                                BLE_GATT_CHR_PROP_INDICATE)) == 0) continue;
        const uint16_t svc_end = chr_service_end(s_chrs[i].svc_index);
        if (svc_end == 0 || s_chrs[i].val_handle >= svc_end) {
            ESP_LOGI(TAG, "skip dsc: chr[%u] val=%u svc_end=%u (no CCCD room)",
                     (unsigned)i, (unsigned)s_chrs[i].val_handle, (unsigned)svc_end);
            continue;
        }
        s_dsc_scan_index = i;
        rc = ble_gattc_disc_all_dscs(s_conn, s_chrs[i].val_handle, svc_end,
                                     seeker_dsc_cb, NULL);
        if (rc == BLE_HS_EINVAL) {
            ESP_LOGI(TAG, "skip dsc: chr[%u] val=%u svc_end=%u (EINVAL)",
                     (unsigned)i, (unsigned)s_chrs[i].val_handle, (unsigned)svc_end);
            continue;
        }
        if (rc != 0) return session_failf(rc, "disc dsc %d", rc);
        err = wait_discovery(CLI_EVT_DISC_DSC_DONE, &status);
        if (err != ESP_OK) return err;
        if (status != 0) return session_failf(ESP_FAIL, "dsc discovery %d", status);
    }

    if (s_disc_overflow) return session_failf(ESP_FAIL, "too many gatt items");
    return seeker_select_uart();
}

// SEEKER 写:特征支持 no-response 时优先用它(Android SDK writeWithResponse=false);
// 只有 with-response 特征才等待完成回调。
static esp_err_t seeker_send_json(const char *json, size_t len)
{
    if (json == NULL || len == 0) return session_failf(ESP_FAIL, "request build");

    const uint16_t capacity = s_mtu > 3 ? (uint16_t)(s_mtu - 3) : 20;
    if (len > capacity) {
        return session_failf(ESP_FAIL, "request %u > mtu", (unsigned)len);
    }

    int rc;
    if ((s_uart_write_props & BLE_GATT_CHR_PROP_WRITE_NO_RSP) != 0) {
        rc = ble_gattc_write_no_rsp_flat(s_conn, s_uart_write_handle, json,
                                         (uint16_t)len);
        if (rc != 0) return session_failf(rc, "gatt write %d", rc);
        return ESP_OK;
    }

    rc = ble_gattc_write_flat(s_conn, s_uart_write_handle, json, (uint16_t)len,
                              write_done_cb, NULL);
    if (rc != 0) return session_failf(rc, "gatt write %d", rc);

    const uint32_t timeout = step_timeout(CLI_WRITE_TIMEOUT_MS);
    if (timeout == 0) return session_failf(ESP_ERR_TIMEOUT, "session timeout");
    client_msg_t msg;
    esp_err_t err = wait_kind(CLI_EVT_WRITE_DONE, &msg, timeout);
    if (err == ESP_ERR_TIMEOUT) return session_failf(ESP_ERR_TIMEOUT, "write timeout");
    if (err != ESP_OK) return err;
    if (msg.status != 0) return session_failf(ESP_FAIL, "gatt status %d", msg.status);
    return ESP_OK;
}

// 发一条 SEEKER 请求并等待完整响应。响应按 22 字节头分包到达,worker 收齐后
// 再做业务解析;缺包时发 get_ble_pkg 补包请求(最多 3 轮)。
static esp_err_t seeker_transact(const char *method, const char *request)
{
    seeker_assembler_reset(&s_seeker_asm);
    seeker_assembler_begin_request(&s_seeker_asm);
    s_seeker_feed = SEEKER_FEED_NONE;
    s_seeker_body_len = 0;
    s_rx_overflow = false;

    esp_err_t err = seeker_send_json(request, strlen(request));
    if (err != ESP_OK) return err;

    const uint32_t timeout = step_timeout(CLI_SEEKER_REQ_TIMEOUT_MS);
    if (timeout == 0) return session_failf(ESP_ERR_TIMEOUT, "session timeout");

    const int64_t deadline = now_us() + (int64_t)timeout * 1000;
    int64_t last_progress = now_us();
    bool got_data = false;
    unsigned repairs = 0;

    for (;;) {
        if (s_seeker_feed == SEEKER_FEED_COMPLETE) {
            // 响应必须回显请求的 method;不是本次请求的应答(例如上一条的迟到
            // 消息或被拼接的其它 JSON)就丢掉重组状态继续等,直到本请求超时。
            seeker_response_t rsp;
            if (!seeker_parse_response((const char *)s_seeker_body, s_seeker_body_len, &rsp)) {
                return session_failf(ESP_FAIL, "json parse");
            }
            if (strcmp(rsp.method, method) != 0) {
                ESP_LOGD(TAG, "ignore seeker response method=%s (want %s)",
                         rsp.method, method);
                seeker_assembler_reset(&s_seeker_asm);
                s_seeker_feed = SEEKER_FEED_NONE;
                s_seeker_body_len = 0;
                continue;
            }
            return ESP_OK;
        }
        if (s_seeker_feed == SEEKER_FEED_ERROR) return session_failf(ESP_FAIL, "seeker frame error");
        if (s_rx_overflow) return session_failf(ESP_FAIL, "rx overflow");
        if (s_cancel) return ESP_ERR_INVALID_STATE;

        const int64_t left = deadline - now_us();
        if (left <= 0) {
            return session_failf(ESP_ERR_TIMEOUT, "response timeout");
        }
        uint32_t slice = (uint32_t)(left / 1000);
        if (slice > 200) slice = 200;
        if (slice == 0) slice = 1;

        client_msg_t msg;
        err = pump_msg(&msg, slice);
        if (err == ESP_OK) {
            if (msg.kind == CLI_EVT_NOTIFY) {
                if (msg.conn_handle != s_conn || msg.attr_handle != s_uart_notify_handle) {
                    continue;
                }
                size_t body_len = 0;
                s_seeker_feed = seeker_assembler_feed(
                    &s_seeker_asm, msg.data, msg.len, s_seeker_body,
                    sizeof(s_seeker_body), &body_len);
                if (s_seeker_feed == SEEKER_FEED_COMPLETE) {
                    s_seeker_body_len = body_len;
                }
                got_data = true;
                last_progress = now_us();
                continue;
            }
            // MTU/写完成等迟到事件:忽略。
            continue;
        }
        if (err != ESP_ERR_TIMEOUT) return err;

        // 有分片但闲置超过 1s,说明丢包:请求设备重发缺失分包。
        if (seeker_assembler_incomplete(&s_seeker_asm) && got_data &&
            (now_us() - last_progress) >= (int64_t)CLI_REPAIR_IDLE_MS * 1000 &&
            repairs < CLI_REPAIR_ROUNDS) {
            uint16_t missing[SEEKER_MAX_PACKETS];
            const size_t missing_count = seeker_assembler_missing(
                &s_seeker_asm, missing, SEEKER_MAX_PACKETS);
            if (missing_count > 0) {
                char repair[512];
                const size_t repair_len = seeker_build_repair_request(
                    repair, sizeof(repair), method, missing, missing_count);
                if (repair_len == 0) return session_failf(ESP_FAIL, "repair build");
                err = seeker_send_json(repair, repair_len);
                if (err != ESP_OK) return err;
                repairs++;
                last_progress = now_us();
                ESP_LOGI(TAG, "SEEKER repair round %u, missing=%u",
                         (unsigned)repairs, (unsigned)missing_count);
            }
        }
    }
}

static esp_err_t seeker_parse_current(seeker_response_t *resp)
{
    if (s_seeker_body_len == 0) return session_failf(ESP_FAIL, "empty response");
    if (!seeker_parse_response((const char *)s_seeker_body, s_seeker_body_len, resp)) {
        return session_failf(ESP_FAIL, "json parse");
    }
    return ESP_OK;
}

static esp_err_t seeker_session(void)
{
    esp_err_t err = seeker_discover();
    if (err != ESP_OK) return err;

    // Android SDK 的 connectionStableMs=500:发现完成后先稳定一下再订阅通知。
    uint32_t disc_stable = CLI_SEEKER_DISC_STABLE_MS;
    if (disc_stable > remaining_ms()) disc_stable = remaining_ms();
    err = delay_capped(disc_stable);
    if (err == ESP_ERR_INVALID_STATE) return err;
    if (err != ESP_OK) return session_failf(ESP_ERR_TIMEOUT, "session timeout");

    info_set_state(BLE_CLIENT_SUBSCRIBING);
    const bool notify = (s_uart_notify_props & BLE_GATT_CHR_PROP_NOTIFY) != 0;
    err = write_cccd(s_uart_cccd_handle, notify);
    if (err != ESP_OK) return err;

    // SEEKER 的请求是 JSON,按规格请求 247 的 MTU;仍要求协商结果 >=64。
    (void)ble_att_set_preferred_mtu(CLI_MTU_REQUEST);
    err = exchange_mtu();
    if (err != ESP_OK) return err;
    err = require_mtu(CLI_SEEKER_MTU_MIN, "seeker");
    if (err != ESP_OK) return err;
    ESP_LOGI(TAG, "SEEKER mtu=%u", (unsigned)s_mtu);

    uint32_t stable = CLI_SEEKER_STABLE_MS;
    if (stable > remaining_ms()) stable = remaining_ms();
    err = delay_capped(stable);
    if (err == ESP_ERR_INVALID_STATE) return err;
    if (err != ESP_OK) return session_failf(ESP_ERR_TIMEOUT, "session timeout");

    info_set_state(BLE_CLIENT_READING);

    char request[256];
    size_t request_len = seeker_build_request(request, sizeof(request), "device_info", NULL);
    if (request_len == 0) return session_failf(ESP_FAIL, "request build");
    err = seeker_transact("device_info", request);
    if (err != ESP_OK) return err;

    seeker_response_t resp;
    err = seeker_parse_current(&resp);
    if (err != ESP_OK) return err;

    if (resp.code == SEEKER_CODE_SUCCESS) {
        seeker_device_info_t info;
        if (!seeker_parse_device_info((const char *)s_seeker_body, s_seeker_body_len, &info)) {
            return session_failf(ESP_FAIL, "info parse");
        }
        // 规格要求 data 至少有一个版本字符串非空,否则视为无效响应。
        if (info.firmware_version[0] == '\0' && info.system_version[0] == '\0') {
            return session_failf(ESP_FAIL, "info empty");
        }
        info_set_seeker_info(&info);
        if (info.has_token) {
            snprintf(s_token, sizeof(s_token), "%s", info.token);
            s_has_token = true;
        }
    } else if (resp.code == SEEKER_CODE_VERIFY_REQUIRED_1 ||
               resp.code == SEEKER_CODE_VERIFY_REQUIRED_2) {
        // BLE 上没有可用的 random code 通道;不伪造 challenge,直接明确失败。
        return session_failf(ESP_FAIL,
                             "verify unsupported over BLE (random code unavailable)");
    } else {
        return session_failf(ESP_FAIL, "device_info code %d", resp.code);
    }

    request_len = seeker_build_request(request, sizeof(request), "status",
                                       s_has_token ? s_token : NULL);
    if (request_len == 0) return session_failf(ESP_FAIL, "request build");
    err = seeker_transact("status", request);
    if (err != ESP_OK) return err;

    err = seeker_parse_current(&resp);
    if (err != ESP_OK) return err;
    if (resp.code != SEEKER_CODE_SUCCESS) {
        return session_failf(ESP_FAIL, "status code %d", resp.code);
    }
    char hawkeye[24];
    if (!seeker_parse_status((const char *)s_seeker_body, s_seeker_body_len, hawkeye)) {
        return session_failf(ESP_FAIL, "status parse");
    }
    info_set_seeker_status(hawkeye);
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// 会话编排与 worker
// ---------------------------------------------------------------------------

static void session_run(const ble_scan_dev_t *dev)
{
    s_conn = BLE_HS_CONN_HANDLE_NONE;
    s_mtu = 23;
    s_failed = false;
    s_fail_err = 0;
    s_fail_detail[0] = '\0';
    s_rx_overflow = false;
    s_ball_rx_len = 0;
    s_seeker_feed = SEEKER_FEED_NONE;
    s_seeker_body_len = 0;
    s_token[0] = '\0';
    s_has_token = false;
    s_write_path = -1;
    s_cmd_timeout_ms = CLI_NOVA_CMD_TIMEOUT_MS;
    seeker_assembler_reset(&s_seeker_asm);
    drain_queue();

    s_model = device_model_from_name(dev->name);
    s_wire = wire_mode_for_model(s_model);
    info_init(dev, s_model);

    // 只读能力白名单:未实现协议的型号不连接、不发任何帧。
    if (!device_model_readable(s_model)) {
        info_terminal(BLE_CLIENT_UNSUPPORTED, ESP_FAIL, "model unsupported");
        log_terminal_summary(BLE_CLIENT_UNSUPPORTED);
        session_wipe_sensitive();
        return;
    }

    // 连接前先确认扫描条目新鲜:列表可能来自已结束的旧扫描,设备可能已经离开
    // 范围或地址已轮换,直接连会得到 connect status 13 / 0x3E。
    // 判定:当前不在扫描,或条目超过 8s 没有更新 -> 先重新扫描,再等目标出现。
    const ble_scan_state_t pre_scan_state = ble_scan_state();
    s_scan_resume_after = (pre_scan_state == BLE_SCAN_SCANNING);
    // 会话总预算从"一进入读取"就开始计算:刷新扫描的 5s 等待也算在 45s 内。
    s_deadline_us = now_us() + (int64_t)CLI_SESSION_CAP_MS * 1000;
    ble_scan_dev_t target = *dev;
    const uint32_t now = (uint32_t)(now_us() / 1000);
    const uint32_t age_ms = now - dev->last_seen_ms;
    if (pre_scan_state != BLE_SCAN_SCANNING || age_ms > CLI_MAX_DEV_AGE_MS) {
        ESP_LOGI(TAG, "stale scan entry: state=%d age=%ums, restart scan before read",
                 (int)pre_scan_state, (unsigned)age_ms);
        const uint32_t refresh_baseline_ms = now;
        const esp_err_t rerr = ble_scan_restart();
        if (rerr != ESP_OK && rerr != ESP_ERR_INVALID_STATE) {
            // INVALID_STATE 是"已经在扫描";其它错误只记录,仍尝试等目标出现。
            ESP_LOGW(TAG, "ble_scan_restart failed: %s", esp_err_to_name(rerr));
        }
        if (!wait_for_fresh_entry(dev, refresh_baseline_ms, &target)) {
            // 目标没有在刷新后重新出现:恢复会话前的扫描语义再报错。
            if (!s_scan_resume_after && ble_scan_state() == BLE_SCAN_SCANNING) {
                (void)ble_scan_pause();
            }
            if (s_cancel) {
                info_terminal(BLE_CLIENT_FAILED, ESP_ERR_INVALID_STATE, "cancelled");
            } else {
                info_terminal(BLE_CLIENT_FAILED, ESP_FAIL,
                              "device not found after rescan");
            }
            log_terminal_summary(BLE_CLIENT_FAILED);
            session_wipe_sensitive();
            return;
        }
        ESP_LOGI(TAG, "fresh scan entry: rssi=%d", (int)target.rssi);
    }

    // 连接会和扫描抢射频:刷新完成后暂停扫描,只有确实暂停成功才在结束时恢复。
    s_paused_by_client = false;
    if (ble_scan_state() == BLE_SCAN_SCANNING && ble_scan_pause() == ESP_OK) {
        s_paused_by_client = true;
    }

    esp_err_t err = cli_connect(&target);
    if (err == ESP_OK) {
        info_set_state(BLE_CLIENT_DISCOVERING);
        switch (s_model) {
        case DEVICE_MODEL_SEEKER:
            err = seeker_session();
            break;
        case DEVICE_MODEL_NOVA:
            s_cmd_timeout_ms = CLI_NOVA_CMD_TIMEOUT_MS;
            err = ball_session(CLI_NOVA_STABLE_MS, true);
            break;
        case DEVICE_MODEL_OMNI:
        case DEVICE_MODEL_HALO:
            s_cmd_timeout_ms = CLI_OMNI_CMD_TIMEOUT_MS;
            err = ball_session(CLI_OMNI_STABLE_MS, true);
            break;
        case DEVICE_MODEL_PACE:
            s_cmd_timeout_ms = CLI_PACE_CMD_TIMEOUT_MS;
            err = ball_session(CLI_PACE_STABLE_MS, true);
            break;
        case DEVICE_MODEL_AURA:
            // AURA 不做 0x07/0x08 登录;若 0x01 被拒绝只报 result,不尝试认证。
            s_cmd_timeout_ms = CLI_AURA_CMD_TIMEOUT_MS;
            err = ball_session(CLI_AURA_STABLE_MS, false);
            break;
        default:
            err = session_failf(ESP_FAIL, "model unsupported");
            break;
        }
    }

    // 用户取消优先级最高:即使底层返回 INVALID_STATE,也给出明确 detail。
    if (s_cancel) {
        s_failed = true;
        s_fail_err = ESP_ERR_INVALID_STATE;
        snprintf(s_fail_detail, sizeof(s_fail_detail), "cancelled");
    }

    if (s_conn != BLE_HS_CONN_HANDLE_NONE) {
        (void)ble_gap_terminate(s_conn, CLI_HCI_REASON_USER_TERM);
        wait_disconnect(CLI_DISCONNECT_WAIT_MS);
        s_conn = BLE_HS_CONN_HANDLE_NONE;
    }
    drain_queue();
    info_set_connected(false);

    if (s_paused_by_client) {
        if (s_scan_resume_after) {
            // 会话前就在扫描:恢复扫描。
            (void)ble_scan_resume();
        }
        // 否则保持 PAUSED:会话前是 FINISHED/PAUSED,不把新一轮 20s 扫描留在
        // 后台;UI 显示"已暂停",用户可长按确定重新扫描。
        s_paused_by_client = false;
    }

    ble_client_state_t final_state = BLE_CLIENT_DONE;
    int final_err = 0;
    const char *final_detail = "";
    if (s_failed) {
        final_state = BLE_CLIENT_FAILED;
        final_err = s_fail_err;
        final_detail = s_fail_detail;
    } else if (err != ESP_OK) {
        final_state = BLE_CLIENT_FAILED;
        final_err = ESP_FAIL;
        final_detail = "session aborted";
    }
    info_terminal(final_state, final_err, final_detail);
    log_terminal_summary(final_state);
    session_wipe_sensitive();
}

static void client_worker(void *arg)
{
    (void)arg;
    bool hwm_logged = false;
    for (;;) {
        ble_scan_dev_t dev;
        if (xQueueReceive(s_req_queue, &dev, portMAX_DELAY) != pdTRUE) continue;
        s_cancel = false;
        session_run(&dev);
        info_lock();
        s_busy = false;
        info_unlock();
        if (!hwm_logged) {
            // 只在第一次会话结束后打印一次,给真机验收看任务栈还有多少余量。
            hwm_logged = true;
            ESP_LOGI(TAG, "client stack hwm=%u",
                     (unsigned)uxTaskGetStackHighWaterMark(NULL));
        }
    }
}

// ---------------------------------------------------------------------------
// 公开 API
// ---------------------------------------------------------------------------

esp_err_t ble_client_start(void)
{
    if (s_started) return ESP_ERR_INVALID_STATE;

    if (s_info_mutex == NULL) s_info_mutex = xSemaphoreCreateMutex();
    if (s_info_mutex == NULL) return ESP_ERR_NO_MEM;

    if (s_msg_queue == NULL) {
        s_msg_queue = xQueueCreate(CLI_MSG_QUEUE_DEPTH, sizeof(client_msg_t));
    }
    if (s_msg_queue == NULL) return ESP_ERR_NO_MEM;

    if (s_req_queue == NULL) {
        s_req_queue = xQueueCreate(1, sizeof(ble_scan_dev_t));
    }
    if (s_req_queue == NULL) return ESP_ERR_NO_MEM;

    info_lock();
    memset(&s_info, 0, sizeof(s_info));
    s_info.state = BLE_CLIENT_IDLE;
    s_info.model = DEVICE_MODEL_UNKNOWN;
    s_busy = false;
    info_unlock();

    if (xTaskCreate(client_worker, "ble_client", CLI_TASK_STACK, NULL,
                    CLI_TASK_PRIO, &s_task) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    s_started = true;
    return ESP_OK;
}

esp_err_t ble_client_request_read(const ble_scan_dev_t *dev)
{
    if (dev == NULL) return ESP_ERR_INVALID_STATE;

    // main.c 只在 ble_scan_start() 成功后调用 ble_client_start();若扫描先失败
    // 后重试成功,客户端可能一直没启动。这里按需懒启动,失败则返回明确错误。
    if (!s_started) {
        const esp_err_t start_err = ble_client_start();
        if (start_err != ESP_OK) {
            ESP_LOGE(TAG, "lazy client start failed: %s", esp_err_to_name(start_err));
            return start_err;
        }
    }

    info_lock();
    if (s_busy) {
        info_unlock();
        return ESP_ERR_INVALID_STATE;
    }
    s_busy = true;
    // 先把"连接中"写进快照,UI 不用等 worker 被调度就能显示进度。
    memset(&s_info, 0, sizeof(s_info));
    s_info.state = BLE_CLIENT_CONNECTING;
    s_info.model = device_model_from_name(dev->name);
    snprintf(s_info.device_name, sizeof(s_info.device_name), "%s", dev->name);
    info_unlock();

    char addr[18];
    ble_scan_format_addr(dev->addr, addr);
    const uint32_t now_ms = (uint32_t)(now_us() / 1000);
    ESP_LOGI(TAG, "read request: model=%s name=%s addr=%s type=%u conn=%d age=%ums",
             device_model_name(s_info.model), dev->name, addr,
             (unsigned)dev->addr_type, dev->connectable ? 1 : 0,
             (unsigned)(now_ms - dev->last_seen_ms));

    if (xQueueSend(s_req_queue, dev, 0) != pdTRUE) {
        info_lock();
        s_busy = false;
        info_unlock();
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

esp_err_t ble_client_cancel(void)
{
    // 只置标志,由 worker 在等待点发现并收尾;无会话时也安全。
    s_cancel = true;
    return ESP_OK;
}

void ble_client_get_info(ble_client_info_t *out)
{
    if (out == NULL) return;
    if (s_info_mutex == NULL) {
        memset(out, 0, sizeof(*out));
        out->state = BLE_CLIENT_IDLE;
        out->model = DEVICE_MODEL_UNKNOWN;
        return;
    }
    info_lock();
    *out = s_info;
    info_unlock();
}

bool ble_client_busy(void)
{
    bool busy = false;
    if (s_info_mutex != NULL) {
        info_lock();
        busy = s_busy;
        info_unlock();
    }
    return busy;
}
