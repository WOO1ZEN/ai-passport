// main/ble_scan.c —— BLE 信号扫描服务(NimBLE observer)的生命周期与设备表访问。
//
// 职责划分:
//   * 本文件只做三件事:NimBLE host 的初始化/停止、扫描的启动/暂停/恢复,
//     以及给设备表加锁;
//   * 广播解析、合并、排序、信号等级都在 main/ble_scan_model.c,保持无锁可测;
//   * 界面渲染在 main/ble_scan_ui.c,本文件绝不访问 LVGL。
//
// 并发模型:
//   * NimBLE 的 GAP 回调运行在专用的 host 任务里;UI 任务通过 ble_scan_snapshot()
//     读取设备表。设备表由 s_lock 互斥锁保护,所有公开函数都可从任意任务调用;
//   * 锁的获取顺序固定为"本模块互斥锁 -> NimBLE 内部锁",回调与公开 API 都不在
//     持有本模块锁时等待另一个任务,因此不会互相死锁;
//   * s_state 是单字 volatile 状态,回调里先拿锁再判断,保证暂停/停止与
//     DISC_COMPLETE 的重启不会交错。
//
// 生命周期(与 main/demo_ble.c 保持同一套所有权纪律):
//   start:  nvs_flash_init -> nimble_port_init -> 建锁/信号量 -> sync 回调里开扫描
//   pause:  置 PAUSED(阻止重启) -> ble_gap_disc_cancel()
//   resume: 重新 ble_gap_disc()
//   stop:   ble_gap_disc_cancel -> nimble_port_stop -> 等 host 任务退出 ->
//           vTaskDelete -> nimble_port_deinit。清理失败时保留已分配资源,
//           让下一次 stop 可以重试,而不是把句柄丢掉造成泄漏。
#include "ble_scan.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "host/ble_gap.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nvs_flash.h"

#include <string.h>

static const char *TAG = "ble_scan";

// 等待 NimBLE host 任务退出 nimble_port_run() 的上限。与 demo_ble.c 一致。
#define BLE_SCAN_STOP_TIMEOUT_MS 2000
// 每次开机只扫描这么久:省电,也符合"开机扫一轮、重启再扫"的产品行为。
// 扫描结束后结果表保留(设备仍可用于详情/读取),不会再自动过期。
#define BLE_SCAN_BOOT_DURATION_MS 20000

static ble_scan_table_t s_table;
// 设备表互斥锁。首次 start 时创建,之后常驻:stop 期间 UI/按键任务仍可能调用
// snapshot/clear,删除锁会和这些调用竞争,常驻一个静态句柄更安全。
static SemaphoreHandle_t s_lock;
static SemaphoreHandle_t s_host_stopped;
static TaskHandle_t s_host_task;
static uint8_t s_own_addr_type;
static bool s_initialized;
static bool s_stop_in_progress;
static bool s_host_done;
// true 表示"这一次 start 会话持有 NimBLE host"(start..stop 之间)。stop 先清掉它,
// 回调里的 DISC_COMPLETE 就不再重启扫描。
static volatile bool s_start_requested;
// 用户是否希望正在扫描:pause()/stop() 清掉,start()/resume() 置起。
// 它和 s_start_requested 分开,才能区分"暂停"与"宿主复位":复位后 if 用户没暂停,
// sync 回调可以自动恢复扫描;用户暂停了就保持 PAUSED,等 resume 再开。
static volatile bool s_scan_wanted;
static volatile ble_scan_state_t s_state = BLE_SCAN_OFF;
static volatile int s_error;

static int gap_event(struct ble_gap_event *event, void *arg);

// 毫秒时钟:esp_timer_get_time() 是 64 位微秒,截成 32 位毫秒正好配合模型层的
// 无符号差值比较(跨回绕安全)。
static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static bool lock_take(void)
{
    SemaphoreHandle_t lock = s_lock;
    if (lock == NULL) return false;
    return xSemaphoreTake(lock, portMAX_DELAY) == pdTRUE;
}

static void lock_give(void)
{
    if (s_lock != NULL) xSemaphoreGive(s_lock);
}

// 持有 s_lock 时调用。启动 legacy 主动扫描(filter_duplicates=0 才能看到周期广播
// 的计数变化),失败由调用方记录并转入 FAILED。
static int discovery_start_locked(void)
{
    struct ble_gap_disc_params params = { 0 };
    // 取舍:这是电池随身设备。interval 取 FAST_INTERVAL_MAX(60 ms)、window 取
    // FAST_WINDOW(30 ms),即约 50% 占空比;代价是发现新设备略慢,换来射频占空比
    // 减半、功耗明显下降。改回 FAST_INTERVAL_MIN 可获得连续扫描但耗电翻倍。
    // 注:NimBLE 的 legacy discovery 每个参数只有一个 interval 字段,没有 min/max
    // 两个值,所以这里用 MAX 表达扫描周期的上限。
    params.itvl = BLE_GAP_SCAN_FAST_INTERVAL_MAX;
    params.window = BLE_GAP_SCAN_FAST_WINDOW;
    params.filter_policy = 0;   // 不使用白名单
    params.limited = 0;         // 通用发现流程
    params.passive = 0;         // 主动扫描:发 SCAN_REQ 以拿到扫描响应里的设备名
    params.filter_duplicates = 0;  // 不去重,才能累计每个设备的广播次数
    params.disable_observer_mode = 0;

    // 有限时长:到点后 NimBLE 会发 DISC_COMPLETE,由 handle_disc_complete()
    // 把状态切到 FINISHED,而不是像以前那样无限续扫。
    const int rc = ble_gap_disc(s_own_addr_type, BLE_SCAN_BOOT_DURATION_MS,
                                &params, gap_event, NULL);
    if (rc == 0) s_state = BLE_SCAN_SCANNING;
    return rc;
}

// 持有 s_lock 时调用:记录错误并停在 FAILED,等待 UI 展示原因。
static void fail_locked(int rc)
{
    s_error = rc;
    s_state = BLE_SCAN_FAILED;
}

// 把一条 DISC 事件合并进设备表。广播内容不可信,解析全部交给模型层。
static void handle_disc(const struct ble_gap_event *event)
{
    ble_adv_fields_t fields;
    ble_scan_parse_adv(event->disc.data, event->disc.length_data, &fields);

    ble_scan_dev_t obs;
    memset(&obs, 0, sizeof(obs));
    memcpy(obs.addr, event->disc.addr.val, sizeof(obs.addr));
    obs.addr_type = event->disc.addr.type;
    obs.rssi = event->disc.rssi;
    // 只有 ADV_IND / ADV_DIRECT_IND 才表示"可连接";扫描响应本身不携带该信息,
    // 交给模型层的 OR 语义保留已经观测到的 true。
    const uint8_t type = event->disc.event_type;
    obs.connectable = (type == BLE_HCI_ADV_RPT_EVTYPE_ADV_IND ||
                       type == BLE_HCI_ADV_RPT_EVTYPE_DIR_IND);
    obs.tx_power = fields.has_tx_power ? fields.tx_power : BLE_SCAN_RSSI_UNKNOWN;
    obs.adv_flags = fields.has_flags ? fields.flags : 0;
    if (fields.has_name) memcpy(obs.name, fields.name, sizeof(obs.name));
    // merge() 用 last_seen_ms 同时决定新条目的 first_seen / last_seen,这里传当前时刻。
    obs.last_seen_ms = now_ms();
    obs.first_seen_ms = obs.last_seen_ms;
    obs.seen_count = 1;

    if (!lock_take()) return;
    // 暂停/停止之后仍可能有在途事件:状态不是 SCANNING 就不写入,避免暂停后表格还变。
    if (s_state == BLE_SCAN_SCANNING) {
        ble_scan_dev_t *stored = ble_scan_table_merge(&s_table, &obs);
        // 每台设备只在上机排查时记录一次"是否收到广播名"。空口里很多设备
        // (手机/手表/信标)本来就不广播名称,这条日志能区分"没名字"和"解析坏了"。
        if (stored != NULL && stored->seen_count == 1 && stored->name[0] != '\0') {
            static bool s_logged_named;
            if (!s_logged_named) {
                s_logged_named = true;
                ESP_LOGI(TAG, "收到带名称的广播: \"%s\"", stored->name);
            }
        }
    }
    lock_give();
}

// 有限时长(20 秒)扫描到点后 NimBLE 会发 DISC_COMPLETE。这里把状态切到
// FINISHED 并清掉"还想扫描"的意愿,让后续 on_sync()/控制器复位都不再自动续扫;
// 结果表保留,用户仍可浏览设备、进入详情并读取信息。pause/stop 触发的取消也会
// 走到这里,但它们会先把状态置成 PAUSED/OFF,因此不会被误判为"扫描完成"。
static void handle_disc_complete(void)
{
    if (!lock_take()) return;
    if (s_state == BLE_SCAN_SCANNING && s_start_requested && s_scan_wanted) {
        s_scan_wanted = false;
        s_state = BLE_SCAN_FINISHED;
        ESP_LOGI(TAG, "开机扫描 %d 秒完成,停止扫描(结果保留;重启后重新扫描)",
                 BLE_SCAN_BOOT_DURATION_MS / 1000);
    }
    // PAUSED / OFF / FINISHED / FAILED:静默,不重启。
    lock_give();
}

static int gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_DISC:
        handle_disc(event);
        break;
    case BLE_GAP_EVENT_DISC_COMPLETE:
        handle_disc_complete();
        break;
    default:
        break;
    }
    return 0;
}

static void on_reset(int reason)
{
    ESP_LOGW(TAG, "NimBLE host 复位: reason=%d(同步后会自动恢复扫描)", reason);
    // 控制器/Host 复位:记录原因。若用户处于暂停状态,保留 PAUSED 语义(等 resume);
    // 否则标 FAILED,sync_cb 再次到来时会自动恢复扫描,不需要用户重新 start。
    s_error = reason;
    // 已经完成本轮扫描或用户主动暂停:不要因为控制器复位把它们改写成 FAILED,
    // 也不要让随后的 sync 回调重新开始扫描。
    if (!s_scan_wanted && (s_state == BLE_SCAN_PAUSED || s_state == BLE_SCAN_FINISHED)) return;
    s_state = BLE_SCAN_FAILED;
}

static void on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc == 0) rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    if (rc != 0) {
        on_reset(rc);
        return;
    }
    if (!s_start_requested || !s_scan_wanted) return;

    if (!lock_take()) return;
    // 正常启动是 STARTING;控制器/Host 复位后是 FAILED(见 on_reset)。两种情况下
    // 只要用户仍希望扫描,就在 sync 之后重新开扫,让协议栈自行恢复。
    if (s_state == BLE_SCAN_STARTING || s_state == BLE_SCAN_FAILED) {
        rc = discovery_start_locked();
        if (rc != 0) {
            fail_locked(rc);
        } else {
            s_error = 0;
            ESP_LOGI(TAG, "NimBLE host 已同步,开始 BLE 扫描");
        }
    }
    lock_give();
}

static void host_task(void *arg)
{
    (void)arg;
    nimble_port_run();
    // 与 demo_ble.c 一致:不用 nimble_port_freertos 的封装,因为它会自己保存任务句柄,
    // 且忽略建任务失败。这里由本模块负责创建、确认退出和删除。
    xSemaphoreGive(s_host_stopped);
    for (;;) vTaskSuspend(NULL);
}

esp_err_t ble_scan_start(void)
{
    // 正在运行/暂停:拒绝重复启动,但不把当前可用状态改写为 FAILED。
    if (s_state == BLE_SCAN_STARTING || s_state == BLE_SCAN_SCANNING ||
        s_state == BLE_SCAN_PAUSED) {
        s_error = ESP_ERR_INVALID_STATE;
        return ESP_ERR_INVALID_STATE;
    }

    // FAILED 且 host 仍然存活(例如 ble_gap_disc 失败、或控制器复位):
    // 直接在现有 host 上重试 discovery,不重建协议栈,避免闪断和资源泄漏。
    if (s_initialized) {
        if (s_state != BLE_SCAN_FAILED || !s_host_task ||
            s_stop_in_progress || !s_start_requested) {
            // 清理未完成的 FAILED(见 stop 的 deinit 重试)必须先 stop()。
            s_error = ESP_ERR_INVALID_STATE;
            return ESP_ERR_INVALID_STATE;
        }
        if (!lock_take()) {
            s_error = ESP_ERR_INVALID_STATE;
            return ESP_ERR_INVALID_STATE;
        }
        s_scan_wanted = true;
        const int rc = discovery_start_locked();
        if (rc != 0) {
            fail_locked(rc);
        } else {
            s_error = 0;
            ESP_LOGI(TAG, "NimBLE host 已同步,开始 BLE 扫描");
        }
        lock_give();
        return rc == 0 ? ESP_OK : ESP_FAIL;
    }

    s_state = BLE_SCAN_STARTING;
    s_error = 0;
    s_scan_wanted = true;
    s_stop_in_progress = false;
    s_host_done = false;

    // 互斥锁只需创建一次;常驻到进程结束(见 s_lock 的注释)。
    if (s_lock == NULL) {
        s_lock = xSemaphoreCreateMutex();
        if (s_lock == NULL) {
            s_error = ESP_ERR_NO_MEM;
            s_state = BLE_SCAN_FAILED;
            return ESP_ERR_NO_MEM;
        }
    }
    // 新会话从空表开始,避免上次扫描的陈旧条目在 15 秒内继续显示。
    if (lock_take()) {
        ble_scan_table_clear(&s_table);
        lock_give();
    }

    // NVS 是 NimBLE 绑定信息与控制器配置的依赖。失败时按约定不擦除分区,
    // 直接上报,让调用方决定是否恢复,避免误删配网/校准数据。
    esp_err_t err = nvs_flash_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS 初始化失败: %s", esp_err_to_name(err));
        s_error = err;
        s_state = BLE_SCAN_FAILED;
        return err;
    }

    err = nimble_port_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init 失败: %s", esp_err_to_name(err));
        s_error = err;
        s_state = BLE_SCAN_FAILED;
        return err;
    }
    s_initialized = true;

    s_host_stopped = xSemaphoreCreateBinary();
    if (s_host_stopped == NULL) {
        err = ESP_ERR_NO_MEM;
        goto failed_start;
    }

    // observer 角色只需要 Host 的 sync/reset 回调;扫描参数在回调里设置。
    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb = on_sync;
    s_start_requested = true;

    if (xTaskCreatePinnedToCore(host_task, "nimble_host", NIMBLE_HS_STACK_SIZE,
                               NULL, configMAX_PRIORITIES - 4, &s_host_task,
                               NIMBLE_CORE) != pdPASS) {
        err = ESP_ERR_NO_MEM;
        goto failed_start;
    }
    return ESP_OK;

failed_start:
    // host 任务可能还没创建,不能依赖 nimble_port_stop();交给 stop() 按当前
    // 持有情况清理,清理失败时它会保留资源供重试。
    s_start_requested = false;
    s_scan_wanted = false;
    esp_err_t cleanup = ble_scan_stop();
    if (cleanup != ESP_OK) {
        ESP_LOGE(TAG, "NimBLE 启动清理失败: %s", esp_err_to_name(cleanup));
    }
    s_error = err;
    s_state = BLE_SCAN_FAILED;
    return err;
}

esp_err_t ble_scan_stop(void)
{
    s_start_requested = false;
    s_scan_wanted = false;

    if (!s_initialized) {
        if (lock_take()) {
            s_state = BLE_SCAN_OFF;
            lock_give();
        } else {
            s_state = BLE_SCAN_OFF;
        }
        s_error = 0;
        return ESP_OK;
    }

    // 先置 OFF:此后到达的 DISC_COMPLETE 不会重启扫描,否则会和 nimble_port_stop
    // 抢同一个 host。
    if (lock_take()) {
        s_state = BLE_SCAN_OFF;
        lock_give();
    } else {
        s_state = BLE_SCAN_OFF;
    }

    if (s_host_task && !s_stop_in_progress) {
        (void)ble_gap_disc_cancel();  // 未在扫描时返回 BLE_HS_EALREADY,忽略即可
        const int rc = nimble_port_stop();
        if (rc != 0) {
            ESP_LOGE(TAG, "nimble_port_stop 失败: %d", rc);
            s_error = rc;
            s_state = BLE_SCAN_FAILED;
            return ESP_FAIL;
        }
        s_stop_in_progress = true;
    }

    if (s_host_task && !s_host_done) {
        if (s_host_stopped == NULL ||
            xSemaphoreTake(s_host_stopped, pdMS_TO_TICKS(BLE_SCAN_STOP_TIMEOUT_MS)) != pdTRUE) {
            ESP_LOGE(TAG, "等待 NimBLE host 停止超时");
            s_error = ESP_ERR_TIMEOUT;
            s_state = BLE_SCAN_FAILED;
            return ESP_ERR_TIMEOUT;
        }
        s_host_done = true;
    }

    if (s_host_task) {
        vTaskDelete(s_host_task);
        s_host_task = NULL;
    }

    const esp_err_t err = nimble_port_deinit();
    if (err != ESP_OK) {
        // 保留 s_initialized / s_host_done 等现场,下一次 stop() 直接重试 deinit。
        ESP_LOGE(TAG, "nimble_port_deinit 失败: %s", esp_err_to_name(err));
        s_error = err;
        s_state = BLE_SCAN_FAILED;
        return err;
    }

    s_initialized = false;
    s_stop_in_progress = false;
    s_host_done = false;
    if (s_host_stopped != NULL) {
        vSemaphoreDelete(s_host_stopped);
        s_host_stopped = NULL;
    }
    if (lock_take()) {
        s_state = BLE_SCAN_OFF;
        lock_give();
    } else {
        s_state = BLE_SCAN_OFF;
    }
    s_error = 0;
    return ESP_OK;
}

esp_err_t ble_scan_pause(void)
{
    if (lock_take()) {
        if (s_state != BLE_SCAN_SCANNING) {
            lock_give();
            s_error = ESP_ERR_INVALID_STATE;
            return ESP_ERR_INVALID_STATE;
        }
        // 先清掉扫描意愿并置 PAUSED,再取消:这样即便 DISC_COMPLETE 先到也不会重启
        // 扫描,控制器复位后的 sync 回调也会尊重暂停状态。
        s_scan_wanted = false;
        s_state = BLE_SCAN_PAUSED;
        lock_give();
    } else {
        s_error = ESP_ERR_INVALID_STATE;
        return ESP_ERR_INVALID_STATE;
    }

    const int rc = ble_gap_disc_cancel();
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "停止扫描失败: %d", rc);
        s_error = rc;
        s_state = BLE_SCAN_FAILED;
        return ESP_FAIL;
    }
    s_error = 0;
    return ESP_OK;
}

esp_err_t ble_scan_resume(void)
{
    if (!lock_take()) {
        s_error = ESP_ERR_INVALID_STATE;
        return ESP_ERR_INVALID_STATE;
    }
    // 正常从 PAUSED 恢复;若暂停期间发生过控制器复位(FAILED 但 host 还活着),
    // 也允许 resume 重新开扫,避免用户被永久困在失败状态。
    const bool recoverable = (s_state == BLE_SCAN_FAILED && s_initialized &&
                              s_host_task && s_start_requested);
    if (s_state != BLE_SCAN_PAUSED && !recoverable) {
        lock_give();
        s_error = ESP_ERR_INVALID_STATE;
        return ESP_ERR_INVALID_STATE;
    }

    s_scan_wanted = true;
    const int rc = discovery_start_locked();
    if (rc != 0) {
        fail_locked(rc);
    } else {
        s_error = 0;
    }
    lock_give();
    return rc == 0 ? ESP_OK : ESP_FAIL;
}

// 手动/自动刷新:丢掉旧结果,重新开始一轮 20 秒扫描。
// 只允许在"没有正在扫描"且 host 可用的状态调用:
//   FINISHED / PAUSED / 可恢复的 FAILED;SCANNING、STARTING、OFF 一律拒绝,
// 避免和正在进行的 discovery 抢射频或在不存在的 host 上启动。
esp_err_t ble_scan_restart(void)
{
    if (!lock_take()) {
        s_error = ESP_ERR_INVALID_STATE;
        return ESP_ERR_INVALID_STATE;
    }

    const bool recoverable = (s_state == BLE_SCAN_FAILED && s_initialized &&
                              s_host_task != NULL && s_start_requested &&
                              !s_stop_in_progress);
    if (s_state != BLE_SCAN_FINISHED && s_state != BLE_SCAN_PAUSED && !recoverable) {
        lock_give();
        s_error = ESP_ERR_INVALID_STATE;
        return ESP_ERR_INVALID_STATE;
    }

    // 新一轮从空表开始:旧条目可能已经离开范围,继续显示只会误导用户。
    ble_scan_table_clear(&s_table);
    s_scan_wanted = true;
    const int rc = discovery_start_locked();
    if (rc != 0) {
        fail_locked(rc);
        lock_give();
        ESP_LOGE(TAG, "重新扫描启动失败: %d", rc);
        return ESP_FAIL;
    }
    s_error = 0;
    lock_give();

    ESP_LOGI(TAG, "手动/刷新重新扫描 %d 秒", BLE_SCAN_BOOT_DURATION_MS / 1000);
    return ESP_OK;
}

ble_scan_state_t ble_scan_state(void)
{
    return s_state;
}

int ble_scan_last_error(void)
{
    return s_error;
}

void ble_scan_clear(void)
{
    if (!lock_take()) return;
    ble_scan_table_clear(&s_table);
    lock_give();
}

size_t ble_scan_snapshot(ble_scan_dev_t *out, size_t cap)
{
    if (out == NULL || cap == 0) return 0;
    if (cap > BLE_SCAN_TABLE_CAP) cap = BLE_SCAN_TABLE_CAP;
    if (!lock_take()) return 0;

    const uint32_t now = now_ms();
    // 暂停或"开机 20 秒扫描已完成"时保留已发现的设备(ble_scan.h 的约定):
    // 不做过期清理,排序也不按过期时间过滤,否则停止扫描 15 秒后界面会把结果
    // 清空、详情页误报"已离开范围"。重新开始扫描(仅 PAUSED 可 resume)后,
    // 下一次 snapshot 会正常清掉已经离开范围很久的设备。
    const bool retain = (s_state == BLE_SCAN_PAUSED || s_state == BLE_SCAN_FINISHED);
    const uint32_t stale_ms = retain ? UINT32_MAX : BLE_SCAN_STALE_MS;
    if (!retain) ble_scan_table_expire(&s_table, now, stale_ms);

    size_t order[BLE_SCAN_TABLE_CAP];
    const size_t count = ble_scan_table_rank(&s_table, now, stale_ms, order, cap);
    for (size_t i = 0; i < count; i++) {
        out[i] = s_table.devs[order[i]];
    }
    lock_give();
    return count;
}

size_t ble_scan_count(void)
{
    if (!lock_take()) return 0;
    if (s_state != BLE_SCAN_PAUSED && s_state != BLE_SCAN_FINISHED) {
        ble_scan_table_expire(&s_table, now_ms(), BLE_SCAN_STALE_MS);
    }
    const size_t count = s_table.count;
    lock_give();
    return count;
}
