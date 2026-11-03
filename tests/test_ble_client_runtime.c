// tests/test_ble_client_runtime.c —— BLE 客户端读取会话的主机运行时测试。
//
// 做法:把 tests/scan_stubs 的平台桩和 main/ble_scan.c、main/ble_client.c、协议层
// 一起在主机上编译;NimBLE host 回调由桩同步触发,假 GATT 数据库由测试按场景
// 注册。测试只使用 public API(ble_client_*)和桩提供的注入点驱动状态机,
// 不依赖 main/ 文件的内部静态变量。
//
// 覆盖:NOVA happy path(认证/信息/状态/写帧字节)、NOVA 认证失败、状态字段、
// SEEKER happy path(唯一 UART 发现/分包重组/token)、丢包补包、verify-required、
// 多候选拒绝、连接超时/服务缺失/MTU 过小/MD5 错误/型号不支持/忙时请求/取消,
// 以及快照拷贝与 busy 终态不变量。
#include "scan_stubs/scan_runtime.c"

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "aura_proto.h"
#include "ble_client.h"
#include "ble_scan.h"
#include "scan_stubs/fake_scan.h"
#include "device_proto.h"
#include "framed_proto.h"
#include "md5.h"
#include "nova_proto.h"
#include "seeker_proto.h"

// ---------------------------------------------------------------------------
// 场景配置与假设备
// ---------------------------------------------------------------------------
typedef enum {
    RESP_NOVA_OK = 0,
    RESP_NOVA_AUTH_REJECT,
    RESP_NOVA_CANCEL_ON_STATUS,
    RESP_SEEKER_OK,
    RESP_SEEKER_REPAIR,
    RESP_SEEKER_VERIFY,
    RESP_SEEKER_BAD_MD5,
    RESP_SEEKER_WRONG_METHOD,  // 先回一条 method 不匹配的迟到响应
    RESP_AURA_RESULT_ERROR,    // AURA 0x01 返回 result!=0
    RESP_FRAMED_AUTH_REJECT,   // P1 Framed 0x08 返回 result=1
} resp_mode_t;

// 球机线模式:同名 GATT/句柄,NOVA 是 Bare,OMNI/HALO/PACE 是 P1 Framed,AURA 是 P2。
typedef enum {
    FAMILY_NOVA = 0,
    FAMILY_FRAMED,
    FAMILY_AURA,
} ball_family_t;

static ball_family_t s_family = FAMILY_NOVA;

// 状态/信息应答的额外字段与填充(回归:填充字节不得被当成电量/版本内容)。
static bool s_nova_status_battery;   // NOVA Bare 0x02 第 4 字节 = 电量
static uint8_t s_nova_battery;
static bool s_ball_status_pad4;      // P1 Framed / P2 0x02 第 4 字节 = 纯填充
static uint8_t s_aura_info_pad_len;  // AURA 0x01 payload 末尾填充字节数(0..8)
static uint8_t s_ball_info_pad_len;  // P1 Framed 0x01 末尾填充(精确解析器应拒绝)

static resp_mode_t s_resp = RESP_NOVA_OK;
static uint16_t s_nova_write_handle = 0xFFFF;
static uint16_t s_nova_notify_handle = 0xFFFF;
static uint16_t s_nova_cccd_handle = 0xFFFF;
static uint16_t s_seeker_write_handle = 0xFFFF;
static uint16_t s_seeker_notify_handle = 0xFFFF;
static uint16_t s_seeker_cccd_handle = 0xFFFF;
static uint8_t s_nova_level = 3;
static uint16_t s_nova_code = 0;
static TaskHandle_t s_worker;

// rescan 等待期间的注入动作:在 vTaskDelay 的第 N 次回调时执行一次。
typedef enum {
    DELAY_NONE = 0,
    DELAY_INJECT_FRESH,
    DELAY_CANCEL,
} delay_action_t;
static delay_action_t s_delay_action = DELAY_NONE;
static unsigned s_delay_countdown;


// 假设备收到的请求(便于断言 SEEKER 请求 JSON)。
static char s_requests[12][320];
static unsigned s_request_count;

static void set_time_ms(uint32_t ms)
{
    test_time_us = (int64_t)ms * 1000;
}

static void addr_with_id(uint8_t addr[6], uint8_t id)
{
    memset(addr, 0, 6);
    addr[0] = id;
}

static ble_scan_dev_t make_dev(const char *name, uint8_t id, int rssi)
{
    ble_scan_dev_t dev;
    memset(&dev, 0, sizeof(dev));
    addr_with_id(dev.addr, id);
    dev.addr_type = 0;
    dev.rssi = (int8_t)rssi;
    dev.rssi_ema = (int8_t)rssi;
    dev.connectable = true;
    snprintf(dev.name, sizeof(dev.name), "%s", name);
    dev.last_seen_ms = 1000;
    dev.first_seen_ms = 1000;
    dev.seen_count = 1;
    return dev;
}

static ble_client_info_t info_now(void)
{
    ble_client_info_t info;
    memset(&info, 0xFF, sizeof(info));
    ble_client_get_info(&info);
    return info;
}

static void record_request(const uint8_t *data, uint16_t len)
{
    if (s_request_count >= sizeof(s_requests) / sizeof(s_requests[0])) return;
    const size_t n = len < sizeof(s_requests[0]) - 1 ? len : sizeof(s_requests[0]) - 1;
    memcpy(s_requests[s_request_count], data, n);
    s_requests[s_request_count][n] = '\0';
    s_request_count++;
}

// ---------------------------------------------------------------------------
// NOVA 应答构造与应答器
// ---------------------------------------------------------------------------
static size_t nova_frame(uint8_t *out, uint8_t result, uint8_t opcode,
                         const uint8_t *payload, size_t len)
{
    out[0] = result;
    out[1] = opcode;
    out[2] = (uint8_t)(len & 0xFF);
    out[3] = (uint8_t)((len >> 8) & 0xFF);
    if (len > 0 && payload != NULL) memcpy(&out[4], payload, len);
    return 4 + len;
}

static size_t nova_access_payload(uint8_t *out, const char *sn, const char *code)
{
    const size_t sn_len = strlen(sn);
    const size_t code_len = strlen(code);
    out[0] = (uint8_t)sn_len;
    out[1] = (uint8_t)code_len;
    memcpy(&out[2], sn, sn_len);
    memcpy(&out[2 + sn_len], code, code_len);
    return 2 + sn_len + code_len;
}

static size_t nova_info_payload(uint8_t *out, const char *sn, const char *hw,
                                const char *fw)
{
    const size_t sn_len = strlen(sn);
    const size_t hw_len = strlen(hw);
    const size_t fw_len = strlen(fw);
    out[0] = (uint8_t)sn_len;
    out[1] = (uint8_t)hw_len;
    out[2] = (uint8_t)fw_len;
    memcpy(&out[3], sn, sn_len);
    memcpy(&out[3 + sn_len], hw, hw_len);
    memcpy(&out[3 + sn_len + hw_len], fw, fw_len);
    return 3 + sn_len + hw_len + fw_len;
}

static size_t nova_status_payload(uint8_t *out, uint8_t level, uint16_t code)
{
    out[0] = level;
    out[1] = (uint8_t)(code & 0xFF);
    out[2] = (uint8_t)((code >> 8) & 0xFF);
    return 3;
}

static void nova_emit(uint8_t opcode, const uint8_t *payload, size_t len,
                      uint8_t result)
{
    uint8_t frame[NOVA_MAX_RESPONSE];
    const size_t n = nova_frame(frame, result, opcode, payload, len);
    test_gatt_emit_notify(s_nova_notify_handle, frame, n);
}

#define NOVA_TEST_SN "O30241001620"
#define NOVA_TEST_CODE "kAgk4M"

static void nova_on_request(const uint8_t *data, uint16_t len)
{
    if (len < 3) return;
    uint8_t payload[NOVA_MAX_PAYLOAD];
    size_t payload_len = 0;

    switch (data[0]) {
    case NOVA_OP_ACCESS_INFO:
        payload_len = nova_access_payload(payload, NOVA_TEST_SN, NOVA_TEST_CODE);
        nova_emit(NOVA_OP_ACCESS_INFO, payload, payload_len, 0);
        break;
    case NOVA_OP_ACCESS_VERIFY: {
        // 客户端必须发送 32 字节 ASCII 私钥:在这里按真机算法复算并核对。
        char expected[33];
        assert(nova_make_private_key((const uint8_t *)NOVA_TEST_SN, strlen(NOVA_TEST_SN),
                                     (const uint8_t *)NOVA_TEST_CODE, strlen(NOVA_TEST_CODE),
                                     expected));
        assert(len == 3 + 32);
        assert(memcmp(&data[3], expected, 32) == 0);
        nova_emit(NOVA_OP_ACCESS_VERIFY, NULL, 0,
                  s_resp == RESP_NOVA_AUTH_REJECT ? 1 : 0);
        break;
    }
    case NOVA_OP_DEVICE_INFO:
        payload_len = nova_info_payload(payload, NOVA_TEST_SN, "V01", "30.0.5-30.0.6");
        nova_emit(NOVA_OP_DEVICE_INFO, payload, payload_len, 0);
        break;
    case NOVA_OP_STATUS:
        if (s_resp == RESP_NOVA_CANCEL_ON_STATUS) {
            // 模拟用户此时取消:发请求后不再应答。
            assert(ble_client_cancel() == ESP_OK);
            break;
        }
        payload_len = nova_status_payload(payload, s_nova_level, s_nova_code);
        if (s_nova_status_battery) {
            payload[payload_len++] = s_nova_battery;
        }
        nova_emit(NOVA_OP_STATUS, payload, payload_len, 0);
        break;
    default:
        break;
    }
}

// ---------------------------------------------------------------------------
// SEEKER 应答构造与应答器
// ---------------------------------------------------------------------------
static uint8_t s_seeker_packets[SEEKER_MAX_PACKETS][300];
static size_t s_seeker_packet_lens[SEEKER_MAX_PACKETS];
static size_t s_seeker_packet_total;

// 22 字节 ASCII 头(16 字符 MD5 中段 + 3 位包号 + 3 位总包数)+ 正文分片。
static size_t seeker_frame(uint8_t *out, const uint8_t *body, size_t body_len,
                           size_t chunk_size, size_t index, size_t total)
{
    char digest[33];
    md5_hex_upper(body, body_len, digest);
    const size_t off = (index - 1) * chunk_size;
    size_t chunk_len = body_len > off ? body_len - off : 0;
    if (chunk_len > chunk_size) chunk_len = chunk_size;
    memcpy(out, &digest[8], 16);
    snprintf((char *)&out[16], 4, "%03u", (unsigned)index);
    snprintf((char *)&out[19], 4, "%03u", (unsigned)total);
    memcpy(&out[22], &body[off], chunk_len);
    return 22 + chunk_len;
}

static size_t seeker_prepare(const char *body, size_t chunk_size)
{
    const size_t body_len = strlen(body);
    size_t total = (body_len + chunk_size - 1) / chunk_size;
    if (total == 0) total = 1;
    assert(total <= SEEKER_MAX_PACKETS);
    for (size_t i = 0; i < total; i++) {
        s_seeker_packet_lens[i] = seeker_frame(s_seeker_packets[i],
                                               (const uint8_t *)body, body_len,
                                               chunk_size, i + 1, total);
    }
    s_seeker_packet_total = total;
    return total;
}

static void seeker_emit(size_t index)
{
    assert(index >= 1 && index <= s_seeker_packet_total);
    test_gatt_emit_notify(s_seeker_notify_handle, s_seeker_packets[index - 1],
                          s_seeker_packet_lens[index - 1]);
}

static const char *SEEKER_INFO_JSON =
    "{\"method\":\"device_info\",\"code\":1000,\"data\":{"
    "\"firmware_version\":\"1.2.2\",\"system_version\":\"01.44.01\","
    "\"sn\":\"S13012211180131\",\"token\":\"session-token\"}}";

static const char *SEEKER_STATUS_JSON =
    "{\"code\":1000,\"data\":{\"hawkeye\":\"ready\"},\"method\":\"status\",\"msg\":\"SUCCESS\"}";

static void seeker_on_request(const uint8_t *data, uint16_t len)
{
    record_request(data, len);
    if (strstr(s_requests[s_request_count - 1], "get_ble_pkg") != NULL) {
        // 补包请求:补发 device_info 的第 2 包。
        assert(strstr(s_requests[s_request_count - 1], "\"device_info\"") != NULL);
        assert(strstr(s_requests[s_request_count - 1], "\"list\"") != NULL);
        seeker_emit(2);
        return;
    }
    if (strstr(s_requests[s_request_count - 1], "\"device_info\"") != NULL) {
        if (s_resp == RESP_SEEKER_VERIFY) {
            const char *json =
                "{\"method\":\"device_info\",\"code\":1002,\"data\":{"
                "\"sn\":\"S11022109170019\",\"random\":\"123456\"}}";
            seeker_prepare(json, 200);
            seeker_emit(1);
            return;
        }
        if (s_resp == RESP_SEEKER_WRONG_METHOD) {
            // 先投递一条 method=status 的迟到消息,再投递正确的 device_info。
            seeker_prepare(SEEKER_STATUS_JSON, 200);
            seeker_emit(1);
            seeker_prepare(SEEKER_INFO_JSON, 80);
            for (size_t i = 1; i <= s_seeker_packet_total; i++) seeker_emit(i);
            return;
        }
        if (s_resp == RESP_SEEKER_BAD_MD5) {
            seeker_prepare(SEEKER_INFO_JSON, 200);
            // 篡改正文但不改头里的 MD5:重组完成后应报 MD5 校验失败。
            s_seeker_packets[0][22] ^= 0x01;
            seeker_emit(1);
            return;
        }
        if (s_resp == RESP_SEEKER_REPAIR) {
            const size_t total = seeker_prepare(SEEKER_INFO_JSON, 50);
            assert(total == 3);
            seeker_emit(1);
            seeker_emit(3);  // 故意丢掉第 2 包
        } else {
            const size_t total = seeker_prepare(SEEKER_INFO_JSON, 80);
            for (size_t i = 1; i <= total; i++) seeker_emit(i);
        }
        return;
    }
    if (strstr(s_requests[s_request_count - 1], "\"status\"") != NULL) {
        seeker_prepare(SEEKER_STATUS_JSON, 200);
        seeker_emit(1);
    }
}

// ---------------------------------------------------------------------------
// OMNI / HALO / PACE(P1 Framed)与 AURA(P2 Frame_ID)应答构造与应答器
// ---------------------------------------------------------------------------
// Framed 请求:41 14 | len u16LE(=plen+5) | opcode | plen u16LE | payload | CRC
// Framed 响应:45 54 | len u16LE(=plen+6) | result | opcode | plen u16LE | payload | CRC
static size_t framed_response(uint8_t *out, uint8_t result, uint8_t opcode,
                              const uint8_t *payload, size_t plen)
{
    out[0] = FRAMED_DEV_HEADER0;
    out[1] = FRAMED_DEV_HEADER1;
    const uint16_t len = (uint16_t)(plen + 6);
    out[2] = (uint8_t)(len & 0xFF);
    out[3] = (uint8_t)(len >> 8);
    out[4] = result;
    out[5] = opcode;
    out[6] = (uint8_t)(plen & 0xFF);
    out[7] = (uint8_t)(plen >> 8);
    if (plen > 0 && payload != NULL) memcpy(&out[8], payload, plen);
    const uint16_t crc = framed_crc16(&out[2], len);
    out[8 + plen] = (uint8_t)(crc & 0xFF);
    out[9 + plen] = (uint8_t)(crc >> 8);
    return 10 + plen;
}

// AURA 响应(Frame_ID 布局):45 54 | len(=plen+8) | 00 00 | result | opcode | plen | payload | CRC
static size_t aura_frame_response(uint8_t *out, uint8_t result, uint8_t opcode,
                                  const uint8_t *payload, size_t plen)
{
    out[0] = FRAMED_DEV_HEADER0;
    out[1] = FRAMED_DEV_HEADER1;
    const uint16_t len = (uint16_t)(plen + 8);
    out[2] = (uint8_t)(len & 0xFF);
    out[3] = (uint8_t)(len >> 8);
    out[4] = 0x00;
    out[5] = 0x00;
    out[6] = result;
    out[7] = opcode;
    out[8] = (uint8_t)(plen & 0xFF);
    out[9] = (uint8_t)(plen >> 8);
    if (plen > 0 && payload != NULL) memcpy(&out[10], payload, plen);
    const uint16_t crc = framed_crc16(&out[2], len);
    out[10 + plen] = (uint8_t)(crc & 0xFF);
    out[11 + plen] = (uint8_t)(crc >> 8);
    return 12 + plen;
}

// AURA 兼容的 Opcode(LE) 响应:45 54 | len(=plen+4) | opcode u16LE | payload | CRC
static size_t aura_opcode_le_response(uint8_t *out, uint8_t opcode,
                                      const uint8_t *payload, size_t plen)
{
    out[0] = FRAMED_DEV_HEADER0;
    out[1] = FRAMED_DEV_HEADER1;
    const uint16_t len = (uint16_t)(plen + 4);
    out[2] = (uint8_t)(len & 0xFF);
    out[3] = (uint8_t)(len >> 8);
    out[4] = opcode;
    out[5] = 0x00;
    if (plen > 0 && payload != NULL) memcpy(&out[6], payload, plen);
    const uint16_t crc = framed_crc16(&out[2], len);
    out[6 + plen] = (uint8_t)(crc & 0xFF);
    out[7 + plen] = (uint8_t)(crc >> 8);
    return 8 + plen;
}

#define BALL_TEST_SN "P16000000121"
#define BALL_TEST_CODE "TFwXJY"
#define BALL_TEST_HW "1.0.0"
#define BALL_TEST_FW "1.0.0-100100100"

static void framed_emit(uint8_t opcode, const uint8_t *payload, size_t len,
                        uint8_t result)
{
    uint8_t frame[FRAMED_MAX_RESPONSE];
    const size_t n = framed_response(frame, result, opcode, payload, len);
    test_gatt_emit_notify(s_nova_notify_handle, frame, n);
}

static void framed_on_request(const uint8_t *data, uint16_t len)
{
    // 41 14 | len u16 | opcode | plen u16 | payload | crc
    if (len < 7 || data[0] != FRAMED_APP_HEADER0 || data[1] != FRAMED_APP_HEADER1) return;
    const uint8_t opcode = data[4];
    uint8_t payload[NOVA_MAX_PAYLOAD];
    size_t payload_len = 0;

    switch (opcode) {
    case NOVA_OP_ACCESS_INFO:
        payload_len = nova_access_payload(payload, BALL_TEST_SN, BALL_TEST_CODE);
        framed_emit(NOVA_OP_ACCESS_INFO, payload, payload_len, 0);
        break;
    case NOVA_OP_ACCESS_VERIFY: {
        char expected[33];
        assert(nova_make_private_key((const uint8_t *)BALL_TEST_SN, strlen(BALL_TEST_SN),
                                     (const uint8_t *)BALL_TEST_CODE,
                                     strlen(BALL_TEST_CODE), expected));
        assert(len == 7 + 32 + 2);
        assert(memcmp(&data[7], expected, 32) == 0);
        framed_emit(NOVA_OP_ACCESS_VERIFY, NULL, 0,
                    s_resp == RESP_FRAMED_AUTH_REJECT ? 1 : 0);
        break;
    }
    case NOVA_OP_DEVICE_INFO:
        payload_len = nova_info_payload(payload, BALL_TEST_SN, BALL_TEST_HW,
                                        BALL_TEST_FW);
        for (uint8_t i = 0; i < s_ball_info_pad_len; i++) {
            payload[payload_len++] = (uint8_t)(0xD0 + i);
        }
        framed_emit(NOVA_OP_DEVICE_INFO, payload, payload_len, 0);
        break;
    case NOVA_OP_STATUS:
        payload_len = nova_status_payload(payload, 3, 1);
        if (s_ball_status_pad4) payload[payload_len++] = 0xA5;  // 填充,不是电量
        framed_emit(NOVA_OP_STATUS, payload, payload_len, 0);
        break;
    default:
        break;
    }
}

static void aura_emit_info(uint8_t result)
{
    uint8_t payload[NOVA_MAX_PAYLOAD];
    size_t payload_len = result == 0
        ? nova_info_payload(payload, BALL_TEST_SN, BALL_TEST_HW, BALL_TEST_FW)
        : 0;
    // AURA 参考解析器容忍 0x01 末尾填充字节:客户端必须能忽略它们。
    for (uint8_t i = 0; result == 0 && i < s_aura_info_pad_len; i++) {
        payload[payload_len++] = (uint8_t)(0xE0 + i);
    }
    uint8_t frame[AURA_MAX_RESPONSE];
    const size_t n = aura_frame_response(frame, result, NOVA_OP_DEVICE_INFO,
                                         payload, payload_len);
    test_gatt_emit_notify(s_nova_notify_handle, frame, n);
}

static void aura_on_request(const uint8_t *data, uint16_t len)
{
    // 41 14 | len u16 | 00 00 | opcode | plen u16 | payload | crc
    if (len < 9 || data[0] != FRAMED_APP_HEADER0 || data[1] != FRAMED_APP_HEADER1) return;
    if (data[4] != 0x00 || data[5] != 0x00) return;
    const uint8_t opcode = data[6];

    if (opcode == NOVA_OP_DEVICE_INFO) {
        aura_emit_info(s_resp == RESP_AURA_RESULT_ERROR ? 1 : 0);
    } else if (opcode == NOVA_OP_STATUS) {
        // 状态用兼容的 Opcode(LE) 布局回复,顺带覆盖两种上行布局。
        uint8_t payload[NOVA_MAX_PAYLOAD];
        size_t payload_len = nova_status_payload(payload, 2, 0x0101);
        if (s_ball_status_pad4) payload[payload_len++] = 0x5A;  // 填充,不是电量
        uint8_t frame[AURA_MAX_RESPONSE];
        const size_t n = aura_opcode_le_response(frame, NOVA_OP_STATUS,
                                                 payload, payload_len);
        test_gatt_emit_notify(s_nova_notify_handle, frame, n);
    }
}

// ---------------------------------------------------------------------------
// 写请求总入口(桩在 ble_gattc_write_* 内同步调用)
// ---------------------------------------------------------------------------
static void on_write(uint16_t conn_handle, uint16_t attr_handle,
                     const uint8_t *data, uint16_t len)
{
    (void)conn_handle;
    if (attr_handle == s_nova_write_handle) {
        record_request(data, len);
        if (s_family == FAMILY_FRAMED) {
            framed_on_request(data, len);
        } else if (s_family == FAMILY_AURA) {
            aura_on_request(data, len);
        } else {
            nova_on_request(data, len);
        }
    } else if (attr_handle == s_seeker_write_handle) {
        seeker_on_request(data, len);
    }
}

// ---------------------------------------------------------------------------
// 假 GATT 数据库与场景初始化
// ---------------------------------------------------------------------------
static void setup_nova_db(void)
{
    const int svc = test_gatt_add_service(NOVA_SERVICE_UUID, 0x0010, 0x0020);
    assert(svc >= 0);
    assert(test_gatt_add_chr(svc, NOVA_WRITE_UUID, 0x0012, 0x0013,
                             BLE_GATT_CHR_PROP_WRITE, 0, false) >= 0);
    assert(test_gatt_add_chr(svc, NOVA_NOTIFY_UUID, 0x0015, 0x0016,
                             BLE_GATT_CHR_PROP_NOTIFY, 0x0017, false) >= 0);
    s_nova_write_handle = 0x0013;
    s_nova_notify_handle = 0x0016;
    s_nova_cccd_handle = 0x0017;
    s_seeker_write_handle = 0xFFFF;  // 同一端口号,靠属性区分协议
    s_seeker_notify_handle = 0xFFFF;
    s_seeker_cccd_handle = 0xFFFF;
}

// 球机共用一组 GATT 句柄(OMNI/HALO/PACE/AURA 与 NOVA 完全相同)。
static void setup_ball_db(void)
{
    setup_nova_db();
}

static void setup_seeker_db(void)
{
    const int svc = test_gatt_add_service("0000fff0-0000-1000-8000-00805f9b34fb",
                                          0x0010, 0x0020);
    assert(svc >= 0);
    assert(test_gatt_add_chr(svc, "0000fff1-0000-1000-8000-00805f9b34fb",
                             0x0012, 0x0013, BLE_GATT_CHR_PROP_WRITE_NO_RSP,
                             0, false) >= 0);
    assert(test_gatt_add_chr(svc, "0000fff2-0000-1000-8000-00805f9b34fb",
                             0x0015, 0x0016, BLE_GATT_CHR_PROP_NOTIFY,
                             0x0017, false) >= 0);
    s_seeker_write_handle = 0x0013;
    s_seeker_notify_handle = 0x0016;
    s_seeker_cccd_handle = 0x0017;
    s_nova_write_handle = 0xFFFF;
    s_nova_notify_handle = 0xFFFF;
    s_nova_cccd_handle = 0xFFFF;
}

// 每个场景开始:清空假设备/注入状态并重新挂写钩子。
static void scenario_begin(void)
{
    test_gatt_reset();
    test_fake_scan_reset();
    // client 测试里这两个桩计数只被假扫描的 pause/resume 使用,按场景清零,
    // 便于新增用例做绝对计数断言(既有用例仍用 before/after 差值)。
    test_disc_calls = 0;
    test_disc_cancel_calls = 0;
    test_delay_hook = NULL;
    s_delay_countdown = 0;
    s_delay_action = DELAY_NONE;
    test_gatt_set_write_hook(on_write);
    s_request_count = 0;
    s_family = FAMILY_NOVA;
    s_resp = RESP_NOVA_OK;
    s_nova_status_battery = false;
    s_nova_battery = 0;
    s_ball_status_pad4 = false;
    s_aura_info_pad_len = 0;
    s_ball_info_pad_len = 0;
    s_nova_write_handle = 0xFFFF;
    s_seeker_write_handle = 0xFFFF;
    s_nova_level = 3;
    s_nova_code = 0;
    memset(s_requests, 0, sizeof(s_requests));
}

// 投递请求 -> 驱动 worker -> 等它回到空闲队列(不预置扫描状态)。
static void run_session_raw(const ble_scan_dev_t *dev)
{
    assert(!ble_client_busy());
    assert(ble_client_request_read(dev) == ESP_OK);
    if (s_worker == NULL) s_worker = test_last_task;  // 首次 request_read 懒启动 worker
    assert(s_worker != NULL);
    assert(ble_client_busy());
    test_run_worker(s_worker);
    assert(!ble_client_busy());
}

// 常规回归场景:扫描处于 SCANNING,且目标条目刚刚更新(last_seen=now),
// 因此会话不会走"过期自动刷新"分支。
static void run_session(const ble_scan_dev_t *dev)
{
    ble_scan_dev_t fresh = *dev;
    fresh.last_seen_ms = (uint32_t)(test_time_us / 1000);
    fresh.first_seen_ms = fresh.last_seen_ms;
    fresh.seen_count = 1;
    test_fake_scan_set_state(BLE_SCAN_SCANNING);
    test_fake_scan_add_dev(&fresh);
    run_session_raw(&fresh);
}

// 统计写记录里发往指定属性的包数。
static unsigned writes_to(uint16_t attr)
{
    unsigned n = 0;
    for (unsigned i = 0; i < test_gatt_write_count; i++) {
        if (test_gatt_writes[i].attr_handle == attr) n++;
    }
    return n;
}

static const struct test_gatt_write_record *write_at(uint16_t attr, unsigned ordinal)
{
    unsigned n = 0;
    for (unsigned i = 0; i < test_gatt_write_count; i++) {
        if (test_gatt_writes[i].attr_handle != attr) continue;
        if (n == ordinal) return &test_gatt_writes[i];
        n++;
    }
    return NULL;
}

// ---------------------------------------------------------------------------
// 1) NOVA happy path
// ---------------------------------------------------------------------------
static void test_nova_happy(void)
{
    scenario_begin();
    setup_nova_db();
    s_resp = RESP_NOVA_OK;

    const ble_scan_dev_t dev = make_dev("NOVA_ABC123", 0x11, -50);
    const unsigned cancel_before = test_disc_cancel_calls;
    const unsigned disc_before = test_disc_calls;
    const unsigned connect_before = test_connect_calls;

    run_session(&dev);

    // 首次 request_read 触发了懒启动,此后重复 start 被拒绝。
    assert(ble_client_start() == ESP_ERR_INVALID_STATE);

    // 扫描恰好暂停/恢复一次。
    assert(test_disc_cancel_calls == cancel_before + 1);
    assert(test_disc_calls == disc_before + 1);
    assert(ble_scan_state() == BLE_SCAN_SCANNING);
    assert(test_connect_calls == connect_before + 1);
    assert(test_connect_peer.val[0] == 0x11);
    assert(test_connect_peer.type == 0);

    // 真机 bug 回归:NimBLE 把 scan_itvl/scan_window 原样塞进 HCI LE Create
    // Connection;留 0 会被控制器拒为 Invalid HCI Command Parameters
    // (0x212 = 530,屏幕显示 "读取失败: connect 530")。这里锁定合法值。
    assert(test_conn_params.scan_itvl >= 0x0004 &&
           test_conn_params.scan_itvl <= 0x4000);
    assert(test_conn_params.scan_window >= 0x0004 &&
           test_conn_params.scan_window <= test_conn_params.scan_itvl);
    assert(test_conn_params.itvl_min == 12 && test_conn_params.itvl_max == 24);
    assert(test_conn_params.supervision_timeout == 400);

    // 最终快照。
    const ble_client_info_t info = info_now();
    assert(info.state == BLE_CLIENT_DONE);
    assert(info.model == DEVICE_MODEL_NOVA);
    assert(!info.connected);
    assert(strcmp(info.device_name, "NOVA_ABC123") == 0);
    assert(strcmp(info.sn, "O30241001620") == 0);
    assert(strcmp(info.firmware_version, "30.0.5-30.0.6") == 0);
    assert(strcmp(info.hardware_version, "V01") == 0);
    assert(info.status_valid);
    assert(info.nova_level == 3);
    assert(info.nova_code == 0);
    assert(!info.battery_valid);  // 0x02 只有 3 字节:电量未知
    assert(info.last_error == 0);

    // CCCD = 0x0001,且只写一次。
    assert(writes_to(s_nova_cccd_handle) == 1);
    const struct test_gatt_write_record *cccd = write_at(s_nova_cccd_handle, 0);
    assert(cccd->len == 2 && cccd->data[0] == 0x01 && cccd->data[1] == 0x00);

    // 四个 Bare 请求按顺序 0x07/0x08/0x01/0x02,字节与 nova_build_request 一致。
    assert(writes_to(s_nova_write_handle) == 4);
    const uint8_t ops[] = {NOVA_OP_ACCESS_INFO, NOVA_OP_ACCESS_VERIFY,
                           NOVA_OP_DEVICE_INFO, NOVA_OP_STATUS};
    for (unsigned i = 0; i < 4; i++) {
        const struct test_gatt_write_record *rec = write_at(s_nova_write_handle, i);
        assert(rec != NULL);
        uint8_t expected[NOVA_MAX_REQUEST];
        size_t expected_len;
        if (ops[i] == NOVA_OP_ACCESS_VERIFY) {
            char key[33];
            assert(nova_make_private_key((const uint8_t *)NOVA_TEST_SN,
                                         strlen(NOVA_TEST_SN),
                                         (const uint8_t *)NOVA_TEST_CODE,
                                         strlen(NOVA_TEST_CODE), key));
            expected_len = nova_build_request(ops[i], key, 32, expected, sizeof(expected));
        } else {
            expected_len = nova_build_request(ops[i], NULL, 0, expected, sizeof(expected));
        }
        assert(expected_len != 0);
        assert(rec->len == expected_len);
        assert(memcmp(rec->data, expected, expected_len) == 0);
        assert(rec->with_response);  // NOVA 优先 with-response
    }
    assert(test_disc_all_svcs_calls == 0);      // NOVA 用 disc_svc_by_uuid
    assert(test_disc_svc_by_uuid_calls >= 1);
    assert(test_exchange_mtu_calls == 1);
    assert(test_write_no_rsp_calls == 0);
}

// ---------------------------------------------------------------------------
// 2) NOVA 认证失败
// ---------------------------------------------------------------------------
static void test_nova_auth_rejected(void)
{
    scenario_begin();
    setup_nova_db();
    s_resp = RESP_NOVA_AUTH_REJECT;

    const ble_scan_dev_t dev = make_dev("NOVA_REJECT", 0x12, -55);
    const unsigned cancel_before = test_disc_cancel_calls;
    const unsigned disc_before = test_disc_calls;

    run_session(&dev);

    const ble_client_info_t info = info_now();
    assert(info.state == BLE_CLIENT_FAILED);
    assert(!info.connected);
    assert(strstr(info.detail, "auth rejected") != NULL);
    assert(strcmp(info.sn, "") == 0);  // 没有读到信息
    assert(test_disc_cancel_calls == cancel_before + 1);
    assert(test_disc_calls == disc_before + 1);

    // 认证被拒后不得再发 0x01/0x02。
    assert(writes_to(s_nova_write_handle) == 2);
    const struct test_gatt_write_record *last = write_at(s_nova_write_handle, 1);
    assert(last->data[0] == NOVA_OP_ACCESS_VERIFY);
}

// ---------------------------------------------------------------------------
// 3) NOVA 0x01/0x02 字段:level=4, code=0x0102(LE)
// ---------------------------------------------------------------------------
static void test_nova_status_fields(void)
{
    scenario_begin();
    setup_nova_db();
    s_resp = RESP_NOVA_OK;
    s_nova_level = 4;      // 发球中
    s_nova_code = 0x0102;  // u16 LE:低字节 0x02、高字节 0x01

    const ble_scan_dev_t dev = make_dev("NOVA_STATUS", 0x13, -60);
    run_session(&dev);

    const ble_client_info_t info = info_now();
    assert(info.state == BLE_CLIENT_DONE);
    assert(info.status_valid);
    assert(info.nova_level == 4);
    assert(info.nova_code == 0x0102);  // 小端解析正确
    // 0x01 的三段变长字段顺序:sn / hardware / firmware。
    assert(strcmp(info.sn, "O30241001620") == 0);
    assert(strcmp(info.hardware_version, "V01") == 0);
    assert(strcmp(info.firmware_version, "30.0.5-30.0.6") == 0);
}

// ---------------------------------------------------------------------------
// 4) SEEKER happy path
// ---------------------------------------------------------------------------
static void test_seeker_happy(void)
{
    scenario_begin();
    setup_seeker_db();
    s_resp = RESP_SEEKER_OK;

    const ble_scan_dev_t dev = make_dev("SEEKER2211180131", 0x21, -45);
    const unsigned cancel_before = test_disc_cancel_calls;
    const unsigned disc_before = test_disc_calls;

    run_session(&dev);

    const ble_client_info_t info = info_now();
    assert(info.state == BLE_CLIENT_DONE);
    assert(info.model == DEVICE_MODEL_SEEKER);
    assert(!info.connected);
    assert(strcmp(info.sn, "S13012211180131") == 0);
    assert(strcmp(info.firmware_version, "1.2.2") == 0);       // APK
    assert(strcmp(info.hardware_version, "01.44.01") == 0);    // system_version
    assert(info.status_valid);
    assert(strcmp(info.seeker_state, "ready") == 0);
    assert(!info.battery_valid);  // 本实现不读 battery_status
    assert(test_disc_cancel_calls == cancel_before + 1);
    assert(test_disc_calls == disc_before + 1);

    // 唯一 UART 发现使用全服务/全特征发现,且订阅 CCCD。
    assert(test_disc_all_svcs_calls >= 1);
    assert(test_disc_all_chrs_calls >= 1);
    assert(writes_to(s_seeker_cccd_handle) == 1);
    // 第一次 device_info 不带 token;status 带 access_token=session-token。
    assert(s_request_count >= 2);
    assert(strstr(s_requests[0], "\"device_info\"") != NULL);
    assert(strstr(s_requests[0], "access_token") == NULL);
    assert(strstr(s_requests[1], "\"status\"") != NULL);
    assert(strstr(s_requests[1], "\"access_token\":\"session-token\"") != NULL);
    // SEEKER 写特征只有 no-response(请求 JSON 不等待写完成)。
    assert(test_write_no_rsp_calls >= 2);
}

// ---------------------------------------------------------------------------
// 5) SEEKER 丢包补包
// ---------------------------------------------------------------------------
static void test_seeker_repair(void)
{
    scenario_begin();
    setup_seeker_db();
    s_resp = RESP_SEEKER_REPAIR;

    const ble_scan_dev_t dev = make_dev("SEEKER_REPAIR", 0x22, -52);
    run_session(&dev);

    const ble_client_info_t info = info_now();
    assert(info.state == BLE_CLIENT_DONE);
    assert(strcmp(info.sn, "S13012211180131") == 0);
    assert(strcmp(info.firmware_version, "1.2.2") == 0);

    // 找到补包请求:method=get_ble_pkg,requestMethod=device_info,list=[2]。
    unsigned repair_idx = 0;
    bool found_repair = false;
    for (unsigned i = 0; i < s_request_count; i++) {
        if (strstr(s_requests[i], "get_ble_pkg") != NULL) {
            repair_idx = i;
            found_repair = true;
            break;
        }
    }
    assert(found_repair);
    assert(strstr(s_requests[repair_idx], "\"method\":\"get_ble_pkg\"") != NULL);
    assert(strstr(s_requests[repair_idx], "\"requestMethod\":\"device_info\"") != NULL);
    assert(strstr(s_requests[repair_idx], "\"list\":[2]") != NULL);
    // 补包后仍继续走 status。
    bool found_status = false;
    for (unsigned i = repair_idx + 1; i < s_request_count; i++) {
        if (strstr(s_requests[i], "\"status\"") != NULL) found_status = true;
    }
    assert(found_status);
}

static void test_seeker_ignores_wrong_method(void)
{
    scenario_begin();
    setup_seeker_db();
    s_resp = RESP_SEEKER_WRONG_METHOD;

    const ble_scan_dev_t dev = make_dev("SEEKER_WRONG_METHOD", 0x25, -57);
    run_session(&dev);

    const ble_client_info_t info = info_now();
    assert(info.state == BLE_CLIENT_DONE);
    assert(strcmp(info.sn, "S13012211180131") == 0);
    assert(strcmp(info.firmware_version, "1.2.2") == 0);
}

// SEEKER notify 值句柄已是服务最后一个属性(没有 CCCD 空间):应跳过描述符发现,
// 唯一 UART 候选为 0 并干净失败,而不是用 BLE_HS_EINVAL 中止。
static void test_seeker_dsc_no_cccd_room(void)
{
    scenario_begin();
    const int svc = test_gatt_add_service("0000fff0-0000-1000-8000-00805f9b34fb",
                                          0x0010, 0x0020);
    assert(svc >= 0);
    assert(test_gatt_add_chr(svc, "0000fff1-0000-1000-8000-00805f9b34fb",
                             0x0012, 0x0013, BLE_GATT_CHR_PROP_WRITE_NO_RSP,
                             0, false) >= 0);
    // val_handle == svc_end:末尾没有 CCCD 空间。
    assert(test_gatt_add_chr(svc, "0000fff2-0000-1000-8000-00805f9b34fb",
                             0x001F, 0x0020, BLE_GATT_CHR_PROP_NOTIFY,
                             0x0021, false) >= 0);
    s_seeker_write_handle = 0x0013;
    s_seeker_notify_handle = 0x0020;
    s_resp = RESP_SEEKER_OK;

    const ble_scan_dev_t dev = make_dev("SEEKER_NO_CCCD_ROOM", 0x26, -58);
    const unsigned cancel_before = test_disc_cancel_calls;
    const unsigned disc_before = test_disc_calls;
    run_session(&dev);

    const ble_client_info_t info = info_now();
    assert(info.state == BLE_CLIENT_FAILED);
    assert(strstr(info.detail, "uart candidates 0") != NULL);
    assert(strstr(info.detail, "disc dsc") == NULL);
    assert(test_disc_all_dscs_calls == 0);  // 范围预判后不再调用
    assert(test_disc_cancel_calls == cancel_before + 1);
    assert(test_disc_calls == disc_before + 1);
    assert(s_request_count == 0);
}

// 描述符发现返回 BLE_HS_EINVAL:只跳过该候选并继续,最终以候选不足失败。
static void test_seeker_dsc_einval_skipped(void)
{
    scenario_begin();
    setup_seeker_db();
    s_resp = RESP_SEEKER_OK;
    test_disc_all_dscs_result = BLE_HS_EINVAL;

    const ble_scan_dev_t dev = make_dev("SEEKER_DSC_EINVAL", 0x27, -59);
    const unsigned cancel_before = test_disc_cancel_calls;
    const unsigned disc_before = test_disc_calls;
    run_session(&dev);

    const ble_client_info_t info = info_now();
    assert(info.state == BLE_CLIENT_FAILED);
    assert(strstr(info.detail, "uart candidates 0") != NULL);
    assert(strstr(info.detail, "disc dsc") == NULL);
    assert(test_disc_all_dscs_calls >= 1);
    assert(test_disc_cancel_calls == cancel_before + 1);
    assert(test_disc_calls == disc_before + 1);
    assert(s_request_count == 0);
}

// ---------------------------------------------------------------------------
// 6) SEEKER verify-required:明确失败,不伪造 challenge
// ---------------------------------------------------------------------------
static void test_seeker_verify_required(void)
{
    scenario_begin();
    setup_seeker_db();
    s_resp = RESP_SEEKER_VERIFY;

    const ble_scan_dev_t dev = make_dev("SEEKER_VERIFY", 0x23, -58);
    const unsigned cancel_before = test_disc_cancel_calls;
    const unsigned disc_before = test_disc_calls;
    run_session(&dev);

    const ble_client_info_t info = info_now();
    assert(info.state == BLE_CLIENT_FAILED);
    assert(strstr(info.detail, "verify") != NULL);
    assert(strstr(info.detail, "random") != NULL);
    assert(test_disc_cancel_calls == cancel_before + 1);
    assert(test_disc_calls == disc_before + 1);
    // 只发了 device_info;没有伪造 verify 请求,也没有再发 status。
    assert(s_request_count == 1);
    assert(strstr(s_requests[0], "\"verify\"") == NULL);
}

// ---------------------------------------------------------------------------
// 7) SEEKER 多个 UART 候选:拒绝猜测
// ---------------------------------------------------------------------------
static void test_seeker_multiple_candidates(void)
{
    scenario_begin();
    const int svc = test_gatt_add_service("0000fff0-0000-1000-8000-00805f9b34fb",
                                          0x0010, 0x0030);
    assert(svc >= 0);
    assert(test_gatt_add_chr(svc, "0000fff1-0000-1000-8000-00805f9b34fb",
                             0x0012, 0x0013, BLE_GATT_CHR_PROP_WRITE_NO_RSP,
                             0, false) >= 0);
    assert(test_gatt_add_chr(svc, "0000fff3-0000-1000-8000-00805f9b34fb",
                             0x0018, 0x0019, BLE_GATT_CHR_PROP_WRITE_NO_RSP,
                             0, false) >= 0);
    assert(test_gatt_add_chr(svc, "0000fff2-0000-1000-8000-00805f9b34fb",
                             0x0015, 0x0016, BLE_GATT_CHR_PROP_NOTIFY,
                             0x0017, false) >= 0);
    s_seeker_write_handle = 0x0013;
    s_seeker_notify_handle = 0x0016;
    s_resp = RESP_SEEKER_OK;

    const ble_scan_dev_t dev = make_dev("SEEKER_TWO_UART", 0x24, -60);
    const unsigned cancel_before = test_disc_cancel_calls;
    const unsigned disc_before = test_disc_calls;
    run_session(&dev);

    const ble_client_info_t info = info_now();
    assert(info.state == BLE_CLIENT_FAILED);
    assert(strstr(info.detail, "uart candidates") != NULL);
    assert(strstr(info.detail, "2") != NULL);
    assert(test_disc_cancel_calls == cancel_before + 1);
    assert(test_disc_calls == disc_before + 1);
    // 失败发生在写任何业务请求之前。
    assert(s_request_count == 0);
}

// ---------------------------------------------------------------------------
// 8) 失败路径与取消
// ---------------------------------------------------------------------------
static void test_connect_timeout(void)
{
    scenario_begin();
    setup_nova_db();
    test_connect_mode = TEST_CONNECT_SILENT;  // 不回调 CONNECT -> 超时

    const ble_scan_dev_t dev = make_dev("NOVA_TIMEOUT", 0x31, -70);
    const unsigned cancel_before = test_disc_cancel_calls;
    const unsigned disc_before = test_disc_calls;
    run_session(&dev);

    const ble_client_info_t info = info_now();
    assert(info.state == BLE_CLIENT_FAILED);
    assert(strstr(info.detail, "connect timeout") != NULL);
    assert(test_disc_cancel_calls == cancel_before + 1);  // 扫描已恢复
    assert(test_disc_calls == disc_before + 1);
    assert(s_request_count == 0);
}

static void test_connect_timeout_late_connect_reaped(void)
{
    scenario_begin();
    setup_nova_db();
    test_connect_mode = TEST_CONNECT_LATE;  // 取消后才回调 CONNECT

    const ble_scan_dev_t dev = make_dev("NOVA_LATE_CONNECT", 0x39, -70);
    const unsigned cancel_before = test_disc_cancel_calls;
    const unsigned disc_before = test_disc_calls;
    run_session(&dev);

    const ble_client_info_t info = info_now();
    assert(info.state == BLE_CLIENT_FAILED);
    assert(strstr(info.detail, "connect timeout") != NULL);
    // 迟到的 CONNECT 必须被 terminate,不能留下孤儿连接(占满 MAX_CONNECTIONS=1)。
    assert(test_disconnect_calls == 1);
    struct ble_gap_conn_desc desc;
    assert(ble_gap_conn_find(1, &desc) != 0);
    assert(test_disc_cancel_calls == cancel_before + 1);
    assert(test_disc_calls == disc_before + 1);
    assert(s_request_count == 0);
}

static void test_nova_service_not_found(void)
{
    scenario_begin();
    // 不注册任何服务。
    const ble_scan_dev_t dev = make_dev("NOVA_NOSVC", 0x32, -65);
    run_session(&dev);

    const ble_client_info_t info = info_now();
    assert(info.state == BLE_CLIENT_FAILED);
    assert(strstr(info.detail, "service not found") != NULL);
    assert(s_request_count == 0);
}

static void test_seeker_mtu_too_small(void)
{
    scenario_begin();
    setup_seeker_db();
    test_mtu_value = 100;  // 64..157:对 NOVA 够用,对 SEEKER 不够(真机单包 150B)
    s_resp = RESP_SEEKER_OK;

    const ble_scan_dev_t dev = make_dev("SEEKER_LOW_MTU", 0x33, -66);
    run_session(&dev);

    const ble_client_info_t info = info_now();
    assert(info.state == BLE_CLIENT_FAILED);
    assert(strstr(info.detail, "mtu 100 < 158") != NULL);
    assert(strstr(info.detail, "seeker") != NULL);
    assert(s_request_count == 0);
}

// NOVA 的 MTU 事件可能丢失/由对端先发起交换:此时应保留 ble_att_mtu() 已知值,
// 而不是退回默认 23 误判失败。
static void test_nova_keeps_known_mtu(void)
{
    scenario_begin();
    setup_nova_db();
    test_exchange_mtu_result = BLE_HS_EALREADY;  // 本端 exchange 失败/已在进行
    test_att_mtu = 247;                          // 但对端已完成交换

    const ble_scan_dev_t dev = make_dev("NOVA_KNOWN_MTU", 0x3A, -60);
    run_session(&dev);

    const ble_client_info_t info = info_now();
    assert(info.state == BLE_CLIENT_DONE);
    assert(test_exchange_mtu_calls == 1);
    assert(strcmp(info.sn, "O30241001620") == 0);
}

// exchange 失败且确实没有已知 MTU 时,NOVA 仍在 23 上报错(不会静默通过)。
static void test_nova_mtu_too_small(void)
{
    scenario_begin();
    setup_nova_db();
    test_exchange_mtu_result = BLE_HS_EALREADY;
    test_att_mtu = 23;  // 没有可用的已协商值

    const ble_scan_dev_t dev = make_dev("NOVA_LOW_MTU", 0x3B, -61);
    run_session(&dev);

    const ble_client_info_t info = info_now();
    assert(info.state == BLE_CLIENT_FAILED);
    assert(strstr(info.detail, "mtu 23 < 64") != NULL);
    assert(s_request_count == 0);
}

static void test_seeker_md5_mismatch(void)
{
    scenario_begin();
    setup_seeker_db();
    s_resp = RESP_SEEKER_BAD_MD5;

    const ble_scan_dev_t dev = make_dev("SEEKER_BAD_MD5", 0x34, -68);
    run_session(&dev);

    const ble_client_info_t info = info_now();
    assert(info.state == BLE_CLIENT_FAILED);
    assert(strstr(info.detail, "frame error") != NULL);  // MD5 校验失败
}

static void test_unsupported_model(void)
{
    scenario_begin();
    const ble_scan_dev_t dev = make_dev("WIDGET_XYZ", 0x35, -60);  // 未知前缀
    const unsigned cancel_before = test_disc_cancel_calls;
    const unsigned disc_before = test_disc_calls;
    run_session(&dev);

    const ble_client_info_t info = info_now();
    assert(info.state == BLE_CLIENT_UNSUPPORTED);
    assert(info.model == DEVICE_MODEL_UNKNOWN);
    assert(!info.connected);
    assert(strstr(info.detail, "unsupported") != NULL);
    assert(test_connect_calls == 0);  // 不连接
    assert(test_disc_cancel_calls == cancel_before);  // 不暂停扫描
    assert(test_disc_calls == disc_before);
    assert(s_request_count == 0);
}

static void test_request_while_busy(void)
{
    scenario_begin();
    setup_nova_db();

    ble_scan_dev_t dev = make_dev("NOVA_BUSY", 0x36, -50);
    dev.last_seen_ms = (uint32_t)(test_time_us / 1000);
    test_fake_scan_set_state(BLE_SCAN_SCANNING);
    test_fake_scan_add_dev(&dev);
    assert(!ble_client_busy());
    assert(ble_client_request_read(&dev) == ESP_OK);
    assert(ble_client_busy());
    // 会话进行中(worker 未被调度)第二次请求必须被忽略。
    assert(ble_client_request_read(&dev) == ESP_ERR_INVALID_STATE);
    assert(ble_client_busy());
    test_run_worker(s_worker);
    assert(!ble_client_busy());
    assert(info_now().state == BLE_CLIENT_DONE);
}

static void test_cancel_session(void)
{
    scenario_begin();
    setup_nova_db();
    s_resp = RESP_NOVA_CANCEL_ON_STATUS;  // 发 0x02 时模拟用户取消

    const ble_scan_dev_t dev = make_dev("NOVA_CANCEL", 0x37, -55);
    const unsigned cancel_before = test_disc_cancel_calls;
    const unsigned disc_before = test_disc_calls;
    run_session(&dev);

    const ble_client_info_t info = info_now();
    assert(info.state == BLE_CLIENT_FAILED);
    assert(strstr(info.detail, "cancelled") != NULL);
    assert(!info.connected);
    assert(test_disc_cancel_calls == cancel_before + 1);  // 取消后恢复扫描
    assert(test_disc_calls == disc_before + 1);
    // 无会话时 cancel 是空操作。
    assert(ble_client_cancel() == ESP_OK);
    assert(ble_client_cancel() == ESP_OK);
}

// ---------------------------------------------------------------------------
// 9) 快照拷贝 / busy 终态不变量
// ---------------------------------------------------------------------------
static void test_snapshot_invariants(void)
{
    scenario_begin();
    setup_nova_db();

    const ble_scan_dev_t dev = make_dev("NOVA_SNAPSHOT", 0x38, -50);
    run_session(&dev);
    assert(!ble_client_busy());

    ble_client_info_t a;
    ble_client_info_t b;
    memset(&a, 0x00, sizeof(a));
    memset(&b, 0xAA, sizeof(b));
    ble_client_get_info(&a);
    ble_client_get_info(&b);
    assert(a.state == BLE_CLIENT_DONE);
    assert(memcmp(&a, &b, sizeof(a)) == 0);  // getter 返回同一份快照拷贝
    // 快照内容不被后续 getter 调用改变(值拷贝,不是内部指针)。
    ble_client_info_t c;
    ble_client_get_info(&c);
    c.state = BLE_CLIENT_IDLE;
    assert(info_now().state == BLE_CLIENT_DONE);
    // NULL 输出安全。
    ble_client_get_info(NULL);
}


// ---------------------------------------------------------------------------
// OMNI / HALO / PACE(P1 Framed + 登录)与 AURA(P2,免登录)的完整会话
// ---------------------------------------------------------------------------
static void run_framed_session(const char *name, device_model_t expect_model,
                               uint8_t id, uint32_t min_elapsed_ms,
                               bool early_notify, bool status_pad4)
{
    scenario_begin();
    setup_ball_db();
    s_family = FAMILY_FRAMED;
    s_ball_status_pad4 = status_pad4;
    // true = 设备在 with-response 写完成之前就回 notify(真机可能出现的顺序),
    // 客户端必须先把数据缓存起来而不是丢弃。
    test_write_notify_before_done = early_notify;

    const ble_scan_dev_t dev = make_dev(name, id, -55);
    const unsigned cancel_before = test_disc_cancel_calls;
    const unsigned disc_before = test_disc_calls;
    const int64_t started_us = test_time_us;

    run_session(&dev);

    const int64_t elapsed_us = test_time_us - started_us;
    assert(elapsed_us >= (int64_t)min_elapsed_ms * 1000);

    const ble_client_info_t info = info_now();
    assert(info.state == BLE_CLIENT_DONE);
    assert(info.model == expect_model);
    assert(!info.connected);
    assert(strcmp(info.sn, BALL_TEST_SN) == 0);
    assert(strcmp(info.firmware_version, BALL_TEST_FW) == 0);
    assert(strcmp(info.hardware_version, BALL_TEST_HW) == 0);
    assert(info.status_valid);
    assert(info.nova_level == 3);
    assert(info.nova_code == 1);
    assert(!info.battery_valid);  // P1 Framed 0x02 第 4 字节是填充,不是电量
    assert(info.last_error == 0);

    // 扫描恰好暂停/恢复一次,会话结束主动断开。
    assert(test_disc_cancel_calls == cancel_before + 1);
    assert(test_disc_calls == disc_before + 1);
    assert(test_disconnect_calls >= 1);

    // 四条业务请求都是 P1 Framed 字节:0x07 / 0x08(key) / 0x01 / 0x02。
    assert(writes_to(s_nova_write_handle) == 4);
    uint8_t expected[FRAMED_MAX_REQUEST];
    size_t expected_len = framed_build_request(NOVA_OP_ACCESS_INFO, NULL, 0,
                                               expected, sizeof(expected));
    assert(expected_len != 0);
    const struct test_gatt_write_record *rec = write_at(s_nova_write_handle, 0);
    assert(rec->len == expected_len && memcmp(rec->data, expected, expected_len) == 0);

    char key[33];
    assert(nova_make_private_key((const uint8_t *)BALL_TEST_SN, strlen(BALL_TEST_SN),
                                 (const uint8_t *)BALL_TEST_CODE,
                                 strlen(BALL_TEST_CODE), key));
    expected_len = framed_build_request(NOVA_OP_ACCESS_VERIFY, key, 32,
                                        expected, sizeof(expected));
    rec = write_at(s_nova_write_handle, 1);
    assert(rec->len == expected_len && memcmp(rec->data, expected, expected_len) == 0);

    expected_len = framed_build_request(NOVA_OP_DEVICE_INFO, NULL, 0,
                                        expected, sizeof(expected));
    rec = write_at(s_nova_write_handle, 2);
    assert(rec->len == expected_len && memcmp(rec->data, expected, expected_len) == 0);

    expected_len = framed_build_request(NOVA_OP_STATUS, NULL, 0,
                                        expected, sizeof(expected));
    rec = write_at(s_nova_write_handle, 3);
    assert(rec->len == expected_len && memcmp(rec->data, expected, expected_len) == 0);
    assert(test_write_no_rsp_calls == 0);  // 球机写特征走 with-response
}

static void test_omni_happy(void)
{
    run_framed_session("OMNI_TEST01", DEVICE_MODEL_OMNI, 0x41, 4000, false, false);
}

static void test_omni_early_notify(void)
{
    run_framed_session("OMNI_EARLY", DEVICE_MODEL_OMNI, 0x44, 4000, true, false);
}

static void test_halo_happy(void)
{
    run_framed_session("HALO_TEST01", DEVICE_MODEL_HALO, 0x42, 4000, false, false);
}

static void test_pace_happy(void)
{
    run_framed_session("PACE_TEST01", DEVICE_MODEL_PACE, 0x43, 2000, false, false);
}

static void run_aura_session(uint8_t id, uint8_t info_result, bool expect_ok,
                             bool early_notify, bool status_pad4,
                             uint8_t info_pad_len)
{
    scenario_begin();
    setup_ball_db();
    s_family = FAMILY_AURA;
    s_resp = info_result == 0 ? RESP_NOVA_OK : RESP_AURA_RESULT_ERROR;
    test_write_notify_before_done = early_notify;
    s_ball_status_pad4 = status_pad4;
    s_aura_info_pad_len = info_pad_len;

    const ble_scan_dev_t dev = make_dev("AURA_TEST01", id, -52);
    const unsigned cancel_before = test_disc_cancel_calls;
    const unsigned disc_before = test_disc_calls;
    const int64_t started_us = test_time_us;

    run_session(&dev);

    const int64_t elapsed_us = test_time_us - started_us;
    assert(elapsed_us >= 1000000);  // AURA 至少 1s 稳定等待

    const ble_client_info_t info = info_now();
    assert(test_disc_cancel_calls == cancel_before + 1);
    assert(test_disc_calls == disc_before + 1);

    if (!expect_ok) {
        assert(info.state == BLE_CLIENT_FAILED);
        assert(info.model == DEVICE_MODEL_AURA);
        assert(strstr(info.detail, "result") != NULL);
        // 0x01 被拒后不得尝试登录(0x07/0x08),也不发 0x02。
        assert(writes_to(s_nova_write_handle) == 1);
        const struct test_gatt_write_record *only = write_at(s_nova_write_handle, 0);
        uint8_t expected[AURA_MAX_REQUEST];
        const size_t expected_len = aura_build_request(NOVA_OP_DEVICE_INFO, NULL, 0,
                                                       expected, sizeof(expected));
        assert(expected_len != 0);
        assert(only->len == expected_len &&
               memcmp(only->data, expected, expected_len) == 0);
        return;
    }

    assert(info.state == BLE_CLIENT_DONE);
    assert(info.model == DEVICE_MODEL_AURA);
    assert(!info.connected);
    assert(strcmp(info.sn, BALL_TEST_SN) == 0);
    assert(strcmp(info.firmware_version, BALL_TEST_FW) == 0);
    assert(strcmp(info.hardware_version, BALL_TEST_HW) == 0);
    assert(info.status_valid);
    assert(info.nova_level == 2);
    assert(info.nova_code == 0x0101);
    assert(!info.battery_valid);  // P2 0x02 第 4 字节是填充,不是电量

    // 没有登录:只有 0x01 和 0x02 两条业务写,且都是 P2 Frame_ID 请求。
    assert(writes_to(s_nova_write_handle) == 2);
    uint8_t expected[AURA_MAX_REQUEST];
    const struct test_gatt_write_record *rec = write_at(s_nova_write_handle, 0);
    size_t expected_len = aura_build_request(NOVA_OP_DEVICE_INFO, NULL, 0,
                                             expected, sizeof(expected));
    assert(expected_len != 0);
    assert(rec->len == expected_len && memcmp(rec->data, expected, expected_len) == 0);
    rec = write_at(s_nova_write_handle, 1);
    expected_len = aura_build_request(NOVA_OP_STATUS, NULL, 0,
                                      expected, sizeof(expected));
    assert(expected_len != 0);
    assert(rec->len == expected_len && memcmp(rec->data, expected, expected_len) == 0);
    // 不得出现 0x07/0x08(任何一条业务写的 opcode 都不是 7/8)。
    for (unsigned i = 0; i < 2; i++) {
        const struct test_gatt_write_record *w = write_at(s_nova_write_handle, i);
        assert(w->data[6] != NOVA_OP_ACCESS_INFO);
        assert(w->data[6] != NOVA_OP_ACCESS_VERIFY);
    }
}

static void test_aura_happy(void)
{
    run_aura_session(0x51, 0, true, false, false, 0);
}

static void test_aura_early_notify(void)
{
    run_aura_session(0x53, 0, true, true, false, 0);
}

static void test_aura_info_rejected(void)
{
    run_aura_session(0x52, 1, false, false, false, 0);
}

// AURA 0x02 第 4 字节是填充:不能被当成电量。
static void test_aura_status_padding(void)
{
    run_aura_session(0x54, 0, true, false, true, 0);
}

// AURA 0x01 末尾容忍 2..3 个填充字节(不允许把填充拼进版本串)。
static void test_aura_info_padded(void)
{
    run_aura_session(0x55, 0, true, false, false, 3);
}

// OMNI P1 0x02 第 4 字节是填充:不能被当成电量。
static void test_omni_status_padding(void)
{
    run_framed_session("OMNI_PAD", DEVICE_MODEL_OMNI, 0x46, 4000, false, true);
}

// OMNI P1 认证被拒(0x08 result=1):FAILED auth rejected,不得继续读 0x01/0x02。
static void test_omni_auth_rejected(void)
{
    scenario_begin();
    setup_ball_db();
    s_family = FAMILY_FRAMED;
    s_resp = RESP_FRAMED_AUTH_REJECT;

    const ble_scan_dev_t dev = make_dev("OMNI_REJECT", 0x47, -55);
    const unsigned cancel_before = test_disc_cancel_calls;
    const unsigned disc_before = test_disc_calls;
    run_session(&dev);

    const ble_client_info_t info = info_now();
    assert(info.state == BLE_CLIENT_FAILED);
    assert(info.model == DEVICE_MODEL_OMNI);
    assert(strstr(info.detail, "auth rejected") != NULL);
    assert(strcmp(info.sn, "") == 0);
    assert(test_disc_cancel_calls == cancel_before + 1);
    assert(test_disc_calls == disc_before + 1);
    // 只有 0x07 与 0x08;拒绝之后不得再发 0x01/0x02(NOVA Bare 4B 电量逻辑无关)。
    assert(writes_to(s_nova_write_handle) == 2);
    const struct test_gatt_write_record *only = write_at(s_nova_write_handle, 1);
    assert(only->data[4] == NOVA_OP_ACCESS_VERIFY);
}

// OMNI 等球机的 0x01 使用精确长度解析:多出的填充字节必须报 "info payload"。
static void test_omni_info_padding_rejected(void)
{
    scenario_begin();
    setup_ball_db();
    s_family = FAMILY_FRAMED;
    s_ball_info_pad_len = 3;

    const ble_scan_dev_t dev = make_dev("OMNI_INFO_PAD", 0x49, -56);
    const unsigned cancel_before = test_disc_cancel_calls;
    const unsigned disc_before = test_disc_calls;
    run_session(&dev);

    const ble_client_info_t info = info_now();
    assert(info.state == BLE_CLIENT_FAILED);
    assert(info.model == DEVICE_MODEL_OMNI);
    assert(strstr(info.detail, "info payload") != NULL);
    assert(strcmp(info.sn, "") == 0);
    assert(test_disc_cancel_calls == cancel_before + 1);
    assert(test_disc_calls == disc_before + 1);
    // 0x07 / 0x08 / 0x01 之后失败,不得再发 0x02。
    assert(writes_to(s_nova_write_handle) == 3);
    const struct test_gatt_write_record *last = write_at(s_nova_write_handle, 2);
    assert(last->data[4] == NOVA_OP_DEVICE_INFO);
}

// NOVA Bare 的 0x02 第 4 字节是真电量:必须保持 battery_valid/数值。
static void test_nova_status_battery(void)
{
    scenario_begin();
    setup_nova_db();
    s_family = FAMILY_NOVA;
    s_nova_status_battery = true;
    s_nova_battery = 87;

    const ble_scan_dev_t dev = make_dev("NOVA_BATTERY", 0x48, -50);
    run_session(&dev);

    const ble_client_info_t info = info_now();
    assert(info.state == BLE_CLIENT_DONE);
    assert(info.model == DEVICE_MODEL_NOVA);
    assert(info.status_valid);
    assert(info.nova_level == 3);
    assert(info.nova_code == 0);
    assert(info.battery_valid);
    assert(info.battery == 87);
}


// ---------------------------------------------------------------------------
// 过期条目自动刷新(连接前 rescan)与连接重试
// ---------------------------------------------------------------------------
static ble_scan_dev_t s_stale_pending;
static bool s_stale_provide_fresh;


static void stale_delay_hook(void)
{
    if (s_delay_action == DELAY_NONE) return;
    if (s_delay_countdown > 0) {
        s_delay_countdown--;
        return;
    }
    const delay_action_t action = s_delay_action;
    s_delay_action = DELAY_NONE;  // 只执行一次
    if (action == DELAY_INJECT_FRESH) {
        ble_scan_dev_t fresh = s_stale_pending;
        fresh.last_seen_ms = (uint32_t)(test_time_us / 1000) + 1;  // 比 baseline 新
        fresh.first_seen_ms = fresh.last_seen_ms;
        fresh.rssi = -38;
        fresh.rssi_ema = -38;
        fresh.seen_count = 1;
        test_fake_scan_add_dev(&fresh);
    } else if (action == DELAY_CANCEL) {
        assert(ble_client_cancel() == ESP_OK);
    }
}

static void arm_delay_action(delay_action_t action, unsigned countdown)
{
    s_delay_action = action;
    s_delay_countdown = countdown;
    test_delay_hook = stale_delay_hook;
}

// 假扫描器 restart 成功后注入"刚扫到的同一设备";last_seen 为当前假时钟。
static void stale_restart_hook(void)
{
    if (!s_stale_provide_fresh) return;
    ble_scan_dev_t fresh = s_stale_pending;
    // 新广播必须严格晚于 refresh baseline(客户端用 wrap-safe int32 差值判断)。
    fresh.last_seen_ms = (uint32_t)(test_time_us / 1000) + 1;
    fresh.first_seen_ms = fresh.last_seen_ms;
    fresh.rssi = -40;
    fresh.rssi_ema = -40;
    fresh.seen_count = 1;
    test_fake_scan_add_dev(&fresh);
}

// 构造"扫描已结束 + 目标条目已过期"的场景,返回给 request_read 的旧条目。
static ble_scan_dev_t setup_stale_case(const char *name, uint8_t id,
                                       bool fresh_after_restart)
{
    scenario_begin();
    setup_nova_db();
    s_family = FAMILY_NOVA;

    ble_scan_dev_t dev = make_dev(name, id, -75);
    const uint32_t now = (uint32_t)(test_time_us / 1000);
    dev.last_seen_ms = now - 20000;  // 远超 CLI_MAX_DEV_AGE_MS(8s)
    dev.first_seen_ms = dev.last_seen_ms;
    test_fake_scan_set_state(BLE_SCAN_FINISHED);
    test_fake_scan_add_dev(&dev);

    s_stale_pending = dev;
    s_stale_provide_fresh = fresh_after_restart;
    test_fake_scan_restart_hook = stale_restart_hook;
    return dev;
}

// 扫描已结束 + 条目过期 -> restart 一次,目标重新出现 -> 继续完成读取。
// 会话前不是 SCANNING,结束后保持 PAUSED(不把新一轮 20s 扫描留在后台)。
static void test_stale_entry_refresh_success(void)
{
    const ble_scan_dev_t dev = setup_stale_case("NOVA_STALE", 0x61, true);

    run_session_raw(&dev);

    const ble_client_info_t info = info_now();
    assert(info.state == BLE_CLIENT_DONE);
    assert(info.model == DEVICE_MODEL_NOVA);
    assert(strcmp(info.sn, "O30241001620") == 0);
    assert(test_fake_scan_restart_calls == 1);
    assert(test_fake_scan_pause_calls == 1);
    assert(test_fake_scan_resume_calls == 0);       // 会话前非 SCANNING:不恢复扫描
    assert(ble_scan_state() == BLE_SCAN_PAUSED);
    assert(test_connect_calls == 1);
    // 兼容既有计数语义:pause 计一次 disc cancel,restart 不产生 resume。
    assert(test_disc_cancel_calls == 1);
    assert(test_disc_calls == 0);
}

// 扫描已结束 + 条目过期 -> restart 后目标没有重新出现 -> FAILED,不连接。
static void test_stale_entry_not_found(void)
{
    const ble_scan_dev_t dev = setup_stale_case("NOVA_GONE", 0x62, false);

    const int64_t started_us = test_time_us;
    run_session_raw(&dev);
    const int64_t elapsed_us = test_time_us - started_us;

    const ble_client_info_t info = info_now();
    assert(info.state == BLE_CLIENT_FAILED);
    assert(strstr(info.detail, "device not found after rescan") != NULL);
    assert(test_fake_scan_restart_calls == 1);
    // rescan 后目标不存在:把新扫描暂停一次,恢复"会话前不在扫描"的语义。
    assert(test_fake_scan_pause_calls == 1);
    assert(test_fake_scan_resume_calls == 0);
    assert(ble_scan_state() == BLE_SCAN_PAUSED);
    assert(test_connect_calls == 0);  // 没有发起连接
    assert(test_disc_cancel_calls == 1);
    assert(test_disc_calls == 0);
    assert(!ble_client_busy());
    // 刷新等待约 5s(计入 45s 会话预算),没有无限等待。
    assert(elapsed_us >= 5000000);
    assert(elapsed_us < 45000000);
}

// 扫描状态已是 FINISHED(即使条目 age 很新):仍要先刷新扫描。
static void test_stale_state_finished_only(void)
{
    scenario_begin();
    setup_nova_db();
    s_family = FAMILY_NOVA;

    ble_scan_dev_t dev = make_dev("NOVA_FINISHED_STATE", 0x67, -60);
    dev.last_seen_ms = (uint32_t)(test_time_us / 1000);  // age 新鲜
    test_fake_scan_set_state(BLE_SCAN_FINISHED);
    test_fake_scan_add_dev(&dev);
    s_stale_pending = dev;
    s_stale_provide_fresh = true;
    test_fake_scan_restart_hook = stale_restart_hook;

    run_session_raw(&dev);

    const ble_client_info_t info = info_now();
    assert(info.state == BLE_CLIENT_DONE);
    assert(test_fake_scan_restart_calls == 1);  // 仅凭"不在扫描"就触发刷新
    assert(test_fake_scan_pause_calls == 1);
    assert(test_fake_scan_resume_calls == 0);   // 会话前非 SCANNING
    assert(ble_scan_state() == BLE_SCAN_PAUSED);
}

// 扫描中但条目 age > 8s:restart 被 INVALID_STATE 拒绝;若等待期间没有新广播,
// 客户端必须按"fresh-entry 规则"失败,而不是拿旧条目继续连接。
static void test_stale_age_while_scanning_no_fresh(void)
{
    scenario_begin();
    setup_nova_db();
    s_family = FAMILY_NOVA;

    ble_scan_dev_t dev = make_dev("NOVA_OLD_ENTRY", 0x68, -70);
    const uint32_t now = (uint32_t)(test_time_us / 1000);
    dev.last_seen_ms = now - 20000;
    dev.first_seen_ms = dev.last_seen_ms;
    test_fake_scan_set_state(BLE_SCAN_SCANNING);
    test_fake_scan_add_dev(&dev);

    const int64_t started_us = test_time_us;
    run_session_raw(&dev);
    const int64_t elapsed_us = test_time_us - started_us;

    const ble_client_info_t info = info_now();
    assert(info.state == BLE_CLIENT_FAILED);
    assert(strstr(info.detail, "device not found after rescan") != NULL);
    assert(test_fake_scan_restart_calls == 1);   // 尝试过 restart(INVALID_STATE 忽略)
    assert(test_fake_scan_pause_calls == 0);     // 会话前就是 SCANNING:不恢复/不暂停
    assert(test_fake_scan_resume_calls == 0);
    assert(ble_scan_state() == BLE_SCAN_SCANNING);
    assert(test_connect_calls == 0);
    // 5s rescan 等待在 fake 时钟上可见,但远小于 45s 会话上限。
    assert(elapsed_us >= 5000000);
    assert(elapsed_us < 45000000);
}

// 同上,但等待期间注入一条 last_seen 更新的广播 -> DONE,使用新条目继续。
static void test_stale_age_while_scanning_with_fresh(void)
{
    scenario_begin();
    setup_nova_db();
    s_family = FAMILY_NOVA;

    ble_scan_dev_t dev = make_dev("NOVA_OLD_ENTRY2", 0x69, -70);
    const uint32_t now = (uint32_t)(test_time_us / 1000);
    dev.last_seen_ms = now - 20000;
    dev.first_seen_ms = dev.last_seen_ms;
    test_fake_scan_set_state(BLE_SCAN_SCANNING);
    test_fake_scan_add_dev(&dev);
    s_stale_pending = dev;
    arm_delay_action(DELAY_INJECT_FRESH, 3);  // 第 4 次轮询时到达新广播

    run_session_raw(&dev);

    const ble_client_info_t info = info_now();
    assert(info.state == BLE_CLIENT_DONE);
    assert(strcmp(info.sn, "O30241001620") == 0);
    assert(test_fake_scan_restart_calls == 1);
    assert(test_fake_scan_pause_calls == 1);
    assert(test_fake_scan_resume_calls == 1);   // 会话前 SCANNING,结束恢复
    assert(ble_scan_state() == BLE_SCAN_SCANNING);
    assert(test_connect_calls == 1);
}

// rescan 等待期间用户取消:FAILED detail "cancelled"(不是 device not found),
// last_error=INVALID_STATE,busy=false,不发起连接,扫描语义恢复为 PAUSED。
static void test_cancel_during_rescan(void)
{
    const ble_scan_dev_t dev = setup_stale_case("NOVA_CANCEL_RESCAN", 0x6A, false);
    arm_delay_action(DELAY_CANCEL, 2);

    run_session_raw(&dev);

    const ble_client_info_t info = info_now();
    assert(info.state == BLE_CLIENT_FAILED);
    assert(strcmp(info.detail, "cancelled") == 0);
    assert(info.last_error == ESP_ERR_INVALID_STATE);
    assert(!info.connected);
    assert(!ble_client_busy());
    assert(test_fake_scan_restart_calls == 1);
    assert(test_fake_scan_pause_calls == 1);   // 恢复"会话前不在扫描"
    assert(test_fake_scan_resume_calls == 0);
    assert(ble_scan_state() == BLE_SCAN_PAUSED);
    assert(test_connect_calls == 0);
}

// 扫描中 + 条目新鲜:不 restart,暂停/恢复各一次,结束后仍在扫描。
static void test_fresh_entry_no_restart(void)
{
    scenario_begin();
    setup_nova_db();
    s_family = FAMILY_NOVA;

    const ble_scan_dev_t dev = make_dev("NOVA_FRESH", 0x63, -50);
    run_session(&dev);

    const ble_client_info_t info = info_now();
    assert(info.state == BLE_CLIENT_DONE);
    assert(test_fake_scan_restart_calls == 0);
    assert(test_fake_scan_pause_calls == 1);
    assert(test_fake_scan_resume_calls == 1);
    assert(ble_scan_state() == BLE_SCAN_SCANNING);
}

// 连接第一次返回 13/574 -> 只重试一次(共 2 次 connect),然后成功。
static void test_connect_retry_once_success(int fail_status)
{
    scenario_begin();
    setup_nova_db();
    s_family = FAMILY_NOVA;
    test_connect_fail_status = fail_status;
    test_connect_fail_attempts = 1;

    const ble_scan_dev_t dev = make_dev("NOVA_RETRY", 0x64, -50);
    run_session(&dev);

    const ble_client_info_t info = info_now();
    assert(info.state == BLE_CLIENT_DONE);
    assert(test_connect_calls == 2);  // 第一次失败 + 重试一次
    assert(test_fake_scan_pause_calls == 1);
    assert(test_fake_scan_resume_calls == 1);
}

static void test_connect_retry_timeout_success(void)
{
    test_connect_retry_once_success(BLE_HS_ETIMEOUT);  // 13
}

static void test_connect_retry_3e_success(void)
{
    test_connect_retry_once_success(574);  // 0x3E:连接建立失败
}

// 连续两次 13 / 574:不无限重试,FAILED 且 detail 为短字符串。
static void test_connect_two_timeouts_fail(int fail_status, const char *want_detail)
{
    scenario_begin();
    setup_nova_db();
    s_family = FAMILY_NOVA;
    test_connect_fail_status = fail_status;
    test_connect_fail_attempts = 2;

    const ble_scan_dev_t dev = make_dev("NOVA_NO_CONN", 0x65, -50);
    run_session(&dev);

    const ble_client_info_t info = info_now();
    assert(info.state == BLE_CLIENT_FAILED);
    assert(strstr(info.detail, want_detail) != NULL);
    assert(test_connect_calls == 2);  // 第一次 + 唯一一次重试
    assert(test_fake_scan_pause_calls == 1);
    assert(test_fake_scan_resume_calls == 1);
    assert(ble_scan_state() == BLE_SCAN_SCANNING);
}

static void test_connect_two_timeouts_13(void)
{
    test_connect_two_timeouts_fail(BLE_HS_ETIMEOUT, "connect timeout");
}

static void test_connect_two_failures_3e(void)
{
    test_connect_two_timeouts_fail(574, "connect failed 0x3E");
}

// 不可重试的状态码(例如 5/ENOENT):只尝试一次,detail "connect rejected"。
static void test_connect_non_retryable(void)
{
    scenario_begin();
    setup_nova_db();
    s_family = FAMILY_NOVA;
    test_connect_fail_status = 5;
    test_connect_fail_attempts = 1;

    const ble_scan_dev_t dev = make_dev("NOVA_REJECTED", 0x66, -50);
    run_session(&dev);

    const ble_client_info_t info = info_now();
    assert(info.state == BLE_CLIENT_FAILED);
    assert(strstr(info.detail, "connect rejected") != NULL);
    assert(test_connect_calls == 1);  // 不重试
    assert(test_fake_scan_pause_calls == 1);
    assert(test_fake_scan_resume_calls == 1);
}

int main(void)
{
    set_time_ms(1000);

    // client 运行时测试使用可控的假 ble_scan 实现(不链接真实 main/ble_scan.c):
    // 前置条件仍是"扫描处于 SCANNING",由假扫描器提供。
    test_fake_scan_reset();
    test_fake_scan_set_state(BLE_SCAN_SCANNING);
    assert(ble_scan_state() == BLE_SCAN_SCANNING);
    assert(ble_scan_count() == 0);

    // main.c 只在 ble_scan_start() 成功后调用 ble_client_start();客户端现在也
    // 支持 request_read() 懒启动,所以这里不显式 start,由第一个用例验证。
    assert(!ble_client_busy());
    assert(info_now().state == BLE_CLIENT_IDLE);

    test_nova_happy();
    test_nova_auth_rejected();
    test_nova_status_fields();
    test_omni_happy();
    test_omni_early_notify();
    test_halo_happy();
    test_pace_happy();
    test_aura_happy();
    test_aura_early_notify();
    test_aura_info_rejected();
    test_aura_status_padding();
    test_aura_info_padded();
    test_omni_status_padding();
    test_omni_auth_rejected();
    test_omni_info_padding_rejected();
    test_nova_status_battery();
    test_seeker_happy();
    test_seeker_repair();
    test_seeker_ignores_wrong_method();
    test_seeker_dsc_no_cccd_room();
    test_seeker_dsc_einval_skipped();
    test_seeker_verify_required();
    test_seeker_multiple_candidates();
    test_connect_timeout();
    test_connect_timeout_late_connect_reaped();
    test_nova_service_not_found();
    test_nova_keeps_known_mtu();
    test_nova_mtu_too_small();
    test_seeker_mtu_too_small();
    test_seeker_md5_mismatch();
    test_unsupported_model();
    test_request_while_busy();
    test_cancel_session();
    test_snapshot_invariants();
    test_stale_entry_refresh_success();
    test_stale_entry_not_found();
    test_stale_state_finished_only();
    test_stale_age_while_scanning_no_fresh();
    test_stale_age_while_scanning_with_fresh();
    test_cancel_during_rescan();
    test_fresh_entry_no_restart();
    test_connect_retry_timeout_success();
    test_connect_retry_3e_success();
    test_connect_two_timeouts_13();
    test_connect_two_failures_3e();
    test_connect_non_retryable();

    puts("BLE client runtime tests: PASS");
    return 0;
}
