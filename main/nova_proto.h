// main/nova_proto.h —— NOVA 发球机的 Bare 线格式、接入认证与只读信息解析(纯 C)。
//
// 线格式(无魔数/无 CRC):
//   请求 [opcode u8][payload_len u16 LE][payload]
//   响应 [result u8][opcode u8][payload_len u16 LE][payload]
// 只读目标:0x01 设备信息(SN/硬件版本/固件版本)、0x02 设备状态(level/code);
// 认证:0x07 取 SN+动态码 -> MD5 私钥 -> 0x08 校验。
//
// 本模块不依赖 ESP-IDF/LVGL,可在主机上单测;GATT 连接与收发在 main/ble_client.c。
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// GATT(NOVA 与 OMNI/HALO 共用同一组自定义 UART 服务;本应用先用在 NOVA 上)。
#define NOVA_SERVICE_UUID "02f00000-0000-0000-0000-00000000fe00"
#define NOVA_WRITE_UUID   "02f00000-0000-0000-0000-00000000ff01"
#define NOVA_NOTIFY_UUID  "02f00000-0000-0000-0000-00000000ff02"

// 单帧容量。0x01/0x02 的 payload 很小;留 512 字节上限与 Android SDK 的 512B 约束一致。
#define NOVA_MAX_PAYLOAD 512
#define NOVA_MAX_REQUEST (3 + NOVA_MAX_PAYLOAD)
#define NOVA_MAX_RESPONSE (4 + NOVA_MAX_PAYLOAD)

// 接入认证动态码长度约束(与 Android SDK 一致)。
#define NOVA_MAX_SN_LEN 120
#define NOVA_MIN_CODE_LEN 4
#define NOVA_MAX_CODE_LEN 10

typedef enum {
    NOVA_OP_DEVICE_INFO = 0x01,
    NOVA_OP_STATUS = 0x02,
    NOVA_OP_ACCESS_INFO = 0x07,
    NOVA_OP_ACCESS_VERIFY = 0x08,
    NOVA_OP_BATTERY = 0x11,
} nova_opcode_t;

typedef struct {
    uint8_t result;
    uint8_t opcode;
    uint16_t payload_len;
    uint8_t payload[NOVA_MAX_PAYLOAD];
} nova_response_t;

// 设备信息(0x01):payload = [sn_len][hw_len][fw_len][SN][硬件版本][固件版本]
typedef struct {
    char sn[NOVA_MAX_SN_LEN + 1];
    char hardware_version[48];
    char firmware_version[48];
} nova_device_info_t;

// 接入信息(0x07):payload = [sn_len][code_len][SN][动态码]
typedef struct {
    char sn[NOVA_MAX_SN_LEN + 1];
    size_t sn_len;
    char code[NOVA_MAX_CODE_LEN + 1];
    size_t code_len;
} nova_access_info_t;

typedef enum {
    NOVA_STATE_UNINITIALIZED = 0,
    NOVA_STATE_INITIALIZING = 2,
    NOVA_STATE_IDLE = 3,
    NOVA_STATE_SERVING = 4,
    NOVA_STATE_TRANSITION = 5,
    NOVA_STATE_PAUSED = 6,
    NOVA_STATE_FAULT = 101,
    NOVA_STATE_SELF_REPAIR = 200,
    NOVA_STATE_UNKNOWN = 0xFF,
} nova_state_t;

// 组 Bare 请求。参数不合法或缓冲区不足返回 0。
size_t nova_build_request(uint8_t opcode, const void *payload, size_t payload_len,
                          uint8_t *out, size_t out_size);

// 解析单条 Bare 响应;结构不合法返回 false。
bool nova_parse_response(const uint8_t *data, size_t len, nova_response_t *out);

// 从累积缓冲区中提取所有完整响应:半包保留、粘包逐帧、坏长度重同步。
// 返回写入 out 的帧数;*buf_len 更新为剩余未消费字节数。
size_t nova_extract_responses(uint8_t *buf, size_t *buf_len,
                              nova_response_t *out, size_t max_out);

// 0x01 / 0x07 / 0x02 解析。payload 结构不符返回 false。
bool nova_parse_device_info(const nova_response_t *resp, nova_device_info_t *out);
bool nova_parse_access_info(const nova_response_t *resp, nova_access_info_t *out);
bool nova_parse_status(const nova_response_t *resp, nova_state_t *state, uint16_t *code);

// 认证私钥:key = lowercase_hex(md5(sn_bytes || salted_sn || code_bytes)),输出 32 字符 + NUL。
bool nova_make_private_key(const uint8_t *sn, size_t sn_len,
                           const uint8_t *code, size_t code_len,
                           char out_hex[33]);

// 状态等级映射(未知等级返回 NOVA_STATE_UNKNOWN,不猜测语义)。
nova_state_t nova_state_from_level(uint8_t level);
