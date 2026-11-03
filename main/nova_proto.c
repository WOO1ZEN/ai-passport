// main/nova_proto.c —— NOVA Bare 线格式、只读信息解析与接入认证私钥(纯 C)。
//
// 线格式(真机抓包实测,无魔数/无 CRC):
//   请求 [opcode u8][payload_len u16 LE][payload]
//   响应 [result u8][opcode u8][payload_len u16 LE][payload]
// 解析规则与 Android SDK(BareFrameCodec.kt / ResponseStreamDecoder.kt)以及
// ble_nova(protocol/frames.py)一致:result 仅接受 0/1,opcode 必须在已知集合内,
// 长度异常时丢弃 1 字节重新同步。
//
// 0x01 布局(顺序:SN、硬件版本、固件版本):
//   [sn_len][hw_len][fw_len][SN][hw][fw],且 plen == 3+sn_len+hw_len+fw_len
// 0x02 布局:[level][code u16 LE][可选 battery],本模块只取 level/code。
#include "nova_proto.h"

#include <string.h>

#include "device_proto.h"

// 已知 opcode 集合:用于无魔数协议的重同步判定。不要只枚举本模块用到的
// 5 个 opcode,否则会把合法的状态/心跳上报当成噪声丢弃。
static bool nova_opcode_known(uint8_t opcode)
{
    switch (opcode) {
    case 0x01:  // QUERY_DEVICE_INFO
    case 0x02:  // QUERY_DEVICE_STATUS
    case 0x03:  // 零点偏移
    case 0x04:  // 部件开关
    case 0x05:  // NOVA_SERVE_INFO
    case 0x06:
    case 0x07:  // FETCH_DYNAMIC_CODE
    case 0x08:  // VERIFY_ACCESS
    case 0x09:  // 零点出厂值
    case 0x0B:  // SERVE_COUNT_REPORT
    case 0x11:  // BATTERY_REPORT
    case 0x12:  // PHOTOELECTRIC_REPORT
    case 0x13:  // COUNTDOWN_REPORT
    case 0x40:
    case 0x41:
    case 0x42:
    case 0x43:
    case 0x44:
    case 0x45:
    case 0x5E:  // AURA_BALL_KIND(同一 opcode 空间)
    case 0x80:  // SERVE_CONTROL
    case 0x81:  // START_SERVE_PARAM
    case 0x82:  // HEARTBEAT_OMNI
    case 0x83:  // HEARTBEAT_NOVA
    case 0x84:
    case 0x85:
    case 0x86:  // HEARTBEAT_ACK
    case 0x87:
    case 0x88:
    case 0x89:
    case 0x8A:
    case 0xC0:
    case 0xC1:
    case 0xC2:
    case 0xC3:
    case 0xC4:
    case 0xC5:
    case 0xC6:  // OTA_RESULT_REPORT
    case 0xC7:  // OTA_RESET_REPORT
        return true;
    default:
        return false;
    }
}

size_t nova_build_request(uint8_t opcode, const void *payload, size_t payload_len,
                          uint8_t *out, size_t out_size)
{
    if (out == NULL || payload_len > NOVA_MAX_PAYLOAD ||
        out_size < 3u + payload_len) {
        return 0;
    }
    if (payload_len > 0 && payload == NULL) {
        return 0;
    }

    out[0] = opcode;
    out[1] = (uint8_t)(payload_len & 0xFFu);
    out[2] = (uint8_t)((payload_len >> 8) & 0xFFu);
    if (payload_len > 0) {
        memcpy(out + 3, payload, payload_len);
    }
    return 3u + payload_len;
}

bool nova_parse_response(const uint8_t *data, size_t len, nova_response_t *out)
{
    if (data == NULL || out == NULL || len < 4) {
        return false;
    }

    const uint8_t result = data[0];
    const uint8_t opcode = data[1];
    const uint16_t payload_len = (uint16_t)(data[2] | ((uint16_t)data[3] << 8));

    // result 只允许 0/1;payload 上限与 Android SDK 的 512B 约束一致。
    if (result > 1 || payload_len > NOVA_MAX_PAYLOAD) {
        return false;
    }
    if (len != 4u + payload_len) {
        return false;
    }

    out->result = result;
    out->opcode = opcode;
    out->payload_len = payload_len;
    if (payload_len > 0) {
        memcpy(out->payload, data + 4, payload_len);
    }
    return true;
}

size_t nova_extract_responses(uint8_t *buf, size_t *buf_len,
                              nova_response_t *out, size_t max_out)
{
    if (buf == NULL || buf_len == NULL || out == NULL || max_out == 0) {
        return 0;
    }

    const size_t len = *buf_len;
    size_t consumed = 0;
    size_t count = 0;

    while (len - consumed >= 4 && count < max_out) {
        const uint8_t *p = buf + consumed;
        const uint8_t result = p[0];
        const uint8_t opcode = p[1];
        const uint16_t payload_len =
            (uint16_t)(p[2] | ((uint16_t)p[3] << 8));

        // 无魔数协议:result/opcode/plen 任一异常都按 1 字节滑动重新同步。
        if (result > 1 || payload_len > NOVA_MAX_PAYLOAD ||
            !nova_opcode_known(opcode)) {
            consumed++;
            continue;
        }

        const size_t total = 4u + payload_len;
        if (len - consumed < total) {
            break;  // 半包:保留剩余字节等下一次 notify
        }

        nova_response_t *dst = &out[count];
        dst->result = result;
        dst->opcode = opcode;
        dst->payload_len = payload_len;
        if (payload_len > 0) {
            memcpy(dst->payload, p + 4, payload_len);
        }
        count++;
        consumed += total;
    }

    // 把未消费的半包/噪声前移,调用方保留 *buf_len 字节继续累积。
    if (consumed > 0) {
        memmove(buf, buf + consumed, len - consumed);
        *buf_len = len - consumed;
    }
    return count;
}

// ---------------------------------------------------------------------------
// Payload 解析
// ---------------------------------------------------------------------------

bool nova_parse_device_info(const nova_response_t *resp, nova_device_info_t *out)
{
    if (resp == NULL || out == NULL || resp->opcode != NOVA_OP_DEVICE_INFO ||
        resp->payload_len < 3) {
        return false;
    }

    const size_t sn_len = resp->payload[0];
    const size_t hw_len = resp->payload[1];
    const size_t fw_len = resp->payload[2];

    // 长度前缀必须与 payload 总长严格一致,且能放进目标缓冲区(留 NUL)。
    if (resp->payload_len != 3u + sn_len + hw_len + fw_len || sn_len > NOVA_MAX_SN_LEN ||
        hw_len >= sizeof(out->hardware_version) ||
        fw_len >= sizeof(out->firmware_version)) {
        return false;
    }

    memcpy(out->sn, resp->payload + 3, sn_len);
    out->sn[sn_len] = '\0';
    memcpy(out->hardware_version, resp->payload + 3 + sn_len, hw_len);
    out->hardware_version[hw_len] = '\0';
    memcpy(out->firmware_version, resp->payload + 3 + sn_len + hw_len, fw_len);
    out->firmware_version[fw_len] = '\0';
    return true;
}

bool nova_parse_access_info(const nova_response_t *resp, nova_access_info_t *out)
{
    if (resp == NULL || out == NULL || resp->opcode != NOVA_OP_ACCESS_INFO ||
        resp->payload_len < 2) {
        return false;
    }

    const size_t sn_len = resp->payload[0];
    const size_t code_len = resp->payload[1];

    // 与 Android SDK / C++ 参考实现一致:SN ≤120,动态码 4..10,长度严格匹配。
    if (sn_len > NOVA_MAX_SN_LEN || code_len < NOVA_MIN_CODE_LEN ||
        code_len > NOVA_MAX_CODE_LEN ||
        resp->payload_len != 2u + sn_len + code_len) {
        return false;
    }

    memcpy(out->sn, resp->payload + 2, sn_len);
    out->sn[sn_len] = '\0';
    out->sn_len = sn_len;
    memcpy(out->code, resp->payload + 2 + sn_len, code_len);
    out->code[code_len] = '\0';
    out->code_len = code_len;
    return true;
}

bool nova_parse_status(const nova_response_t *resp, nova_state_t *state, uint16_t *code)
{
    if (resp == NULL || state == NULL || code == NULL ||
        resp->opcode != NOVA_OP_STATUS || resp->payload_len < 3) {
        return false;
    }

    // 官方 Android 解析器要求恰好 3 字节;ESP32 参考实现允许第 4 字节是电量。
    // 这里宽松接受 plen>=3,避免未来固件多带一字节就整条状态失败。
    const uint8_t level = resp->payload[0];
    *code = (uint16_t)(resp->payload[1] | ((uint16_t)resp->payload[2] << 8));
    *state = nova_state_from_level(level);
    return true;
}

bool nova_make_private_key(const uint8_t *sn, size_t sn_len,
                           const uint8_t *code, size_t code_len,
                           char out_hex[33])
{
    // 与 SEEKER verify 共用同一算法,统一在 device_proto 中实现,避免两份实现漂移。
    return device_auth_make_private_key(sn, sn_len, code, code_len, out_hex);
}

nova_state_t nova_state_from_level(uint8_t level)
{
    // 等级语义只采用有依据的取值;1/10 等 best-effort 值不猜测,统一 UNKNOWN。
    switch (level) {
    case 0:
        return NOVA_STATE_UNINITIALIZED;
    case 2:
        return NOVA_STATE_INITIALIZING;
    case 3:
        return NOVA_STATE_IDLE;
    case 4:
        return NOVA_STATE_SERVING;
    case 5:
        return NOVA_STATE_TRANSITION;
    case 6:
        return NOVA_STATE_PAUSED;
    case 101:
        return NOVA_STATE_FAULT;
    case 200:
        return NOVA_STATE_SELF_REPAIR;
    default:
        return NOVA_STATE_UNKNOWN;
    }
}
