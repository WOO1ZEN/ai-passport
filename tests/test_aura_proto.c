// tests/test_aura_proto.c —— AURA P2("Frame_ID")线格式的主机单元测试。
//
// 契约来源:main/aura_proto.h。只依赖 C11 标准库与 assert(),不链接 ESP-IDF/LVGL。
// 解析结果复用 nova_response_t / nova_parse_*;CRC 复用 main/framed_proto.c。
//
// 黄金向量出处(逐字节):
// - 0x01 请求 `41 14 07 00 00 00 01 00 00 3C 00`:aura_ble_test/tests/
//   test_p2_protocol.py:38、Kotlin AuraCompatibilityTest.kt:29、spec §6 V1。
// - 0x02 请求 `41 14 07 00 00 00 02 00 00 CC 00`、0x08 查询请求
//   `41 14 09 00 00 00 08 02 00 00 00 80 D9`:spec §6 V3/V6。
// - Opcode(LE) 0x01 应答(43B,CRC 05 C8)、0x02 应答(crc 14 00 / 75 90):
//   spec §6 V2/V4,厂商 XLSX / test_p2_protocol.py:285-297。
// - Frame_ID 实机 ACK `45 54 08 00 00 00 00 8C 00 00 80 46`:spec §6 V5
//   (aura_ble_test/crash.log:1592)。
// - Frame_ID 0x01/0x02 应答:按 spec §2 规则由 Python 参考实现独立构造,
//   CRC 已用 crc16.py 算法复核;实机 0x01/0x02 直接抓包尚缺(spec §8.4)。
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "aura_proto.h"
#include "framed_proto.h"
#include "nova_proto.h"

// ---------------------------------------------------------------------------
// 测试辅助
// ---------------------------------------------------------------------------
static size_t hex_to_bytes(const char *hex, uint8_t *out, size_t out_cap)
{
    size_t count = 0;
    int hi = -1;
    for (const char *p = hex; *p != '\0'; p++) {
        int nibble;
        if (*p >= '0' && *p <= '9') {
            nibble = *p - '0';
        } else if (*p >= 'a' && *p <= 'f') {
            nibble = *p - 'a' + 10;
        } else if (*p >= 'A' && *p <= 'F') {
            nibble = *p - 'A' + 10;
        } else {
            continue;
        }
        if (hi < 0) {
            hi = nibble;
        } else {
            assert(count < out_cap);
            out[count++] = (uint8_t)((hi << 4) | nibble);
            hi = -1;
        }
    }
    assert(hi < 0);
    return count;
}

static void assert_hex_equals(const uint8_t *actual, size_t actual_len,
                              const char *expected_hex)
{
    uint8_t expected[600];
    const size_t expected_len = hex_to_bytes(expected_hex, expected,
                                             sizeof(expected));
    assert(actual_len == expected_len);
    assert(memcmp(actual, expected, actual_len) == 0);
}

// 测试侧组 Frame_ID 设备应答:45 54 | Length(=plen+8) | 00 00 | Result | Op |
// Plen | Payload | CRC。
static size_t build_frame_id_response(uint8_t result, uint8_t opcode,
                                      const uint8_t *payload, size_t payload_len,
                                      uint8_t *out, size_t out_cap)
{
    assert(out_cap >= 12 + payload_len);
    const uint16_t length = (uint16_t)(payload_len + 8);
    out[0] = FRAMED_DEV_HEADER0;
    out[1] = FRAMED_DEV_HEADER1;
    out[2] = (uint8_t)(length & 0xFF);
    out[3] = (uint8_t)(length >> 8);
    out[4] = 0x00;
    out[5] = 0x00;
    out[6] = result;
    out[7] = opcode;
    out[8] = (uint8_t)(payload_len & 0xFF);
    out[9] = (uint8_t)(payload_len >> 8);
    if (payload_len > 0) {
        memcpy(out + 10, payload, payload_len);
    }
    const uint16_t crc = framed_crc16(out + 2, length);
    out[10 + payload_len] = (uint8_t)(crc & 0xFF);
    out[11 + payload_len] = (uint8_t)(crc >> 8);
    return 12 + payload_len;
}

// 测试侧组 Opcode(LE) 设备应答:45 54 | Length(=plen+4) | Opcode u16LE |
// Payload | CRC(无 Result 字节)。
static size_t build_opcode_response(uint16_t opcode_u16, const uint8_t *payload,
                                    size_t payload_len, uint8_t *out,
                                    size_t out_cap)
{
    assert(out_cap >= 8 + payload_len);
    const uint16_t length = (uint16_t)(payload_len + 4);
    out[0] = FRAMED_DEV_HEADER0;
    out[1] = FRAMED_DEV_HEADER1;
    out[2] = (uint8_t)(length & 0xFF);
    out[3] = (uint8_t)(length >> 8);
    out[4] = (uint8_t)(opcode_u16 & 0xFF);
    out[5] = (uint8_t)(opcode_u16 >> 8);
    if (payload_len > 0) {
        memcpy(out + 6, payload, payload_len);
    }
    const uint16_t crc = framed_crc16(out + 2, length);
    out[6 + payload_len] = (uint8_t)(crc & 0xFF);
    out[7 + payload_len] = (uint8_t)(crc >> 8);
    return 8 + payload_len;
}

// 设备信息 payload:[snLen][hwLen][fwLen][SN][HW][FW]。
static size_t make_info_payload(uint8_t *out, size_t cap)
{
    const char *sn = "T04221200001";
    const char *hw = "1.0.0";
    const char *fw = "1.0.0-100100100";
    const size_t sn_len = strlen(sn);
    const size_t hw_len = strlen(hw);
    const size_t fw_len = strlen(fw);
    const size_t total = 3 + sn_len + hw_len + fw_len;
    assert(cap >= total);
    out[0] = (uint8_t)sn_len;
    out[1] = (uint8_t)hw_len;
    out[2] = (uint8_t)fw_len;
    memcpy(out + 3, sn, sn_len);
    memcpy(out + 3 + sn_len, hw, hw_len);
    memcpy(out + 3 + sn_len + hw_len, fw, fw_len);
    return total;
}

// ---------------------------------------------------------------------------
// 请求组包(固定 Frame_ID 布局)
// ---------------------------------------------------------------------------
static void test_build_request(void)
{
    uint8_t frame[AURA_MAX_REQUEST];

    // 0x01 信息查询黄金帧。
    size_t n = aura_build_request(0x01, NULL, 0, frame, sizeof(frame));
    assert_hex_equals(frame, n, "41 14 07 00 00 00 01 00 00 3C 00");

    // 0x02 状态查询黄金帧。
    n = aura_build_request(0x02, NULL, 0, frame, sizeof(frame));
    assert_hex_equals(frame, n, "41 14 07 00 00 00 02 00 00 CC 00");

    // 0x08 查询(2 字节球类参数):Length=9、整帧 13B。
    const uint8_t sport_ball[2] = {0x00, 0x00};
    n = aura_build_request(0x08, sport_ball, sizeof(sport_ball), frame,
                           sizeof(frame));
    assert_hex_equals(frame, n, "41 14 09 00 00 00 08 02 00 00 00 80 D9");

    // 结构检查:Length=plen+7,CRC 覆盖 Length 起的 length 字节,低字节在前。
    const uint8_t payload[] = {0xAA, 0xBB, 0xCC};
    n = aura_build_request(0x14, payload, sizeof(payload), frame, sizeof(frame));
    assert(n == 14);
    assert(frame[0] == 0x41 && frame[1] == 0x14);
    assert(frame[2] == 0x0A && frame[3] == 0x00);  // Length = 7 + 3
    assert(frame[4] == 0x00 && frame[5] == 0x00);  // Frame_ID 恒 0000
    assert(frame[6] == 0x14);
    assert(frame[7] == 0x03 && frame[8] == 0x00);
    assert(memcmp(frame + 9, payload, sizeof(payload)) == 0);
    {
        const uint16_t crc = framed_crc16(frame + 2, 10);
        assert(frame[12] == (uint8_t)(crc & 0xFF));
        assert(frame[13] == (uint8_t)(crc >> 8));
    }

    // 非法参数。
    static uint8_t big[AURA_MAX_PAYLOAD + 1];
    assert(aura_build_request(0x01, big, sizeof(big), frame, sizeof(frame)) == 0);
    assert(aura_build_request(0x01, NULL, 1, frame, sizeof(frame)) == 0);
    assert(aura_build_request(0x01, NULL, 0, frame, 10) == 0);
    assert(aura_build_request(0x01, NULL, 0, NULL, 32) == 0);
}

// ---------------------------------------------------------------------------
// 提取:Frame_ID 布局
// ---------------------------------------------------------------------------
static void test_extract_frame_id(void)
{
    uint8_t buf[256];
    nova_response_t out[4];
    uint8_t info_payload[64];
    const size_t info_len = make_info_payload(info_payload, sizeof(info_payload));

    // Frame_ID 0x01 信息应答:按 spec §2 规则构造(CRC 已独立复核)。
    size_t n = hex_to_bytes(
        "45 54 2B 00 00 00 00 01 23 00 0C 05 0F"
        "54 30 34 32 32 31 32 30 30 30 30 31"   // "T04221200001"
        "31 2E 30 2E 30"                        // "1.0.0"
        "31 2E 30 2E 30 2D 31 30 30 31 30 30 31 30 30"  // "1.0.0-100100100"
        "59 53",
        buf, sizeof(buf));
    assert(n == 12 + info_len);
    size_t buf_len = n;
    assert(aura_extract_responses(buf, &buf_len, out, 4) == 1);
    assert(out[0].result == 0 && out[0].opcode == 0x01 &&
           out[0].payload_len == info_len);
    nova_device_info_t info;
    assert(nova_parse_device_info(&out[0], &info));
    assert(strcmp(info.sn, "T04221200001") == 0);
    assert(strcmp(info.hardware_version, "1.0.0") == 0);
    assert(strcmp(info.firmware_version, "1.0.0-100100100") == 0);
    assert(buf_len == 0);

    // Frame_ID 0x02 状态应答:level=3 / code=1。
    n = hex_to_bytes("45 54 0B 00 00 00 00 02 03 00 03 01 00 97 B2", buf,
                     sizeof(buf));
    buf_len = n;
    assert(aura_extract_responses(buf, &buf_len, out, 4) == 1);
    assert(out[0].result == 0 && out[0].opcode == 0x02 && out[0].payload_len == 3);
    nova_state_t state = NOVA_STATE_UNKNOWN;
    uint16_t code = 0xFFFF;
    assert(nova_parse_status(&out[0], &state, &code));
    assert(state == NOVA_STATE_IDLE && code == 1);

    // 实机 Frame_ID ACK(空 payload,opcode 0x8C):result 在 Frame_ID 头。
    n = hex_to_bytes("45 54 08 00 00 00 00 8C 00 00 80 46", buf, sizeof(buf));
    buf_len = n;
    assert(aura_extract_responses(buf, &buf_len, out, 4) == 1);
    assert(out[0].result == 0 && out[0].opcode == 0x8C && out[0].payload_len == 0);
    assert(buf_len == 0);
}

// ---------------------------------------------------------------------------
// 提取:Opcode(LE) 布局
// ---------------------------------------------------------------------------
static void test_extract_opcode_layout(void)
{
    uint8_t buf[256];
    nova_response_t out[4];

    // 0x02 状态:45 54 07 00 | 02 00 | 00 00 00 | 14 00。
    size_t n = hex_to_bytes("45 54 07 00 02 00 00 00 00 14 00", buf, sizeof(buf));
    size_t buf_len = n;
    assert(aura_extract_responses(buf, &buf_len, out, 4) == 1);
    assert(out[0].result == 0 && out[0].opcode == 0x02 && out[0].payload_len == 3);
    assert(out[0].payload[0] == 0 && out[0].payload[1] == 0 && out[0].payload[2] == 0);
    nova_state_t state = NOVA_STATE_UNKNOWN;
    uint16_t code = 0xFFFF;
    assert(nova_parse_status(&out[0], &state, &code));
    assert(state == NOVA_STATE_UNINITIALIZED && code == 0);

    // 0x02 状态:level=2、code=0x0101。
    n = hex_to_bytes("45 54 07 00 02 00 02 01 01 75 90", buf, sizeof(buf));
    buf_len = n;
    assert(aura_extract_responses(buf, &buf_len, out, 4) == 1);
    assert(aura_extract_responses(buf, &buf_len, out, 4) == 0);
    assert(nova_parse_status(&out[0], &state, &code));
    assert(state == NOVA_STATE_INITIALIZING && code == 0x0101);

    // Opcode(LE) 0x01 信息应答(43B,厂商 Excel/测试向量,CRC 05 C8)。
    n = hex_to_bytes(
        "45 54 27 00 01 00 0C 05 0F"
        "54 30 34 32 32 31 32 30 30 30 30 31"
        "31 2E 30 2E 30"
        "31 2E 30 2E 30 2D 31 30 30 31 30 30 31 30 30"
        "05 C8",
        buf, sizeof(buf));
    assert(n == 43);
    buf_len = n;
    assert(aura_extract_responses(buf, &buf_len, out, 4) == 1);
    assert(out[0].result == 0 && out[0].opcode == 0x01 && out[0].payload_len == 35);
    nova_device_info_t info;
    assert(nova_parse_device_info(&out[0], &info));
    assert(strcmp(info.sn, "T04221200001") == 0);
    assert(strcmp(info.hardware_version, "1.0.0") == 0);
    assert(strcmp(info.firmware_version, "1.0.0-100100100") == 0);
    assert(buf_len == 0);

    // Opcode(LE) 空 payload:Length=4 → 整帧 8B,result 视为 0。
    const uint8_t empty[] = {0x02};
    uint8_t small[16];
    const size_t small_len = build_opcode_response(0x11, empty, 0, small,
                                                   sizeof(small));
    assert(small_len == 8);
    memcpy(buf, small, small_len);
    buf_len = small_len;
    assert(aura_extract_responses(buf, &buf_len, out, 4) == 1);
    assert(out[0].result == 0 && out[0].opcode == 0x11 && out[0].payload_len == 0);
}

// ---------------------------------------------------------------------------
// 0x01 设备信息解析:接受尾部填充(aura_parse_device_info)
//
// 依据:aura_ble_test p2_parsers.py 的 0x01 解析只要求长度足够、忽略声明
// 结束后的尾部填充;而 nova_parse_device_info 要求 plen 精确等于
// 3+snLen+hwLen+fwLen。两者的安全上限(SN<=120、hw/fw 留 NUL)相同。
// ---------------------------------------------------------------------------
static void test_parse_device_info_padding(void)
{
    uint8_t payload[64];
    const size_t info_len = make_info_payload(payload, sizeof(payload));
    assert(info_len == 35);

    nova_response_t resp;
    nova_device_info_t tolerant;
    nova_device_info_t strict;

    // 1) 精确长度:两个解析器都必须成功。
    memset(&resp, 0, sizeof(resp));
    resp.opcode = NOVA_OP_DEVICE_INFO;
    resp.payload_len = (uint16_t)info_len;
    memcpy(resp.payload, payload, info_len);
    assert(aura_parse_device_info(&resp, &tolerant));
    assert(nova_parse_device_info(&resp, &strict));
    assert(strcmp(tolerant.sn, "T04221200001") == 0);
    assert(strcmp(tolerant.hardware_version, "1.0.0") == 0);
    assert(strcmp(tolerant.firmware_version, "1.0.0-100100100") == 0);
    assert(strcmp(tolerant.sn, strict.sn) == 0);
    assert(strcmp(tolerant.hardware_version, strict.hardware_version) == 0);
    assert(strcmp(tolerant.firmware_version, strict.firmware_version) == 0);

    // 2) 2 字节尾部填充:aura 成功(忽略填充),nova 严格解析必须失败。
    resp.payload_len = (uint16_t)(info_len + 2);
    resp.payload[info_len] = 0xEE;
    resp.payload[info_len + 1] = 0x7F;
    memset(&tolerant, 0x7F, sizeof(tolerant));
    assert(aura_parse_device_info(&resp, &tolerant));
    assert(strcmp(tolerant.sn, "T04221200001") == 0);
    assert(strcmp(tolerant.hardware_version, "1.0.0") == 0);
    assert(strcmp(tolerant.firmware_version, "1.0.0-100100100") == 0);
    assert(!nova_parse_device_info(&resp, &strict));

    // 3) 3 字节填充(与任务描述一致),字符串必须在声明长度处结束,
    //    填充内容不得进入 sn/hw/fw。
    resp.payload_len = (uint16_t)(info_len + 3);
    resp.payload[info_len] = 'X';
    resp.payload[info_len + 1] = 'Y';
    resp.payload[info_len + 2] = 'Z';
    assert(aura_parse_device_info(&resp, &tolerant));
    assert(strcmp(tolerant.sn, "T04221200001") == 0);
    assert(strcmp(tolerant.hardware_version, "1.0.0") == 0);
    assert(strcmp(tolerant.firmware_version, "1.0.0-100100100") == 0);
    assert(strlen(tolerant.firmware_version) == 15);
    assert(tolerant.firmware_version[15] == '\0');

    // 4) 尾部填充也可以来自真实帧提取(Frame_ID 布局)。
    uint8_t frame[128];
    uint8_t padded[64];
    memcpy(padded, payload, info_len);
    padded[info_len] = 0x5A;
    padded[info_len + 1] = 0xA5;
    size_t frame_len = build_frame_id_response(0, 0x01, padded,
                                               info_len + 2, frame,
                                               sizeof(frame));
    nova_response_t frames[2];
    size_t buf_len = frame_len;
    assert(aura_extract_responses(frame, &buf_len, frames, 2) == 1);
    assert(frames[0].payload_len == info_len + 2);
    assert(aura_parse_device_info(&frames[0], &tolerant));
    assert(strcmp(tolerant.sn, "T04221200001") == 0);
    assert(!nova_parse_device_info(&frames[0], &strict));

    // 5) Opcode(LE) 布局同样接受尾部填充。
    const size_t opcode_frame_len = build_opcode_response(0x01, padded,
                                                          info_len + 2, frame,
                                                          sizeof(frame));
    buf_len = opcode_frame_len;
    assert(aura_extract_responses(frame, &buf_len, frames, 2) == 1);
    assert(frames[0].opcode == 0x01 && frames[0].result == 0);
    assert(aura_parse_device_info(&frames[0], &tolerant));
    assert(strcmp(tolerant.hardware_version, "1.0.0") == 0);

    // 6) 截断:声明长度比实际少 1 字节 → 失败。
    resp.payload_len = (uint16_t)(info_len - 1);
    assert(!aura_parse_device_info(&resp, &tolerant));

    // 7) 长度前缀比 payload 声明的更长 → 失败(越界读取必须被拒绝)。
    resp.payload_len = (uint16_t)info_len;
    resp.payload[1] = 6;  // hw_len 5→6,used 变为 36 > 35
    assert(!aura_parse_device_info(&resp, &tolerant));
    resp.payload[1] = 5;
    resp.payload[2] = 16;  // fw_len 15→16
    assert(!aura_parse_device_info(&resp, &tolerant));
    resp.payload[2] = 15;
    resp.payload[0] = 13;  // sn_len 12→13
    assert(!aura_parse_device_info(&resp, &tolerant));

    // 8) 长度前缀超过静态上限:NUL 放不下 → 失败。
    memset(&resp, 0, sizeof(resp));
    resp.opcode = NOVA_OP_DEVICE_INFO;
    resp.payload_len = 200;
    resp.payload[0] = 121;  // SN > NOVA_MAX_SN_LEN
    resp.payload[1] = 1;
    resp.payload[2] = 1;
    assert(!aura_parse_device_info(&resp, &tolerant));
    resp.payload[0] = 1;
    resp.payload[1] = (uint8_t)sizeof(tolerant.hardware_version);  // 48:无 NUL 空间
    assert(!aura_parse_device_info(&resp, &tolerant));
    resp.payload[1] = 1;
    resp.payload[2] = (uint8_t)sizeof(tolerant.firmware_version);  // 48
    assert(!aura_parse_device_info(&resp, &tolerant));

    // 9) payload 太短(3 字节前缀不完整 / 空 payload)→ 失败。
    resp.payload_len = 0;
    assert(!aura_parse_device_info(&resp, &tolerant));
    resp.payload_len = 2;
    assert(!aura_parse_device_info(&resp, &tolerant));

    // 10) opcode 不是 0x01 / NULL 参数 → 失败。
    resp.payload_len = (uint16_t)info_len;
    memcpy(resp.payload, payload, info_len);
    resp.opcode = NOVA_OP_STATUS;
    assert(!aura_parse_device_info(&resp, &tolerant));
    resp.opcode = NOVA_OP_DEVICE_INFO;
    assert(!aura_parse_device_info(NULL, &tolerant));
    assert(!aura_parse_device_info(&resp, NULL));
}

// ---------------------------------------------------------------------------
// 流式重组:两种布局混合 / 半包 / 拆头 / 坏 CRC / 坏长度
// ---------------------------------------------------------------------------
static void test_extract_streaming(void)
{
    uint8_t buf[512];
    nova_response_t out[4];
    uint8_t info_payload[64];
    const size_t info_len = make_info_payload(info_payload, sizeof(info_payload));

    uint8_t fid_info[128];
    const size_t fid_info_len = build_frame_id_response(0, 0x01, info_payload,
                                                        info_len, fid_info,
                                                        sizeof(fid_info));
    uint8_t op_status[32];
    const uint8_t status_payload[3] = {2, 1, 1};
    const size_t op_status_len = build_opcode_response(0x02, status_payload, 3,
                                                       op_status,
                                                       sizeof(op_status));

    // 两种布局粘包:Frame_ID 信息 + Opcode(LE) 状态,一次取两帧。
    memcpy(buf, fid_info, fid_info_len);
    memcpy(buf + fid_info_len, op_status, op_status_len);
    size_t buf_len = fid_info_len + op_status_len;
    assert(aura_extract_responses(buf, &buf_len, out, 4) == 2);
    assert(out[0].opcode == 0x01 && out[0].result == 0);
    assert(out[1].opcode == 0x02 && out[1].result == 0);
    assert(buf_len == 0);

    // 半包:先给整帧的 6 字节(Length 域已完整但 payload 未到齐)。
    buf_len = 6;
    memcpy(buf, fid_info, 6);
    assert(aura_extract_responses(buf, &buf_len, out, 4) == 0);
    assert(buf_len == 6);
    memcpy(buf + buf_len, fid_info + 6, fid_info_len - 6);
    buf_len += fid_info_len - 6;
    assert(aura_extract_responses(buf, &buf_len, out, 4) == 1);
    assert(out[0].opcode == 0x01 && buf_len == 0);

    // 只收到帧头:保留等待。
    memcpy(buf, fid_info, 2);
    buf_len = 2;
    assert(aura_extract_responses(buf, &buf_len, out, 4) == 0);
    assert(buf_len == 2);

    // 拆成单字节帧头:0x45 单独到达时只保留该字节。
    buf[0] = FRAMED_DEV_HEADER0;
    buf_len = 1;
    assert(aura_extract_responses(buf, &buf_len, out, 4) == 0);
    assert(buf_len == 1 && buf[0] == FRAMED_DEV_HEADER0);
    memcpy(buf + 1, fid_info + 1, fid_info_len - 1);
    buf_len += fid_info_len - 1;
    assert(aura_extract_responses(buf, &buf_len, out, 4) == 1);
    assert(out[0].opcode == 0x01 && buf_len == 0);

    // 坏 CRC 后紧跟合法帧:逐字节重同步。
    memcpy(buf, fid_info, fid_info_len);
    buf[fid_info_len - 1] ^= 0xFF;
    memcpy(buf + fid_info_len, op_status, op_status_len);
    buf_len = fid_info_len + op_status_len;
    assert(aura_extract_responses(buf, &buf_len, out, 4) == 1);
    assert(out[0].opcode == 0x02 && buf_len == 0);

    // 帧头前噪声。
    buf[0] = 0x00;
    buf[1] = 0x41;
    buf[2] = 0x14;
    memcpy(buf + 3, op_status, op_status_len);
    buf_len = 3 + op_status_len;
    assert(aura_extract_responses(buf, &buf_len, out, 4) == 1);
    assert(out[0].opcode == 0x02 && buf_len == 0);

    // Length 太小(<4)视为噪声:丢 1 字节后继续。
    buf[0] = FRAMED_DEV_HEADER0;
    buf[1] = FRAMED_DEV_HEADER1;
    buf[2] = 0x03;
    buf[3] = 0x00;
    memcpy(buf + 4, op_status, op_status_len);
    buf_len = 4 + op_status_len;
    assert(aura_extract_responses(buf, &buf_len, out, 4) == 1);
    assert(out[0].opcode == 0x02 && buf_len == 0);

    // 超出 4096 的 Length 按噪声处理。
    buf[0] = FRAMED_DEV_HEADER0;
    buf[1] = FRAMED_DEV_HEADER1;
    buf[2] = 0x0D;
    buf[3] = 0x10;  // 0x100D + 4 > 4096
    memcpy(buf + 4, op_status, op_status_len);
    buf_len = 4 + op_status_len;
    assert(aura_extract_responses(buf, &buf_len, out, 4) == 1);
    assert(out[0].opcode == 0x02 && buf_len == 0);

    // Frame_ID 布局但 Length 与 Plen 不自洽(CRC 仍合法):判坏帧并重同步。
    {
        uint8_t bad[16];
        const uint16_t length = 9;  // Frame_ID 最小 Length 应为 8
        bad[0] = FRAMED_DEV_HEADER0;
        bad[1] = FRAMED_DEV_HEADER1;
        bad[2] = (uint8_t)(length & 0xFF);
        bad[3] = (uint8_t)(length >> 8);
        bad[4] = 0x00;
        bad[5] = 0x00;
        bad[6] = 0x00;  // result
        bad[7] = 0x01;  // opcode
        bad[8] = 0x00;  // plen = 0(应等于 length-8 = 1)
        bad[9] = 0x00;
        bad[10] = 0xAA;
        const uint16_t crc = framed_crc16(bad + 2, length);
        bad[11] = (uint8_t)(crc & 0xFF);
        bad[12] = (uint8_t)(crc >> 8);
        const size_t bad_len = 13;
        memcpy(buf, bad, bad_len);
        memcpy(buf + bad_len, op_status, op_status_len);
        buf_len = bad_len + op_status_len;
        assert(aura_extract_responses(buf, &buf_len, out, 4) == 1);
        assert(out[0].opcode == 0x02 && buf_len == 0);
    }

    // max_out 限制:一帧后保留剩余,再次调用可继续取。
    memcpy(buf, fid_info, fid_info_len);
    memcpy(buf + fid_info_len, op_status, op_status_len);
    buf_len = fid_info_len + op_status_len;
    assert(aura_extract_responses(buf, &buf_len, out, 1) == 1);
    assert(out[0].opcode == 0x01);
    assert(buf_len == op_status_len);
    assert(aura_extract_responses(buf, &buf_len, out, 1) == 1);
    assert(out[0].opcode == 0x02 && buf_len == 0);

    // 参数保护。
    buf_len = 0;
    assert(aura_extract_responses(NULL, &buf_len, out, 4) == 0);
    assert(aura_extract_responses(buf, NULL, out, 4) == 0);
    assert(aura_extract_responses(buf, &buf_len, NULL, 4) == 0);
    assert(aura_extract_responses(buf, &buf_len, out, 0) == 0);
}

// ---------------------------------------------------------------------------
// 端到端(免登录):0x01 信息 → 0x02 状态,不得出现 0x07/0x08
// ---------------------------------------------------------------------------
static void test_no_auth_roundtrip(void)
{
    uint8_t frame[AURA_MAX_RESPONSE];
    nova_response_t out[2];
    uint8_t info_payload[64];
    const size_t info_len = make_info_payload(info_payload, sizeof(info_payload));

    // AURA 会话只发 0x01/0x02,不发 0x07/0x08(免登录)。
    uint8_t requests[2][AURA_MAX_REQUEST];
    size_t request_lens[2];
    request_lens[0] = aura_build_request(0x01, NULL, 0, requests[0], sizeof(requests[0]));
    request_lens[1] = aura_build_request(0x02, NULL, 0, requests[1], sizeof(requests[1]));
    assert(request_lens[0] == 11 && request_lens[1] == 11);
    const uint8_t request_ops[2] = {requests[0][6], requests[1][6]};
    assert(request_ops[0] == 0x01 && request_ops[1] == 0x02);

    // 0x01 应答:Frame_ID 与 Opcode(LE) 两种布局都要能解析。
    size_t buf_len = build_frame_id_response(0, 0x01, info_payload, info_len,
                                             frame, sizeof(frame));
    assert(aura_extract_responses(frame, &buf_len, out, 2) == 1);
    nova_device_info_t info;
    assert(nova_parse_device_info(&out[0], &info));
    assert(strcmp(info.sn, "T04221200001") == 0);

    buf_len = build_opcode_response(0x01, info_payload, info_len, frame,
                                    sizeof(frame));
    assert(aura_extract_responses(frame, &buf_len, out, 2) == 1);
    assert(nova_parse_device_info(&out[0], &info));
    assert(strcmp(info.firmware_version, "1.0.0-100100100") == 0);

    // 0x02 应答(两种布局)都映射到同一状态表。
    const uint8_t status_payload[3] = {3, 1, 0};
    nova_state_t state = NOVA_STATE_UNKNOWN;
    uint16_t code = 0xFFFF;
    buf_len = build_frame_id_response(0, 0x02, status_payload, 3, frame,
                                      sizeof(frame));
    assert(aura_extract_responses(frame, &buf_len, out, 2) == 1);
    assert(nova_parse_status(&out[0], &state, &code));
    assert(state == NOVA_STATE_IDLE && code == 1);

    buf_len = build_opcode_response(0x02, status_payload, 3, frame, sizeof(frame));
    assert(aura_extract_responses(frame, &buf_len, out, 2) == 1);
    assert(nova_parse_status(&out[0], &state, &code));
    assert(state == NOVA_STATE_IDLE && code == 1);

    // 设备拒绝 0x01 时 result 原样上报(由 client 判定失败,不尝试认证)。
    buf_len = build_frame_id_response(1, 0x01, NULL, 0, frame, sizeof(frame));
    assert(aura_extract_responses(frame, &buf_len, out, 2) == 1);
    assert(out[0].result == 1 && out[0].opcode == 0x01);
}

int main(void)
{
    test_build_request();
    test_extract_frame_id();
    test_extract_opcode_layout();
    test_parse_device_info_padding();
    test_extract_streaming();
    test_no_auth_roundtrip();
    puts("AURA P2 proto tests: PASS");
    return 0;
}
