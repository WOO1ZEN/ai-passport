// tests/test_framed_proto.c —— OMNI / HALO / PACE 的 P1 Framed 线格式主机单测。
//
// 契约来源:main/framed_proto.h。只依赖 C11 标准库与 assert(),不链接
// ESP-IDF/LVGL。解析结果复用 nova_response_t / nova_parse_*。
//
// 黄金向量出处(逐字节):
// - CRC:
//   * 05 00 01 00 00 → 线上 B9 C0、05 00 07 00 00 → 59 C1、空数据 → FF FF、
//     "123456789" → 37 4B:/tmp/omni_protocol_spec.md §6d,以及
//     ble_control Crc16Test.kt / pace_ble_test src/utils/crc16.py。
//   * 请求帧 41 14 05 00 01/02/07 00 00 + CRC:ProtocolCodecTest.kt
//     testFramedCodec_encodeRequest、spec §6(a)(b)、PACE §7 V1/V2。
// - 0x01 应答真机帧(45B):
//   omni_halo_ble_sdk/tests/test_client.py:92-113(经 spec §6b 转录),
//   解析 sn=T04221200001 hw=1.0.0 fw=1.0.0-100100100。
// - 0x08 认证成功应答 45 54 06 00 00 08 00 00 81 BF:ProtocolCodecTest.kt
//   testBareCodec_decodeResponse_validCrc / spec §6(a)。
// - 0x07 应答 SN=P16000000121 动态码=TFwXJY 与私钥 4adf...:spec §6(a),
//   私钥与 CheckCodeAuthenticatorTest.kt 一致。
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "framed_proto.h"
#include "md5.h"
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

// 测试侧独立组设备应答帧(与 framed_build_request 对偶),用于构造半包/粘包数据。
static size_t build_device_frame(uint8_t result, uint8_t opcode,
                                 const uint8_t *payload, size_t payload_len,
                                 uint8_t *out, size_t out_cap)
{
    assert(out_cap >= 10 + payload_len);
    const uint16_t length = (uint16_t)(payload_len + 6);
    out[0] = FRAMED_DEV_HEADER0;
    out[1] = FRAMED_DEV_HEADER1;
    out[2] = (uint8_t)(length & 0xFF);
    out[3] = (uint8_t)(length >> 8);
    out[4] = result;
    out[5] = opcode;
    out[6] = (uint8_t)(payload_len & 0xFF);
    out[7] = (uint8_t)(payload_len >> 8);
    if (payload_len > 0) {
        memcpy(out + 8, payload, payload_len);
    }
    const uint16_t crc = framed_crc16(out + 2, length);
    out[8 + payload_len] = (uint8_t)(crc & 0xFF);
    out[9 + payload_len] = (uint8_t)(crc >> 8);
    return 10 + payload_len;
}

// ---------------------------------------------------------------------------
// CRC16 / Modbus
// ---------------------------------------------------------------------------
static void test_crc16(void)
{
    // 空数据:初值 0xFFFF 原样输出,线上 FF FF。
    assert(framed_crc16(NULL, 0) == 0xFFFFu);
    assert(framed_crc16((const uint8_t *)"", 0) == 0xFFFFu);

    const uint8_t v1_data[] = {0x05, 0x00, 0x01, 0x00, 0x00};
    assert(framed_crc16(v1_data, sizeof(v1_data)) == 0xC0B9u);

    const uint8_t v2_data[] = {0x05, 0x00, 0x07, 0x00, 0x00};
    assert(framed_crc16(v2_data, sizeof(v2_data)) == 0xC159u);

    // CRC 只对传入长度负责;改长度不会读到后面数据。
    assert(framed_crc16(v1_data, 3) != framed_crc16(v1_data, 5));

    // 标准 Modbus 自检向量 "123456789" → 37 4B。
    const uint8_t check[] = "123456789";
    assert(framed_crc16(check, 9) == 0x4B37u);

    // 请求帧尾字节必须低字节在前:B9 C0 / 59 C1。
    uint8_t frame[32];
    assert(framed_build_request(0x01, NULL, 0, frame, sizeof(frame)) == 9);
    assert(frame[7] == 0xB9 && frame[8] == 0xC0);
    assert(framed_build_request(0x07, NULL, 0, frame, sizeof(frame)) == 9);
    assert(frame[7] == 0x59 && frame[8] == 0xC1);
}

// ---------------------------------------------------------------------------
// 请求组包
// ---------------------------------------------------------------------------
static void test_build_request(void)
{
    uint8_t frame[FRAMED_MAX_REQUEST];

    // 0x01 设备信息查询(真机 TX 黄金帧)。
    size_t n = framed_build_request(0x01, NULL, 0, frame, sizeof(frame));
    assert_hex_equals(frame, n, "41 14 05 00 01 00 00 B9 C0");

    // 0x07 取动态码(与 ble_control ProtocolCodecTest.kt 的 Framed 请求一致)。
    n = framed_build_request(0x07, NULL, 0, frame, sizeof(frame));
    assert_hex_equals(frame, n, "41 14 05 00 07 00 00 59 C1");

    // 0x02 状态查询。
    n = framed_build_request(0x02, NULL, 0, frame, sizeof(frame));
    assert_hex_equals(frame, n, "41 14 05 00 02 00 00 49 C0");

    // 0x08 私钥(SN=P16000000121 + TFwXJY 算出的 4adf...):
    // TX 帧来源 spec §6(a)(CRC C9 7D 已由 Crc16Test.kt 交叉验证)。
    char key[33];
    assert(nova_make_private_key((const uint8_t *)"P16000000121", 12,
                                 (const uint8_t *)"TFwXJY", 6, key));
    assert(strcmp(key, "4adf57c6005bf74e9d921ebab767b2b2") == 0);
    n = framed_build_request(0x08, key, 32, frame, sizeof(frame));
    assert(n == 41);
    assert_hex_equals(frame, n,
                      "41 14 25 00 08 20 00"
                      "34 61 64 66 35 37 63 36 30 30 35 62 66 37 34 65"
                      "39 64 39 32 31 65 62 61 62 37 36 37 62 32 62 32"
                      "C9 7D");

    // 带 payload 的通用帧:长度 = plen + 5,CRC 覆盖整段数据域。
    const uint8_t payload[] = {0x11, 0x22, 0x33};
    n = framed_build_request(0x80, payload, sizeof(payload), frame, sizeof(frame));
    assert(n == 12);
    assert(frame[0] == 0x41 && frame[1] == 0x14);
    assert(frame[2] == 0x08 && frame[3] == 0x00);  // Length = 3 + 5
    assert(frame[4] == 0x80);
    assert(frame[5] == 0x03 && frame[6] == 0x00);
    assert(memcmp(frame + 7, payload, sizeof(payload)) == 0);
    {
        const uint16_t crc = framed_crc16(frame + 2, 8);
        assert(frame[10] == (uint8_t)(crc & 0xFF));
        assert(frame[11] == (uint8_t)(crc >> 8));
    }

    // 非法参数:超长 payload / 缓冲区不足 / 空 payload 指针。
    static uint8_t big[FRAMED_MAX_PAYLOAD + 1];
    assert(framed_build_request(0x01, big, sizeof(big), frame, sizeof(frame)) == 0);
    assert(framed_build_request(0x01, NULL, 1, frame, sizeof(frame)) == 0);
    assert(framed_build_request(0x01, NULL, 0, frame, 8) == 0);
    assert(framed_build_request(0x01, NULL, 0, NULL, 32) == 0);
}

// ---------------------------------------------------------------------------
// 响应提取:黄金帧
// ---------------------------------------------------------------------------
static void test_extract_goldens(void)
{
    uint8_t buf[256];
    nova_response_t out[4];

    // 0x08 认证成功应答。
    size_t n = hex_to_bytes("45 54 06 00 00 08 00 00 81 BF", buf, sizeof(buf));
    size_t buf_len = n;
    assert(framed_extract_responses(buf, &buf_len, out, 4) == 1);
    assert(out[0].result == 0 && out[0].opcode == 0x08 && out[0].payload_len == 0);
    assert(buf_len == 0);

    // 0x01 设备信息真机帧:复用 nova_parse_device_info 解析。
    n = hex_to_bytes(
        "45 54 29 00 00 01 23 00 0C 05 0F"
        "54 30 34 32 32 31 32 30 30 30 30 31"   // "T04221200001"
        "31 2E 30 2E 30"                        // "1.0.0"
        "31 2E 30 2E 30 2D 31 30 30 31 30 30 31 30 30"  // "1.0.0-100100100"
        "F7 C9",
        buf, sizeof(buf));
    assert(n == 45);
    buf_len = n;
    assert(framed_extract_responses(buf, &buf_len, out, 4) == 1);
    assert(out[0].result == 0 && out[0].opcode == 0x01 && out[0].payload_len == 35);
    nova_device_info_t info;
    assert(nova_parse_device_info(&out[0], &info));
    assert(strcmp(info.sn, "T04221200001") == 0);
    assert(strcmp(info.hardware_version, "1.0.0") == 0);
    assert(strcmp(info.firmware_version, "1.0.0-100100100") == 0);
    assert(buf_len == 0);

    // 0x02 状态帧 level=3 / code=1。
    n = hex_to_bytes("45 54 09 00 00 02 03 00 03 01 00 D4 D2", buf, sizeof(buf));
    buf_len = n;
    assert(framed_extract_responses(buf, &buf_len, out, 4) == 1);
    assert(out[0].result == 0 && out[0].opcode == 0x02 && out[0].payload_len == 3);
    nova_state_t state = NOVA_STATE_UNKNOWN;
    uint16_t code = 0xFFFF;
    assert(nova_parse_status(&out[0], &state, &code));
    assert(state == NOVA_STATE_IDLE && code == 1);

    // 粘包:三帧一次取出,顺序不变。
    uint8_t status_frame[32];
    const uint8_t status_payload[3] = {3, 1, 0};
    const size_t status_len = build_device_frame(0, 0x02, status_payload, 3,
                                                 status_frame, sizeof(status_frame));
    n = hex_to_bytes("45 54 06 00 00 08 00 00 81 BF", buf, sizeof(buf));
    memcpy(buf + n, status_frame, status_len);
    n += status_len;
    buf_len = n;
    assert(framed_extract_responses(buf, &buf_len, out, 4) == 2);
    assert(out[0].opcode == 0x08 && out[1].opcode == 0x02);
    assert(buf_len == 0);

    // max_out 限制:取走一帧后保留剩余帧,再调用可继续取。
    memcpy(buf, status_frame, status_len);
    memcpy(buf + status_len, status_frame, status_len);
    buf_len = status_len * 2;
    assert(framed_extract_responses(buf, &buf_len, out, 1) == 1);
    assert(buf_len == status_len);
    assert(framed_extract_responses(buf, &buf_len, out, 1) == 1);
    assert(buf_len == 0);
}

// ---------------------------------------------------------------------------
// 响应提取:半包 / 拆头 / 噪声 / 坏 CRC / 坏长度
// ---------------------------------------------------------------------------
static void test_extract_streaming(void)
{
    uint8_t golden[64];
    uint8_t buf[512];
    nova_response_t out[4];
    const size_t golden_len = hex_to_bytes(
        "45 54 29 00 00 01 23 00 0C 05 0F"
        "54 30 34 32 32 31 32 30 30 30 30 31 31 2E 30 2E 30"
        "31 2E 30 2E 30 2D 31 30 30 31 30 30 31 30 30 F7 C9",
        golden, sizeof(golden));
    uint8_t status_frame[32];
    const uint8_t status_payload[3] = {3, 1, 0};
    const size_t status_len = build_device_frame(0, 0x02, status_payload, 3,
                                                 status_frame, sizeof(status_frame));

    // 半包:先给前 10 字节,长度域合法但整帧未到,必须保留等待。
    size_t buf_len = 10;
    memcpy(buf, golden, 10);
    assert(framed_extract_responses(buf, &buf_len, out, 4) == 0);
    assert(buf_len == 10);
    memcpy(buf + buf_len, golden + 10, golden_len - 10);
    buf_len += golden_len - 10;
    assert(framed_extract_responses(buf, &buf_len, out, 4) == 1);
    assert(out[0].opcode == 0x01 && buf_len == 0);

    // 只收到帧头 + 半个 Length 域:保留等待。
    memcpy(buf, golden, 3);
    buf_len = 3;
    assert(framed_extract_responses(buf, &buf_len, out, 4) == 0);
    assert(buf_len == 3);

    // 拆成单字节帧头:0x45 单独到达时保留最后 1 字节。
    buf[0] = FRAMED_DEV_HEADER0;
    buf_len = 1;
    assert(framed_extract_responses(buf, &buf_len, out, 4) == 0);
    assert(buf_len == 1 && buf[0] == FRAMED_DEV_HEADER0);
    memcpy(buf + buf_len, golden + 1, golden_len - 1);
    buf_len += golden_len - 1;
    assert(framed_extract_responses(buf, &buf_len, out, 4) == 1);
    assert(out[0].opcode == 0x01 && buf_len == 0);

    // 帧头前噪声 + 坏 CRC 后紧跟合法帧:逐字节重同步后仍能取出合法帧。
    memcpy(buf, golden, golden_len);
    buf[golden_len - 1] ^= 0xFF;  // 故意破坏 CRC
    memcpy(buf + golden_len, status_frame, status_len);
    buf_len = golden_len + status_len;
    size_t frames = framed_extract_responses(buf, &buf_len, out, 4);
    assert(frames == 1);
    assert(out[0].opcode == 0x02 && buf_len == 0);

    // 帧头前有普通噪声。
    buf[0] = 0x00;
    buf[1] = 0x41;
    buf[2] = 0x14;
    memcpy(buf + 3, status_frame, status_len);
    buf_len = 3 + status_len;
    assert(framed_extract_responses(buf, &buf_len, out, 4) == 1);
    assert(out[0].opcode == 0x02 && buf_len == 0);

    // Length 太小(<6)视为噪声,丢 1 字节后找到后面的合法帧。
    buf[0] = 0x45;
    buf[1] = 0x54;
    buf[2] = 0x03;
    buf[3] = 0x00;
    memcpy(buf + 4, status_frame, status_len);
    buf_len = 4 + status_len;
    assert(framed_extract_responses(buf, &buf_len, out, 4) == 1);
    assert(out[0].opcode == 0x02 && buf_len == 0);

    // Length 超过 4096:按噪声处理并重同步。
    buf[0] = 0x45;
    buf[1] = 0x54;
    buf[2] = 0x0D;
    buf[3] = 0x10;  // 0x100D + 4 > 4096
    memcpy(buf + 4, status_frame, status_len);
    buf_len = 4 + status_len;
    assert(framed_extract_responses(buf, &buf_len, out, 4) == 1);
    assert(out[0].opcode == 0x02 && buf_len == 0);

    // CRC 合法但 Length 与 Plen 不自洽:仍应判定为坏帧并重同步。
    uint8_t inconsistent[16];
    {
        const uint16_t length = 7;  // plen 字段填 0,Length 应为 6
        inconsistent[0] = 0x45;
        inconsistent[1] = 0x54;
        inconsistent[2] = (uint8_t)(length & 0xFF);
        inconsistent[3] = (uint8_t)(length >> 8);
        inconsistent[4] = 0x00;
        inconsistent[5] = 0x02;
        inconsistent[6] = 0x00;
        inconsistent[7] = 0x00;
        inconsistent[8] = 0xAA;  // 多余字节
        const uint16_t crc = framed_crc16(inconsistent + 2, length);
        inconsistent[9] = (uint8_t)(crc & 0xFF);
        inconsistent[10] = (uint8_t)(crc >> 8);
        const size_t inconsistent_len = 11;
        memcpy(buf, inconsistent, inconsistent_len);
        memcpy(buf + inconsistent_len, status_frame, status_len);
        buf_len = inconsistent_len + status_len;
        assert(framed_extract_responses(buf, &buf_len, out, 4) == 1);
        assert(out[0].opcode == 0x02 && buf_len == 0);
    }

    // 参数保护。
    buf_len = 0;
    assert(framed_extract_responses(NULL, &buf_len, out, 4) == 0);
    assert(framed_extract_responses(buf, NULL, out, 4) == 0);
    assert(framed_extract_responses(buf, &buf_len, NULL, 4) == 0);
    assert(framed_extract_responses(buf, &buf_len, out, 0) == 0);
}

// ---------------------------------------------------------------------------
// 端到端:0x07 → 0x08 认证 → 0x01 信息 → 0x02 状态
// ---------------------------------------------------------------------------
static void test_auth_info_status_roundtrip(void)
{
    uint8_t frame[FRAMED_MAX_RESPONSE];
    nova_response_t out[2];
    size_t buf_len;

    // 0x07 请求。
    size_t req_len = framed_build_request(0x07, NULL, 0, frame, sizeof(frame));
    assert_hex_equals(frame, req_len, "41 14 05 00 07 00 00 59 C1");

    // 0x07 应答(真机格式,SN=P16000000121 / 动态码=TFwXJY)。
    const char *access_payload = "\x0C\x06" "P16000000121" "TFwXJY";
    const size_t access_len = 2 + 12 + 6;
    buf_len = build_device_frame(0, 0x07, (const uint8_t *)access_payload,
                                 access_len, frame, sizeof(frame));
    assert(framed_extract_responses(frame, &buf_len, out, 2) == 1);
    nova_access_info_t access;
    assert(nova_parse_access_info(&out[0], &access));
    assert(strcmp(access.sn, "P16000000121") == 0);
    assert(strcmp(access.code, "TFwXJY") == 0);

    // 本地 MD5 → 0x08 请求(黄金帧见 spec §6a)。
    char key[33];
    assert(nova_make_private_key((const uint8_t *)access.sn, access.sn_len,
                                 (const uint8_t *)access.code, access.code_len,
                                 key));
    assert(strcmp(key, "4adf57c6005bf74e9d921ebab767b2b2") == 0);
    req_len = framed_build_request(0x08, key, 32, frame, sizeof(frame));
    assert(req_len == 41);
    assert(memcmp(frame, "\x41\x14\x25\x00\x08\x20\x00", 7) == 0);
    assert(memcmp(frame + 7, key, 32) == 0);

    // 0x08 成功应答。
    buf_len = hex_to_bytes("45 54 06 00 00 08 00 00 81 BF", frame, sizeof(frame));
    assert(framed_extract_responses(frame, &buf_len, out, 2) == 1);
    assert(out[0].result == 0 && out[0].opcode == 0x08);

    // 0x01 设备信息。
    const char *info_payload =
        "\x0C\x05\x0F" "T04221200001" "1.0.0" "1.0.0-100100100";
    const size_t info_len = 3 + 12 + 5 + 15;
    buf_len = build_device_frame(0, 0x01, (const uint8_t *)info_payload, info_len,
                                 frame, sizeof(frame));
    assert(framed_extract_responses(frame, &buf_len, out, 2) == 1);
    nova_device_info_t info;
    assert(nova_parse_device_info(&out[0], &info));
    assert(strcmp(info.sn, "T04221200001") == 0);
    assert(strcmp(info.hardware_version, "1.0.0") == 0);
    assert(strcmp(info.firmware_version, "1.0.0-100100100") == 0);

    // 0x02 状态:认证后查询,level=3/code=1(空闲待命)。
    const uint8_t status_payload[3] = {3, 1, 0};
    buf_len = build_device_frame(0, 0x02, status_payload, sizeof(status_payload),
                                 frame, sizeof(frame));
    assert(framed_extract_responses(frame, &buf_len, out, 2) == 1);
    nova_state_t state = NOVA_STATE_UNKNOWN;
    uint16_t code = 0xFFFF;
    assert(nova_parse_status(&out[0], &state, &code));
    assert(state == NOVA_STATE_IDLE && code == 1);

    // 认证失败帧 result=1 必须原样上报给上层(由 client 判 auth rejected)。
    buf_len = build_device_frame(1, 0x08, NULL, 0, frame, sizeof(frame));
    assert(framed_extract_responses(frame, &buf_len, out, 2) == 1);
    assert(out[0].result == 1 && out[0].opcode == 0x08);
}

int main(void)
{
    test_crc16();
    test_build_request();
    test_extract_goldens();
    test_extract_streaming();
    test_auth_info_status_roundtrip();
    puts("P1 framed proto tests: PASS");
    return 0;
}
