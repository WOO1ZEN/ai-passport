// tests/test_nova_proto.c —— NOVA Bare 协议层的主机单元测试。
//
// 契约来源:main/nova_proto.h、main/device_proto.h、main/md5.h。
// 只依赖 C11 标准库与 assert(),不链接 ESP-IDF/LVGL/动态内存。
//
// 黄金向量出处(逐字节复现):
// - 0x07 / 0x01 / 0x02 / 0x08 真机抓包:ble_nova/tests/offline/test_bare_wire.py
//   的 GOLDEN_* 常量(2026-10-13 对 NOVA_O30241001620 实测),以及
//   /tmp/nova_protocol_spec.md §7 的 V1..V5。
// - 组包向量:ble_control/app/src/test/java/.../ProtocolCodecTest.kt
//   testBareCodec_encodeRequest_* 与 test_bare_wire.py::test_bare_request_goldens。
// - 认证私钥:P16000000121/TFwXJY 等 6 组来自
//   ble_control/app/src/test/java/.../CheckCodeAuthenticatorTest.kt
//   testPythonGoldenVectors;O30241001620/kAgk4M 来自 nova 规范 §7 V2。
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "device_proto.h"
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
            continue;  // 允许空格分隔
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

static size_t build_response(uint8_t result, uint8_t opcode, const uint8_t *payload,
                             size_t payload_len, uint8_t *out, size_t out_cap)
{
    assert(out_cap >= 4 + payload_len);
    out[0] = result;
    out[1] = opcode;
    out[2] = (uint8_t)(payload_len & 0xFFu);
    out[3] = (uint8_t)((payload_len >> 8) & 0xFFu);
    if (payload_len > 0) {
        memcpy(out + 4, payload, payload_len);
    }
    return 4 + payload_len;
}

static nova_response_t parse_exact(const uint8_t *data, size_t len)
{
    nova_response_t resp;

    memset(&resp, 0, sizeof(resp));
    assert(nova_parse_response(data, len, &resp));
    return resp;
}

// ---------------------------------------------------------------------------
// 组包
// ---------------------------------------------------------------------------
static void test_build_request_goldens(void)
{
    uint8_t out[600];
    char hex[1200];

    // 0x07 空 payload → 07 00 00(ProtocolCodecTest.kt 对应向量)。
    assert(nova_build_request(NOVA_OP_ACCESS_INFO, NULL, 0, out, sizeof(out)) == 3);
    assert(memcmp(out, "\x07\x00\x00", 3) == 0);

    // 0x02 空 payload → 02 00 00(ble_nova test_bare_wire.py)。
    assert(nova_build_request(NOVA_OP_STATUS, NULL, 0, out, sizeof(out)) == 3);
    assert(memcmp(out, "\x02\x00\x00", 3) == 0);
    // 0x01 空 payload → 01 00 00。
    assert(nova_build_request(NOVA_OP_DEVICE_INFO, NULL, 0, out, sizeof(out)) == 3);
    assert(memcmp(out, "\x01\x00\x00", 3) == 0);

    // 0x08 私钥 32 字节 ASCII 'a' → 08 20 00 + 61*32。
    uint8_t key[32];
    memset(key, 'a', sizeof(key));
    assert(nova_build_request(NOVA_OP_ACCESS_VERIFY, key, sizeof(key), out,
                              sizeof(out)) == 3 + 32);
    assert(out[0] == 0x08 && out[1] == 0x20 && out[2] == 0x00);
    for (size_t i = 0; i < 32; i++) {
        assert(out[3 + i] == 'a');
    }

    // 0x83 心跳 6 字节 host id → 83 06 00 aabbccddeeff。
    const uint8_t host_id[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
    assert(nova_build_request(0x83, host_id, sizeof(host_id), out,
                              sizeof(out)) == 9);
    const uint8_t hb_expected[9] = {0x83, 0x06, 0x00, 0xAA, 0xBB,
                                    0xCC, 0xDD, 0xEE, 0xFF};
    assert(memcmp(out, hb_expected, sizeof(hb_expected)) == 0);

    // 长度越界 / 缓冲区不足 / 指针非法都返回 0。
    uint8_t big[NOVA_MAX_PAYLOAD + 1];
    memset(big, 0x11, sizeof(big));
    assert(nova_build_request(0x80, big, sizeof(big), out, sizeof(out)) == 0);
    assert(nova_build_request(0x80, "abc", 3, out, 5) == 0);
    assert(nova_build_request(0x80, NULL, 3, out, sizeof(out)) == 0);
    assert(nova_build_request(0x80, "abc", 3, NULL, 10) == 0);

    // 完整 hex 输出便于人工核对:0x01 请求即为 "010000"。
    size_t n = nova_build_request(NOVA_OP_DEVICE_INFO, NULL, 0, out, sizeof(out));
    for (size_t i = 0; i < n; i++) {
        (void)snprintf(hex + i * 2, 3, "%02x", out[i]);
    }
    assert(strcmp(hex, "010000") == 0);
}

// ---------------------------------------------------------------------------
// 响应解析(单帧)
// ---------------------------------------------------------------------------
static void test_parse_response(void)
{
    static const char *const golden_07 =
        "00 07 14 00 0c 06 "
        "4f3330323431303031363230"  // sn_len=12, dyn_len=6, "O30241001620"
        "6b41676b344d";              // "kAgk4M"
    uint8_t buf[64];
    size_t n = hex_to_bytes(golden_07, buf, sizeof(buf));

    nova_response_t resp = parse_exact(buf, n);
    assert(resp.result == 0);
    assert(resp.opcode == NOVA_OP_ACCESS_INFO);
    assert(resp.payload_len == 20);

    // result 只接受 0/1。
    n = build_response(2, NOVA_OP_STATUS, NULL, 0, buf, sizeof(buf));
    assert(!nova_parse_response(buf, n, &resp));

    // 长度与 payload_len 不一致。
    n = build_response(0, NOVA_OP_STATUS, (const uint8_t *)"\x00\x00\x00", 3,
                       buf, sizeof(buf));
    assert(nova_parse_response(buf, n, &resp));
    assert(!nova_parse_response(buf, n - 1, &resp));

    // payload_len 超过 512 视为噪声。
    const uint8_t too_long[4] = {0x00, 0x02, 0x01, 0x02};  // plen = 0x0201
    assert(!nova_parse_response(too_long, sizeof(too_long), &resp));

    // 半帧 / 空输入。
    assert(!nova_parse_response(buf, 3, &resp));
    assert(!nova_parse_response(NULL, 8, &resp));
    assert(!nova_parse_response(buf, n, NULL));
}

// ---------------------------------------------------------------------------
// 流式提取:半包 / 粘包 / 坏长度重同步
// ---------------------------------------------------------------------------
static void test_extract_responses(void)
{
    static const char *const golden_01 =
        "00 01 1f 00 0c 03 0d "
        "4f3330323431303031363230"  // SN
        "563031"                    // V01
        "33302e302e352d33302e302e36";  // 30.0.5-30.0.6
    static const char *const golden_02 = "00 02 03 00 00 00 00";
    static const char *const ack_08 = "00 08 00 00";

    uint8_t frame01[64];
    uint8_t frame02[16];
    uint8_t ack[8];
    const size_t len01 = hex_to_bytes(golden_01, frame01, sizeof(frame01));
    const size_t len02 = hex_to_bytes(golden_02, frame02, sizeof(frame02));
    const size_t len_ack = hex_to_bytes(ack_08, ack, sizeof(ack));

    nova_response_t out[4];
    uint8_t buf[128];
    size_t buf_len;

    // 粘包:0x02 + 0x08 一次提取两帧。
    memcpy(buf, frame02, len02);
    memcpy(buf + len02, ack, len_ack);
    buf_len = len02 + len_ack;
    assert(nova_extract_responses(buf, &buf_len, out, 4) == 2);
    assert(out[0].opcode == NOVA_OP_STATUS && out[0].result == 0);
    assert(out[1].opcode == NOVA_OP_ACCESS_VERIFY && out[1].result == 0);
    assert(buf_len == 0);

    // 半包:先给 0x01 帧的前 10 字节,剩余字节等到第二次 feed。
    buf_len = 10;
    memcpy(buf, frame01, 10);
    assert(nova_extract_responses(buf, &buf_len, out, 4) == 0);
    assert(buf_len == 10);
    memcpy(buf + buf_len, frame01 + 10, len01 - 10);
    buf_len += len01 - 10;
    assert(nova_extract_responses(buf, &buf_len, out, 4) == 1);
    assert(out[0].opcode == NOVA_OP_DEVICE_INFO && buf_len == 0);

    // 噪声重同步:非法 result / 未知 opcode / 非法长度都应被逐字节丢弃。
    const uint8_t noise[] = {0xEE, 0xEE, 0x00, 0xAA, 0x00, 0x00, 0x00, 0x02,
                             0xFF, 0x7F};
    memcpy(buf, noise, sizeof(noise));
    memcpy(buf + sizeof(noise), frame02, len02);
    buf_len = sizeof(noise) + len02;
    assert(nova_extract_responses(buf, &buf_len, out, 4) == 1);
    assert(out[0].opcode == NOVA_OP_STATUS && buf_len == 0);

    // max_out 限制:一帧后保留剩余半包,再次调用可继续提取。
    memcpy(buf, frame02, len02);
    memcpy(buf + len02, ack, len_ack);
    buf_len = len02 + len_ack;
    assert(nova_extract_responses(buf, &buf_len, out, 1) == 1);
    assert(out[0].opcode == NOVA_OP_STATUS);
    assert(buf_len == len_ack);
    assert(nova_extract_responses(buf, &buf_len, out, 1) == 1);
    assert(out[0].opcode == NOVA_OP_ACCESS_VERIFY && buf_len == 0);

    // 参数保护。
    assert(nova_extract_responses(NULL, &buf_len, out, 4) == 0);
    assert(nova_extract_responses(buf, NULL, out, 4) == 0);
    buf_len = 0;
    assert(nova_extract_responses(buf, &buf_len, out, 0) == 0);
}

// ---------------------------------------------------------------------------
// Payload 解析:0x01 / 0x07 / 0x02
// ---------------------------------------------------------------------------
static void test_parse_payloads(void)
{
    static const char *const golden_07 =
        "00 07 14 00 0c 06 4f3330323431303031363230 6b41676b344d";
    static const char *const golden_01 =
        "00 01 1f 00 0c 03 0d 4f3330323431303031363230 563031 "
        "33302e302e352d33302e302e36";
    static const char *const golden_02 = "00 02 03 00 00 00 00";
    static const char *const auth_ok = "00 08 00 00";

    uint8_t buf[64];
    nova_response_t resp;

    // 0x07 接入信息。
    size_t n = hex_to_bytes(golden_07, buf, sizeof(buf));
    resp = parse_exact(buf, n);
    nova_access_info_t access;
    memset(&access, 0, sizeof(access));
    assert(nova_parse_access_info(&resp, &access));
    assert(strcmp(access.sn, "O30241001620") == 0);
    assert(access.sn_len == 12);
    assert(strcmp(access.code, "kAgk4M") == 0);
    assert(access.code_len == 6);
    // opcode 不符 / 结构不足都失败。
    resp.opcode = NOVA_OP_DEVICE_INFO;
    assert(!nova_parse_access_info(&resp, &access));
    resp.opcode = NOVA_OP_ACCESS_INFO;
    resp.payload_len = 2;
    assert(!nova_parse_access_info(&resp, &access));
    resp.payload_len = 19;  // 2+12+6=20,少 1 字节
    assert(!nova_parse_access_info(&resp, &access));

    // 0x01 设备信息。
    n = hex_to_bytes(golden_01, buf, sizeof(buf));
    resp = parse_exact(buf, n);
    nova_device_info_t info;
    memset(&info, 0, sizeof(info));
    assert(nova_parse_device_info(&resp, &info));
    assert(strcmp(info.sn, "O30241001620") == 0);
    assert(strcmp(info.hardware_version, "V01") == 0);
    assert(strcmp(info.firmware_version, "30.0.5-30.0.6") == 0);
    // 长度必须严格等于 3+sn+hw+fw:多 1 字节或长度前缀改动都失败。
    resp.payload_len = (uint16_t)(resp.payload_len - 1);
    assert(!nova_parse_device_info(&resp, &info));
    resp = parse_exact(buf, n);
    resp.payload[1] = 4;  // hw_len 2→4,总长不再匹配
    assert(!nova_parse_device_info(&resp, &info));
    // opcode 不符。
    resp = parse_exact(buf, n);
    resp.opcode = NOVA_OP_STATUS;
    assert(!nova_parse_device_info(&resp, &info));

    // 0x02 状态:真机 plen=3,level=0/code=0。
    n = hex_to_bytes(golden_02, buf, sizeof(buf));
    resp = parse_exact(buf, n);
    nova_state_t state = NOVA_STATE_UNKNOWN;
    uint16_t code = 0xFFFF;
    assert(nova_parse_status(&resp, &state, &code));
    assert(state == NOVA_STATE_UNINITIALIZED && code == 0);

    // 容忍第 4 字节(未来固件可能附电量);level=3, code=0x1234。
    const uint8_t status4[4] = {3, 0x34, 0x12, 0x64};
    n = build_response(0, NOVA_OP_STATUS, status4, sizeof(status4), buf,
                       sizeof(buf));
    resp = parse_exact(buf, n);
    state = NOVA_STATE_UNKNOWN;
    assert(nova_parse_status(&resp, &state, &code));
    assert(state == NOVA_STATE_IDLE && code == 0x1234);
    // payload < 3 失败。
    n = build_response(0, NOVA_OP_STATUS, status4, 2, buf, sizeof(buf));
    resp = parse_exact(buf, n);
    assert(!nova_parse_status(&resp, &state, &code));

    // 0x08 认证成功帧:result=0,空 payload。
    n = hex_to_bytes(auth_ok, buf, sizeof(buf));
    resp = parse_exact(buf, n);
    assert(resp.opcode == NOVA_OP_ACCESS_VERIFY && resp.payload_len == 0);

    // 状态等级映射:只映射有依据的等级,其余 UNKNOWN。
    assert(nova_state_from_level(0) == NOVA_STATE_UNINITIALIZED);
    assert(nova_state_from_level(2) == NOVA_STATE_INITIALIZING);
    assert(nova_state_from_level(3) == NOVA_STATE_IDLE);
    assert(nova_state_from_level(4) == NOVA_STATE_SERVING);
    assert(nova_state_from_level(5) == NOVA_STATE_TRANSITION);
    assert(nova_state_from_level(6) == NOVA_STATE_PAUSED);
    assert(nova_state_from_level(101) == NOVA_STATE_FAULT);
    assert(nova_state_from_level(200) == NOVA_STATE_SELF_REPAIR);
    assert(nova_state_from_level(1) == NOVA_STATE_UNKNOWN);
    assert(nova_state_from_level(10) == NOVA_STATE_UNKNOWN);
    assert(nova_state_from_level(255) == NOVA_STATE_UNKNOWN);
}

// ---------------------------------------------------------------------------
// 0x01/0x02/0x07/0x08 请求-响应往返
// ---------------------------------------------------------------------------
static void test_round_trip(void)
{
    uint8_t request[NOVA_MAX_REQUEST];
    uint8_t frame[NOVA_MAX_RESPONSE];
    nova_response_t out[2];
    size_t buf_len;

    // 0x07:请求 07 00 00,响应为真机黄金帧。
    size_t req_len = nova_build_request(NOVA_OP_ACCESS_INFO, NULL, 0, request,
                                        sizeof(request));
    assert(req_len == 3 && memcmp(request, "\x07\x00\x00", 3) == 0);
    size_t frame_len = hex_to_bytes(
        "00 07 14 00 0c 06 4f3330323431303031363230 6b41676b344d", frame,
        sizeof(frame));
    buf_len = frame_len;
    assert(nova_extract_responses(frame, &buf_len, out, 2) == 1);
    nova_access_info_t access;
    assert(nova_parse_access_info(&out[0], &access));
    char key[33];
    assert(nova_make_private_key((const uint8_t *)access.sn, access.sn_len,
                                 (const uint8_t *)access.code, access.code_len,
                                 key));

    // 0x08:用上一步的私钥组包,设备回 00 08 00 00。
    req_len = nova_build_request(NOVA_OP_ACCESS_VERIFY, key, 32, request,
                                 sizeof(request));
    assert(req_len == 35 && request[0] == 0x08 && request[1] == 0x20);
    assert(memcmp(request + 3, key, 32) == 0);
    frame_len = hex_to_bytes("00 08 00 00", frame, sizeof(frame));
    buf_len = frame_len;
    assert(nova_extract_responses(frame, &buf_len, out, 2) == 1);
    assert(out[0].result == 0 && out[0].opcode == NOVA_OP_ACCESS_VERIFY);

    // 0x01:请求 01 00 00,响应 golden_01。
    req_len = nova_build_request(NOVA_OP_DEVICE_INFO, NULL, 0, request,
                                 sizeof(request));
    assert(req_len == 3 && memcmp(request, "\x01\x00\x00", 3) == 0);
    frame_len = hex_to_bytes(
        "00 01 1f 00 0c 03 0d 4f3330323431303031363230 563031 "
        "33302e302e352d33302e302e36",
        frame, sizeof(frame));
    buf_len = frame_len;
    assert(nova_extract_responses(frame, &buf_len, out, 2) == 1);
    nova_device_info_t info;
    assert(nova_parse_device_info(&out[0], &info));
    assert(strcmp(info.firmware_version, "30.0.5-30.0.6") == 0);

    // 0x02:请求 02 00 00,响应 golden_02。
    req_len = nova_build_request(NOVA_OP_STATUS, NULL, 0, request,
                                 sizeof(request));
    assert(req_len == 3 && memcmp(request, "\x02\x00\x00", 3) == 0);
    frame_len = hex_to_bytes("00 02 03 00 00 00 00", frame, sizeof(frame));
    buf_len = frame_len;
    assert(nova_extract_responses(frame, &buf_len, out, 2) == 1);
    nova_state_t state;
    uint16_t code;
    assert(nova_parse_status(&out[0], &state, &code));
    assert(state == NOVA_STATE_UNINITIALIZED && code == 0);
}

// ---------------------------------------------------------------------------
// 接入认证私钥
// ---------------------------------------------------------------------------
static void test_private_key(void)
{
    char key[33];

    // CheckCodeAuthenticatorTest.kt::testPythonGoldenVectors(逐字节一致)。
    assert(nova_make_private_key((const uint8_t *)"P16000000121", 12,
                                 (const uint8_t *)"TFwXJY", 6, key));
    assert(strcmp(key, "4adf57c6005bf74e9d921ebab767b2b2") == 0);

    assert(nova_make_private_key((const uint8_t *)"O30241001620", 12,
                                 (const uint8_t *)"A1b2C3", 6, key));
    assert(strcmp(key, "c83bc73a73387a648f2cbf327c1ade1f") == 0);

    assert(nova_make_private_key((const uint8_t *)"X31250300001", 12,
                                 (const uint8_t *)"Zq9wLm", 6, key));
    assert(strcmp(key, "d059fdaa1f0a1aa1f89ec2059800e782") == 0);

    assert(nova_make_private_key((const uint8_t *)"SN123", 5,
                                 (const uint8_t *)"abcd", 4, key));
    assert(strcmp(key, "8cdcb3d9d9497a5a5b384f99f0a9f844") == 0);

    assert(nova_make_private_key((const uint8_t *)"SN123", 5,
                                 (const uint8_t *)"abcdefghij", 10, key));
    assert(strcmp(key, "45c556d49c7dccff4ed8f1e4789a0c1e") == 0);

    assert(nova_make_private_key(NULL, 0, (const uint8_t *)"abcd", 4, key));
    assert(strcmp(key, "e2fc714c4727ee9395f324cd2e7f331f") == 0);

    // 真机黄金向量:SN=O30241001620 + dynamic=kAgk4M
    // (/tmp/nova_protocol_spec.md §7 V2,与 hardware-read-report.md 一致)。
    assert(nova_make_private_key((const uint8_t *)"O30241001620", 12,
                                 (const uint8_t *)"kAgk4M", 6, key));
    assert(strcmp(key, "2dfe0b272db98d6baf06f428d9d6c59d") == 0);

    // 越界/非法参数。
    static const uint8_t long_sn[121] = {0};
    assert(!nova_make_private_key(long_sn, 121, (const uint8_t *)"abcd", 4, key));
    assert(!nova_make_private_key((const uint8_t *)"SN123", 5,
                                  (const uint8_t *)"abc", 3, key));
    assert(!nova_make_private_key((const uint8_t *)"SN123", 5,
                                  (const uint8_t *)"abcdefghijk", 11, key));
    assert(!nova_make_private_key((const uint8_t *)"SN123", 5, NULL, 4, key));
    assert(!nova_make_private_key((const uint8_t *)"SN123", 5,
                                  (const uint8_t *)"abcd", 4, NULL));
    assert(!nova_make_private_key(NULL, 3, (const uint8_t *)"abcd", 4, key));

    // SN 含高位字节(0xFF)时必须按无符号取模:0xFF % 36 = 3 → SALT[3]='g'。
    // 参考实现的有符号 char 会在这里产生分歧,本用例锁定正确行为。
    {
        static const char salt[] = DEVICE_AUTH_SALT;
        char material[8];
        const uint8_t high_sn[1] = {0xFF};
        material[0] = (char)0xFF;
        material[1] = salt[0xFF % 36];
        memcpy(material + 2, "abcd", 4);
        char expected[33];
        md5_hex_lower(material, 6, expected);
        assert(nova_make_private_key(high_sn, 1, (const uint8_t *)"abcd", 4, key));
        assert(strcmp(key, expected) == 0);
    }

    // 私钥必须是 32 位小写 hex + NUL。
    assert(strlen(key) == 32);
    for (size_t i = 0; i < 32; i++) {
        const char c = key[i];
        assert((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'));
    }
}

// ---------------------------------------------------------------------------
// 型号分类
// ---------------------------------------------------------------------------
static void test_device_classification(void)
{
    // Android DeviceCatalog.kt / DeviceProfiles.kt 的顺序与大小写规则。
    assert(device_model_from_name("SEEKER2211180131") == DEVICE_MODEL_SEEKER);
    assert(device_model_from_name("seeker123") == DEVICE_MODEL_SEEKER);
    assert(device_model_from_name("  FUTUREMIND SEEKER 01  ") == DEVICE_MODEL_SEEKER);
    assert(device_model_from_name("futuremind seeker") == DEVICE_MODEL_SEEKER);
    assert(device_model_from_name("NOVA_O30241001620") == DEVICE_MODEL_NOVA);
    assert(device_model_from_name("nova_abc") == DEVICE_MODEL_NOVA);
    assert(device_model_from_name("OMNI_01") == DEVICE_MODEL_OMNI);
    assert(device_model_from_name("OMNI-01") == DEVICE_MODEL_OMNI);
    assert(device_model_from_name("HALO_01") == DEVICE_MODEL_HALO);
    assert(device_model_from_name("HALO-01") == DEVICE_MODEL_HALO);
    assert(device_model_from_name("AURA_01") == DEVICE_MODEL_AURA);
    assert(device_model_from_name("PACE_01") == DEVICE_MODEL_PACE);
    assert(device_model_from_name(" ESP32 ") == DEVICE_MODEL_UNKNOWN);
    assert(device_model_from_name("NOVAX") == DEVICE_MODEL_UNKNOWN);
    assert(device_model_from_name("") == DEVICE_MODEL_UNKNOWN);
    assert(device_model_from_name(NULL) == DEVICE_MODEL_UNKNOWN);

    assert(strcmp(device_model_name(DEVICE_MODEL_NOVA), "NOVA") == 0);
    assert(strcmp(device_model_name(DEVICE_MODEL_SEEKER), "SEEKER") == 0);
    assert(strcmp(device_model_name(DEVICE_MODEL_UNKNOWN), "UNKNOWN") == 0);

    // 六款型号都支持只读信息读取;UNKNOWN 不支持。
    assert(device_model_readable(DEVICE_MODEL_NOVA));
    assert(device_model_readable(DEVICE_MODEL_SEEKER));
    assert(device_model_readable(DEVICE_MODEL_OMNI));
    assert(device_model_readable(DEVICE_MODEL_HALO));
    assert(device_model_readable(DEVICE_MODEL_AURA));
    assert(device_model_readable(DEVICE_MODEL_PACE));
    assert(!device_model_readable(DEVICE_MODEL_UNKNOWN));

    // device_proto 的私钥算法与 nova_make_private_key 必须完全一致。
    char key_nova[33];
    char key_device[33];
    assert(nova_make_private_key((const uint8_t *)"SN123", 5,
                                 (const uint8_t *)"abcd", 4, key_nova));
    assert(device_auth_make_private_key((const uint8_t *)"SN123", 5,
                                        (const uint8_t *)"abcd", 4,
                                        key_device));
    assert(strcmp(key_nova, key_device) == 0);
}

int main(void)
{
    test_build_request_goldens();
    test_parse_response();
    test_extract_responses();
    test_parse_payloads();
    test_round_trip();
    test_private_key();
    test_device_classification();
    puts("NOVA proto tests: PASS");
    return 0;
}
