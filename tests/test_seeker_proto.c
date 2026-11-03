// tests/test_seeker_proto.c —— SEEKER(鹰眼)分包重组 / JSON 解析 / 请求组包的主机单测。
//
// 契约来源:main/seeker_proto.h、main/md5.h。
// 只依赖 C11 标准库与 assert(),不链接 ESP-IDF/LVGL/动态内存;测试里允许 printf。
//
// 黄金向量出处:
// - 真机抓包 single-packet status(97 字节,含 22 字节头):
//   ble_control/app/src/test/java/.../SeekerPacketAssemblerTest.kt 的 golden 常量,
//   以及 /tmp/seeker_protocol_spec.md §7(头部 MD5 中段 018E205E14131822 已验证)。
// - device_info 响应字段:fragment 见 SeekerProtocolTest.kt:28。
// - 多包构造规则(16 位 MD5 中段 + %03d/%03X 包号 + 正文):
//   SeekerPacketAssemblerTest.kt::packets()。
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "md5.h"
#include "seeker_proto.h"

// 真机单包状态响应(2026-10-28 抓包)。
static const char SEEKER_GOLDEN_STATUS[] =
    "018E205E14131822001001"
    "{\"code\":1000,\"data\":{\"hawkeye\":\"uninit\"},\"method\":\"status\","
    "\"msg\":\"SUCCESS\"}";

// Android SeekerProtocolTest.kt:28 的 device_info fixture。
static const char SEEKER_INFO_FIXTURE[] =
    "{\"method\":\"device_info\",\"code\":1000,\"data\":"
    "{\"firmware_version\":\"1.0.1\",\"system_version\":\"FG05\","
    "\"sn\":\"S11022109170019\",\"token\":\"session-token\"}}";

// ---------------------------------------------------------------------------
// 测试辅助:按真机规则把整段正文切成若干 notify 包。
// ---------------------------------------------------------------------------
static size_t make_packets(const char *body, size_t body_len, size_t chunk,
                           bool hex_mode, uint8_t *storage, size_t storage_cap,
                           size_t offsets[SEEKER_MAX_PACKETS],
                           size_t lengths[SEEKER_MAX_PACKETS], size_t *count)
{
    char hash[33];

    assert(chunk > 0 && body_len > 0);
    md5_hex_upper(body, body_len, hash);  // 头里只放 MD5 大写 hex 的第 8..23 个字符
    const size_t total = (body_len + chunk - 1) / chunk;
    assert(total >= 1 && total <= SEEKER_MAX_PACKETS);

    size_t pos = 0;
    for (size_t i = 0; i < total; i++) {
        const size_t body_off = i * chunk;
        size_t slice = body_len - body_off;
        if (slice > chunk) {
            slice = chunk;
        }
        assert(pos + SEEKER_PACKET_HEADER_LEN + slice <= storage_cap);

        char num[8];
        memcpy(storage + pos, hash + 8, 16);
        if (hex_mode) {
            snprintf(num, sizeof(num), "%03zX", i + 1);
        } else {
            snprintf(num, sizeof(num), "%03zu", i + 1);
        }
        memcpy(storage + pos + 16, num, 3);
        if (hex_mode) {
            snprintf(num, sizeof(num), "%03zX", total);
        } else {
            snprintf(num, sizeof(num), "%03zu", total);
        }
        memcpy(storage + pos + 19, num, 3);
        memcpy(storage + pos + SEEKER_PACKET_HEADER_LEN, body + body_off, slice);

        offsets[i] = pos;
        lengths[i] = SEEKER_PACKET_HEADER_LEN + slice;
        pos += lengths[i];
    }
    *count = total;
    return pos;
}

// ---------------------------------------------------------------------------
// 真机黄金帧
// ---------------------------------------------------------------------------
static void test_real_device_golden(void)
{
    seeker_assembler_t assembler;
    uint8_t out[256];
    size_t out_len = 0;

    assert(strlen(SEEKER_GOLDEN_STATUS) == 97);
    seeker_assembler_reset(&assembler);
    assert(seeker_assembler_feed(&assembler,
                                 (const uint8_t *)SEEKER_GOLDEN_STATUS,
                                 strlen(SEEKER_GOLDEN_STATUS), out, sizeof(out),
                                 &out_len) == SEEKER_FEED_COMPLETE);
    // 正文必须逐字节等于抓包正文(去掉 22 字节头)。
    assert(out_len == strlen(SEEKER_GOLDEN_STATUS) - SEEKER_PACKET_HEADER_LEN);
    assert(out_len == 75);
    assert(memcmp(out, SEEKER_GOLDEN_STATUS + SEEKER_PACKET_HEADER_LEN, out_len) == 0);

    seeker_response_t response;
    assert(seeker_parse_response((const char *)out, out_len, &response));
    assert(response.code == 1000);
    assert(strcmp(response.method, "status") == 0);
    assert(strcmp(response.msg, "SUCCESS") == 0);

    char hawkeye[24];
    assert(seeker_parse_status((const char *)out, out_len, hawkeye));
    assert(strcmp(hawkeye, "uninit") == 0);

    // 独立复核头部 MD5 中段:MD5(正文) 大写 hex 的第 8..23 个字符。
    char digest[33];
    md5_hex_upper(SEEKER_GOLDEN_STATUS + SEEKER_PACKET_HEADER_LEN, 75, digest);
    assert(strcmp(digest, "93B85796018E205E141318228814189B") == 0);
    assert(memcmp(digest + 8, SEEKER_GOLDEN_STATUS, 16) == 0);
}

// ---------------------------------------------------------------------------
// 9 包级消息重组:十进制 / 十六进制计数器、乱序、重复包
// ---------------------------------------------------------------------------
static void test_multi_packet_reassembly(void)
{
    static uint8_t storage[SEEKER_MAX_BODY + 2 * SEEKER_MAX_PACKETS];
    static uint8_t out[SEEKER_MAX_BODY];
    static size_t offsets[SEEKER_MAX_PACKETS];
    static size_t lengths[SEEKER_MAX_PACKETS];
    char body[1024];

    // 与真机 9 包版本响应同量级的 body:填充 900 字符,chunk=8 → 113 包超出
    // SEEKER_MAX_PACKETS,故使用 8 包以上但不超过 64 包的长度(约 360 字节)。
    const char *filler =
        "0123456789abcdef0123456789abcdef0123456789abcdef0123456789ab"
        "cdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789"
        "abcdef0123456789abcdef0123456789abcdef0123456789abcdef01234567"
        "89abcdef0123456789abcdef0123456789abcdef0123456789abcdef012345"
        "6789abcdef0123456789abcdef";
    int written = snprintf(body, sizeof(body),
                           "{\"method\":\"device_info\",\"code\":1000,\"data\":"
                           "{\"firmware_version\":\"1.2.2\",\"system_version\":"
                           "\"01.44.01\",\"sn\":\"S13012211180131\","
                           "\"token\":\"session-token\",\"filler\":\"%s\"}}",
                           filler);
    assert(written > 0);
    const size_t body_len = (size_t)written;

    // 十进制模式:chunk=12 → 约 33 包,计数器无 A-F。
    size_t count = 0;
    size_t total_bytes = make_packets(body, body_len, 12, false, storage,
                                      sizeof(storage), offsets, lengths, &count);
    assert(count > 9 && count <= SEEKER_MAX_PACKETS);
    seeker_assembler_t assembler;
    seeker_assembler_reset(&assembler);
    size_t out_len = 0;
    // 乱序 + 重复:先喂后半部分(含重复),再喂前半部分。
    for (size_t i = count; i > count / 2; i--) {
        assert(seeker_assembler_feed(&assembler, storage + offsets[i - 1],
                                     lengths[i - 1], out, sizeof(out),
                                     &out_len) == SEEKER_FEED_NONE);
        assert(seeker_assembler_feed(&assembler, storage + offsets[i - 1],
                                     lengths[i - 1], out, sizeof(out),
                                     &out_len) == SEEKER_FEED_NONE);
    }
    assert(seeker_assembler_incomplete(&assembler));
    seeker_feed_result_t result = SEEKER_FEED_NONE;
    for (size_t i = count / 2; i > 0; i--) {
        result = seeker_assembler_feed(&assembler, storage + offsets[i - 1],
                                       lengths[i - 1], out, sizeof(out),
                                       &out_len);
    }
    assert(result == SEEKER_FEED_COMPLETE);
    assert(out_len == body_len);
    assert(memcmp(out, body, body_len) == 0);
    assert(!seeker_assembler_incomplete(&assembler));

    // 重组后的 JSON 字段。
    seeker_device_info_t info;
    assert(seeker_parse_device_info((const char *)out, out_len, &info));
    assert(strcmp(info.sn, "S13012211180131") == 0);
    assert(strcmp(info.firmware_version, "1.2.2") == 0);
    assert(strcmp(info.system_version, "01.44.01") == 0);
    assert(strcmp(info.token, "session-token") == 0);
    assert(info.has_token);

    // 十六进制模式:同一正文,计数器 %03X,首包总包数含 A-F 即锁定 hex。
    total_bytes = make_packets(body, body_len, 12, true, storage, sizeof(storage),
                               offsets, lengths, &count);
    assert(total_bytes > 0 && count > 9);
    seeker_assembler_reset(&assembler);
    result = SEEKER_FEED_NONE;
    for (size_t i = 0; i < count; i++) {
        result = seeker_assembler_feed(&assembler, storage + offsets[i],
                                       lengths[i], out, sizeof(out), &out_len);
        if (i + 1 < count) {
            assert(result == SEEKER_FEED_NONE);
        }
    }
    assert(result == SEEKER_FEED_COMPLETE && out_len == body_len);
    assert(memcmp(out, body, body_len) == 0);

    // 首包总包数文本含 A-F 的情况:27 包 → total "01B",第一包就锁定 hex
    // (对应 Android SeekerPacketAssemblerTest.kt 的 210 字节 / 8 字节切片用例)。
    char body210[256];
    char fill[202];
    memset(fill, 'a', sizeof(fill) - 1);
    fill[sizeof(fill) - 1] = '\0';
    const int n210 = snprintf(body210, sizeof(body210), "{\"msg\":\"%s\"}",
                              fill);
    assert(n210 > 0 && (size_t)n210 == 211);
    total_bytes = make_packets(body210, (size_t)n210, 8, true, storage,
                               sizeof(storage), offsets, lengths, &count);
    assert(total_bytes > 0 && count == 27);
    seeker_assembler_reset(&assembler);
    result = SEEKER_FEED_NONE;
    for (size_t i = 0; i < count; i++) {
        result = seeker_assembler_feed(&assembler, storage + offsets[i],
                                       lengths[i], out, sizeof(out), &out_len);
        if (i + 1 < count) {
            assert(result == SEEKER_FEED_NONE);
        }
    }
    assert(result == SEEKER_FEED_COMPLETE && out_len == (size_t)n210);
    assert(memcmp(out, body210, (size_t)n210) == 0);
}

// ---------------------------------------------------------------------------
// 与真机 9 包版本响应同构的消息(100 字节正文 / 12 字节切片 → 9 包)
// ---------------------------------------------------------------------------
static void test_nine_packet_version_message(void)
{
    static uint8_t storage[SEEKER_MAX_BODY + 2 * SEEKER_MAX_PACKETS];
    static uint8_t out[512];
    static size_t offsets[SEEKER_MAX_PACKETS];
    static size_t lengths[SEEKER_MAX_PACKETS];
    const char body[] =
        "{\"method\":\"device_info\",\"code\":1000,\"data\":"
        "{\"firmware_version\":\"1.2.2\",\"system_version\":\"01.44.01\"}}";
    const size_t body_len = strlen(body);
    assert(body_len == 100);

    // 十进制与十六进制两种计数器格式(≤9 时两种格式字符相同,均需正确解析)。
    for (int hex_mode = 0; hex_mode < 2; hex_mode++) {
        size_t count = 0;
        const size_t total_bytes =
            make_packets(body, body_len, 12, hex_mode != 0, storage,
                         sizeof(storage), offsets, lengths, &count);
        assert(total_bytes > 0 && count == 9);

        seeker_assembler_t assembler;
        seeker_assembler_reset(&assembler);
        size_t out_len = 0;
        seeker_feed_result_t result = SEEKER_FEED_NONE;
        // 故意乱序:从第 9 包倒着喂。
        for (size_t i = count; i > 0; i--) {
            result = seeker_assembler_feed(&assembler, storage + offsets[i - 1],
                                           lengths[i - 1], out, sizeof(out),
                                           &out_len);
            if (i > 1) {
                assert(result == SEEKER_FEED_NONE);
            }
        }
        assert(result == SEEKER_FEED_COMPLETE);
        assert(out_len == body_len && memcmp(out, body, body_len) == 0);

        seeker_device_info_t info;
        assert(seeker_parse_device_info((const char *)out, out_len, &info));
        assert(strcmp(info.firmware_version, "1.2.2") == 0);
        assert(strcmp(info.system_version, "01.44.01") == 0);
    }
}

// ---------------------------------------------------------------------------
// 回归:小写十六进制计数器(真机图片总包数 "1c2" = 450 风格)
//
// SeekerPacketAssembler.kt:42-43 先把 3 字符计数器转大写再检查 A-F,所以小写
// a-f 同样要锁定十六进制。修复前只检查 'A'-'F',会把 "00a" 当十进制解析失败。
// ---------------------------------------------------------------------------
static void test_lowercase_hex_counters(void)
{
    static uint8_t storage[SEEKER_MAX_BODY + 2 * SEEKER_MAX_PACKETS];
    static uint8_t out[SEEKER_MAX_BODY];
    static size_t offsets[SEEKER_MAX_PACKETS];
    static size_t lengths[SEEKER_MAX_PACKETS];
    char body[400];

    // 场景 1:10 包,总包数大写 "00A" → 改成小写 "00a";index 10 也是 "00a"。
    static const char version[] =
        "{\"method\":\"device_info\",\"code\":1000,\"data\":"
        "{\"firmware_version\":\"1.2.2\",\"system_version\":\"01.44.01\"}}";
    assert(strlen(version) == 100);

    size_t count = 0;
    size_t total_bytes = make_packets(version, strlen(version), 10, true, storage,
                                      sizeof(storage), offsets, lengths, &count);
    assert(total_bytes > 0 && count == 10);
    // 把每个包的 3 字符包号 + 3 字符总包数改成小写,模拟真机小写 hex 输出。
    bool saw_lower = false;
    for (size_t i = 0; i < count; i++) {
        for (size_t j = 16; j < SEEKER_PACKET_HEADER_LEN; j++) {
            uint8_t *c = &storage[offsets[i] + j];
            if (*c >= 'A' && *c <= 'F') {
                *c = (uint8_t)(*c - 'A' + 'a');
                saw_lower = true;
            }
        }
    }
    assert(saw_lower);

    // 乱序喂入:最后一包(index "00a")先到,首包即触发小写 hex 锁定 + 总包数 10。
    seeker_assembler_t assembler;
    seeker_assembler_reset(&assembler);
    size_t out_len = 0;
    seeker_feed_result_t result = SEEKER_FEED_NONE;
    for (size_t i = count; i > 0; i--) {
        result = seeker_assembler_feed(&assembler, storage + offsets[i - 1],
                                       lengths[i - 1], out, sizeof(out),
                                       &out_len);
        if (i > 1) {
            assert(result == SEEKER_FEED_NONE);
        }
    }
    assert(result == SEEKER_FEED_COMPLETE);
    assert(out_len == strlen(version));
    assert(memcmp(out, version, out_len) == 0);

    // 场景 2:28 包,总包数大写 "01C" → 小写 "01c";首包就锁定 hex。
    char fill[263];
    memset(fill, 'a', sizeof(fill) - 1);
    fill[sizeof(fill) - 1] = '\0';
    const int n = snprintf(body, sizeof(body),
                           "{\"method\":\"device_info\",\"code\":1000,"
                           "\"data\":{\"sn\":\"S1\",\"filler\":\"%s\"}}",
                           fill);
    assert(n > 0 && (size_t)n == 329);
    total_bytes = make_packets(body, (size_t)n, 12, true, storage,
                               sizeof(storage), offsets, lengths, &count);
    assert(total_bytes > 0 && count == 28);
    saw_lower = false;
    for (size_t i = 0; i < count; i++) {
        for (size_t j = 16; j < SEEKER_PACKET_HEADER_LEN; j++) {
            uint8_t *c = &storage[offsets[i] + j];
            if (*c >= 'A' && *c <= 'F') {
                *c = (uint8_t)(*c - 'A' + 'a');
                saw_lower = true;
            }
        }
    }
    assert(saw_lower);
    seeker_assembler_reset(&assembler);
    result = SEEKER_FEED_NONE;
    for (size_t i = 0; i < count; i++) {
        result = seeker_assembler_feed(&assembler, storage + offsets[i],
                                       lengths[i], out, sizeof(out), &out_len);
        if (i + 1 < count) {
            assert(result == SEEKER_FEED_NONE);
        }
    }
    assert(result == SEEKER_FEED_COMPLETE && out_len == (size_t)n);
    assert(memcmp(out, body, (size_t)n) == 0);
}

// ---------------------------------------------------------------------------
// 缺包 / 冲突包 / MD5 校验 / 边界错误
// ---------------------------------------------------------------------------
static void test_error_paths(void)
{
    static uint8_t storage[SEEKER_MAX_BODY + 2 * SEEKER_MAX_PACKETS];
    static uint8_t out[SEEKER_MAX_BODY];
    static size_t offsets[SEEKER_MAX_PACKETS];
    static size_t lengths[SEEKER_MAX_PACKETS];
    char body[] = "{\"method\":\"status\",\"msg\":\"鹰眼已就绪\"}";
    size_t count = 0;
    const size_t body_len = strlen(body);

    make_packets(body, body_len, 8, false, storage, sizeof(storage), offsets,
                 lengths, &count);
    assert(count >= 3);

    // 缺包:只喂最后一包,missing 应为 1..count-1;再补齐其余包。
    seeker_assembler_t assembler;
    seeker_assembler_reset(&assembler);
    size_t out_len = 0;
    assert(seeker_assembler_feed(&assembler, storage + offsets[count - 1],
                                 lengths[count - 1], out, sizeof(out),
                                 &out_len) == SEEKER_FEED_NONE);
    assert(seeker_assembler_incomplete(&assembler));
    uint16_t missing[SEEKER_MAX_PACKETS];
    assert(seeker_assembler_missing(&assembler, missing, SEEKER_MAX_PACKETS) ==
           count - 1);
    for (size_t i = 0; i < count - 1; i++) {
        assert(missing[i] == (uint16_t)(i + 1));
    }
    // 缓冲区不足时 missing() 只写 cap 个但仍返回总数。
    memset(missing, 0, sizeof(missing));
    assert(seeker_assembler_missing(&assembler, missing, 1) == count - 1);
    assert(missing[0] == 1 && missing[1] == 0);
    seeker_feed_result_t result = SEEKER_FEED_NONE;
    for (size_t i = 0; i < count - 1; i++) {
        result = seeker_assembler_feed(&assembler, storage + offsets[i],
                                       lengths[i], out, sizeof(out), &out_len);
    }
    assert(result == SEEKER_FEED_COMPLETE && out_len == body_len);
    assert(memcmp(out, body, body_len) == 0);
    // 完成后 missing 为空。
    assert(seeker_assembler_missing(&assembler, missing, SEEKER_MAX_PACKETS) == 0);

    // MD5 校验失败:篡改已重组正文的一个字节 → ERROR。
    uint8_t corrupted[256];
    make_packets(body, body_len, 8, false, corrupted, sizeof(corrupted), offsets,
                 lengths, &count);
    corrupted[offsets[0] + lengths[0] - 1] ^= 0x01;
    seeker_assembler_reset(&assembler);
    result = SEEKER_FEED_NONE;
    for (size_t i = 0; i < count; i++) {
        result = seeker_assembler_feed(&assembler, corrupted + offsets[i],
                                       lengths[i], out, sizeof(out), &out_len);
    }
    assert(result == SEEKER_FEED_ERROR);

    // 重复包正文冲突 → ERROR。
    make_packets(body, body_len, 8, false, storage, sizeof(storage), offsets,
                 lengths, &count);
    seeker_assembler_reset(&assembler);
    assert(seeker_assembler_feed(&assembler, storage + offsets[0], lengths[0],
                                 out, sizeof(out), &out_len) == SEEKER_FEED_NONE);
    uint8_t conflict[128];
    memcpy(conflict, storage + offsets[0], lengths[0]);
    conflict[lengths[0] - 1] = '!';
    assert(seeker_assembler_feed(&assembler, conflict, lengths[0], out,
                                 sizeof(out), &out_len) == SEEKER_FEED_ERROR);

    // 总包数在批次中途改变 → ERROR(头哈希相同,只改总包数)。
    char hash[33];
    md5_hex_upper(body, body_len, hash);
    uint8_t changed_total[128];
    size_t n = 0;
    memcpy(changed_total + n, hash + 8, 16);
    n += 16;
    memcpy(changed_total + n, "001", 3);
    n += 3;
    memcpy(changed_total + n, "004", 3);
    n += 3;
    memcpy(changed_total + n, body, body_len);
    n += body_len;
    seeker_assembler_reset(&assembler);
    assert(seeker_assembler_feed(&assembler, storage + offsets[0], lengths[0],
                                 out, sizeof(out), &out_len) == SEEKER_FEED_NONE);
    assert(seeker_assembler_feed(&assembler, changed_total, n, out, sizeof(out),
                                 &out_len) == SEEKER_FEED_ERROR);

    // 头无法识别(非 22 字节 / 含非 hex 字符)→ ERROR。
    seeker_assembler_reset(&assembler);
    const char *bad_header = "md5+1+2+payload";
    assert(seeker_assembler_feed(&assembler, (const uint8_t *)bad_header,
                                 strlen(bad_header), out, sizeof(out),
                                 &out_len) == SEEKER_FEED_ERROR);
    uint8_t non_hex[64];
    memset(non_hex, 'G', SEEKER_PACKET_HEADER_LEN);
    memcpy(non_hex, "AAAAAAAAAAAAAAAA", 16);
    memcpy(non_hex + 16, "001001", 6);
    non_hex[3] = 'G';  // 头部混入非 hex 字符,必须整体拒绝
    non_hex[SEEKER_PACKET_HEADER_LEN] = 'x';
    assert(seeker_assembler_feed(&assembler, non_hex,
                                 SEEKER_PACKET_HEADER_LEN + 1, out, sizeof(out),
                                 &out_len) == SEEKER_FEED_ERROR);

    // 包号/总包数越界:index=0、total=0、total 超过静态上限 64。
    uint8_t header_pkt[64];
    memcpy(header_pkt, hash + 8, 16);
    memcpy(header_pkt + 16, "000", 3);
    memcpy(header_pkt + 19, "003", 3);
    memcpy(header_pkt + 22, "ab", 2);
    seeker_assembler_reset(&assembler);
    assert(seeker_assembler_feed(&assembler, header_pkt, 24, out, sizeof(out),
                                 &out_len) == SEEKER_FEED_ERROR);
    memcpy(header_pkt + 16, "001", 3);
    memcpy(header_pkt + 19, "000", 3);
    assert(seeker_assembler_feed(&assembler, header_pkt, 24, out, sizeof(out),
                                 &out_len) == SEEKER_FEED_ERROR);
    memcpy(header_pkt + 19, "065", 3);  // 65 包 > SEEKER_MAX_PACKETS
    assert(seeker_assembler_feed(&assembler, header_pkt, 24, out, sizeof(out),
                                 &out_len) == SEEKER_FEED_ERROR);

    // 正文超过 SEEKER_MAX_BODY → ERROR。
    static uint8_t huge_packet[SEEKER_MAX_BODY + SEEKER_PACKET_HEADER_LEN + 8];
    static uint8_t huge_out[SEEKER_MAX_BODY + 16];
    seeker_assembler_reset(&assembler);
    memset(huge_packet, 'a', sizeof(huge_packet));
    md5_hex_upper("x", 1, hash);
    memcpy(huge_packet, hash + 8, 16);
    memcpy(huge_packet + 16, "001", 3);
    memcpy(huge_packet + 19, "001", 3);
    seeker_assembler_reset(&assembler);
    assert(seeker_assembler_feed(&assembler, huge_packet, sizeof(huge_packet),
                                 huge_out, sizeof(huge_out),
                                 &out_len) == SEEKER_FEED_ERROR);

    // 完成时输出缓冲区不足 → ERROR。
    char small_body[] = "{\"a\":\"0123456789\"}";
    make_packets(small_body, strlen(small_body), 32, false, storage,
                 sizeof(storage), offsets, lengths, &count);
    assert(count == 1);
    seeker_assembler_reset(&assembler);
    assert(seeker_assembler_feed(&assembler, storage + offsets[0], lengths[0],
                                 out, 4, &out_len) == SEEKER_FEED_ERROR);

    // 空包 / NULL 参数。
    seeker_assembler_reset(&assembler);
    assert(seeker_assembler_feed(&assembler, (const uint8_t *)"x", 0, out,
                                 sizeof(out), &out_len) == SEEKER_FEED_NONE);
    assert(seeker_assembler_feed(NULL, storage, 24, out, sizeof(out), &out_len) ==
           SEEKER_FEED_ERROR);
    assert(seeker_assembler_feed(&assembler, NULL, 24, out, sizeof(out),
                                 &out_len) == SEEKER_FEED_ERROR);
    assert(seeker_assembler_feed(&assembler, storage, 24, out, sizeof(out),
                                 NULL) == SEEKER_FEED_ERROR);
    assert(!seeker_assembler_incomplete(NULL));
    assert(seeker_assembler_missing(NULL, missing, 4) == 0);
}

// ---------------------------------------------------------------------------
// 已完成消息去重窗口 / begin_request / reset
// ---------------------------------------------------------------------------
static void test_completed_window(void)
{
    seeker_assembler_t assembler;
    uint8_t out[256];
    size_t out_len = 0;

    seeker_assembler_reset(&assembler);
    assert(seeker_assembler_feed(&assembler,
                                 (const uint8_t *)SEEKER_GOLDEN_STATUS,
                                 strlen(SEEKER_GOLDEN_STATUS), out, sizeof(out),
                                 &out_len) == SEEKER_FEED_COMPLETE);
    // 同一批次重复到达:在去重窗口内直接忽略。
    assert(seeker_assembler_feed(&assembler,
                                 (const uint8_t *)SEEKER_GOLDEN_STATUS,
                                 strlen(SEEKER_GOLDEN_STATUS), out, sizeof(out),
                                 &out_len) == SEEKER_FEED_NONE);
    // begin_request 清空去重窗口(新请求可能收到相同内容)。
    seeker_assembler_begin_request(&assembler);
    assert(seeker_assembler_feed(&assembler,
                                 (const uint8_t *)SEEKER_GOLDEN_STATUS,
                                 strlen(SEEKER_GOLDEN_STATUS), out, sizeof(out),
                                 &out_len) == SEEKER_FEED_COMPLETE);
    // reset 清空模式/批次/去重。
    seeker_assembler_reset(&assembler);
    assert(!seeker_assembler_incomplete(&assembler));
    assert(seeker_assembler_feed(&assembler,
                                 (const uint8_t *)SEEKER_GOLDEN_STATUS,
                                 strlen(SEEKER_GOLDEN_STATUS), out, sizeof(out),
                                 &out_len) == SEEKER_FEED_COMPLETE);
    seeker_assembler_reset(NULL);
    seeker_assembler_begin_request(NULL);
}

// ---------------------------------------------------------------------------
// 原始 JSON 透传模式(模拟器/兼容固件)
// ---------------------------------------------------------------------------
static void test_raw_json_passthrough(void)
{
    const char *body = SEEKER_GOLDEN_STATUS + SEEKER_PACKET_HEADER_LEN;
    const size_t body_len = strlen(body);
    seeker_assembler_t assembler;
    uint8_t out[128];
    size_t out_len = 0;

    // 先收到纯空白 notify:不锁定模式,也不透传。
    seeker_assembler_reset(&assembler);
    assert(seeker_assembler_feed(&assembler, (const uint8_t *)"\r\n\t", 3, out,
                                 sizeof(out), &out_len) == SEEKER_FEED_NONE);

    // 之后按 7 字节切片:首包以 '{' 锁定原始 JSON 模式,后续整包透传。
    size_t total = 0;
    for (size_t off = 0; off < body_len; off += 7) {
        size_t slice = body_len - off;
        if (slice > 7) {
            slice = 7;
        }
        out_len = 0;
        const seeker_feed_result_t result = seeker_assembler_feed(
            &assembler, (const uint8_t *)body + off, slice, out, sizeof(out),
            &out_len);
        assert(result == SEEKER_FEED_COMPLETE);
        assert(out_len == slice);
        assert(memcmp(out, body + off, slice) == 0);
        total += slice;
    }
    assert(total == body_len);

    // 输出容量不足 → ERROR。
    seeker_assembler_reset(&assembler);
    assert(seeker_assembler_feed(&assembler, (const uint8_t *)body, 60, out, 4,
                                 &out_len) == SEEKER_FEED_ERROR);
}

// ---------------------------------------------------------------------------
// JSON 解析:envelope / device_info / status / battery
// ---------------------------------------------------------------------------
static void test_json_parse(void)
{
    seeker_response_t response;

    // 顶层 code/method/msg。
    assert(seeker_parse_response(SEEKER_INFO_FIXTURE,
                                 strlen(SEEKER_INFO_FIXTURE), &response));
    assert(response.code == 1000);
    assert(strcmp(response.method, "device_info") == 0);
    assert(response.msg[0] == '\0');

    // code 缺失 → -1(spec:absent code defaults to -1)。
    const char *no_code = "{\"method\":\"status\",\"msg\":\"ok\"}";
    assert(seeker_parse_response(no_code, strlen(no_code), &response));
    assert(response.code == -1);
    assert(strcmp(response.method, "status") == 0);

    // 转义引号/反斜杠(JSON 值: a "b" \ c)。
    const char *escaped =
        "{\"method\":\"status\",\"msg\":\"a \\\"b\\\" \\\\ c\",\"code\":1000}";
    assert(seeker_parse_response(escaped, strlen(escaped), &response));
    assert(response.code == 1000);
    assert(strcmp(response.msg, "a \"b\" \\ c") == 0);

    // \uXXXX 解码(含 UTF-16 代理对)。
    const char *unicode =
        "{\"method\":\"status\",\"msg\":\"\\u9E70\\u773C \\uD83D\\uDE00\","
        "\"code\":1000}";
    assert(seeker_parse_response(unicode, strlen(unicode), &response));
    assert(strcmp(response.msg, "\xE9\xB9\xB0\xE7\x9C\xBC \xF0\x9F\x98\x80") == 0);

    // 嵌套对象里的同名键不能覆盖顶层字段。
    const char *nested =
        "{\"data\":{\"code\":99,\"method\":\"nested\"},\"code\":1000,"
        "\"method\":\"status\",\"msg\":\"ok\"}";
    assert(seeker_parse_response(nested, strlen(nested), &response));
    assert(response.code == 1000 && strcmp(response.method, "status") == 0);

    // 非法 JSON → false。
    const char *invalid[] = {"not json", "{\"code\":1000", "{\"code\":1000}x",
                             "[]", "{\"code\":\"1000\"}"};
    for (size_t i = 0; i < sizeof(invalid) / sizeof(invalid[0]); i++) {
        if (i == 4) {
            // 字符串 code 仍是合法 JSON,但 code 保持 -1。
            assert(seeker_parse_response(invalid[i], strlen(invalid[i]),
                                         &response));
            assert(response.code == -1);
        } else {
            assert(!seeker_parse_response(invalid[i], strlen(invalid[i]),
                                          &response));
        }
    }
    assert(!seeker_parse_response(NULL, 0, &response));
    assert(!seeker_parse_response("{}", 2, NULL));

    // 空对象合法。
    assert(seeker_parse_response("{}", 2, &response));
    assert(response.code == -1 && response.method[0] == '\0');

    // device_info 字段。
    seeker_device_info_t info;
    assert(seeker_parse_device_info(SEEKER_INFO_FIXTURE,
                                    strlen(SEEKER_INFO_FIXTURE), &info));
    assert(strcmp(info.sn, "S11022109170019") == 0);
    assert(strcmp(info.firmware_version, "1.0.1") == 0);
    assert(strcmp(info.system_version, "FG05") == 0);
    assert(strcmp(info.token, "session-token") == 0);
    assert(info.has_token);

    // 1004 挑战响应只有 sn/random:字段缺失时保持空串,不报错。
    const char *challenge =
        "{\"method\":\"device_info\",\"code\":1004,\"data\":"
        "{\"sn\":\"S11022109170019\",\"random\":\"123456\"}}";
    memset(&info, 0x7F, sizeof(info));
    assert(seeker_parse_device_info(challenge, strlen(challenge), &info));
    assert(strcmp(info.sn, "S11022109170019") == 0);
    assert(info.firmware_version[0] == '\0');
    assert(info.system_version[0] == '\0');
    assert(!info.has_token);

    // data 缺失 / data 不是对象 → false。
    const char *no_data = "{\"code\":1000,\"method\":\"device_info\"}";
    assert(!seeker_parse_device_info(no_data, strlen(no_data), &info));
    const char *array_data =
        "{\"code\":1000,\"method\":\"device_info\",\"data\":[{}]}";
    assert(!seeker_parse_device_info(array_data, strlen(array_data), &info));

    // 超长 value 安全截断,仍以 NUL 结尾。
    char long_token[300];
    memset(long_token, 't', sizeof(long_token));
    long_token[sizeof(long_token) - 1] = '\0';
    char long_json[512];
    int n = snprintf(long_json, sizeof(long_json),
                     "{\"code\":1000,\"method\":\"device_info\",\"data\":"
                     "{\"sn\":\"SN\",\"token\":\"%s\"}}",
                     long_token);
    assert(n > 0);
    assert(seeker_parse_device_info(long_json, (size_t)n, &info));
    assert(strlen(info.token) == sizeof(info.token) - 1);

    // status:data.hawkeye;报文里含引号内的花括号也不能影响解析。
    const char *status_fixture =
        "{\"method\":\"status\",\"msg\":\"鹰眼 { \\\"ready\\\" }\","
        "\"data\":[{}]}";
    char hawkeye[24];
    assert(!seeker_parse_status(status_fixture, strlen(status_fixture), hawkeye));
    const char *status =
        "{\"code\":1000,\"method\":\"status\",\"data\":{\"hawkeye\":\"ready\"}}";
    assert(seeker_parse_status(status, strlen(status), hawkeye));
    assert(strcmp(hawkeye, "ready") == 0);
    const char *status_missing =
        "{\"code\":1000,\"method\":\"status\",\"data\":{}}";
    assert(!seeker_parse_status(status_missing, strlen(status_missing), hawkeye));
    const char *status_wrong_type =
        "{\"code\":1000,\"method\":\"status\",\"data\":{\"hawkeye\":7}}";
    assert(!seeker_parse_status(status_wrong_type, strlen(status_wrong_type),
                                hawkeye));

    // battery_status:data.electricity_quantity,只接受 0..100 的整数。
    int percent = -1;
    const char *battery =
        "{\"code\":1000,\"method\":\"battery_status\",\"data\":"
        "{\"electricity_quantity\":87}}";
    assert(seeker_parse_battery(battery, strlen(battery), &percent));
    assert(percent == 87);
    const char *battery_zero =
        "{\"code\":1000,\"data\":{\"electricity_quantity\":0}}";
    assert(seeker_parse_battery(battery_zero, strlen(battery_zero), &percent));
    assert(percent == 0);
    const char *bad_battery[] = {
        "{\"code\":1000,\"data\":{\"electricity_quantity\":101}}",
        "{\"code\":1000,\"data\":{\"electricity_quantity\":-1}}",
        "{\"code\":1000,\"data\":{\"electricity_quantity\":\"87\"}}",
        "{\"code\":1000,\"data\":{}}",
    };
    for (size_t i = 0; i < sizeof(bad_battery) / sizeof(bad_battery[0]); i++) {
        assert(!seeker_parse_battery(bad_battery[i], strlen(bad_battery[i]),
                                     &percent));
    }
}

// ---------------------------------------------------------------------------
// 请求组包(JSON 必须逐字符一致)
// ---------------------------------------------------------------------------
static void test_request_builders(void)
{
    char out[256];
    size_t n;

    n = seeker_build_request(out, sizeof(out), "device_info", NULL);
    assert(n == strlen("{\"method\":\"device_info\"}"));
    assert(strcmp(out, "{\"method\":\"device_info\"}") == 0);

    n = seeker_build_request(out, sizeof(out), "status", "session-token");
    assert(n == strlen("{\"method\":\"status\",\"access_token\":\"session-token\"}"));
    assert(strcmp(out, "{\"method\":\"status\",\"access_token\":\"session-token\"}") == 0);

    // 空 token 视为未提供。
    n = seeker_build_request(out, sizeof(out), "status", "");
    assert(strcmp(out, "{\"method\":\"status\"}") == 0);

    // method 中的引号/反斜杠必须转义。
    n = seeker_build_request(out, sizeof(out), "a\"b\\c", NULL);
    assert(strcmp(out, "{\"method\":\"a\\\"b\\\\c\"}") == 0);

    // 溢出 / 非法参数返回 0。
    assert(seeker_build_request(out, 10, "device_info", NULL) == 0);
    assert(seeker_build_request(NULL, sizeof(out), "device_info", NULL) == 0);
    assert(seeker_build_request(out, sizeof(out), NULL, NULL) == 0);

    // 补包请求。
    const uint16_t missing[] = {1, 2, 3};
    n = seeker_build_repair_request(out, sizeof(out), "device_info", missing, 3);
    const char *expected_repair =
        "{\"method\":\"get_ble_pkg\",\"requestMethod\":\"device_info\","
        "\"list\":[1,2,3]}";
    assert(n == strlen(expected_repair));
    assert(strcmp(out, expected_repair) == 0);

    const uint16_t one[] = {7};
    n = seeker_build_repair_request(out, sizeof(out), "status", one, 1);
    assert(strcmp(out,
                  "{\"method\":\"get_ble_pkg\",\"requestMethod\":\"status\","
                  "\"list\":[7]}") == 0);

    n = seeker_build_repair_request(out, sizeof(out), "status", NULL, 0);
    assert(strcmp(out,
                  "{\"method\":\"get_ble_pkg\",\"requestMethod\":\"status\","
                  "\"list\":[]}") == 0);

    // 非法参数 / 缓冲区不足。
    assert(seeker_build_repair_request(out, 8, "device_info", missing, 3) == 0);
    assert(seeker_build_repair_request(NULL, sizeof(out), "device_info", missing,
                                       3) == 0);
    assert(seeker_build_repair_request(out, sizeof(out), NULL, missing, 3) == 0);
    assert(seeker_build_repair_request(out, sizeof(out), "device_info", NULL, 3) == 0);
}

int main(void)
{
    test_real_device_golden();
    test_multi_packet_reassembly();
    test_nine_packet_version_message();
    test_lowercase_hex_counters();
    test_error_paths();
    test_completed_window();
    test_raw_json_passthrough();
    test_json_parse();
    test_request_builders();
    puts("SEEKER proto tests: PASS");
    return 0;
}
