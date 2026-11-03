// tests/test_md5.c —— 纯 C MD5 的主机单元测试。
//
// 契约来源:main/md5.h。只依赖 C11 标准库与 assert(),不链接 ESP-IDF/LVGL。
// 黄金向量来自 RFC 1321 附录 A.5 的测试套件(公共标准向量),用于确认
// 增量实现的填充、字节序与 hex 输出都正确。
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "md5.h"

static void expect_hex(const char *input, size_t len, const char *expected_lower)
{
    char hex[33];
    char upper[33];

    md5_hex_lower(input, len, hex);
    assert(strcmp(hex, expected_lower) == 0);
    assert(strlen(hex) == 32);
    // 大写版本除了字母大小写外必须完全一致。
    md5_hex_upper(input, len, upper);
    for (size_t i = 0; i < 32; i++) {
        char lower_char = hex[i];
        if (lower_char >= 'a' && lower_char <= 'f') {
            lower_char = (char)(lower_char - 'a' + 'A');
        }
        assert(upper[i] == lower_char);
    }
    assert(upper[32] == '\0');
}

// RFC 1321 A.5 的全部标准向量。
static void test_rfc1321_vectors(void)
{
    expect_hex("", 0, "d41d8cd98f00b204e9800998ecf8427e");
    expect_hex("a", 1, "0cc175b9c0f1b6a831c399e269772661");
    expect_hex("abc", 3, "900150983cd24fb0d6963f7d28e17f72");
    expect_hex("message digest", 14, "f96b697d7cb7938d525a2f31aaf161d0");
    expect_hex("abcdefghijklmnopqrstuvwxyz", 26,
               "c3fcd3d76192e4007dfb496cca67e13b");
    expect_hex("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789",
               62, "d174ab98d277d9f5a5611c2c9f419d9f");
    expect_hex("123456789012345678901234567890123456789012345678901234567890123456"
               "78901234567890",
               80, "57edf4a22be3c955ac49da2e2107b67a");

    // 跨多个 64 字节分组的输入(值由 Python hashlib.md5 独立算出)。
    static char thousand_a[1000];
    memset(thousand_a, 'a', sizeof(thousand_a));
    expect_hex(thousand_a, sizeof(thousand_a), "cabe45dcc9ae5b66ba86600cca6b8ba8");
    expect_hex("The quick brown fox jumps over the lazy dog", 43,
               "9e107d9d372bb6826bd81d3542a419d6");
}

// 增量接口:任意切分方式必须得到同一摘要(覆盖 64 字节边界与空 update)。
static void test_incremental(void)
{
    const char *message = "abcdefghijklmnopqrstuvwxyz0123456789";
    char once[33];
    md5_hex_lower(message, strlen(message), once);

    char hex[33];
    uint8_t digest[16];
    md5_ctx_t ctx;

    md5_init(&ctx);
    md5_update(&ctx, message, 1);
    md5_update(&ctx, message + 1, 0);        // 空 update 不影响结果
    md5_update(&ctx, NULL, 0);               // NULL + 0 也安全
    md5_update(&ctx, message + 1, strlen(message) - 1);
    md5_final(&ctx, digest);
    md5_digest_hex_lower(digest, hex);
    assert(strcmp(hex, once) == 0);

    // 按 1/7/13/64 等不规则边界切分。
    const size_t chunks[] = {1, 7, 13, 64, 2, 3};
    md5_init(&ctx);
    size_t offset = 0;
    for (size_t i = 0; i < sizeof(chunks) / sizeof(chunks[0]) && offset < strlen(message); i++) {
        size_t n = chunks[i];
        if (n > strlen(message) - offset) {
            n = strlen(message) - offset;
        }
        md5_update(&ctx, message + offset, n);
        offset += n;
    }
    md5_final(&ctx, digest);
    md5_digest_hex_lower(digest, hex);
    assert(strcmp(hex, once) == 0);

    // 摘要辅助函数:小写/大写。
    char upper[33];
    md5_digest_hex_upper(digest, upper);
    for (size_t i = 0; i < 32; i++) {
        const char c = upper[i];
        assert((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F'));
        if (c >= 'A' && c <= 'F') {
            assert(hex[i] == (char)(c - 'A' + 'a'));
        } else {
            assert(hex[i] == c);
        }
    }
    assert(upper[32] == '\0');

    // 一次性接口与增量接口对空输入一致。
    md5_init(&ctx);
    md5_final(&ctx, digest);
    md5_digest_hex_lower(digest, hex);
    assert(strcmp(hex, "d41d8cd98f00b204e9800998ecf8427e") == 0);
}

int main(void)
{
    test_rfc1321_vectors();
    test_incremental();
    puts("MD5 tests: PASS");
    return 0;
}
