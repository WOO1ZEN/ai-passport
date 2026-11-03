// main/md5.c —— 纯 C MD5(RFC 1321)增量实现,只用于本地协议校验,不用于安全存储。
//
// 设计说明:
// - 不依赖 ESP-IDF / LVGL / 动态内存,同一份代码可在设备与主机测试中编译;
// - 增量接口按 RFC 1321 的 Init/Update/Final 语义实现,调用方负责在栈上
//   分配 md5_ctx_t;
// - 私钥与动态码属于敏感输入,本文件不做任何日志输出。
#include "md5.h"

#include <stdbool.h>
#include <string.h>

// 轮函数常量(K[i] = floor(2^32 * abs(sin(i + 1))),整数枚举以避免浮点)。
static const uint32_t MD5_K[64] = {
    0xd76aa478u, 0xe8c7b756u, 0x242070dbu, 0xc1bdceeeu,
    0xf57c0fafu, 0x4787c62au, 0xa8304613u, 0xfd469501u,
    0x698098d8u, 0x8b44f7afu, 0xffff5bb1u, 0x895cd7beu,
    0x6b901122u, 0xfd987193u, 0xa679438eu, 0x49b40821u,
    0xf61e2562u, 0xc040b340u, 0x265e5a51u, 0xe9b6c7aau,
    0xd62f105du, 0x02441453u, 0xd8a1e681u, 0xe7d3fbc8u,
    0x21e1cde6u, 0xc33707d6u, 0xf4d50d87u, 0x455a14edu,
    0xa9e3e905u, 0xfcefa3f8u, 0x676f02d9u, 0x8d2a4c8au,
    0xfffa3942u, 0x8771f681u, 0x6d9d6122u, 0xfde5380cu,
    0xa4beea44u, 0x4bdecfa9u, 0xf6bb4b60u, 0xbebfbc70u,
    0x289b7ec6u, 0xeaa127fau, 0xd4ef3085u, 0x04881d05u,
    0xd9d4d039u, 0xe6db99e5u, 0x1fa27cf8u, 0xc4ac5665u,
    0xf4292244u, 0x432aff97u, 0xab9423a7u, 0xfc93a039u,
    0x655b59c3u, 0x8f0ccc92u, 0xffeff47du, 0x85845dd1u,
    0x6fa87e4fu, 0xfe2ce6e0u, 0xa3014314u, 0x4e0811a1u,
    0xf7537e82u, 0xbd3af235u, 0x2ad7d2bbu, 0xeb86d391u,
};

// 每轮循环左移位数。
static const uint8_t MD5_S[64] = {
    7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22,
    5, 9, 14, 20, 5, 9, 14, 20, 5, 9, 14, 20, 5, 9, 14, 20,
    4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23,
    6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21,
};

static uint32_t md5_rotl32(uint32_t x, uint32_t c)
{
    return (x << c) | (x >> (32u - c));
}

// 处理一个 64 字节分组;消息字按小端读入。
static void md5_transform(uint32_t state[4], const uint8_t block[64])
{
    uint32_t m[16];

    for (size_t i = 0; i < 16; i++) {
        m[i] = (uint32_t)block[i * 4] |
               ((uint32_t)block[i * 4 + 1] << 8) |
               ((uint32_t)block[i * 4 + 2] << 16) |
               ((uint32_t)block[i * 4 + 3] << 24);
    }

    uint32_t a = state[0];
    uint32_t b = state[1];
    uint32_t c = state[2];
    uint32_t d = state[3];

    for (size_t i = 0; i < 64; i++) {
        uint32_t f;
        size_t g;
        if (i < 16) {
            f = (b & c) | (~b & d);
            g = i;
        } else if (i < 32) {
            f = (d & b) | (~d & c);
            g = (5 * i + 1) % 16;
        } else if (i < 48) {
            f = b ^ c ^ d;
            g = (3 * i + 5) % 16;
        } else {
            f = c ^ (b | ~d);
            g = (7 * i) % 16;
        }
        const uint32_t tmp = d;
        d = c;
        c = b;
        b = b + md5_rotl32(a + f + MD5_K[i] + m[g], MD5_S[i]);
        a = tmp;
    }

    state[0] += a;
    state[1] += b;
    state[2] += c;
    state[3] += d;
}

void md5_init(md5_ctx_t *ctx)
{
    if (ctx == NULL) {
        return;
    }
    ctx->state[0] = 0x67452301u;
    ctx->state[1] = 0xefcdab89u;
    ctx->state[2] = 0x98badcfeu;
    ctx->state[3] = 0x10325476u;
    ctx->count = 0;
    memset(ctx->buffer, 0, sizeof(ctx->buffer));
}

void md5_update(md5_ctx_t *ctx, const void *data, size_t len)
{
    if (ctx == NULL || data == NULL || len == 0) {
        return;
    }

    const uint8_t *p = (const uint8_t *)data;
    size_t index = (size_t)(ctx->count % 64u);
    ctx->count += len;

    // 先把缓冲区补满一个分组。
    size_t part = 64u - index;
    size_t i = 0;
    if (len >= part) {
        memcpy(&ctx->buffer[index], p, part);
        md5_transform(ctx->state, ctx->buffer);
        for (i = part; i + 63u < len; i += 64u) {
            md5_transform(ctx->state, &p[i]);
        }
        index = 0;
    }
    // 剩余不足一组的字节留在缓冲区,等下一次 update / final。
    memcpy(&ctx->buffer[index], &p[i], len - i);
}

void md5_final(md5_ctx_t *ctx, uint8_t out[16])
{
    if (ctx == NULL || out == NULL) {
        return;
    }

    // 填充:0x80 + 若干 0x00,使总长度 ≡ 56 (mod 64),末尾再放 64 位比特长度。
    const uint64_t bits = ctx->count * 8u;
    const size_t index = (size_t)(ctx->count % 64u);
    const size_t pad_len = (index < 56u) ? (56u - index) : (120u - index);
    uint8_t pad[128];

    memset(pad, 0, sizeof(pad));
    pad[0] = 0x80;
    md5_update(ctx, pad, pad_len);

    uint8_t len_le[8];
    for (size_t i = 0; i < sizeof(len_le); i++) {
        len_le[i] = (uint8_t)(bits >> (8u * i));
    }
    md5_update(ctx, len_le, sizeof(len_le));

    // 摘要按小端输出。
    for (size_t i = 0; i < 4; i++) {
        out[i * 4] = (uint8_t)(ctx->state[i]);
        out[i * 4 + 1] = (uint8_t)(ctx->state[i] >> 8);
        out[i * 4 + 2] = (uint8_t)(ctx->state[i] >> 16);
        out[i * 4 + 3] = (uint8_t)(ctx->state[i] >> 24);
    }
}

// 16 字节摘要转 32 字符 hex + NUL。
static void md5_digest_hex(const uint8_t digest[16], char out[33], bool upper)
{
    const char *table = upper ? "0123456789ABCDEF" : "0123456789abcdef";

    for (size_t i = 0; i < 16; i++) {
        out[i * 2] = table[digest[i] >> 4];
        out[i * 2 + 1] = table[digest[i] & 0x0Fu];
    }
    out[32] = '\0';
}

void md5_hex_lower(const void *data, size_t len, char out[33])
{
    md5_ctx_t ctx;
    uint8_t digest[16];

    if (out == NULL) {
        return;
    }
    md5_init(&ctx);
    md5_update(&ctx, data, len);
    md5_final(&ctx, digest);
    md5_digest_hex(digest, out, false);
}

void md5_hex_upper(const void *data, size_t len, char out[33])
{
    md5_ctx_t ctx;
    uint8_t digest[16];

    if (out == NULL) {
        return;
    }
    md5_init(&ctx);
    md5_update(&ctx, data, len);
    md5_final(&ctx, digest);
    md5_digest_hex(digest, out, true);
}

void md5_digest_hex_lower(const uint8_t digest[16], char out[33])
{
    if (digest == NULL || out == NULL) {
        return;
    }
    md5_digest_hex(digest, out, false);
}

void md5_digest_hex_upper(const uint8_t digest[16], char out[33])
{
    if (digest == NULL || out == NULL) {
        return;
    }
    md5_digest_hex(digest, out, true);
}
