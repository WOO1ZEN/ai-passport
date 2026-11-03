// main/md5.h —— 纯 C MD5(RFC 1321),用于 NOVA 接入认证和 SEEKER 分包校验。
//
// 不依赖 ESP-IDF:同一份实现在设备与主机测试里编译。安全性要求:私钥/动态码
// 不得打印,调用方只在内存里短存。
#pragma once

#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint32_t state[4];
    uint64_t count;      // 已处理的字节数
    uint8_t buffer[64];
} md5_ctx_t;

void md5_init(md5_ctx_t *ctx);
void md5_update(md5_ctx_t *ctx, const void *data, size_t len);
void md5_final(md5_ctx_t *ctx, uint8_t out[16]);

// 一次性计算并输出 32 字符 hex + NUL(小写/大写)。
void md5_hex_lower(const void *data, size_t len, char out[33]);
void md5_hex_upper(const void *data, size_t len, char out[33]);

// 已计算摘要的 hex 输出。
void md5_digest_hex_lower(const uint8_t digest[16], char out[33]);
void md5_digest_hex_upper(const uint8_t digest[16], char out[33]);
