// main/seeker_proto.c —— SEEKER(鹰眼)BLE 分包重组、轻量 JSON 解析与请求组包(纯 C)。
//
// 真机实测的分包格式(22 字节 ASCII 头 + 正文):
//   [16 字符大写 MD5 中段][3 字符包号][3 字符总包数][正文...]
//   MD5 中段 = MD5(完整正文) 大写 hex 的第 8..23 个字符;包号从 1 开始;
//   计数器默认十进制,只要 6 个计数字符里出现 A-F 就整体切换成十六进制。
// 首包第一个非空白字节是 '{' 时视为未封装的原始 JSON(模拟器/兼容固件)。
//
// 没有动态内存:每个 assembler 自带 arena(最多 SEEKER_MAX_BODY 字节),
// 超过上限或包数超过 SEEKER_MAX_PACKETS 一律返回 ERROR,由调用方断开重连。
// JSON 解析只提取只读所需字段,不构建 DOM,也不做任何日志输出(token/pwd 不落日志)。
#include "seeker_proto.h"

#include <string.h>

#include "md5.h"

// 模式哨兵:hash[0] == '\0' 表示尚未锁定;hash == "#RAW" 表示原始 JSON 模式;
// 其余情况 hash 保存分包模式当前(或最近一次)批次的 16 位大写 MD5 中段。
// 结构体没有独立的模式字段,故借用 hash(原始 JSON 批次不使用 hash)。
#define SEEKER_RAW_SENTINEL "#RAW"

#define SEEKER_COUNTER_DIGITS 3
#define SEEKER_MAX_COUNTER 4095

static bool seeker_is_space(uint8_t c)
{
    return c == ' ' || c == '\t' || c == '\r' || c == '\n';
}

static bool seeker_is_hex(uint8_t c)
{
    return (c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') ||
           (c >= 'a' && c <= 'f');
}

static uint8_t seeker_hex_nibble(uint8_t c)
{
    if (c >= '0' && c <= '9') {
        return (uint8_t)(c - '0');
    }
    if (c >= 'a' && c <= 'f') {
        return (uint8_t)(c - 'a' + 10);
    }
    return (uint8_t)(c - 'A' + 10);
}

static uint8_t seeker_to_upper(uint8_t c)
{
    return (c >= 'a' && c <= 'z') ? (uint8_t)(c - 'a' + 'A') : c;
}

// 解析 3 字符计数器。radix 为 10 或 16;越界(>4095)或其他字符返回 false。
static bool seeker_parse_counter(const char text[SEEKER_COUNTER_DIGITS],
                                 unsigned radix, uint16_t *out)
{
    unsigned value = 0;

    for (size_t i = 0; i < SEEKER_COUNTER_DIGITS; i++) {
        const uint8_t c = (uint8_t)text[i];
        unsigned digit;
        if (c >= '0' && c <= '9') {
            digit = (unsigned)(c - '0');
        } else if (radix == 16 && c >= 'A' && c <= 'F') {
            digit = (unsigned)(c - 'A' + 10);
        } else {
            return false;
        }
        value = value * radix + digit;
    }
    if (value > SEEKER_MAX_COUNTER) {
        return false;
    }
    *out = (uint16_t)value;
    return true;
}

static bool seeker_hash_completed(const seeker_assembler_t *a, const char hash[17])
{
    for (size_t i = 0; i < a->completed_count && i < SEEKER_MAX_COMPLETED; i++) {
        if (strcmp(a->completed[i], hash) == 0) {
            return true;
        }
    }
    return false;
}

// 最近 4 个已完成 MD5 中段的去重窗口(先进先出)。
static void seeker_hash_remember(seeker_assembler_t *a, const char hash[17])
{
    if (a->completed_count < SEEKER_MAX_COMPLETED) {
        memcpy(a->completed[a->completed_count], hash, 17);
        a->completed_count++;
        return;
    }
    memmove(a->completed[0], a->completed[1],
            (SEEKER_MAX_COMPLETED - 1) * sizeof(a->completed[0]));
    memcpy(a->completed[SEEKER_MAX_COMPLETED - 1], hash, 17);
}

void seeker_assembler_reset(seeker_assembler_t *a)
{
    if (a == NULL) {
        return;
    }
    memset(a, 0, sizeof(*a));
}

void seeker_assembler_begin_request(seeker_assembler_t *a)
{
    if (a == NULL) {
        return;
    }
    // 只清空"最近完成"窗口:同一连接里迟到的旧响应不能被下一请求误用。
    a->completed_count = 0;
}

// 判断首包模式:第一个非空白字节是 '{' 则锁定原始 JSON 模式。
// 返回 false 表示整包都是空白(无法判定,保持未锁定)。
static bool seeker_latch_mode(seeker_assembler_t *a, const uint8_t *packet,
                              size_t len)
{
    if (a->hash[0] != '\0') {
        return true;
    }
    for (size_t i = 0; i < len; i++) {
        if (seeker_is_space(packet[i])) {
            continue;
        }
        if (packet[i] == '{') {
            memcpy(a->hash, SEEKER_RAW_SENTINEL, sizeof(SEEKER_RAW_SENTINEL));
        }
        // 非 '{' 时不在这里写 hash,等分包处理成功后自然保存 16 位哈希。
        return true;
    }
    return false;
}

static seeker_feed_result_t seeker_feed_framed(seeker_assembler_t *a,
                                               const uint8_t *packet, size_t len,
                                               uint8_t *out, size_t out_cap,
                                               size_t *out_len)
{
    // 单包必须严格大于 22 字节(至少 1 字节正文)。
    if (len <= SEEKER_PACKET_HEADER_LEN) {
        return SEEKER_FEED_ERROR;
    }
    for (size_t i = 0; i < SEEKER_PACKET_HEADER_LEN; i++) {
        if (!seeker_is_hex(packet[i])) {
            return SEEKER_FEED_ERROR;
        }
    }

    char hash[17];
    for (size_t i = 0; i < 16; i++) {
        hash[i] = (char)seeker_to_upper(packet[i]);
    }
    hash[16] = '\0';

    // 已完成的同一批次(迟到的重复包)直接忽略。
    if (seeker_hash_completed(a, hash)) {
        return SEEKER_FEED_NONE;
    }

    // 任意计数字符出现 A-F 即锁定十六进制;否则按十进制。
    // Android SeekerPacketAssembler.kt 会先把 3 字符计数器转大写再判断,因此
    // 小写 a-f 同样要锁定 hex(真机图片总包数实为小写 "1c2" = 450)。
    bool hex_latched = false;
    for (size_t i = 16; i < SEEKER_PACKET_HEADER_LEN; i++) {
        const uint8_t c = seeker_to_upper(packet[i]);
        if (c >= 'A' && c <= 'F') {
            hex_latched = true;
        }
    }

    if (!a->active) {
        // 新批次:hash 同时作为"已锁定分包模式"的标记保留到下次 reset。
        memcpy(a->hash, hash, sizeof(hash));
        a->active = true;
        a->hex = false;
        a->total = 0;
        a->present = 0;
        a->arena_len = 0;
        memset(a->offsets, 0, sizeof(a->offsets));
        memset(a->lengths, 0, sizeof(a->lengths));
    } else if (strcmp(a->hash, hash) != 0) {
        // 结构体只支持单个未完成批次;多消息交织视为不可恢复错误。
        return SEEKER_FEED_ERROR;
    }

    const bool was_hex = a->hex;
    if (hex_latched) {
        a->hex = true;
    }
    const unsigned radix = a->hex ? 16u : 10u;

    char index_text[SEEKER_COUNTER_DIGITS];
    char total_text[SEEKER_COUNTER_DIGITS];
    for (size_t i = 0; i < SEEKER_COUNTER_DIGITS; i++) {
        index_text[i] = (char)seeker_to_upper(packet[16 + i]);
        total_text[i] = (char)seeker_to_upper(packet[19 + i]);
    }

    uint16_t index = 0;
    uint16_t total = 0;
    if (!seeker_parse_counter(index_text, radix, &index) ||
        !seeker_parse_counter(total_text, radix, &total)) {
        return SEEKER_FEED_ERROR;
    }
    // 总包数 1..4095(协议范围),但同时受静态内存上限约束(实测最多 9 包)。
    if (total < 1 || total > SEEKER_MAX_PACKETS) {
        return SEEKER_FEED_ERROR;
    }
    if (a->total == 0) {
        a->total = total;
    } else if (a->total != total) {
        // 特殊情况:本包才锁定十六进制,而总包数文本不含 A-F —— 同一段文本
        // 从十进制改按十六进制解释(例如 "010" 由 10 变 16)。只有当初保存的
        // 值与当前文本的十进制解释一致时,才认定为同一批次;否则是总包数冲突。
        bool reinterpreted = false;
        if (!was_hex && a->hex) {
            uint16_t decimal_total = 0;
            if (seeker_parse_counter(total_text, 10, &decimal_total) &&
                decimal_total == a->total) {
                a->total = total;
                reinterpreted = true;
            }
        }
        if (!reinterpreted) {
            return SEEKER_FEED_ERROR;
        }
    }
    if (index < 1 || index > a->total) {
        return SEEKER_FEED_ERROR;
    }

    const uint8_t *body = packet + SEEKER_PACKET_HEADER_LEN;
    const size_t body_len = len - SEEKER_PACKET_HEADER_LEN;
    const size_t slot = (size_t)index - 1;

    if (a->lengths[slot] != 0) {
        // 重复包:正文一致则忽略,不一致则冲突(不可恢复)。
        if (a->lengths[slot] != body_len ||
            memcmp(a->arena + a->offsets[slot], body, body_len) != 0) {
            return SEEKER_FEED_ERROR;
        }
        return SEEKER_FEED_NONE;
    }

    if (a->arena_len + body_len > SEEKER_MAX_BODY) {
        return SEEKER_FEED_ERROR;
    }
    a->offsets[slot] = (uint16_t)a->arena_len;
    a->lengths[slot] = (uint16_t)body_len;
    memcpy(a->arena + a->arena_len, body, body_len);
    a->arena_len += body_len;
    a->present++;

    if (a->present != a->total) {
        return SEEKER_FEED_NONE;
    }

    // 消息完整:按包号升序拼接并校验 MD5 中段。
    if (out == NULL || out_cap < a->arena_len) {
        return SEEKER_FEED_ERROR;
    }

    md5_ctx_t ctx;
    uint8_t digest[16];
    char digest_hex[33];
    size_t written = 0;
    md5_init(&ctx);
    for (size_t i = 0; i < a->total; i++) {
        const size_t off = a->offsets[i];
        const size_t seg = a->lengths[i];
        md5_update(&ctx, a->arena + off, seg);
        memcpy(out + written, a->arena + off, seg);
        written += seg;
    }
    md5_final(&ctx, digest);
    md5_digest_hex_upper(digest, digest_hex);

    if (memcmp(digest_hex + 8, a->hash, 16) != 0) {
        return SEEKER_FEED_ERROR;
    }

    seeker_hash_remember(a, a->hash);
    a->active = false;
    a->present = 0;
    a->total = 0;
    a->hex = false;
    // a->hash 保留最近批次的 16 位哈希,作为"分包模式已锁定"的标记。
    a->arena_len = 0;
    *out_len = written;
    return SEEKER_FEED_COMPLETE;
}

seeker_feed_result_t seeker_assembler_feed(seeker_assembler_t *a,
                                           const uint8_t *packet, size_t len,
                                           uint8_t *out, size_t out_cap,
                                           size_t *out_len)
{
    if (a == NULL || packet == NULL || out_len == NULL) {
        return SEEKER_FEED_ERROR;
    }
    *out_len = 0;
    if (len == 0) {
        return SEEKER_FEED_NONE;
    }
    if (!seeker_latch_mode(a, packet, len)) {
        return SEEKER_FEED_NONE;
    }

    // 原始 JSON 模式:整包透传,由调用方拼接/J 解析(不解析 22 字节头)。
    if (strcmp(a->hash, SEEKER_RAW_SENTINEL) == 0) {
        if (out == NULL || out_cap < len) {
            return SEEKER_FEED_ERROR;
        }
        memcpy(out, packet, len);
        *out_len = len;
        return SEEKER_FEED_COMPLETE;
    }

    return seeker_feed_framed(a, packet, len, out, out_cap, out_len);
}

size_t seeker_assembler_missing(const seeker_assembler_t *a, uint16_t *out,
                                size_t cap)
{
    if (a == NULL || !a->active || out == NULL || cap == 0) {
        return 0;
    }

    size_t missing = 0;
    for (uint16_t index = 1; index <= a->total; index++) {
        if (a->lengths[index - 1] == 0) {
            if (missing < cap) {
                out[missing] = index;
            }
            missing++;
        }
    }
    return missing;
}

bool seeker_assembler_incomplete(const seeker_assembler_t *a)
{
    if (a == NULL) {
        return false;
    }
    // active 只在收到第一包后置位,完成/复位后清零。
    return a->active && a->present > 0;
}

// ---------------------------------------------------------------------------
// 轻量 JSON 解析
// ---------------------------------------------------------------------------
//
// 只支持顶层对象的键值提取与一层嵌套(data 对象内部)。字符串值按 JSON 规则
// 反转义(含 \" \\ \/ \b \f \n \r \t 与 \uXXXX);不做完整 schema 校验。

typedef enum {
    SEEKER_JSON_NONE = 0,
    SEEKER_JSON_STRING,
    SEEKER_JSON_OBJECT,
    SEEKER_JSON_ARRAY,
    SEEKER_JSON_NUMBER,
    SEEKER_JSON_LITERAL,
} seeker_json_type_t;

typedef struct {
    const char *start;  // 指向值首字节
    size_t len;         // 值的原始字节长度
    seeker_json_type_t type;
} seeker_json_value_t;

static void json_skip_ws(const char *json, size_t len, size_t *pos)
{
    while (*pos < len) {
        const char c = json[*pos];
        if (c != ' ' && c != '\t' && c != '\r' && c != '\n') {
            break;
        }
        (*pos)++;
    }
}

// pos 指向双引号字符;成功时移到收尾双引号之后。
static bool json_skip_string(const char *json, size_t len, size_t *pos)
{
    if (*pos >= len || json[*pos] != '"') {
        return false;
    }
    (*pos)++;
    while (*pos < len) {
        const char c = json[*pos];
        if (c == '\\') {
            if (*pos + 1 >= len) {
                return false;
            }
            *pos += 2;
            continue;
        }
        (*pos)++;
        if (c == '"') {
            return true;
        }
    }
    return false;
}

// 跳过一个 JSON 值;成功时 pos 指到值末尾之后。
static bool json_skip_value(const char *json, size_t len, size_t *pos)
{
    json_skip_ws(json, len, pos);
    if (*pos >= len) {
        return false;
    }

    const char c = json[*pos];
    if (c == '"') {
        return json_skip_string(json, len, pos);
    }
    if (c == '{' || c == '[') {
        size_t depth = 0;
        while (*pos < len) {
            const char cur = json[*pos];
            if (cur == '"') {
                if (!json_skip_string(json, len, pos)) {
                    return false;
                }
                continue;
            }
            if (cur == '{' || cur == '[') {
                depth++;
            } else if (cur == '}' || cur == ']') {
                depth--;
                if (depth == 0) {
                    (*pos)++;
                    return true;
                }
            }
            (*pos)++;
        }
        return false;
    }
    if (c == 't') {
        if (*pos + 4 > len || memcmp(json + *pos, "true", 4) != 0) {
            return false;
        }
        *pos += 4;
        return true;
    }
    if (c == 'f') {
        if (*pos + 5 > len || memcmp(json + *pos, "false", 5) != 0) {
            return false;
        }
        *pos += 5;
        return true;
    }
    if (c == 'n') {
        if (*pos + 4 > len || memcmp(json + *pos, "null", 4) != 0) {
            return false;
        }
        *pos += 4;
        return true;
    }

    // 数字:[-]digits[.digits][eE[+-]digits]。
    const size_t start = *pos;
    if (json[*pos] == '-') {
        (*pos)++;
    }
    size_t digits = 0;
    while (*pos < len && json[*pos] >= '0' && json[*pos] <= '9') {
        (*pos)++;
        digits++;
    }
    if (digits == 0) {
        return false;
    }
    if (*pos < len && json[*pos] == '.') {
        (*pos)++;
        while (*pos < len && json[*pos] >= '0' && json[*pos] <= '9') {
            (*pos)++;
        }
    }
    if (*pos < len && (json[*pos] == 'e' || json[*pos] == 'E')) {
        (*pos)++;
        if (*pos < len && (json[*pos] == '+' || json[*pos] == '-')) {
            (*pos)++;
        }
        while (*pos < len && json[*pos] >= '0' && json[*pos] <= '9') {
            (*pos)++;
        }
    }
    return *pos > start;
}

static seeker_json_type_t json_value_type(const char *json, size_t pos)
{
    const char c = json[pos];
    if (c == '"') {
        return SEEKER_JSON_STRING;
    }
    if (c == '{') {
        return SEEKER_JSON_OBJECT;
    }
    if (c == '[') {
        return SEEKER_JSON_ARRAY;
    }
    if (c == 't' || c == 'f' || c == 'n') {
        return SEEKER_JSON_LITERAL;
    }
    return SEEKER_JSON_NUMBER;
}

// 在对象里查找顶层键。json 必须从 '{' 开始。
static bool json_find_field(const char *json, size_t len, const char *key,
                            seeker_json_value_t *out)
{
    size_t i = 0;
    json_skip_ws(json, len, &i);
    if (i >= len || json[i] != '{') {
        return false;
    }
    i++;

    const size_t key_len = strlen(key);
    for (;;) {
        json_skip_ws(json, len, &i);
        if (i >= len) {
            return false;
        }
        if (json[i] == '}') {
            return false;
        }
        if (json[i] != '"') {
            return false;
        }

        const size_t key_start = i + 1;
        if (!json_skip_string(json, len, &i)) {
            return false;
        }
        const size_t key_end = i - 1;  // 收尾双引号位置
        json_skip_ws(json, len, &i);
        if (i >= len || json[i] != ':') {
            return false;
        }
        i++;
        json_skip_ws(json, len, &i);

        const size_t value_start = i;
        if (!json_skip_value(json, len, &i)) {
            return false;
        }
        const size_t value_end = i;

        if (key_end - key_start == key_len &&
            memcmp(json + key_start, key, key_len) == 0) {
            out->start = json + value_start;
            out->len = value_end - value_start;
            out->type = json_value_type(json, value_start);
            return true;
        }

        json_skip_ws(json, len, &i);
        if (i < len && json[i] == ',') {
            i++;
            continue;
        }
        return false;
    }
}

// 校验并返回顶层对象起始位置;允许首尾空白。
static bool json_root_object(const char *json, size_t len)
{
    size_t i = 0;
    json_skip_ws(json, len, &i);
    if (i >= len || json[i] != '{') {
        return false;
    }
    if (!json_skip_value(json, len, &i)) {
        return false;
    }
    json_skip_ws(json, len, &i);
    return i == len;
}

static bool json_decode_hex4(const char *src, uint32_t *code)
{
    uint32_t value = 0;
    for (size_t i = 0; i < 4; i++) {
        const uint8_t c = (uint8_t)src[i];
        if (!seeker_is_hex(c)) {
            return false;
        }
        value = (value << 4) | seeker_hex_nibble(c);
    }
    *code = value;
    return true;
}

// 把码点编码成 UTF-8 追加到 dst;容量不足返回 false(调用方截断)。
static bool json_append_codepoint(char *dst, size_t cap, size_t *pos,
                                  uint32_t code)
{
    uint8_t tmp[4];
    size_t n;
    if (code <= 0x7F) {
        tmp[0] = (uint8_t)code;
        n = 1;
    } else if (code <= 0x7FF) {
        tmp[0] = (uint8_t)(0xC0 | (code >> 6));
        tmp[1] = (uint8_t)(0x80 | (code & 0x3F));
        n = 2;
    } else if (code <= 0xFFFF) {
        tmp[0] = (uint8_t)(0xE0 | (code >> 12));
        tmp[1] = (uint8_t)(0x80 | ((code >> 6) & 0x3F));
        tmp[2] = (uint8_t)(0x80 | (code & 0x3F));
        n = 3;
    } else {
        tmp[0] = (uint8_t)(0xF0 | (code >> 18));
        tmp[1] = (uint8_t)(0x80 | ((code >> 12) & 0x3F));
        tmp[2] = (uint8_t)(0x80 | ((code >> 6) & 0x3F));
        tmp[3] = (uint8_t)(0x80 | (code & 0x3F));
        n = 4;
    }
    if (*pos + n >= cap) {
        return false;
    }
    memcpy(dst + *pos, tmp, n);
    *pos += n;
    return true;
}

// 反转义 JSON 字符串值;输出 NUL 结尾,超长时安全截断(不拆多字节字符)。
static void json_unescape_string(const seeker_json_value_t *value, char *dst,
                                 size_t cap)
{
    size_t out = 0;
    if (dst == NULL || cap == 0) {
        return;
    }
    if (value->type != SEEKER_JSON_STRING || value->len < 2) {
        dst[0] = '\0';
        return;
    }

    const char *src = value->start + 1;      // 跳过开引号
    const size_t src_len = value->len - 2;   // 去掉首尾引号
    size_t i = 0;
    while (i < src_len && out + 1 < cap) {
        const uint8_t c = (uint8_t)src[i];
        if (c != '\\') {
            dst[out++] = (char)c;
            i++;
            continue;
        }
        if (i + 1 >= src_len) {
            break;
        }
        const char esc = src[i + 1];
        i += 2;
        switch (esc) {
        case '"':
            dst[out++] = '"';
            break;
        case '\\':
            dst[out++] = '\\';
            break;
        case '/':
            dst[out++] = '/';
            break;
        case 'b':
            dst[out++] = '\b';
            break;
        case 'f':
            dst[out++] = '\f';
            break;
        case 'n':
            dst[out++] = '\n';
            break;
        case 'r':
            dst[out++] = '\r';
            break;
        case 't':
            dst[out++] = '\t';
            break;
        case 'u': {
            if (i + 4 > src_len) {
                i = src_len;
                break;
            }
            uint32_t code = 0;
            if (!json_decode_hex4(src + i, &code)) {
                i += 4;
                break;
            }
            i += 4;
            // UTF-16 代理对:高代理 + \uXXXX 低代理合并成一个码点。
            if (code >= 0xD800 && code <= 0xDBFF && i + 6 <= src_len &&
                src[i] == '\\' && src[i + 1] == 'u') {
                uint32_t low = 0;
                if (json_decode_hex4(src + i + 2, &low) && low >= 0xDC00 &&
                    low <= 0xDFFF) {
                    code = 0x10000 + ((code - 0xD800) << 10) + (low - 0xDC00);
                    i += 6;
                }
            }
            if (!json_append_codepoint(dst, cap, &out, code)) {
                i = src_len;
            }
            break;
        }
        default:
            // 非法转义:按原字符保留,避免静默吞数据。
            dst[out++] = esc;
            break;
        }
    }
    dst[out] = '\0';
}

static bool json_parse_int(const seeker_json_value_t *value, int *out)
{
    if (value->type != SEEKER_JSON_NUMBER || value->len == 0) {
        return false;
    }

    size_t i = 0;
    bool negative = false;
    if (value->start[i] == '-') {
        negative = true;
        i++;
    }
    if (i >= value->len) {
        return false;
    }

    int result = 0;
    for (; i < value->len; i++) {
        const char c = value->start[i];
        if (c < '0' || c > '9') {
            return false;  // 只读字段都是整数,不接受小数/指数
        }
        result = result * 10 + (c - '0');
    }
    *out = negative ? -result : result;
    return true;
}

bool seeker_parse_response(const char *json, size_t len, seeker_response_t *out)
{
    if (json == NULL || out == NULL || !json_root_object(json, len)) {
        return false;
    }

    out->code = -1;
    out->method[0] = '\0';
    out->msg[0] = '\0';

    seeker_json_value_t value;
    if (json_find_field(json, len, "code", &value)) {
        int code = 0;
        if (json_parse_int(&value, &code)) {
            out->code = code;
        }
    }
    if (json_find_field(json, len, "method", &value)) {
        json_unescape_string(&value, out->method, sizeof(out->method));
    }
    if (json_find_field(json, len, "msg", &value)) {
        json_unescape_string(&value, out->msg, sizeof(out->msg));
    }
    return true;
}

// 取顶层 data 对象;不存在或不是对象时返回 false。
static bool json_get_data_object(const char *json, size_t len,
                                 seeker_json_value_t *data)
{
    seeker_json_value_t value;
    if (!json_find_field(json, len, "data", &value) ||
        value.type != SEEKER_JSON_OBJECT) {
        return false;
    }
    *data = value;
    return true;
}

bool seeker_parse_device_info(const char *json, size_t len,
                              seeker_device_info_t *out)
{
    if (json == NULL || out == NULL || !json_root_object(json, len)) {
        return false;
    }
    memset(out, 0, sizeof(*out));

    seeker_json_value_t data;
    if (!json_get_data_object(json, len, &data)) {
        return false;
    }

    seeker_json_value_t value;
    if (json_find_field(data.start, data.len, "sn", &value)) {
        json_unescape_string(&value, out->sn, sizeof(out->sn));
    }
    if (json_find_field(data.start, data.len, "firmware_version", &value)) {
        json_unescape_string(&value, out->firmware_version,
                             sizeof(out->firmware_version));
    }
    if (json_find_field(data.start, data.len, "system_version", &value)) {
        json_unescape_string(&value, out->system_version,
                             sizeof(out->system_version));
    }
    if (json_find_field(data.start, data.len, "token", &value)) {
        json_unescape_string(&value, out->token, sizeof(out->token));
        out->has_token = out->token[0] != '\0';
    }
    return true;
}

bool seeker_parse_status(const char *json, size_t len, char hawkeye[24])
{
    if (json == NULL || hawkeye == NULL || !json_root_object(json, len)) {
        return false;
    }

    seeker_json_value_t data;
    if (!json_get_data_object(json, len, &data)) {
        return false;
    }
    seeker_json_value_t value;
    if (!json_find_field(data.start, data.len, "hawkeye", &value) ||
        value.type != SEEKER_JSON_STRING) {
        return false;
    }
    json_unescape_string(&value, hawkeye, 24);
    return hawkeye[0] != '\0';
}

bool seeker_parse_battery(const char *json, size_t len, int *percent)
{
    if (json == NULL || percent == NULL || !json_root_object(json, len)) {
        return false;
    }

    seeker_json_value_t data;
    if (!json_get_data_object(json, len, &data)) {
        return false;
    }
    seeker_json_value_t value;
    if (!json_find_field(data.start, data.len, "electricity_quantity", &value)) {
        return false;
    }
    int quantity = 0;
    if (!json_parse_int(&value, &quantity) || quantity < 0 || quantity > 100) {
        return false;
    }
    *percent = quantity;
    return true;
}

// ---------------------------------------------------------------------------
// 请求组包
// ---------------------------------------------------------------------------
//
// 手工拼接,不使用 stdio/snprintf;转义 \" 和 \\,避免把未可信字符串直接注入
// JSON(method/token 来自本固件,仍按防御性方式处理)。

static bool json_append_raw(char *out, size_t cap, size_t *pos, const char *text)
{
    const size_t len = strlen(text);
    if (*pos + len >= cap) {
        return false;
    }
    memcpy(out + *pos, text, len);
    *pos += len;
    return true;
}

static bool json_append_escaped(char *out, size_t cap, size_t *pos,
                                const char *text)
{
    for (const char *p = text; *p != '\0'; p++) {
        const char c = *p;
        const char *escaped = NULL;
        switch (c) {
        case '"':
            escaped = "\\\"";
            break;
        case '\\':
            escaped = "\\\\";
            break;
        case '\b':
            escaped = "\\b";
            break;
        case '\f':
            escaped = "\\f";
            break;
        case '\n':
            escaped = "\\n";
            break;
        case '\r':
            escaped = "\\r";
            break;
        case '\t':
            escaped = "\\t";
            break;
        default:
            break;
        }
        if (escaped != NULL) {
            if (!json_append_raw(out, cap, pos, escaped)) {
                return false;
            }
        } else {
            if (*pos + 1 >= cap) {
                return false;
            }
            out[(*pos)++] = c;
        }
    }
    return true;
}

static bool json_append_uint(char *out, size_t cap, size_t *pos, unsigned value)
{
    char digits[8];
    size_t count = 0;
    if (value == 0) {
        digits[count++] = '0';
    } else {
        while (value > 0 && count < sizeof(digits)) {
            digits[count++] = (char)('0' + (value % 10));
            value /= 10;
        }
    }
    if (*pos + count >= cap) {
        return false;
    }
    for (size_t i = 0; i < count; i++) {
        out[(*pos)++] = digits[count - 1 - i];
    }
    return true;
}

size_t seeker_build_request(char *out, size_t cap, const char *method,
                            const char *access_token)
{
    if (out == NULL || cap == 0 || method == NULL) {
        return 0;
    }

    size_t pos = 0;
    if (!json_append_raw(out, cap, &pos, "{\"method\":\"") ||
        !json_append_escaped(out, cap, &pos, method)) {
        return 0;
    }
    if (access_token != NULL && access_token[0] != '\0') {
        if (!json_append_raw(out, cap, &pos, "\",\"access_token\":\"") ||
            !json_append_escaped(out, cap, &pos, access_token)) {
            return 0;
        }
    }
    if (!json_append_raw(out, cap, &pos, "\"}") || pos + 1 > cap) {
        return 0;
    }
    out[pos] = '\0';
    return pos;
}

// 补包请求:{"method":"get_ble_pkg","requestMethod":<原方法>,"list":[...]}。
// 已知限制(接口未提供 token 参数,头文件不可改):Android 客户端会用原请求的
// args(可能含 access_token)重建补包请求,而本函数只能带 method/requestMethod/list。
// 对当前只读流程可接受:device_info 是首个请求且不带 token,status/battery_status
// 响应都是单包、不会触发补包;因此暂不扩展接口。
size_t seeker_build_repair_request(char *out, size_t cap,
                                   const char *request_method,
                                   const uint16_t *missing,
                                   size_t missing_count)
{
    if (out == NULL || cap == 0 || request_method == NULL ||
        (missing_count > 0 && missing == NULL)) {
        return 0;
    }

    size_t pos = 0;
    if (!json_append_raw(out, cap, &pos, "{\"method\":\"get_ble_pkg\","
                                       "\"requestMethod\":\"") ||
        !json_append_escaped(out, cap, &pos, request_method) ||
        !json_append_raw(out, cap, &pos, "\",\"list\":[")) {
        return 0;
    }
    for (size_t i = 0; i < missing_count; i++) {
        if (i > 0 && !json_append_raw(out, cap, &pos, ",")) {
            return 0;
        }
        if (!json_append_uint(out, cap, &pos, (unsigned)missing[i])) {
            return 0;
        }
    }
    if (!json_append_raw(out, cap, &pos, "]}") || pos + 1 > cap) {
        return 0;
    }
    out[pos] = '\0';
    return pos;
}
