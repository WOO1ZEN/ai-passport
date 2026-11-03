// main/device_proto.c —— 设备型号识别与接入认证私钥算法的纯 C 实现。
//
// 分类规则来自 Android SDK(ble_control/.../device/DeviceCatalog.kt 与
// DeviceProfiles.kt):先按 SEEKER 前缀(去掉首尾空白、忽略大小写)判定,
// 再依次尝试发球机前缀。当前六款型号都实现了只读信息/状态读取:NOVA=Bare、
// OMNI/HALO/PACE=P1 Framed、AURA(AURA_/P2_)=P2 Frame_ID、SEEKER=JSON。
//
// 私钥算法与 NOVA 0x07/0x08、SEEKER verify 完全一致:
//   key = lowercase_hex(md5(sn || salted_sn || code)),
//   salted_sn[i] = SALT[ (uint8_t)sn[i] % 36 ]
// 注意 sn[i] 必须按无符号字节取模(Android C++ 参考实现的有符号 char 是已知缺陷)。
#include "device_proto.h"

#include <string.h>

#include "md5.h"

// 与 main/nova_proto.h / CheckCodeAuthenticator.kt 中的约束保持一致。
#define DEVICE_AUTH_MAX_SN_LEN 120
#define DEVICE_AUTH_MIN_CODE_LEN 4
#define DEVICE_AUTH_MAX_CODE_LEN 10

// 36 字节 ASCII 盐(数组多留 1 字节给字符串结尾 NUL);长度由头文件常量固定。
static const uint8_t DEVICE_AUTH_SALT_BYTES[DEVICE_AUTH_SALT_LEN + 1] =
    "Mjgx1jAwXDBaMFcxCz3JBgNVBAYT4kJF7Rkw";

// 不区分大小写地判断 name 是否以 prefix 开头(仅 ASCII 大小写折叠)。
static bool ascii_starts_with_ignore_case(const char *name, const char *prefix)
{
    if (name == NULL || prefix == NULL) {
        return false;
    }
    while (*prefix != '\0') {
        char a = *name++;
        if (a == '\0') {
            return false;
        }
        if (a >= 'A' && a <= 'Z') {
            a = (char)(a - 'A' + 'a');
        }
        char b = *prefix++;
        if (b >= 'A' && b <= 'Z') {
            b = (char)(b - 'A' + 'a');
        }
        if (a != b) {
            return false;
        }
    }
    return true;
}

// 跳过首尾空白后的起始位置;找不到非空白内容时返回 NULL。
static const char *skip_leading_space(const char *s)
{
    while (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n') {
        s++;
    }
    return (*s == '\0') ? NULL : s;
}

// 计算去掉首尾空白后的名称长度(用于 SEEKER 前缀匹配,避免 " seeker01" 漏判)。
static size_t trimmed_len(const char *s)
{
    size_t end = strlen(s);
    while (end > 0) {
        const char c = s[end - 1];
        if (c != ' ' && c != '\t' && c != '\r' && c != '\n') {
            break;
        }
        end--;
    }
    return end;
}

static bool starts_with_prefix_trimmed(const char *name, const char *prefix)
{
    const char *start = skip_leading_space(name);
    if (start == NULL) {
        return false;
    }
    const size_t len = trimmed_len(start);
    const size_t prefix_len = strlen(prefix);
    if (len < prefix_len) {
        return false;
    }
    // 复制到定长小缓冲后复用统一的大小写不敏感比较,避免改写 source 指针。
    char head[32];
    if (prefix_len >= sizeof(head)) {
        return false;
    }
    memcpy(head, start, prefix_len);
    head[prefix_len] = '\0';
    return ascii_starts_with_ignore_case(head, prefix);
}

device_model_t device_model_from_name(const char *name)
{
    if (name == NULL || *name == '\0') {
        return DEVICE_MODEL_UNKNOWN;
    }

    // 顺序与 Android SDK 一致:SEEKER 优先(仅这一步按 trim 后匹配)。
    if (starts_with_prefix_trimmed(name, "SEEKER") ||
        starts_with_prefix_trimmed(name, "FUTUREMIND SEEKER")) {
        return DEVICE_MODEL_SEEKER;
    }

    // 发球机家族:直接按原始名称做忽略大小写前缀匹配。
    if (ascii_starts_with_ignore_case(name, "NOVA_")) {
        return DEVICE_MODEL_NOVA;
    }
    if (ascii_starts_with_ignore_case(name, "OMNI_") ||
        ascii_starts_with_ignore_case(name, "OMNI-")) {
        return DEVICE_MODEL_OMNI;
    }
    if (ascii_starts_with_ignore_case(name, "HALO_") ||
        ascii_starts_with_ignore_case(name, "HALO-")) {
        return DEVICE_MODEL_HALO;
    }
    // AURA 的 P2 线格式;部分固件广播名为 "P2_<SN>"(参考项目文档示例),统一按
    // AURA 处理,否则会落到 UNKNOWN 而永远读不到。
    if (ascii_starts_with_ignore_case(name, "AURA_") ||
        ascii_starts_with_ignore_case(name, "P2_")) {
        return DEVICE_MODEL_AURA;
    }
    // PACE 在 Android SDK 中没有注册适配器;这里保留型号枚举以便 UI 区分,
    // 只读实现按 P1 Framed(与 OMNI/HALO 同族)读取信息/状态。
    if (ascii_starts_with_ignore_case(name, "PACE")) {
        return DEVICE_MODEL_PACE;
    }
    return DEVICE_MODEL_UNKNOWN;
}

const char *device_model_name(device_model_t model)
{
    switch (model) {
    case DEVICE_MODEL_NOVA:
        return "NOVA";
    case DEVICE_MODEL_SEEKER:
        return "SEEKER";
    case DEVICE_MODEL_OMNI:
        return "OMNI";
    case DEVICE_MODEL_HALO:
        return "HALO";
    case DEVICE_MODEL_AURA:
        return "AURA";
    case DEVICE_MODEL_PACE:
        return "PACE";
    case DEVICE_MODEL_UNKNOWN:
    default:
        return "UNKNOWN";
    }
}

bool device_model_readable(device_model_t model)
{
    // 六款型号都实现了只读信息/状态读取:
    // NOVA=Bare、SEEKER=JSON、OMNI/HALO/PACE=P1 Framed、AURA=P2 Frame_ID。
    return model == DEVICE_MODEL_NOVA || model == DEVICE_MODEL_SEEKER ||
           model == DEVICE_MODEL_OMNI || model == DEVICE_MODEL_HALO ||
           model == DEVICE_MODEL_AURA || model == DEVICE_MODEL_PACE;
}

bool device_auth_make_private_key(const uint8_t *sn, size_t sn_len,
                                  const uint8_t *code, size_t code_len,
                                  char out_hex[33])
{
    if (out_hex == NULL) {
        return false;
    }
    if (sn_len > DEVICE_AUTH_MAX_SN_LEN) {
        return false;
    }
    if (code == NULL || code_len < DEVICE_AUTH_MIN_CODE_LEN ||
        code_len > DEVICE_AUTH_MAX_CODE_LEN) {
        return false;
    }
    if (sn_len > 0 && sn == NULL) {
        return false;
    }

    uint8_t salted[DEVICE_AUTH_MAX_SN_LEN];
    for (size_t i = 0; i < sn_len; i++) {
        // 关键:按无符号字节取模,不能使用有符号 char。
        salted[i] = DEVICE_AUTH_SALT_BYTES[(uint8_t)sn[i] % DEVICE_AUTH_SALT_LEN];
    }

    md5_ctx_t ctx;
    uint8_t digest[16];
    md5_init(&ctx);
    md5_update(&ctx, sn, sn_len);
    md5_update(&ctx, salted, sn_len);
    md5_update(&ctx, code, code_len);
    md5_final(&ctx, digest);
    md5_digest_hex_lower(digest, out_hex);
    return true;
}
