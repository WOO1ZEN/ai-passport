// main/device_proto.h —— 设备型号识别与"可读取信息"能力判定。
//
// 只做名称分类,不依赖 ESP-IDF/LVGL;扫描结果里的设备名决定了后续走哪条协议。
// 名称分类只是"选择协议家族"的第一步,真正的设备身份还要由 GATT 发现成功来确认。
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    DEVICE_MODEL_UNKNOWN = 0,
    DEVICE_MODEL_NOVA,     // NOVA_ 前缀:NOVA Bare 协议
    DEVICE_MODEL_SEEKER,   // SEEKER / FUTUREMIND SEEKER 前缀:JSON over BLE
    DEVICE_MODEL_OMNI,     // OMNI_ / OMNI- 前缀:P1 Framed + 0x07/0x08 登录
    DEVICE_MODEL_HALO,     // HALO_ / HALO- 前缀:P1 Framed + 0x07/0x08 登录
    DEVICE_MODEL_AURA,     // AURA_ / P2_ 前缀:P2 Frame_ID,无需登录,直接读信息
    DEVICE_MODEL_PACE,     // PACE 前缀:P1 Framed + 0x07/0x08 登录
} device_model_t;

// 名称分类。规则与 Android SDK 一致:先 SEEKER(不区分大小写),再发球机前缀。
device_model_t device_model_from_name(const char *name);

// 型号短名:"NOVA"/"SEEKER"/"OMNI"/"HALO"/"AURA"/"PACE"/"UNKNOWN"。
const char *device_model_name(device_model_t model);

// 当前固件是否已实现"连接并读取信息/状态/版本"。
bool device_model_readable(device_model_t model);

// 通用接入认证私钥算法(NOVA/OMNI/HALO/PACE 的 0x07/0x08 与 SEEKER verify 共用):
//   key = lowercase_hex(md5(sn || salted_sn || code)),salted_sn[i] = SALT[sn[i] % 36]
// 输出 32 字节小写 hex + NUL。SN 最长 120 字节,动态码 4..10 字节;越界返回 false。
#define DEVICE_AUTH_SALT "Mjgx1jAwXDBaMFcxCz3JBgNVBAYT4kJF7Rkw"
#define DEVICE_AUTH_SALT_LEN 36
bool device_auth_make_private_key(const uint8_t *sn, size_t sn_len,
                                  const uint8_t *code, size_t code_len,
                                  char out_hex[33]);
