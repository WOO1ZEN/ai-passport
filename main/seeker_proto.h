// main/seeker_proto.h —— SEEKER 鹰眼的 BLE 分包重组、JSON 字段解析与请求组包(纯 C)。
//
// 真机实测的 BLE 应用层封装(优先于旧文档):
//   每个 notify = 22 字节 ASCII 头 + 正文
//   [16 字符大写 MD5 中段][3 字符包号][3 字符总包数]
//   MD5 中段 = 完整正文 MD5 的大写 hex 第 8..23 个字符;包号从 1 开始;
//   包号/总包数通常为十进制,出现 A-F 时按十六进制解释(真机图片总包数 "1c2" = 450)。
// 若 notify 第一个非空白字节是 '{',则视为未加封装的原始 JSON(模拟器/其他固件)。
//
// JSON 层:{"method":..,"code":1000,"msg":..,"data":{...}};请求不带 id,响应通过
// method 回归匹配。只读目标:device_info / status / battery_status。
//
// 本模块不依赖 ESP-IDF/LVGL,可在主机上单测;GATT 连接与收发在 main/ble_client.c。
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define SEEKER_PACKET_HEADER_LEN 22
#define SEEKER_MAX_PACKETS 64          // 只读响应实测最多 9 包;留余量,超出即报错
#define SEEKER_MAX_BODY 8192           // 设备信息/状态响应远小于此
#define SEEKER_MAX_COMPLETED 4         // 最近完成的 MD5 中段去重窗口
#define SEEKER_CODE_SUCCESS 1000
#define SEEKER_CODE_VERIFY_REQUIRED_1 1002
#define SEEKER_CODE_VERIFY_REQUIRED_2 1004

typedef enum {
    SEEKER_FEED_NONE = 0,   // 收到一个分片,但消息尚未完整
    SEEKER_FEED_COMPLETE,   // 消息已完整且 MD5 校验通过,*out_len 为正文长度
    SEEKER_FEED_ERROR,      // 头/包号/重复包/MD5 校验失败,调用方应断开重连
} seeker_feed_result_t;

typedef struct {
    bool active;                         // 是否有一个未完成的消息批次
    bool hex;                            // 包号/总包数按十六进制解释
    uint16_t total;                      // 总包数
    uint16_t present;                    // 已收到的包数
    char hash[17];                       // 16 位大写 MD5 中段 + NUL
    uint16_t offsets[SEEKER_MAX_PACKETS];// 按包号(1-based)记录的 arena 偏移
    uint16_t lengths[SEEKER_MAX_PACKETS];// 按包号记录的正文长度
    uint8_t arena[SEEKER_MAX_BODY];      // 按到达顺序存放的正文分片
    size_t arena_len;
    char completed[SEEKER_MAX_COMPLETED][17];
    size_t completed_count;
} seeker_assembler_t;

void seeker_assembler_reset(seeker_assembler_t *a);
// 开始一个新请求:清空"最近完成"去重窗口(避免同一连接的迟到响应被误用)。
void seeker_assembler_begin_request(seeker_assembler_t *a);

// 喂一包 notify 数据。完成时把完整正文写入 out(容量不足返回 ERROR)。
seeker_feed_result_t seeker_assembler_feed(seeker_assembler_t *a,
                                           const uint8_t *packet, size_t len,
                                           uint8_t *out, size_t out_cap,
                                           size_t *out_len);

// 当前未完成批次里缺失的包号(升序),最多写 cap 个;返回缺失总数。
size_t seeker_assembler_missing(const seeker_assembler_t *a, uint16_t *out, size_t cap);

// 是否已收到至少一包但消息未完整(用于判断是否要发补包请求)。
bool seeker_assembler_incomplete(const seeker_assembler_t *a);

// ---- JSON 解析(轻量:按键名提取字符串/整数,处理引号与反斜杠转义) ----

typedef struct {
    int code;                 // 缺失时为 -1
    char method[32];
    char msg[48];
} seeker_response_t;

// 解析顶层 code/method/msg。json 必须是完整 UTF-8 JSON 对象。
bool seeker_parse_response(const char *json, size_t len, seeker_response_t *out);

// device_info 的 data 字段;firmware_version=APK 版本,system_version=系统固件版本。
typedef struct {
    char sn[48];
    char firmware_version[40];
    char system_version[40];
    char token[64];
    bool has_token;
} seeker_device_info_t;
bool seeker_parse_device_info(const char *json, size_t len, seeker_device_info_t *out);

// status 的 data.hawkeye 字符串(uninit/ready/ota/running/running_trailing/pause/error)。
bool seeker_parse_status(const char *json, size_t len, char hawkeye[24]);

// battery_status 的 data.electricity_quantity(0..100);缺失返回 false。
bool seeker_parse_battery(const char *json, size_t len, int *percent);

// ---- 请求组包 ----

// {"method":"..."} 或 {"method":"...","access_token":"..."}(token 为 NULL/空时省略)。
size_t seeker_build_request(char *out, size_t cap, const char *method,
                            const char *access_token);

// {"method":"get_ble_pkg","requestMethod":"...","list":[..]} 补包请求。
size_t seeker_build_repair_request(char *out, size_t cap, const char *request_method,
                                   const uint16_t *missing, size_t missing_count);
