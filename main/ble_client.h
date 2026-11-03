// main/ble_client.h —— 设备详情页的"连接并读取信息/状态/版本"服务。
//
// 职责边界:
//   * main/ble_client.c    NimBLE central:连接、GATT 发现、订阅、请求/应答、超时;
//   * main/nova_proto.c    NOVA Bare 协议与认证(纯逻辑);
//   * main/framed_proto.c  OMNI/HALO/PACE 的 P1 Framed 协议(纯逻辑);
//   * main/aura_proto.c    AURA 的 P2 Frame_ID 协议(纯逻辑,无需登录);
//   * main/seeker_proto.c  SEEKER 分包重组/JSON/请求(纯逻辑);
//   * main/ble_scan_ui.c   界面展示。
//
// 并发与生命周期:
//   * ble_client_start() 必须在 NimBLE host 已启动(ble_scan_start 成功)后调用;
//   * ble_client_request_read() 只投递请求,不阻塞;内部任务负责暂停扫描、
//     连接、读取、断开、恢复扫描;
//   * 所有 getter 带内部锁,可从任意任务调用;返回的快照可跨任务使用;
//   * 同一时刻只允许一个读取会话;busy 时新请求被忽略。
#pragma once

#include "ble_scan_model.h"
#include "device_proto.h"
#include "esp_err.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    BLE_CLIENT_IDLE = 0,        // 未连接,也没有进行中的读取
    BLE_CLIENT_CONNECTING,      // 正在建立 GATT 连接
    BLE_CLIENT_DISCOVERING,     // 正在发现服务/特征
    BLE_CLIENT_SUBSCRIBING,     // 正在订阅通知
    BLE_CLIENT_AUTHENTICATING,  // NOVA/OMNI/HALO/PACE 的 0x07/0x08 接入认证
    BLE_CLIENT_READING,         // 正在读取信息/状态
    BLE_CLIENT_DONE,            // 本次读取完成(已断开)
    BLE_CLIENT_FAILED,          // 失败(已断开),last_error/detail 给出原因
    BLE_CLIENT_UNSUPPORTED,     // 该型号暂不支持读取
} ble_client_state_t;

typedef struct {
    ble_client_state_t state;
    device_model_t model;
    char device_name[BLE_SCAN_NAME_MAX];
    bool connected;
    char sn[48];
    // NOVA:固件版本形如 "1.0.0-1.0.0";SEEKER:APK 版本 firmware_version。
    char firmware_version[40];
    // NOVA:硬件版本 hardware_version;SEEKER:系统固件版本 system_version。
    char hardware_version[40];
    bool status_valid;
    uint8_t nova_level;      // NOVA 0x02 状态等级
    uint16_t nova_code;      // NOVA 0x02 状态码
    char seeker_state[24];   // SEEKER data.hawkeye 原始状态字符串
    bool battery_valid;
    uint8_t battery;         // 0..100
    int last_error;          // esp_err_t 或 NimBLE/GATT 返回码
    char detail[64];         // ASCII 诊断文本(串口/界面"详情"用,不含中文)
} ble_client_info_t;

// 创建设备客户端任务。重复调用返回 ESP_ERR_INVALID_STATE。
esp_err_t ble_client_start(void);

// 请求连接 dev 并读取信息/状态:会自行暂停/恢复扫描。同一时刻只允许一个会话,
// busy 时忽略并返回 ESP_ERR_INVALID_STATE。dev 会被复制,调用后可释放。
esp_err_t ble_client_request_read(const ble_scan_dev_t *dev);

// 请求取消当前会话(不阻塞)。无会话时返回 ESP_OK。
esp_err_t ble_client_cancel(void);

// 读取当前快照(线程安全)。
void ble_client_get_info(ble_client_info_t *out);

// 是否有会话在进行中(不含 DONE/FAILED 的静态结果)。
bool ble_client_busy(void);
