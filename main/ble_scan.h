// main/ble_scan.h —— BLE 扫描服务(NimBLE observer)对应用层的接口。
//
// 职责边界:
//   * 本模块只负责 NimBLE 的初始化/扫描生命周期与设备表的线程安全访问;
//   * 广播解析、设备合并、排序、信号等级在 main/ble_scan_model.c(纯逻辑);
//   * 界面渲染在 main/ble_scan_ui.c。
//
// 并发约定:除特别说明外,所有函数都可从任意任务调用;内部持有互斥锁。
// 扫描回调运行在 NimBLE host 任务,不会在持有 LVGL 锁时被调用。
#pragma once

#include "ble_scan_model.h"
#include "esp_err.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    BLE_SCAN_OFF = 0,     // 未初始化(或已 stop)
    BLE_SCAN_STARTING,    // NimBLE host 已启动,等待 sync 后开始扫描
    BLE_SCAN_SCANNING,    // 正在扫描
    BLE_SCAN_PAUSED,      // 扫描被用户暂停,结果表保留
    BLE_SCAN_FINISHED,    // 开机后的 20 秒扫描已结束,结果保留;重启后重新扫描
    BLE_SCAN_FAILED,      // 初始化或扫描失败,last_error 给出原因
} ble_scan_state_t;

// 初始化 NimBLE 并开始扫描。成功返回 ESP_OK 并进入 STARTING/SCANNING。
// 每次 start(即每次开机)只扫描 BLE_SCAN_BOOT_DURATION_MS(20 秒),结束后进入
// BLE_SCAN_FINISHED 并保留结果表;重启应用后重新开始下一轮 20 秒扫描。
// 重复调用或 NVS/内存不足返回错误;失败后状态为 BLE_SCAN_FAILED。
esp_err_t ble_scan_start(void);

// 取消扫描、停止并反初始化 NimBLE host。必须在 BLE 回调之外调用。
// 未启动时返回 ESP_OK;清理失败时保留已分配资源以便重试。
esp_err_t ble_scan_stop(void);

// 暂停扫描(保留已发现设备)。仅 SCANNING 状态下有效。
esp_err_t ble_scan_pause(void);

// 从暂停恢复扫描(重新获得一个完整的 20 秒扫描窗口)。仅 PAUSED / 可恢复的
// FAILED 状态下有效。
esp_err_t ble_scan_resume(void);

// 重新开始一轮 20 秒扫描:清空当前结果表,再从 FINISHED / PAUSED / 可恢复的
// FAILED 状态进入 SCANNING。用于列表页长按"确定"手动重扫,以及读取前发现
// 扫描结果已过期时刷新。SCANNING 状态下返回 ESP_ERR_INVALID_STATE。
esp_err_t ble_scan_restart(void);

ble_scan_state_t ble_scan_state(void);

// 最近一次失败的原因:NimBLE 返回码或 esp_err_t;成功时为 0。
int ble_scan_last_error(void);

// 清空结果表(扫描继续)。可从 UI 任务调用。
void ble_scan_clear(void);

// 取按信号强度排序的设备快照(强→弱),最多 cap 条,返回实际写入条数。
// 内部会先清理过期设备,再复制并排序;输出可以安全地跨任务使用。
size_t ble_scan_snapshot(ble_scan_dev_t *out, size_t cap);

// 当前未过期的设备数量(不排序、不复制设备内容)。
size_t ble_scan_count(void);
