// main/ble_scan_ui.h —— BLE 信号扫描应用的界面层(深色扫描器/雷达主题)。
//
// 与 rest 的分工:
//   * main/ble_scan.c        扫描生命周期与设备表(NimBLE observer);
//   * main/ble_scan_ui.c     列表页/详情页/设备信息页、按键语义、250 ms 刷新;
//   * 本头文件只暴露应用入口,不暴露任何 LVGL 类型。
//
// 线程/锁约定:
//   * LVGL 非线程安全,所有 lv_* 调用都必须处于 LVGL 任务或 bsp_lvgl_lock() 内;
//   * 本模块的 250 ms 刷新定时器由 LVGL 任务驱动,回调内不再加锁;
//   * ble_scan_ui_handle_key() 从应用输入任务调用,内部自行 bsp_lvgl_lock(250);
//   * ble_scan_ui_create()/ble_scan_ui_destroy() 由调用方持锁后调用。
#pragma once

#include "bsp_button.h"

// 创建列表页/详情页/设备信息页三个屏幕、以及 250 ms 刷新定时器,并立即完成首次渲染。
// 调用前提:bsp_display_init() 与 bsp_lvgl_init() 已成功,且【调用方已持有
// bsp_lvgl_lock()】。重复调用会被忽略(返回后仍只有一套屏幕/定时器)。
void ble_scan_ui_create(void);

// 删除刷新定时器与三个屏幕,并释放本模块持有的 LVGL 对象引用。
// 调用前提:【调用方已持有 bsp_lvgl_lock()】。本应用界面常驻,正常流程不会调用;
// 保留给界面重建/退出路径,调用后需重新 ble_scan_ui_create() 才会再出现界面。
void ble_scan_ui_destroy(void);

// 处理一次按键事件,由应用输入任务调用(不得在已持有 LVGL 锁的上下文调用)。
// 内部获取 bsp_lvgl_lock(250),拿不到锁时直接丢弃本次事件。
// 只处理 BSP_BTN_CLICK / BSP_BTN_LONG,其余事件(PRESS/DOUBLE/RELEASE)忽略。
void ble_scan_ui_handle_key(bsp_btn_t btn, bsp_btn_ev_t ev);

// 推送一次电量百分比(0..100;< 0 表示不可用,显示 "--")。
// 由应用主任务在 LVGL 锁之外读好 bsp_battery_soc() 后调用,内部自行加锁刷新
// 两个页面的电量标签,可从任意任务调用。界面上电后约 1 秒内会收到第一次推送。
void ble_scan_ui_set_battery(int percent);
