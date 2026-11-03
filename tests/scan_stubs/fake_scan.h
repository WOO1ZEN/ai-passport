// tests/scan_stubs/fake_scan.h —— client 运行时测试用的假 ble_scan API 控制面。
//
// main/ble_client.c 通过 ble_scan.h 读取扫描状态/设备表,并在连接前做过期刷新。
// client 运行时测试不链接真实 main/ble_scan.c(那需要真实 NimBLE 扫描),而是链接
// tests/scan_stubs/fake_scan.c 里的可控实现:测试可以设置扫描状态、往表里放设备、
// 统计 pause/resume/restart 次数,并在 restart 时通过钩子注入"刚扫到的设备"。
#pragma once

#include "ble_scan.h"

// 清空假扫描器状态、设备表与全部计数(不影响桩的 GATT/任务状态)。
void test_fake_scan_reset(void);

// 直接设置/读取扫描状态(用于构造 FINISHED/PAUSED/FAILED/SCANNING 场景)。
void test_fake_scan_set_state(ble_scan_state_t state);
ble_scan_state_t test_fake_scan_get_state(void);

// 往设备表里加一条(同 addr+type 覆盖);last_seen_ms 由测试控制。
void test_fake_scan_add_dev(const ble_scan_dev_t *dev);
size_t test_fake_scan_dev_count(void);

// restart 行为注入:非 0 时 restart 失败(状态 FAILED、last_error=该值)。
extern int test_fake_scan_restart_result;
// restart 成功时是否清空设备表(默认 true,模拟"新一轮从空表开始")。
extern bool test_fake_scan_clear_on_restart;
// restart 成功、清空表之后调用,用于注入"刚扫到的目标设备"。
extern void (*test_fake_scan_restart_hook)(void);

// 调用计数。
extern unsigned test_fake_scan_restart_calls;
extern unsigned test_fake_scan_pause_calls;
extern unsigned test_fake_scan_resume_calls;
