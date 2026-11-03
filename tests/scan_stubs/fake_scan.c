// tests/scan_stubs/fake_scan.c —— client 运行时测试用的假 ble_scan 实现。
//
// 只提供 main/ble_client.c 会调用的扫描 API(state/pause/resume/restart/snapshot/
// count/last_error/clear/start/stop + format_addr 由真实 ble_scan_model.c 提供)。
// 行为刻意保持简单、确定:设备表按插入顺序保存,不做过期清理(测试自己控制
// last_seen_ms);pause/resume 计数同时累加桩里的 test_disc_cancel_calls /
// test_disc_calls,这样既有的"扫描恰好暂停/恢复一次"断言无需改写语义。
#include "scan_test_stubs.h"
#include "fake_scan.h"

#include <assert.h>
#include <string.h>

static ble_scan_dev_t s_devs[BLE_SCAN_TABLE_CAP];
static size_t s_dev_count;
static ble_scan_state_t s_state = BLE_SCAN_OFF;
static int s_error;

int test_fake_scan_restart_result;
bool test_fake_scan_clear_on_restart = true;
void (*test_fake_scan_restart_hook)(void);
unsigned test_fake_scan_restart_calls;
unsigned test_fake_scan_pause_calls;
unsigned test_fake_scan_resume_calls;

void test_fake_scan_reset(void)
{
    memset(s_devs, 0, sizeof(s_devs));
    s_dev_count = 0;
    s_state = BLE_SCAN_OFF;
    s_error = 0;
    test_fake_scan_restart_result = 0;
    test_fake_scan_clear_on_restart = true;
    test_fake_scan_restart_hook = NULL;
    test_fake_scan_restart_calls = 0;
    test_fake_scan_pause_calls = 0;
    test_fake_scan_resume_calls = 0;
}

void test_fake_scan_set_state(ble_scan_state_t state)
{
    s_state = state;
}

ble_scan_state_t test_fake_scan_get_state(void)
{
    return s_state;
}

void test_fake_scan_add_dev(const ble_scan_dev_t *dev)
{
    assert(dev != NULL);
    for (size_t i = 0; i < s_dev_count; i++) {
        if (s_devs[i].addr_type == dev->addr_type &&
            memcmp(s_devs[i].addr, dev->addr, sizeof(dev->addr)) == 0) {
            s_devs[i] = *dev;
            return;
        }
    }
    assert(s_dev_count < BLE_SCAN_TABLE_CAP);
    s_devs[s_dev_count++] = *dev;
}

size_t test_fake_scan_dev_count(void)
{
    return s_dev_count;
}

esp_err_t ble_scan_start(void)
{
    s_state = BLE_SCAN_SCANNING;
    s_error = 0;
    return ESP_OK;
}

esp_err_t ble_scan_stop(void)
{
    s_state = BLE_SCAN_OFF;
    return ESP_OK;
}

esp_err_t ble_scan_pause(void)
{
    if (s_state != BLE_SCAN_SCANNING) {
        s_error = ESP_ERR_INVALID_STATE;
        return ESP_ERR_INVALID_STATE;
    }
    s_state = BLE_SCAN_PAUSED;
    test_fake_scan_pause_calls++;
    test_disc_cancel_calls++;  // 与真实实现的"pause -> disc cancel"对齐
    return ESP_OK;
}

esp_err_t ble_scan_resume(void)
{
    if (s_state != BLE_SCAN_PAUSED) {
        s_error = ESP_ERR_INVALID_STATE;
        return ESP_ERR_INVALID_STATE;
    }
    s_state = BLE_SCAN_SCANNING;
    test_fake_scan_resume_calls++;
    test_disc_calls++;  // 与真实实现的"resume -> 新 discovery"对齐
    return ESP_OK;
}

esp_err_t ble_scan_restart(void)
{
    test_fake_scan_restart_calls++;
    if (s_state != BLE_SCAN_FINISHED && s_state != BLE_SCAN_PAUSED &&
        s_state != BLE_SCAN_FAILED) {
        s_error = ESP_ERR_INVALID_STATE;
        return ESP_ERR_INVALID_STATE;
    }
    if (test_fake_scan_restart_result != 0) {
        s_error = test_fake_scan_restart_result;
        s_state = BLE_SCAN_FAILED;
        return ESP_FAIL;
    }
    if (test_fake_scan_clear_on_restart) {
        memset(s_devs, 0, sizeof(s_devs));
        s_dev_count = 0;
    }
    s_state = BLE_SCAN_SCANNING;
    s_error = 0;
    if (test_fake_scan_restart_hook != NULL) test_fake_scan_restart_hook();
    return ESP_OK;
}

ble_scan_state_t ble_scan_state(void)
{
    return s_state;
}

int ble_scan_last_error(void)
{
    return s_error;
}

void ble_scan_clear(void)
{
    memset(s_devs, 0, sizeof(s_devs));
    s_dev_count = 0;
}

size_t ble_scan_snapshot(ble_scan_dev_t *out, size_t cap)
{
    if (out == NULL || cap == 0) return 0;
    size_t n = s_dev_count < cap ? s_dev_count : cap;
    for (size_t i = 0; i < n; i++) out[i] = s_devs[i];
    return n;
}

size_t ble_scan_count(void)
{
    return s_dev_count;
}
