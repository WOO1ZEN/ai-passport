// tests/test_ble_scan_runtime.c —— BLE 扫描服务生命周期/状态机的主机运行时测试。
//
// 做法与 tests/test_demo_ble_runtime.c 相同:把 tests/scan_stubs 的平台桩和
// main/ble_scan.c(以及它依赖的纯逻辑层 main/ble_scan_model.c)直接包含进来,
// 在主机上执行真实的状态机代码。NimBLE host 任务、sync/DISC 回调和时钟都由
// 测试手动驱动,因此不需要硬件、也不会真正并发。
#include "scan_stubs/scan_runtime.c"
#include "../main/ble_scan_model.c"
#include "../main/ble_scan.c"

#include <assert.h>
#include <stdio.h>
#include <string.h>

// ---------------------------------------------------------------------------
// 测试辅助
// ---------------------------------------------------------------------------
static void set_time_ms(uint32_t ms)
{
    test_time_us = (int64_t)ms * 1000;
}

static void addr_with_id(uint8_t addr[6], uint8_t id)
{
    memset(addr, 0, 6);
    addr[0] = id;
}

// 组一段广播 payload:flags + 可选 TX power + 可选完整设备名。
static uint8_t build_adv(uint8_t *buf, const char *name, int tx_power)
{
    uint8_t n = 0;

    buf[n++] = 0x02;
    buf[n++] = 0x01;  // Flags
    buf[n++] = 0x06;

    if (tx_power != BLE_SCAN_RSSI_UNKNOWN) {
        buf[n++] = 0x02;
        buf[n++] = 0x0A;  // TX power
        buf[n++] = (uint8_t)(int8_t)tx_power;
    }
    if (name != NULL && name[0] != '\0') {
        const size_t len = strlen(name);
        assert(len < 24);
        buf[n++] = (uint8_t)(1 + len);
        buf[n++] = 0x09;  // Complete local name
        memcpy(&buf[n], name, len);
        n += (uint8_t)len;
    }
    return n;
}

static void fire_adv(const uint8_t addr[6], uint8_t addr_type, int8_t rssi,
                     uint8_t event_type, const uint8_t *data, uint8_t len)
{
    struct ble_gap_disc_desc desc;

    memset(&desc, 0, sizeof(desc));
    memcpy(desc.addr.val, addr, sizeof(desc.addr.val));
    desc.addr.type = addr_type;
    desc.rssi = rssi;
    desc.event_type = event_type;
    desc.data = data;
    desc.length_data = len;
    test_gap_fire_disc(&desc);
}

// 启动一个完整会话:start 成功 -> 模拟 sync -> 进入 SCANNING。
static void start_scanning(void)
{
    const unsigned disc_before = test_disc_calls;

    assert(ble_scan_start() == ESP_OK);
    assert(ble_scan_state() == BLE_SCAN_STARTING);
    assert(s_initialized);
    assert(s_host_task != NULL);
    assert(test_disc_calls == disc_before);  // discovery 只在 sync 回调里启动
    test_gap_fire_sync();
    assert(ble_scan_state() == BLE_SCAN_SCANNING);
    assert(test_disc_calls == disc_before + 1);
}

// 结束会话:先让 host 任务跑到停机点给出 stopped 信号,再正常 stop。
static void stop_session(void)
{
    assert(s_initialized);
    assert(s_host_task != NULL);
    test_run_worker(s_host_task);
    assert(ble_scan_stop() == ESP_OK);
    assert(ble_scan_state() == BLE_SCAN_OFF);
    assert(!s_initialized);
    assert(s_host_task == NULL);
}

int main(void)
{
    ble_scan_dev_t snap[BLE_SCAN_TABLE_CAP];
    uint8_t addr[6];
    uint8_t adv[32];
    uint8_t n;
    size_t count;
    TaskHandle_t task;

    // ---- 未启动时的接口安全性与错误码 ----
    assert(ble_scan_state() == BLE_SCAN_OFF);
    assert(ble_scan_last_error() == 0);
    assert(ble_scan_stop() == ESP_OK);  // 未启动时 stop 是空操作
    assert(ble_scan_state() == BLE_SCAN_OFF);
    assert(ble_scan_pause() == ESP_ERR_INVALID_STATE);
    assert(ble_scan_resume() == ESP_ERR_INVALID_STATE);
    assert(ble_scan_last_error() == ESP_ERR_INVALID_STATE);
    assert(ble_scan_snapshot(snap, BLE_SCAN_TABLE_CAP) == 0);
    assert(ble_scan_snapshot(NULL, 4) == 0);
    assert(ble_scan_snapshot(snap, 0) == 0);
    assert(ble_scan_count() == 0);
    ble_scan_clear();  // 锁尚未创建,必须安全返回
    assert(ble_scan_count() == 0);

    // ---- start() 失败路径 ----
    // 1) 互斥锁分配失败:状态 FAILED,句柄不保留。
    test_mutex_fails = true;
    assert(ble_scan_start() == ESP_ERR_NO_MEM);
    test_mutex_fails = false;
    assert(ble_scan_state() == BLE_SCAN_FAILED);
    assert(ble_scan_last_error() == ESP_ERR_NO_MEM);
    assert(s_lock == NULL);
    assert(!s_initialized && s_host_task == NULL);
    assert(test_nvs_calls == 0);

    // 2) NVS 初始化失败:不擦除、不初始化 NimBLE,状态 FAILED,可重试。
    test_nvs_result = ESP_FAIL;
    assert(ble_scan_start() == ESP_FAIL);
    test_nvs_result = ESP_OK;
    assert(ble_scan_state() == BLE_SCAN_FAILED);
    assert(ble_scan_last_error() == ESP_FAIL);
    assert(!s_initialized && s_host_task == NULL);
    assert(test_nimble_inits == 0);
    assert(test_nvs_calls == 1);

    // 3) nimble_port_init 失败:资源不保留,状态 FAILED。
    test_nimble_init_result = ESP_FAIL;
    assert(ble_scan_start() == ESP_FAIL);
    test_nimble_init_result = ESP_OK;
    assert(ble_scan_state() == BLE_SCAN_FAILED);
    assert(ble_scan_last_error() == ESP_FAIL);
    assert(!s_initialized && s_host_task == NULL);
    assert(test_nimble_inits == 1);
    assert(test_nimble_deinits == 0);

    // 4) host 停止信号量分配失败:failed_start 走 stop 清理,nimble 被 deinit。
    test_sem_fails = true;
    assert(ble_scan_start() == ESP_ERR_NO_MEM);
    test_sem_fails = false;
    assert(ble_scan_state() == BLE_SCAN_FAILED);
    assert(ble_scan_last_error() == ESP_ERR_NO_MEM);
    assert(!s_initialized && s_host_task == NULL);
    assert(s_host_stopped == NULL);
    assert(test_nimble_inits == 2);
    assert(test_nimble_deinits == 1);

    // 5) host 任务创建失败:同样清理并保留可重试状态。
    unsigned deinits_before = test_nimble_deinits;
    test_create_fails = true;
    assert(ble_scan_start() == ESP_ERR_NO_MEM);
    test_create_fails = false;
    assert(ble_scan_state() == BLE_SCAN_FAILED);
    assert(ble_scan_last_error() == ESP_ERR_NO_MEM);
    assert(!s_initialized && s_host_task == NULL);
    assert(test_nimble_deinits == deinits_before + 1);

    // ---- start() 成功:STARTING,重复 start 被拒绝且不破坏会话 ----
    set_time_ms(1000);
    assert(ble_scan_start() == ESP_OK);
    assert(ble_scan_state() == BLE_SCAN_STARTING);
    assert(ble_scan_last_error() == 0);
    assert(s_initialized && s_host_task != NULL && s_host_stopped != NULL);
    assert(ble_scan_start() == ESP_ERR_INVALID_STATE);
    assert(ble_scan_state() == BLE_SCAN_STARTING);
    assert(ble_scan_last_error() == ESP_ERR_INVALID_STATE);
    // STARTING(host 还没 sync)也不能 restart,且不会启动 discovery。
    assert(ble_scan_restart() == ESP_ERR_INVALID_STATE);
    assert(test_disc_calls == 0);

    // sync 回调进入 SCANNING,并以约定的参数启动主动扫描。
    test_gap_fire_sync();
    assert(ble_scan_state() == BLE_SCAN_SCANNING);
    assert(test_disc_calls == 1);
    assert(test_disc_own_addr_type == 0);
    // 每次开机只扫描 20 秒(有限时长),到点后 NIC 发 DISC_COMPLETE。
    assert(test_disc_duration_ms == 20000);
    // 电池设备:~50% 占空比扫描(interval=FAST_MAX 60ms,window=FAST_WINDOW 30ms)。
    assert(test_disc_params.itvl == BLE_GAP_SCAN_FAST_INTERVAL_MAX);
    assert(test_disc_params.window == BLE_GAP_SCAN_FAST_WINDOW);
    assert(test_disc_params.passive == 0);             // 主动扫描,拿扫描响应里的名字
    assert(test_disc_params.filter_duplicates == 0);   // 不去重,累计广播次数
    assert(test_disc_params.filter_policy == 0);
    assert(test_disc_params.limited == 0);
    assert(test_disc_params.disable_observer_mode == 0);

    // ---- DISC 合并:解析、去重计数、排序 ----
    addr_with_id(addr, 0x11);
    n = build_adv(adv, "Dev1", -8);
    fire_adv(addr, 0, -80, BLE_HCI_ADV_RPT_EVTYPE_ADV_IND, adv, n);
    assert(ble_scan_count() == 1);
    count = ble_scan_snapshot(snap, BLE_SCAN_TABLE_CAP);
    assert(count == 1);
    assert(snap[0].rssi == -80 && snap[0].rssi_ema == -80);
    assert(snap[0].tx_power == -8);
    assert(snap[0].adv_flags == 0x06);
    assert(snap[0].connectable);
    assert(strcmp(snap[0].name, "Dev1") == 0);
    assert(snap[0].first_seen_ms == 1000 && snap[0].last_seen_ms == 1000);
    assert(snap[0].seen_count == 1);

    set_time_ms(2000);
    addr_with_id(addr, 0x22);
    n = build_adv(adv, "Dev2", BLE_SCAN_RSSI_UNKNOWN);
    fire_adv(addr, 0, -50, BLE_HCI_ADV_RPT_EVTYPE_ADV_IND, adv, n);

    set_time_ms(3000);
    addr_with_id(addr, 0x33);
    n = build_adv(adv, "Dev3", BLE_SCAN_RSSI_UNKNOWN);
    fire_adv(addr, 0, -60, BLE_HCI_ADV_RPT_EVTYPE_SCAN_RSP, adv, n);

    count = ble_scan_snapshot(snap, BLE_SCAN_TABLE_CAP);
    assert(count == 3);
    assert(strcmp(snap[0].name, "Dev2") == 0 && snap[0].rssi == -50);  // 信号最强在前
    assert(strcmp(snap[1].name, "Dev3") == 0 && snap[1].rssi == -60);
    assert(strcmp(snap[2].name, "Dev1") == 0 && snap[2].rssi == -80);
    assert(!snap[1].connectable);  // 扫描响应本身不代表可连接
    assert(snap[0].connectable);

    // 扫描响应更新已有设备:EMA 平滑、名称/可连接/TX 保持,计数累加。
    set_time_ms(4000);
    addr_with_id(addr, 0x11);
    n = build_adv(adv, NULL, BLE_SCAN_RSSI_UNKNOWN);
    fire_adv(addr, 0, -70, BLE_HCI_ADV_RPT_EVTYPE_SCAN_RSP, adv, n);
    count = ble_scan_snapshot(snap, BLE_SCAN_TABLE_CAP);
    assert(count == 3);
    assert(strcmp(snap[2].name, "Dev1") == 0);
    assert(snap[2].rssi == -70);
    assert(snap[2].rssi_ema == -77);  // (-80*3 + -70)/4
    assert(snap[2].connectable);      // 只做 OR,不降级
    assert(snap[2].tx_power == -8);
    assert(snap[2].seen_count == 2);
    assert(snap[2].last_seen_ms == 4000 && snap[2].first_seen_ms == 1000);

    // ---- pause:取消扫描,暂停后到达的 DISC 不写入 ----
    unsigned cancels_before = test_disc_cancel_calls;
    assert(ble_scan_pause() == ESP_OK);
    assert(ble_scan_state() == BLE_SCAN_PAUSED);
    assert(test_disc_cancel_calls == cancels_before + 1);
    assert(ble_scan_pause() == ESP_ERR_INVALID_STATE);  // 已在暂停
    assert(ble_scan_last_error() == ESP_ERR_INVALID_STATE);

    set_time_ms(5000);
    addr_with_id(addr, 0x44);
    n = build_adv(adv, "Dev4", BLE_SCAN_RSSI_UNKNOWN);
    fire_adv(addr, 0, -65, BLE_HCI_ADV_RPT_EVTYPE_ADV_IND, adv, n);
    assert(ble_scan_count() == 3);  // 暂停期间的新设备不得进入结果表

    // PAUSED 下 DISC_COMPLETE 不重启扫描。
    unsigned disc_before = test_disc_calls;
    test_gap_fire_disc_complete();
    assert(test_disc_calls == disc_before);
    assert(ble_scan_state() == BLE_SCAN_PAUSED);

    // ---- resume:重新开始扫描,后续 DISC 正常合并 ----
    assert(ble_scan_resume() == ESP_OK);
    assert(ble_scan_state() == BLE_SCAN_SCANNING);
    assert(test_disc_calls == disc_before + 1);
    assert(ble_scan_resume() == ESP_ERR_INVALID_STATE);  // 已在扫描

    set_time_ms(5000);
    fire_adv(addr, 0, -65, BLE_HCI_ADV_RPT_EVTYPE_ADV_IND, adv, n);
    assert(ble_scan_count() == 4);

    // ---- 快照 / 计数遵守 BLE_SCAN_STALE_MS 过期规则 ----
    // t=18000:Dev2(2000) 与 Dev3(3000) 的差值为 16000/15000,已过期;
    // Dev1(4000) 与 Dev4(5000) 仍有效。
    set_time_ms(18000);
    assert(ble_scan_count() == 2);
    count = ble_scan_snapshot(snap, BLE_SCAN_TABLE_CAP);
    assert(count == 2);
    assert(strcmp(snap[0].name, "Dev4") == 0 && snap[0].rssi == -65);
    assert(strcmp(snap[1].name, "Dev1") == 0 && snap[1].rssi == -70);
    // 快照不会修改表内容(再次读取结果一致)。
    assert(ble_scan_snapshot(snap, 1) == 1 && snap[0].rssi == -65);

    // t=20000:剩下两条的差值也 >= 15000,全部过期。
    set_time_ms(20000);
    assert(ble_scan_count() == 0);
    assert(ble_scan_snapshot(snap, BLE_SCAN_TABLE_CAP) == 0);

    // ---- clear:扫描继续,只清空结果表 ----
    set_time_ms(20000);
    addr_with_id(addr, 0x55);
    n = build_adv(adv, "Dev5", BLE_SCAN_RSSI_UNKNOWN);
    fire_adv(addr, 0, -55, BLE_HCI_ADV_RPT_EVTYPE_ADV_IND, adv, n);
    set_time_ms(20100);
    addr_with_id(addr, 0x66);
    n = build_adv(adv, "Dev6", BLE_SCAN_RSSI_UNKNOWN);
    fire_adv(addr, 0, -58, BLE_HCI_ADV_RPT_EVTYPE_ADV_IND, adv, n);
    assert(ble_scan_count() == 2);
    assert(ble_scan_state() == BLE_SCAN_SCANNING);
    ble_scan_clear();
    assert(ble_scan_count() == 0);
    assert(ble_scan_snapshot(snap, BLE_SCAN_TABLE_CAP) == 0);
    // cap 超过表容量时按容量截断。
    assert(ble_scan_snapshot(snap, BLE_SCAN_TABLE_CAP + 100) == 0);

    // ---- 20 秒扫描到点:进入 FINISHED,不自动续扫,结果保留且不再过期 ----
    set_time_ms(30000);
    addr_with_id(addr, 0x77);
    n = build_adv(adv, "Dev7", BLE_SCAN_RSSI_UNKNOWN);
    fire_adv(addr, 0, -42, BLE_HCI_ADV_RPT_EVTYPE_ADV_IND, adv, n);
    assert(ble_scan_count() == 1);
    disc_before = test_disc_calls;
    test_gap_fire_disc_complete();
    assert(ble_scan_state() == BLE_SCAN_FINISHED);
    assert(test_disc_calls == disc_before);          // 完成后不再重启 discovery
    assert(ble_scan_count() == 1);

    // 完成 5 分钟后结果仍保留(不再按 BLE_SCAN_STALE_MS 过期),仍可浏览/读取。
    set_time_ms(330000);
    assert(ble_scan_count() == 1);
    count = ble_scan_snapshot(snap, BLE_SCAN_TABLE_CAP);
    assert(count == 1);
    assert(strcmp(snap[0].name, "Dev7") == 0 && snap[0].rssi == -42);

    // 完成后的广播不再写入;重复 DISC_COMPLETE 保持 FINISHED;不能 pause/start。
    fire_adv(addr, 0, -40, BLE_HCI_ADV_RPT_EVTYPE_ADV_IND, adv, n);
    assert(ble_scan_count() == 1 && snap[0].seen_count == 1);
    test_gap_fire_disc_complete();
    assert(ble_scan_state() == BLE_SCAN_FINISHED);
    assert(test_disc_calls == disc_before);
    assert(ble_scan_pause() == ESP_ERR_INVALID_STATE);
    assert(ble_scan_start() == ESP_ERR_INVALID_STATE);  // 重启应用才会重新扫描

    // ---- ble_scan_restart():FINISHED -> 清空旧结果并重新开始 20 秒扫描 ----
    assert(ble_scan_count() == 1);  // Dev7 在 FINISHED 下仍保留
    unsigned restarts_before = test_disc_calls;
    assert(ble_scan_restart() == ESP_OK);
    assert(ble_scan_state() == BLE_SCAN_SCANNING);
    assert(ble_scan_count() == 0);                 // 新一轮从空表开始
    assert(test_disc_calls == restarts_before + 1);  // 恰好一次新 discovery
    assert(test_disc_duration_ms == 20000);        // 仍然是 20 秒窗口
    assert(ble_scan_last_error() == 0);

    // SCANNING 下 restart 被拒绝,且不会多发一次 discovery。
    restarts_before = test_disc_calls;
    assert(ble_scan_restart() == ESP_ERR_INVALID_STATE);
    assert(test_disc_calls == restarts_before);
    assert(ble_scan_state() == BLE_SCAN_SCANNING);

    // PAUSED 下也可以 restart:先暂停并保留结果,再 restart 清表重扫。
    fire_adv(addr, 0, -44, BLE_HCI_ADV_RPT_EVTYPE_ADV_IND, adv, n);
    assert(ble_scan_count() == 1);
    assert(ble_scan_pause() == ESP_OK);
    assert(ble_scan_state() == BLE_SCAN_PAUSED);
    assert(ble_scan_count() == 1);  // PAUSED 保留结果
    restarts_before = test_disc_calls;
    assert(ble_scan_restart() == ESP_OK);
    assert(ble_scan_state() == BLE_SCAN_SCANNING);
    assert(ble_scan_count() == 0);
    assert(test_disc_calls == restarts_before + 1);
    assert(test_disc_duration_ms == 20000);

    // restart 启动 discovery 失败 -> FAILED + last_error;随后可恢复重启。
    assert(ble_scan_pause() == ESP_OK);  // SCANNING -> PAUSED,restart 才合法
    test_disc_result = BLE_HS_ENOMEM;
    assert(ble_scan_restart() == ESP_FAIL);
    assert(ble_scan_state() == BLE_SCAN_FAILED);
    assert(ble_scan_last_error() == BLE_HS_ENOMEM);
    test_disc_result = 0;
    restarts_before = test_disc_calls;
    assert(ble_scan_restart() == ESP_OK);  // 可恢复的 FAILED
    assert(ble_scan_state() == BLE_SCAN_SCANNING);
    assert(test_disc_calls == restarts_before + 1);
    assert(test_disc_duration_ms == 20000);
    assert(ble_scan_last_error() == 0);

    // ---- stop:等待 host 任务超时 -> 保留任务;重试 -> 清理成功 ----
    task = s_host_task;
    assert(task != NULL);
    unsigned stops_before = test_nimble_stops;
    cancels_before = test_disc_cancel_calls;
    assert(ble_scan_stop() == ESP_ERR_TIMEOUT);
    assert(test_nimble_stops == stops_before + 1);
    assert(test_disc_cancel_calls == cancels_before + 1);
    assert(ble_scan_state() == BLE_SCAN_FAILED);
    assert(ble_scan_last_error() == ESP_ERR_TIMEOUT);
    assert(s_host_task == task && !task->deleted);  // 未确认停机前不删除任务

    test_run_worker(task);
    assert(s_host_stopped != NULL && s_host_stopped->available);

    // deinit 失败:保留 initialized/host_done 现场供下一次 stop 重试。
    test_nimble_deinit_result = ESP_FAIL;
    assert(ble_scan_stop() == ESP_FAIL);
    assert(ble_scan_last_error() == ESP_FAIL);
    assert(ble_scan_state() == BLE_SCAN_FAILED);
    assert(s_initialized);
    assert(task->deleted && s_host_task == NULL);
    assert(test_nimble_stops == stops_before + 1);  // 不重复 stop 已停的 host

    test_nimble_deinit_result = ESP_OK;
    assert(ble_scan_stop() == ESP_OK);
    assert(ble_scan_state() == BLE_SCAN_OFF);
    assert(!s_initialized);
    assert(s_host_task == NULL && s_host_stopped == NULL);
    assert(ble_scan_restart() == ESP_ERR_INVALID_STATE);  // OFF 下不可 restart

    // ---- 清理后可重新 start / stop(状态机复位) ----
    start_scanning();
    stop_session();
    assert(ble_scan_state() == BLE_SCAN_OFF);
    assert(ble_scan_last_error() == 0);

    // ---- sync 回调失败路径:地址准备/推断失败 -> FAILED ----
    assert(ble_scan_start() == ESP_OK);
    disc_before = test_disc_calls;
    test_ensure_addr_result = ESP_FAIL;
    test_gap_fire_sync();
    assert(ble_scan_state() == BLE_SCAN_FAILED);
    assert(ble_scan_last_error() == ESP_FAIL);
    assert(test_disc_calls == disc_before);  // 地址准备失败时不得启动 discovery
    // host 仍存活时的 start() 重试:在现有 host 上重启 discovery,不重建协议栈。
    test_ensure_addr_result = 0;
    assert(ble_scan_start() == ESP_OK);
    assert(ble_scan_state() == BLE_SCAN_SCANNING);
    assert(ble_scan_last_error() == 0);
    assert(test_disc_calls == disc_before + 1);
    stop_session();

    assert(ble_scan_start() == ESP_OK);
    test_infer_addr_result = ESP_FAIL;
    test_gap_fire_sync();
    assert(ble_scan_state() == BLE_SCAN_FAILED);
    assert(ble_scan_last_error() == ESP_FAIL);
    test_infer_addr_result = 0;
    stop_session();

    // ---- sync 中启动 discovery 失败 -> FAILED ----
    assert(ble_scan_start() == ESP_OK);
    test_disc_result = BLE_HS_ENOMEM;
    test_gap_fire_sync();
    assert(ble_scan_state() == BLE_SCAN_FAILED);
    assert(ble_scan_last_error() == BLE_HS_ENOMEM);
    test_disc_result = 0;
    stop_session();

    // ---- 控制器/Host 复位:FAILED,且按用户是否暂停决定恢复方式 ----
    // a) 用户未暂停:下一次 sync 回调自动恢复扫描。
    start_scanning();
    disc_before = test_disc_calls;
    test_gap_fire_reset(ESP_FAIL);
    assert(ble_scan_state() == BLE_SCAN_FAILED);
    assert(ble_scan_last_error() == ESP_FAIL);
    test_gap_fire_sync();
    assert(ble_scan_state() == BLE_SCAN_SCANNING);
    assert(ble_scan_last_error() == 0);
    assert(test_disc_calls == disc_before + 1);
    stop_session();

    // b) resume() 也能从复位后的 FAILED 恢复。
    start_scanning();
    test_gap_fire_reset(ESP_FAIL);
    assert(ble_scan_state() == BLE_SCAN_FAILED);
    disc_before = test_disc_calls;
    assert(ble_scan_resume() == ESP_OK);
    assert(ble_scan_state() == BLE_SCAN_SCANNING);
    assert(test_disc_calls == disc_before + 1);
    stop_session();

    // c) start() 在 host 仍存活时只重启 discovery,不重新初始化 NimBLE。
    start_scanning();
    test_gap_fire_reset(ESP_FAIL);
    assert(ble_scan_state() == BLE_SCAN_FAILED);
    disc_before = test_disc_calls;
    const unsigned inits_before = test_nimble_inits;
    assert(ble_scan_start() == ESP_OK);
    assert(ble_scan_state() == BLE_SCAN_SCANNING);
    assert(test_nimble_inits == inits_before);
    assert(test_disc_calls == disc_before + 1);
    stop_session();

    // d) 暂停期间复位保持 PAUSED 语义,resume 后继续扫描。
    start_scanning();
    assert(ble_scan_pause() == ESP_OK);
    assert(ble_scan_state() == BLE_SCAN_PAUSED);
    test_gap_fire_reset(ESP_FAIL);
    assert(ble_scan_state() == BLE_SCAN_PAUSED);
    assert(ble_scan_last_error() == ESP_FAIL);
    assert(ble_scan_resume() == ESP_OK);
    assert(ble_scan_state() == BLE_SCAN_SCANNING);
    stop_session();

    // ---- pause 的取消失败 / EALREADY 分支 ----
    start_scanning();
    test_disc_cancel_result = BLE_HS_ENOMEM;
    assert(ble_scan_pause() == ESP_FAIL);
    assert(ble_scan_state() == BLE_SCAN_FAILED);
    assert(ble_scan_last_error() == BLE_HS_ENOMEM);
    test_disc_cancel_result = 0;
    stop_session();

    start_scanning();
    test_disc_cancel_result = BLE_HS_EALREADY;  // 已经不在扫描:视为成功
    assert(ble_scan_pause() == ESP_OK);
    assert(ble_scan_state() == BLE_SCAN_PAUSED);
    test_disc_cancel_result = 0;
    assert(ble_scan_resume() == ESP_OK);
    assert(ble_scan_state() == BLE_SCAN_SCANNING);
    stop_session();

    puts("BLE scan runtime tests: PASS");
    return 0;
}
