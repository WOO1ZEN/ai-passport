// tests/scan_stubs/scan_runtime.c —— scan_test_stubs.h 的确定性主机实现。
//
// 与 tests/demo_stubs/demo_runtime.c 同一套调度模型:任务不会真并发,测试
// 通过 test_run_worker() 让 host 任务跑到 vTaskSuspend(NULL) 停机点;NimBLE
// 的 sync/DISC 回调由测试手动触发。所有外部失败都可注入,方便覆盖启动/清理
// 的错误分支。
#include "scan_test_stubs.h"

#include <assert.h>
#include <setjmp.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

struct test_task {
    TaskFunction_t entry;
    void *arg;
    bool deleted;
};

struct test_sem {
    bool available;
    bool deleted;
    bool mutex;
};

static struct test_task s_tasks[32];
static struct test_sem s_sems[64];
static unsigned s_task_count;
static unsigned s_sem_count;
unsigned test_task_deletes;
static TaskHandle_t s_current_task;
TaskHandle_t test_last_task;
void (*test_delay_hook)(void);
static jmp_buf s_worker_park;

struct test_ble_hs_cfg ble_hs_cfg;

bool test_create_fails;
bool test_sem_fails;
bool test_mutex_fails;
esp_err_t test_nvs_result = ESP_OK;
unsigned test_nvs_calls;
esp_err_t test_nimble_init_result = ESP_OK;
unsigned test_nimble_inits;
esp_err_t test_nimble_deinit_result = ESP_OK;
int test_nimble_stop_result;
unsigned test_nimble_stops;
unsigned test_nimble_deinits;
int test_ensure_addr_result;
int test_infer_addr_result;
int test_disc_result;
int test_disc_cancel_result;
unsigned test_disc_calls;
unsigned test_disc_cancel_calls;
struct ble_gap_disc_params test_disc_params;
int32_t test_disc_duration_ms;
int32_t test_disc_duration_ms;
uint8_t test_disc_own_addr_type;
int64_t test_time_us;

static ble_gap_event_fn s_disc_cb;
static void *s_disc_cb_arg;

void test_log(const char *tag, const char *format, ...)
{
#ifdef SCAN_STUB_DEBUG
    va_list ap;
    va_start(ap, format);
    fprintf(stderr, "[%s] ", tag);
    vfprintf(stderr, format, ap);
    fputc('\n', stderr);
    va_end(ap);
#else
    (void)tag;
    (void)format;
#endif
}

const char *esp_err_to_name(esp_err_t error)
{
    (void)error;
    return "test error";
}

int64_t esp_timer_get_time(void)
{
    return test_time_us;
}

uint32_t esp_random(void)
{
    static uint32_t s_random = 0x13572468u;
    s_random = s_random * 1664525u + 1013904223u;
    return s_random;
}

TickType_t xTaskGetTickCount(void)
{
    return (TickType_t)(test_time_us / 1000);
}

unsigned uxTaskGetStackHighWaterMark(TaskHandle_t task)
{
    (void)task;
    return 2048;
}

esp_err_t nvs_flash_init(void)
{
    test_nvs_calls++;
    return test_nvs_result;
}

// ---------------------------------------------------------------------------
// 任务:确定性调度,host 任务只能停在 vTaskSuspend(NULL)
// ---------------------------------------------------------------------------
BaseType_t xTaskCreate(TaskFunction_t entry, const char *name, unsigned stack,
                       void *arg, unsigned priority, TaskHandle_t *task)
{
    (void)name;
    (void)stack;
    (void)priority;
    if (task == NULL) return pdFALSE;
    *task = NULL;
    if (test_create_fails) return pdFALSE;
    assert(s_task_count < sizeof(s_tasks) / sizeof(s_tasks[0]));
    TaskHandle_t created = &s_tasks[s_task_count++];
    *created = (struct test_task){ .entry = entry, .arg = arg };
    *task = created;
    test_last_task = created;
    return pdPASS;
}

BaseType_t xTaskCreatePinnedToCore(TaskFunction_t entry, const char *name,
                                   unsigned stack, void *arg, unsigned priority,
                                   TaskHandle_t *task, int core)
{
    (void)core;
    return xTaskCreate(entry, name, stack, arg, priority, task);
}

void test_run_worker(TaskHandle_t task)
{
    assert(task != NULL && !task->deleted);
    assert(s_current_task == NULL);
    s_current_task = task;
    if (setjmp(s_worker_park) == 0) {
        task->entry(task->arg);
        assert(!"worker unexpectedly returned");
    }
    s_current_task = NULL;
}

void vTaskSuspend(TaskHandle_t task)
{
    assert(task == NULL && s_current_task != NULL);
    longjmp(s_worker_park, 1);
}

void vTaskDelete(TaskHandle_t task)
{
    // 只有拥有者可以回收已停机的任务,任务不能删除自己。
    assert(task != NULL && !task->deleted && s_current_task == NULL);
    task->deleted = true;
    test_task_deletes++;
}

bool test_in_worker(void)
{
    return s_current_task != NULL;
}

void vTaskDelay(TickType_t ticks)
{
    test_time_us += (int64_t)ticks * 1000;
    if (test_delay_hook != NULL) test_delay_hook();
}

// ---------------------------------------------------------------------------
// 信号量:mutex 初始可用,binary 初始不可用
// ---------------------------------------------------------------------------
static SemaphoreHandle_t sem_create(bool mutex)
{
    assert(s_sem_count < sizeof(s_sems) / sizeof(s_sems[0]));
    SemaphoreHandle_t sem = &s_sems[s_sem_count++];
    *sem = (struct test_sem){ .available = mutex, .mutex = mutex };
    return sem;
}

SemaphoreHandle_t xSemaphoreCreateBinary(void)
{
    if (test_sem_fails) return NULL;
    return sem_create(false);
}

SemaphoreHandle_t xSemaphoreCreateMutex(void)
{
    if (test_mutex_fails) return NULL;
    return sem_create(true);
}

BaseType_t xSemaphoreGive(SemaphoreHandle_t sem)
{
    assert(sem != NULL && !sem->deleted);
    sem->available = true;
    return pdTRUE;
}

BaseType_t xSemaphoreTake(SemaphoreHandle_t sem, TickType_t ticks)
{
    (void)ticks;
    assert(sem != NULL && !sem->deleted);
    if (!sem->available) return pdFALSE;
    sem->available = false;
    return pdTRUE;
}

void vSemaphoreDelete(SemaphoreHandle_t sem)
{
    assert(sem != NULL && !sem->deleted);
    sem->deleted = true;
}

// ---------------------------------------------------------------------------
// NimBLE host 生命周期
// ---------------------------------------------------------------------------
esp_err_t nimble_port_init(void)
{
    test_nimble_inits++;
    return test_nimble_init_result;
}

esp_err_t nimble_port_deinit(void)
{
    assert(s_current_task == NULL);
    test_nimble_deinits++;
    return test_nimble_deinit_result;
}

int nimble_port_stop(void)
{
    test_nimble_stops++;
    return test_nimble_stop_result;
}

void nimble_port_run(void)
{
    // 真实实现会一直运行 host 事件循环;主机测试里立即返回,随后 host_task
    // 给出 s_host_stopped 信号并停在 vTaskSuspend。
}

int ble_hs_util_ensure_addr(int privacy)
{
    (void)privacy;
    return test_ensure_addr_result;
}

int ble_hs_id_infer_auto(int privacy, uint8_t *type)
{
    (void)privacy;
    if (type != NULL) *type = 0;
    return test_infer_addr_result;
}

int ble_gap_disc(uint8_t own_addr_type, int32_t duration_ms,
                 const struct ble_gap_disc_params *params,
                 ble_gap_event_fn cb, void *cb_arg)
{
    test_disc_duration_ms = duration_ms;
    assert(params != NULL && cb != NULL);
    test_disc_calls++;
    test_disc_own_addr_type = own_addr_type;
    test_disc_params = *params;
    s_disc_cb = cb;
    s_disc_cb_arg = cb_arg;
    return test_disc_result;
}

int ble_gap_disc_cancel(void)
{
    test_disc_cancel_calls++;
    return test_disc_cancel_result;
}

// ---------------------------------------------------------------------------
// 测试触发点
// ---------------------------------------------------------------------------
void test_gap_fire_sync(void)
{
    assert(ble_hs_cfg.sync_cb != NULL);
    ble_hs_cfg.sync_cb();
}

void test_gap_fire_reset(int reason)
{
    assert(ble_hs_cfg.reset_cb != NULL);
    ble_hs_cfg.reset_cb(reason);
}

static void fire_event(struct ble_gap_event *event)
{
    assert(s_disc_cb != NULL);
    assert(s_disc_cb(event, s_disc_cb_arg) == 0);
}

void test_gap_fire_disc(const struct ble_gap_disc_desc *desc)
{
    struct ble_gap_event event;

    assert(desc != NULL && desc->data != NULL);
    memset(&event, 0, sizeof(event));
    event.type = BLE_GAP_EVENT_DISC;
    event.disc = *desc;
    fire_event(&event);
}

void test_gap_fire_disc_complete(void)
{
    struct ble_gap_event event;

    memset(&event, 0, sizeof(event));
    event.type = BLE_GAP_EVENT_DISC_COMPLETE;
    fire_event(&event);
}

// ===========================================================================
// FreeRTOS 队列:worker 内的空队列读取会 longjmp 到 test_run_worker() 的停机点,
// 下次 test_run_worker() 重新进入任务函数并继续消费(任务状态须在静态变量里)。
// ===========================================================================
struct test_queue {
    unsigned length;
    unsigned item_size;
    unsigned head;
    unsigned count;
    bool deleted;
    uint8_t items[16][288];
};

static struct test_queue s_queues[8];
unsigned test_queue_count;

static struct test_queue *queue_of(QueueHandle_t q)
{
    assert(q != NULL && !q->deleted);
    return q;
}

QueueHandle_t xQueueCreate(unsigned length, unsigned item_size)
{
    if (test_create_fails) return NULL;
    assert(test_queue_count < sizeof(s_queues) / sizeof(s_queues[0]));
    assert(length > 0 && length <= 16);
    assert(item_size > 0 && item_size <= 288);
    QueueHandle_t q = &s_queues[test_queue_count++];
    *q = (struct test_queue){ .length = length, .item_size = item_size };
    return q;
}

BaseType_t xQueueSend(QueueHandle_t queue, const void *item, TickType_t ticks)
{
    (void)ticks;
    struct test_queue *q = queue_of(queue);
    if (item == NULL || q->count >= q->length) return pdFALSE;
    const unsigned slot = (q->head + q->count) % q->length;
    memcpy(q->items[slot], item, q->item_size);
    q->count++;
    return pdTRUE;
}

BaseType_t xQueueReceive(QueueHandle_t queue, void *item, TickType_t ticks)
{
    struct test_queue *q = queue_of(queue);
    if (q->count == 0) {
        if (s_current_task != NULL && ticks == portMAX_DELAY) {
            // 无限等待:停机,下一次 test_run_worker 重新进入任务函数。
            longjmp(s_worker_park, 1);
        }
        // 有限超时:模拟"等满这个超时"并推进假时钟,让上层轮询循环准时超时。
        if (s_current_task != NULL && ticks != 0) {
            test_time_us += (int64_t)ticks * 1000;
        }
        return pdFALSE;
    }
    memcpy(item, q->items[q->head], q->item_size);
    q->head = (q->head + 1) % q->length;
    q->count--;
    return pdTRUE;
}

void vQueueDelete(QueueHandle_t queue)
{
    struct test_queue *q = queue_of(queue);
    q->deleted = true;
}

// ===========================================================================
// NimBLE UUID:16/32/128 位,统一展开成蓝牙 base UUID 后比较。
// ===========================================================================
int ble_uuid_cmp(const ble_uuid_t *uuid1, const ble_uuid_t *uuid2)
{
    if (uuid1 == NULL || uuid2 == NULL) return uuid1 == uuid2 ? 0 : 1;

    uint8_t a[16] = { 0 };
    uint8_t b[16] = { 0 };
    for (int pass = 0; pass < 2; pass++) {
        const ble_uuid_t *u = pass == 0 ? uuid1 : uuid2;
        uint8_t *out = pass == 0 ? a : b;
        if (u->type == BLE_UUID_TYPE_128) {
            const ble_uuid128_t *u128 = (const ble_uuid128_t *)u;
            memcpy(out, u128->value, sizeof(u128->value));
            continue;
        }
        // 16/32 位按蓝牙 base UUID 展开:0000xxxx-0000-1000-8000-00805f9b34fb。
        static const uint8_t base_be[16] = {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00,
            0x80, 0x00, 0x00, 0x80, 0x5f, 0x9b, 0x34, 0xfb,
        };
        memcpy(out, base_be, sizeof(base_be));
        if (u->type == BLE_UUID_TYPE_16) {
            const uint16_t v = ((const ble_uuid16_t *)u)->value;
            out[2] = (uint8_t)(v >> 8);
            out[3] = (uint8_t)(v & 0xff);
        } else {
            const uint32_t v = ((const ble_uuid32_t *)u)->value;
            out[0] = (uint8_t)(v >> 24);
            out[1] = (uint8_t)(v >> 16);
            out[2] = (uint8_t)(v >> 8);
            out[3] = (uint8_t)(v & 0xff);
        }
    }
    // 统一转成 NimBLE 的 little-endian 形式再比较。
    for (int i = 0; i < 8; i++) {
        uint8_t t = a[i];
        a[i] = a[15 - i];
        a[15 - i] = t;
        t = b[i];
        b[i] = b[15 - i];
        b[15 - i] = t;
    }
    return memcmp(a, b, sizeof(a));
}

static int hex_nibble(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

int ble_uuid_from_str(ble_uuid_any_t *uuid, const char *str)
{
    if (uuid == NULL || str == NULL) return BLE_HS_EINVAL;

    char hex[33];
    size_t hex_len = 0;
    for (const char *p = str; *p != '\0'; p++) {
        if (*p == '-') continue;
        if (hex_nibble(*p) < 0 || hex_len >= sizeof(hex) - 1) return BLE_HS_EINVAL;
        hex[hex_len++] = *p;
    }
    hex[hex_len] = '\0';

    if (hex_len == 4) {
        uint16_t v = 0;
        for (size_t i = 0; i < 4; i++) v = (uint16_t)((v << 4) | (unsigned)hex_nibble(hex[i]));
        uuid->u16 = (ble_uuid16_t){ .u = { .type = BLE_UUID_TYPE_16 }, .value = v };
        return 0;
    }
    if (hex_len == 8) {
        uint32_t v = 0;
        for (size_t i = 0; i < 8; i++) v = (v << 4) | (uint32_t)hex_nibble(hex[i]);
        uuid->u32 = (ble_uuid32_t){ .u = { .type = BLE_UUID_TYPE_32 }, .value = v };
        return 0;
    }
    if (hex_len == 32) {
        uuid->u128.u.type = BLE_UUID_TYPE_128;
        for (size_t i = 0; i < 16; i++) {
            const int hi = hex_nibble(hex[(15 - i) * 2]);
            const int lo = hex_nibble(hex[(15 - i) * 2 + 1]);
            uuid->u128.value[i] = (uint8_t)((hi << 4) | lo);
        }
        return 0;
    }
    return BLE_HS_EINVAL;
}

uint16_t ble_uuid_u16(const ble_uuid_t *uuid)
{
    if (uuid == NULL) return 0;
    if (uuid->type == BLE_UUID_TYPE_16) return ((const ble_uuid16_t *)uuid)->value;
    return 0;
}

char *ble_uuid_to_str(const ble_uuid_t *uuid, char *dst)
{
    if (dst == NULL) return dst;
    if (uuid == NULL) {
        dst[0] = '\0';
        return dst;
    }
    if (uuid->type == BLE_UUID_TYPE_16) {
        unsigned v = ((const ble_uuid16_t *)uuid)->value;
        snprintf(dst, 37, "%04x", v);
        return dst;
    }
    if (uuid->type == BLE_UUID_TYPE_32) {
        unsigned v = (unsigned)((const ble_uuid32_t *)uuid)->value;
        snprintf(dst, 37, "%08x", v);
        return dst;
    }
    const ble_uuid128_t *u128 = (const ble_uuid128_t *)uuid;
    snprintf(dst, 37,
             "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
             u128->value[15], u128->value[14], u128->value[13], u128->value[12],
             u128->value[11], u128->value[10], u128->value[9], u128->value[8],
             u128->value[7], u128->value[6], u128->value[5], u128->value[4],
             u128->value[3], u128->value[2], u128->value[1], u128->value[0]);
    return dst;
}

// ===========================================================================
// os_mbuf
// ===========================================================================
int os_mbuf_copydata(const struct os_mbuf *om, int off, int len, void *dst)
{
    if (om == NULL || dst == NULL || off < 0 || len < 0) return -1;
    if ((unsigned)off + (unsigned)len > om->len) return -1;
    memcpy(dst, om->data + off, (size_t)len);
    return 0;
}

static struct os_mbuf s_mbuf_pool[4];
static unsigned s_mbuf_used;

struct os_mbuf *ble_hs_mbuf_from_flat(const void *buf, uint16_t len)
{
    assert(len <= sizeof(s_mbuf_pool[0].data));
    struct os_mbuf *om = &s_mbuf_pool[s_mbuf_used++ % 4];
    om->len = len;
    if (buf != NULL && len > 0) memcpy(om->data, buf, len);
    return om;
}

void os_mbuf_free_chain(struct os_mbuf *om)
{
    (void)om;
}

// ===========================================================================
// NimBLE central:GAP 连接 / GATT 发现 / 写 / 通知(脚本化假设备)
// ===========================================================================
int test_connect_mode = TEST_CONNECT_OK;
int test_connect_peer_status = BLE_HS_ENOTCONN;
int test_connect_fail_status;
unsigned test_connect_fail_attempts;
int test_disconnect_result;
int test_gap_disconnect_reason = 0x13;
unsigned test_connect_calls;
unsigned test_disconnect_calls;
ble_addr_t test_connect_peer;
struct ble_gap_conn_params test_conn_params;
int test_discovery_error;
int test_mtu_value = 247;
int test_att_mtu = 23;
int test_exchange_mtu_result;
int test_disc_all_dscs_result;
int test_write_rsp_status;
int test_write_flat_result;
int test_write_no_rsp_result;
bool test_write_notify_before_done;
unsigned test_disc_all_svcs_calls;
unsigned test_disc_svc_by_uuid_calls;
unsigned test_disc_all_chrs_calls;
unsigned test_disc_all_dscs_calls;
unsigned test_exchange_mtu_calls;
unsigned test_write_flat_calls;
unsigned test_write_no_rsp_calls;
struct test_gatt_write_record test_gatt_writes[32];
unsigned test_gatt_write_count;

static ble_gap_event_fn s_gap_cb;
static void *s_gap_cb_arg;
static uint16_t s_conn_handle;
static bool s_connected;
static struct ble_gap_conn_desc s_conn_desc;
static test_gatt_write_hook_t s_write_hook;

static void gap_emit(struct ble_gap_event *event)
{
    if (s_gap_cb != NULL) (void)s_gap_cb(event, s_gap_cb_arg);
}

static void conn_reset(void)
{
    s_gap_cb = NULL;
    s_gap_cb_arg = NULL;
    s_conn_handle = 0;
    s_connected = false;
    memset(&s_conn_desc, 0, sizeof(s_conn_desc));
}

struct test_gatt_svc_entry {
    bool used;
    char uuid_str[40];
    ble_uuid_any_t uuid;
    uint16_t start_handle;
    uint16_t end_handle;
};

struct test_gatt_chr_entry {
    int service;
    char uuid_str[40];
    ble_uuid_any_t uuid;
    uint16_t def_handle;
    uint16_t val_handle;
    uint8_t properties;
    uint16_t cccd_handle;
    bool cccd_indication;
};

static struct test_gatt_svc_entry s_gatt_svcs[TEST_GATT_MAX_SERVICES];
static struct test_gatt_chr_entry s_gatt_chrs[TEST_GATT_MAX_CHRS];

void test_gatt_reset(void)
{
    memset(s_gatt_svcs, 0, sizeof(s_gatt_svcs));
    memset(s_gatt_chrs, 0, sizeof(s_gatt_chrs));
    memset(test_gatt_writes, 0, sizeof(test_gatt_writes));
    test_gatt_write_count = 0;
    test_connect_mode = TEST_CONNECT_OK;
    test_connect_peer_status = BLE_HS_ENOTCONN;
    test_connect_fail_status = 0;
    test_connect_fail_attempts = 0;
    test_disconnect_result = 0;
    test_gap_disconnect_reason = 0x13;
    test_connect_calls = 0;
    test_disconnect_calls = 0;
    test_discovery_error = 0;
    test_mtu_value = 247;
    test_att_mtu = 23;
    test_exchange_mtu_result = 0;
    test_disc_all_dscs_result = 0;
    test_write_rsp_status = 0;
    test_write_flat_result = 0;
    test_write_no_rsp_result = 0;
    test_write_notify_before_done = false;
    test_delay_hook = NULL;
    test_disc_all_svcs_calls = 0;
    test_disc_svc_by_uuid_calls = 0;
    test_disc_all_chrs_calls = 0;
    test_disc_all_dscs_calls = 0;
    test_exchange_mtu_calls = 0;
    test_write_flat_calls = 0;
    test_write_no_rsp_calls = 0;
    s_write_hook = NULL;
    memset(&test_connect_peer, 0, sizeof(test_connect_peer));
    conn_reset();
}

int test_gatt_add_service(const char *uuid_str, uint16_t start_handle,
                          uint16_t end_handle)
{
    for (int i = 0; i < TEST_GATT_MAX_SERVICES; i++) {
        if (s_gatt_svcs[i].used) continue;
        ble_uuid_any_t uuid;
        if (ble_uuid_from_str(&uuid, uuid_str) != 0) return -1;
        s_gatt_svcs[i].used = true;
        snprintf(s_gatt_svcs[i].uuid_str, sizeof(s_gatt_svcs[i].uuid_str), "%s", uuid_str);
        s_gatt_svcs[i].uuid = uuid;
        s_gatt_svcs[i].start_handle = start_handle;
        s_gatt_svcs[i].end_handle = end_handle;
        return i;
    }
    return -1;
}

int test_gatt_add_chr(int service_index, const char *uuid_str,
                      uint16_t def_handle, uint16_t val_handle,
                      uint8_t properties, uint16_t cccd_handle,
                      bool cccd_indication)
{
    if (service_index < 0 || service_index >= TEST_GATT_MAX_SERVICES ||
        !s_gatt_svcs[service_index].used) {
        return -1;
    }
    for (int i = 0; i < TEST_GATT_MAX_CHRS; i++) {
        if (s_gatt_chrs[i].def_handle != 0) continue;
        ble_uuid_any_t uuid;
        if (ble_uuid_from_str(&uuid, uuid_str) != 0) return -1;
        s_gatt_chrs[i].service = service_index;
        snprintf(s_gatt_chrs[i].uuid_str, sizeof(s_gatt_chrs[i].uuid_str), "%s", uuid_str);
        s_gatt_chrs[i].uuid = uuid;
        s_gatt_chrs[i].def_handle = def_handle;
        s_gatt_chrs[i].val_handle = val_handle;
        s_gatt_chrs[i].properties = properties;
        s_gatt_chrs[i].cccd_handle = cccd_handle;
        s_gatt_chrs[i].cccd_indication = cccd_indication;
        return i;
    }
    return -1;
}

void test_gatt_set_write_hook(test_gatt_write_hook_t hook)
{
    s_write_hook = hook;
}


int ble_gap_connect(uint8_t own_addr_type, const ble_addr_t *peer_addr,
                    int32_t duration_ms,
                    const struct ble_gap_conn_params *params,
                    ble_gap_event_fn cb, void *cb_arg)
{
    if (params != NULL) test_conn_params = *params;
    (void)own_addr_type;
    (void)duration_ms;
    (void)params;
    test_connect_calls++;
    if (peer_addr != NULL) test_connect_peer = *peer_addr;
    s_gap_cb = cb;
    s_gap_cb_arg = cb_arg;

    if (test_connect_mode == TEST_CONNECT_SILENT) return 0;
    if (test_connect_mode == TEST_CONNECT_LATE) return 0;  // 事件稍后由 conn_cancel 投递

    if (test_connect_fail_attempts > 0) {
        // 前 N 次连接注入失败回调(不进 connected 状态)。
        test_connect_fail_attempts--;
        struct ble_gap_event failed;
        memset(&failed, 0, sizeof(failed));
        failed.type = BLE_GAP_EVENT_CONNECT;
        failed.connect.status = test_connect_fail_status;
        gap_emit(&failed);
        return 0;
    }

    struct ble_gap_event event;
    memset(&event, 0, sizeof(event));
    event.type = BLE_GAP_EVENT_CONNECT;
    if (test_connect_mode == TEST_CONNECT_FAIL) {
        event.connect.status = test_connect_peer_status;
        gap_emit(&event);
        return 0;
    }

    s_conn_handle = 1;
    s_connected = true;
    memset(&s_conn_desc, 0, sizeof(s_conn_desc));
    s_conn_desc.conn_handle = s_conn_handle;
    s_conn_desc.role = BLE_GAP_ROLE_MASTER;
    if (peer_addr != NULL) {
        s_conn_desc.peer_id_addr = *peer_addr;
        s_conn_desc.peer_ota_addr = *peer_addr;
    }
    event.connect.status = 0;
    event.connect.conn_handle = s_conn_handle;
    gap_emit(&event);
    return 0;
}

int ble_gap_disconnect(uint16_t conn_handle)
{
    test_disconnect_calls++;
    if (!s_connected || conn_handle != s_conn_handle) return BLE_HS_ENOTCONN;
    s_connected = false;
    struct ble_gap_event event;
    memset(&event, 0, sizeof(event));
    event.type = BLE_GAP_EVENT_DISCONNECT;
    event.disconnect.reason = test_gap_disconnect_reason;
    event.disconnect.conn = s_conn_desc;
    gap_emit(&event);
    return test_disconnect_result;
}

int ble_gap_conn_find(uint16_t handle, struct ble_gap_conn_desc *out_desc)
{
    if (!s_connected || handle != s_conn_handle || out_desc == NULL) return BLE_HS_ENOENT;
    *out_desc = s_conn_desc;
    return 0;
}

int ble_gap_conn_find_by_addr(const ble_addr_t *addr, struct ble_gap_conn_desc *out_desc)
{
    if (!s_connected || addr == NULL || out_desc == NULL) return BLE_HS_ENOENT;
    *out_desc = s_conn_desc;
    return 0;
}

void test_gatt_emit_notify(uint16_t attr_handle, const uint8_t *data, size_t len)
{
    static struct os_mbuf om;
    assert(len <= sizeof(om.data));
    om.len = (uint16_t)len;
    if (len > 0) memcpy(om.data, data, len);
#ifdef SCAN_STUB_DEBUG
    fprintf(stderr, "[stub] notify attr=%u conn=%u len=%zu\n", attr_handle,
            (unsigned)s_conn_handle, len);
#endif

    struct ble_gap_event event;
    memset(&event, 0, sizeof(event));
    event.type = BLE_GAP_EVENT_NOTIFY_RX;
    event.notify_rx.om = &om;
    event.notify_rx.attr_handle = attr_handle;
    event.notify_rx.conn_handle = s_conn_handle;
    gap_emit(&event);
}

void test_gatt_emit_subscribe(uint16_t attr_handle, bool indicate)
{
    struct ble_gap_event event;
    memset(&event, 0, sizeof(event));
    event.type = BLE_GAP_EVENT_SUBSCRIBE;
    event.subscribe.conn_handle = s_conn_handle;
    event.subscribe.attr_handle = attr_handle;
    event.subscribe.cur_notify = indicate ? 0 : 1;
    event.subscribe.cur_indicate = indicate ? 1 : 0;
    gap_emit(&event);
}

int ble_gattc_exchange_mtu(uint16_t conn_handle,
                           ble_gatt_mtu_fn *cb, void *cb_arg)
{
    (void)conn_handle;
    test_exchange_mtu_calls++;
    if (test_exchange_mtu_result != 0) return test_exchange_mtu_result;
    // 握手成功:对端 MTU 生效,ble_att_mtu() 之后返回该值。
    test_att_mtu = test_mtu_value;

    struct ble_gap_event event;
    memset(&event, 0, sizeof(event));
    event.type = BLE_GAP_EVENT_MTU;
    event.mtu.conn_handle = s_conn_handle;
    event.mtu.channel_id = 4;  // BLE_L2CAP_CID_ATT
    event.mtu.value = (uint16_t)test_mtu_value;
    gap_emit(&event);
    if (cb != NULL) {
        struct ble_gatt_error err = { 0 };
        (void)cb(s_conn_handle, &err, (uint16_t)test_mtu_value, cb_arg);
    }
    return 0;
}

static bool gatt_service_matches_range(const struct test_gatt_svc_entry *svc,
                                        uint16_t start_handle, uint16_t end_handle)
{
    // NimBLE 语义:发现 [start,end] 范围内的特征;与服务范围相交即参与。
    return !(svc->end_handle < start_handle || svc->start_handle > end_handle);
}

int ble_gattc_disc_all_svcs(uint16_t conn_handle,
                            ble_gatt_disc_svc_fn *cb, void *cb_arg)
{
    test_disc_all_svcs_calls++;
    if (cb == NULL) return 0;

    struct ble_gatt_error err = { 0 };
    if (test_discovery_error != 0) {
        err.status = (uint16_t)test_discovery_error;
        (void)cb(conn_handle, &err, NULL, cb_arg);
        err.status = BLE_HS_EDONE;
        (void)cb(conn_handle, &err, NULL, cb_arg);
        return 0;
    }
    for (int i = 0; i < TEST_GATT_MAX_SERVICES; i++) {
        if (!s_gatt_svcs[i].used) continue;
        struct ble_gatt_svc svc;
        memset(&svc, 0, sizeof(svc));
        svc.start_handle = s_gatt_svcs[i].start_handle;
        svc.end_handle = s_gatt_svcs[i].end_handle;
        svc.uuid = s_gatt_svcs[i].uuid;
        (void)cb(conn_handle, &err, &svc, cb_arg);
    }
    err.status = BLE_HS_EDONE;
    (void)cb(conn_handle, &err, NULL, cb_arg);
    return 0;
}

int ble_gattc_disc_svc_by_uuid(uint16_t conn_handle, const ble_uuid_t *uuid,
                               ble_gatt_disc_svc_fn *cb, void *cb_arg)
{
    test_disc_svc_by_uuid_calls++;
    if (cb == NULL) return 0;

    struct ble_gatt_error err = { 0 };
    if (test_discovery_error != 0) {
        err.status = (uint16_t)test_discovery_error;
        (void)cb(conn_handle, &err, NULL, cb_arg);
        err.status = BLE_HS_EDONE;
        (void)cb(conn_handle, &err, NULL, cb_arg);
        return 0;
    }
    for (int i = 0; i < TEST_GATT_MAX_SERVICES; i++) {
        if (!s_gatt_svcs[i].used) continue;
        if (ble_uuid_cmp(uuid, &s_gatt_svcs[i].uuid.u) != 0) continue;
        struct ble_gatt_svc svc;
        memset(&svc, 0, sizeof(svc));
        svc.start_handle = s_gatt_svcs[i].start_handle;
        svc.end_handle = s_gatt_svcs[i].end_handle;
        svc.uuid = s_gatt_svcs[i].uuid;
        (void)cb(conn_handle, &err, &svc, cb_arg);
    }
    err.status = BLE_HS_EDONE;
    (void)cb(conn_handle, &err, NULL, cb_arg);
    return 0;
}

int ble_gattc_disc_all_chrs(uint16_t conn_handle, uint16_t start_handle,
                            uint16_t end_handle, ble_gatt_chr_fn *cb,
                            void *cb_arg)
{
    test_disc_all_chrs_calls++;
    if (cb == NULL) return 0;

    struct ble_gatt_error err = { 0 };
    for (int i = 0; i < TEST_GATT_MAX_CHRS; i++) {
        if (s_gatt_chrs[i].def_handle == 0) continue;
        const int svc = s_gatt_chrs[i].service;
        if (!gatt_service_matches_range(&s_gatt_svcs[svc], start_handle, end_handle)) continue;
        struct ble_gatt_chr chr;
        memset(&chr, 0, sizeof(chr));
        chr.def_handle = s_gatt_chrs[i].def_handle;
        chr.val_handle = s_gatt_chrs[i].val_handle;
        chr.properties = s_gatt_chrs[i].properties;
        chr.uuid = s_gatt_chrs[i].uuid;
        (void)cb(conn_handle, &err, &chr, cb_arg);
    }
    err.status = BLE_HS_EDONE;
    (void)cb(conn_handle, &err, NULL, cb_arg);
    return 0;
}

int ble_gattc_disc_all_dscs(uint16_t conn_handle, uint16_t start_handle,
                            uint16_t end_handle, ble_gatt_dsc_fn *cb,
                            void *cb_arg)
{
    (void)end_handle;
    test_disc_all_dscs_calls++;
    if (test_disc_all_dscs_result != 0) return test_disc_all_dscs_result;
    if (cb == NULL) return 0;

    struct ble_gatt_error err = { 0 };
    for (int i = 0; i < TEST_GATT_MAX_CHRS; i++) {
        if (s_gatt_chrs[i].def_handle == 0) continue;
        if (s_gatt_chrs[i].val_handle != start_handle) continue;
        if (s_gatt_chrs[i].cccd_handle == 0) continue;
        struct ble_gatt_dsc dsc;
        memset(&dsc, 0, sizeof(dsc));
        dsc.handle = s_gatt_chrs[i].cccd_handle;
        if (ble_uuid_from_str(&dsc.uuid, "2902") != 0) return BLE_HS_EINVAL;
        (void)cb(conn_handle, &err, s_gatt_chrs[i].val_handle, &dsc, cb_arg);
    }
    err.status = BLE_HS_EDONE;
    (void)cb(conn_handle, &err, 0, NULL, cb_arg);
    return 0;
}

static void gatt_record_write(uint16_t attr_handle, const void *data,
                              uint16_t len, bool with_response)
{
    if (test_gatt_write_count < sizeof(test_gatt_writes) / sizeof(test_gatt_writes[0])) {
        struct test_gatt_write_record *rec = &test_gatt_writes[test_gatt_write_count++];
        memset(rec, 0, sizeof(*rec));
        rec->attr_handle = attr_handle;
        rec->with_response = with_response;
        rec->len = len;
        assert(len <= sizeof(rec->data));
        if (len > 0) memcpy(rec->data, data, len);
    }
#ifdef SCAN_STUB_DEBUG
    fprintf(stderr, "[stub] write attr=%u len=%u hook=%d\n", attr_handle, len,
            s_write_hook != NULL);
#endif
    if (s_write_hook != NULL) s_write_hook(s_conn_handle, attr_handle, data, len);
}

int ble_gattc_write_flat(uint16_t conn_handle, uint16_t attr_handle,
                         const void *data, uint16_t data_len,
                         ble_gatt_attr_fn *cb, void *cb_arg)
{
    (void)conn_handle;
    test_write_flat_calls++;
    if (test_write_flat_result != 0) return test_write_flat_result;

    struct ble_gatt_error err = { .status = (uint16_t)test_write_rsp_status };
    struct ble_gatt_attr attr = { .handle = attr_handle };
    if (!test_write_notify_before_done) {
        if (cb != NULL) (void)cb(s_conn_handle, &err, &attr, cb_arg);
        gatt_record_write(attr_handle, data, data_len, true);
    } else {
        gatt_record_write(attr_handle, data, data_len, true);
        if (cb != NULL) (void)cb(s_conn_handle, &err, &attr, cb_arg);
    }
    return 0;
}

int ble_gattc_write_no_rsp_flat(uint16_t conn_handle, uint16_t attr_handle,
                                const void *data, uint16_t data_len)
{
    (void)conn_handle;
    test_write_no_rsp_calls++;
    if (test_write_no_rsp_result != 0) return test_write_no_rsp_result;
    gatt_record_write(attr_handle, data, data_len, false);
    return 0;
}

int ble_gattc_write(uint16_t conn_handle, uint16_t attr_handle,
                    struct os_mbuf *om, ble_gatt_attr_fn *cb, void *cb_arg)
{
    (void)conn_handle;
    if (om == NULL) return BLE_HS_EINVAL;
    test_write_flat_calls++;
    struct ble_gatt_error err = { .status = (uint16_t)test_write_rsp_status };
    struct ble_gatt_attr attr = { .handle = attr_handle };
    if (!test_write_notify_before_done) {
        if (cb != NULL) (void)cb(s_conn_handle, &err, &attr, cb_arg);
        gatt_record_write(attr_handle, om->data, om->len, true);
    } else {
        gatt_record_write(attr_handle, om->data, om->len, true);
        if (cb != NULL) (void)cb(s_conn_handle, &err, &attr, cb_arg);
    }
    return 0;
}



int ble_gap_conn_cancel(void)
{
    if (test_connect_mode == TEST_CONNECT_LATE && !s_connected) {
        // 模拟取消之后才到的 CONNECT:客户端必须回收这个孤儿连接。
        test_connect_mode = TEST_CONNECT_SILENT;
        s_connected = true;
        s_conn_handle = 1;
        memset(&s_conn_desc, 0, sizeof(s_conn_desc));
        s_conn_desc.conn_handle = s_conn_handle;
        s_conn_desc.role = BLE_GAP_ROLE_MASTER;
        struct ble_gap_event event;
        memset(&event, 0, sizeof(event));
        event.type = BLE_GAP_EVENT_CONNECT;
        event.connect.status = 0;
        event.connect.conn_handle = s_conn_handle;
        gap_emit(&event);
    }
    return 0;
}

int ble_gap_terminate(uint16_t conn_handle, uint8_t hci_reason)
{
    (void)hci_reason;
    test_disconnect_calls++;
    if (!s_connected || conn_handle != s_conn_handle) return BLE_HS_ENOTCONN;
    s_connected = false;
    struct ble_gap_event event;
    memset(&event, 0, sizeof(event));
    event.type = BLE_GAP_EVENT_DISCONNECT;
    event.disconnect.reason = test_gap_disconnect_reason;
    event.disconnect.conn = s_conn_desc;
    gap_emit(&event);
    return test_disconnect_result;
}

int ble_att_set_preferred_mtu(uint16_t mtu)
{
    (void)mtu;
    return 0;
}

uint16_t ble_att_mtu(uint16_t conn_handle)
{
    (void)conn_handle;
    return (uint16_t)test_att_mtu;
}
