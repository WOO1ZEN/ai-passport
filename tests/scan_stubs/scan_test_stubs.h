// tests/scan_stubs/scan_test_stubs.h —— BLE 扫描服务 + BLE central 客户端的主机测试平台边界。
//
// main/ble_scan.c(observer)与 main/ble_client.c(NimBLE central)依赖 FreeRTOS /
// ESP-IDF / NimBLE。本头文件只提供它们用到的类型与函数声明,让真实 .c 文件可以
// 在主机上编译执行;行为由 tests/scan_stubs/scan_runtime.c 实现(确定性任务/
// 队列/信号量、可注入失败、可手动触发的 NimBLE 回调、脚本化的假 GATT 数据库)。
//
// 测试模型(与 tests/demo_stubs/demo_runtime.c 一致):
//   * 没有真并发:worker 任务由 test_run_worker() 驱动,在"会阻塞"的队列读取处
//     通过 longjmp 停机;下一次 test_run_worker() 重新进入任务函数(任务状态必须
//     保存在静态变量里,这也是目标实现的写法)。
//   * NimBLE 回调用测试显式触发;GATT 发现/MTU/写请求的完成回调默认同步触发,
//     模拟"设备立即可用"的成功路径;失败/超时通过 test_* 注入点控制。
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// ---- ESP-IDF ----
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_NO_MEM 0x101
#define ESP_ERR_INVALID_STATE 0x103
#define ESP_ERR_INVALID_ARG 0x102
#define ESP_ERR_TIMEOUT 0x107
#define ESP_ERR_NOT_FOUND 0x105
const char *esp_err_to_name(esp_err_t error);
void test_log(const char *tag, const char *format, ...);
#define ESP_LOGE(...) test_log(__VA_ARGS__)
#define ESP_LOGW(...) test_log(__VA_ARGS__)
#define ESP_LOGI(...) test_log(__VA_ARGS__)
#define ESP_LOGD(...) test_log(__VA_ARGS__)

int64_t esp_timer_get_time(void);
esp_err_t nvs_flash_init(void);
uint32_t esp_random(void);

// ---- FreeRTOS ----
typedef int BaseType_t;
typedef unsigned TickType_t;
typedef struct test_task *TaskHandle_t;
typedef struct test_sem *SemaphoreHandle_t;
typedef struct test_queue *QueueHandle_t;
typedef void (*TaskFunction_t)(void *);
#define pdTRUE 1
#define pdPASS 1
#define pdFALSE 0
#define portMAX_DELAY UINT32_MAX
#define pdMS_TO_TICKS(ms) (ms)
#define configMAX_PRIORITIES 25
#define NIMBLE_HS_STACK_SIZE 4096
#define NIMBLE_CORE 0

BaseType_t xTaskCreate(TaskFunction_t entry, const char *name, unsigned stack,
                       void *arg, unsigned priority, TaskHandle_t *task);
BaseType_t xTaskCreatePinnedToCore(TaskFunction_t entry, const char *name,
                                   unsigned stack, void *arg, unsigned priority,
                                   TaskHandle_t *task, int core);
void vTaskSuspend(TaskHandle_t task);
void vTaskDelete(TaskHandle_t task);
void vTaskDelay(TickType_t ticks);
TickType_t xTaskGetTickCount(void);
unsigned uxTaskGetStackHighWaterMark(TaskHandle_t task);
SemaphoreHandle_t xSemaphoreCreateBinary(void);
SemaphoreHandle_t xSemaphoreCreateMutex(void);
BaseType_t xSemaphoreGive(SemaphoreHandle_t sem);
BaseType_t xSemaphoreTake(SemaphoreHandle_t sem, TickType_t ticks);
void vSemaphoreDelete(SemaphoreHandle_t sem);
QueueHandle_t xQueueCreate(unsigned length, unsigned item_size);
BaseType_t xQueueSend(QueueHandle_t queue, const void *item, TickType_t ticks);
BaseType_t xQueueReceive(QueueHandle_t queue, void *item, TickType_t ticks);
void vQueueDelete(QueueHandle_t queue);

// ---- NimBLE UUID ----
#define BLE_UUID_TYPE_16 16
#define BLE_UUID_TYPE_32 32
#define BLE_UUID_TYPE_128 128

typedef struct {
    uint8_t type;
} ble_uuid_t;

typedef struct {
    ble_uuid_t u;
    uint16_t value;
} ble_uuid16_t;

typedef struct {
    ble_uuid_t u;
    uint32_t value;
} ble_uuid32_t;

typedef struct {
    ble_uuid_t u;
    uint8_t value[16];
} ble_uuid128_t;

typedef union {
    ble_uuid_t u;
    ble_uuid16_t u16;
    ble_uuid32_t u32;
    ble_uuid128_t u128;
} ble_uuid_any_t;

#define BLE_UUID16_DECLARE(uuid16) \
    (ble_uuid16_t) { .u = { .type = BLE_UUID_TYPE_16 }, .value = (uuid16) }
#define BLE_UUID_STR_LEN 37

int ble_uuid_cmp(const ble_uuid_t *uuid1, const ble_uuid_t *uuid2);
uint16_t ble_uuid_u16(const ble_uuid_t *uuid);
char *ble_uuid_to_str(const ble_uuid_t *uuid, char *dst);
int ble_uuid_from_str(ble_uuid_any_t *uuid, const char *str);

// ---- NimBLE os_mbuf(notify 回调里承载字节) ----
struct os_mbuf {
    uint16_t len;
    uint8_t data[512];
};

#define OS_MBUF_PKTLEN(om) ((om)->len)
#define OS_MBUF_DATA(om, type) ((type)((om)->data))
int os_mbuf_copydata(const struct os_mbuf *om, int off, int len, void *dst);
struct os_mbuf *ble_hs_mbuf_from_flat(const void *buf, uint16_t len);
void os_mbuf_free_chain(struct os_mbuf *om);

// ---- NimBLE 地址 / 连接 ----
typedef struct {
    uint8_t val[6];  // NimBLE 里低字节在前
    uint8_t type;    // 0 = 公共,1 = 随机
} ble_addr_t;

struct ble_gap_sec_state {
    unsigned encrypted : 1;
    unsigned authenticated : 1;
    unsigned bonded : 1;
    unsigned key_size : 5;
};

struct ble_gap_conn_desc {
    struct ble_gap_sec_state sec_state;
    ble_addr_t our_id_addr;
    ble_addr_t peer_id_addr;
    ble_addr_t our_ota_addr;
    ble_addr_t peer_ota_addr;
    uint16_t conn_handle;
    uint16_t conn_itvl;
    uint16_t conn_latency;
    uint16_t supervision_timeout;
    uint8_t role;
    uint8_t master_clock_accuracy;
};

struct ble_gap_conn_params {
    uint16_t scan_itvl;
    uint16_t scan_window;
    uint16_t itvl_min;
    uint16_t itvl_max;
    uint16_t latency;
    uint16_t supervision_timeout;
    uint16_t min_ce_len;
    uint16_t max_ce_len;
};

#define BLE_GAP_INITIAL_CONN_ITVL_MIN 24
#define BLE_GAP_INITIAL_CONN_ITVL_MAX 40
#define BLE_GAP_ROLE_MASTER 0
#define BLE_GAP_ROLE_SLAVE 1

// ---- NimBLE:legacy discovery(observer) ----
struct ble_gap_disc_desc {
    uint8_t event_type;
    uint8_t length_data;
    ble_addr_t addr;
    int8_t rssi;
    const uint8_t *data;
};

struct ble_gap_disc_params {
    uint16_t itvl;
    uint16_t window;
    uint8_t filter_policy;
    uint8_t limited;
    uint8_t passive;
    uint8_t filter_duplicates;
    uint8_t disable_observer_mode;
};

struct ble_gap_event {
    int type;
    union {
        struct {
            int status;
            uint16_t conn_handle;
        } connect;
        struct {
            int reason;
            struct ble_gap_conn_desc conn;
        } disconnect;
        struct ble_gap_disc_desc disc;
        struct {
            int reason;
        } disc_complete;
        struct {
            uint16_t conn_handle;
            uint16_t channel_id;
            uint16_t value;
        } mtu;
        struct {
            struct os_mbuf *om;
            uint16_t attr_handle;
            uint16_t conn_handle;
            uint8_t indication;
        } notify_rx;
        struct {
            uint16_t conn_handle;
            uint16_t attr_handle;
            uint8_t reason;
            uint8_t prev_notify : 1;
            uint8_t cur_notify : 1;
            uint8_t prev_indicate : 1;
            uint8_t cur_indicate : 1;
        } subscribe;
    };
};

struct test_ble_hs_cfg {
    void (*reset_cb)(int reason);
    void (*sync_cb)(void);
};
extern struct test_ble_hs_cfg ble_hs_cfg;

#define BLE_HS_FOREVER -1
// 与 NimBLE 一致的错误码(客户端 detail 里可能出现)。
#define BLE_HS_EAGAIN 1
#define BLE_HS_EALREADY 2
#define BLE_HS_EINVAL 3
#define BLE_HS_EMSGSIZE 4
#define BLE_HS_ENOENT 5
#define BLE_HS_ENOMEM 6
#define BLE_HS_ENOTCONN 7
#define BLE_HS_ENOTSUP 8
#define BLE_HS_ECONTROLLER 12
#define BLE_HS_ETIMEOUT 13
#define BLE_HS_EDONE 14
#define BLE_HS_EBUSY 15
#define BLE_HS_ENOTSYNCED 22
#define BLE_HS_EPREEMPTED 29

#define BLE_GAP_EVENT_CONNECT 0
#define BLE_GAP_EVENT_DISCONNECT 1
// 注意:这两个值沿用本桩历史取值,只保证测试内部一致。
#define BLE_GAP_EVENT_DISC 5
#define BLE_GAP_EVENT_DISC_COMPLETE 6
#define BLE_GAP_EVENT_NOTIFY_RX 12
#define BLE_GAP_EVENT_SUBSCRIBE 14
#define BLE_GAP_EVENT_MTU 15
#define BLE_HCI_ADV_RPT_EVTYPE_ADV_IND 0x00
#define BLE_HCI_ADV_RPT_EVTYPE_DIR_IND 0x01
#define BLE_HCI_ADV_RPT_EVTYPE_SCAN_RSP 0x04
#define BLE_GAP_SCAN_FAST_INTERVAL_MIN 0x0030
#define BLE_GAP_SCAN_FAST_INTERVAL_MAX 0x0060
#define BLE_GAP_SCAN_FAST_WINDOW 0x0030
#define BLE_ATT_MTU_DFLT 23
#define BLE_ATT_MTU_MAX 517
#define BLE_L2CAP_CID_ATT 4
#define BLE_HS_CONN_HANDLE_NONE 0xffff

typedef int (*ble_gap_event_fn)(struct ble_gap_event *event, void *arg);

int ble_hs_util_ensure_addr(int privacy);
int ble_hs_id_infer_auto(int privacy, uint8_t *type);
int ble_gap_disc(uint8_t own_addr_type, int32_t duration_ms,
                 const struct ble_gap_disc_params *params,
                 ble_gap_event_fn cb, void *cb_arg);
int ble_gap_disc_cancel(void);
int ble_gap_connect(uint8_t own_addr_type, const ble_addr_t *peer_addr,
                    int32_t duration_ms,
                    const struct ble_gap_conn_params *params,
                    ble_gap_event_fn cb, void *cb_arg);
int ble_gap_disconnect(uint16_t conn_handle);
int ble_gap_conn_find(uint16_t handle, struct ble_gap_conn_desc *out_desc);
int ble_gap_conn_find_by_addr(const ble_addr_t *addr, struct ble_gap_conn_desc *out_desc);
int ble_gap_conn_cancel(void);
int ble_gap_terminate(uint16_t conn_handle, uint8_t hci_reason);
esp_err_t nimble_port_init(void);
esp_err_t nimble_port_deinit(void);
int nimble_port_stop(void);
void nimble_port_run(void);

// ---- NimBLE GATT client ----
#define BLE_GATT_CHR_PROP_BROADCAST 0x01
#define BLE_GATT_CHR_PROP_READ 0x02
#define BLE_GATT_CHR_PROP_WRITE_NO_RSP 0x04
#define BLE_GATT_CHR_PROP_WRITE 0x08
#define BLE_GATT_CHR_PROP_NOTIFY 0x10
#define BLE_GATT_CHR_PROP_INDICATE 0x20

#define BLE_GATT_CHR_F_READ 0x00000002
#define BLE_GATT_CHR_F_WRITE_NO_RSP 0x00000004
#define BLE_GATT_CHR_F_WRITE 0x00000008
#define BLE_GATT_CHR_F_NOTIFY 0x00000010
#define BLE_GATT_CHR_F_INDICATE 0x00000020
#define BLE_GATT_CHR_F_NOTIFY_INDICATE_ENC 0x00008000

struct ble_gatt_error {
    uint16_t status;
    uint16_t att_handle;
};

struct ble_gatt_svc {
    uint16_t start_handle;
    uint16_t end_handle;
    ble_uuid_any_t uuid;
};

struct ble_gatt_chr {
    uint16_t def_handle;
    uint16_t val_handle;
    uint8_t properties;
    ble_uuid_any_t uuid;
};

struct ble_gatt_dsc {
    uint16_t handle;
    ble_uuid_any_t uuid;
};

struct ble_gatt_attr {
    uint16_t handle;
    uint16_t offset;
    struct os_mbuf *om;
};

typedef int ble_gatt_mtu_fn(uint16_t conn_handle,
                            const struct ble_gatt_error *error,
                            uint16_t mtu, void *arg);
typedef int ble_gatt_disc_svc_fn(uint16_t conn_handle,
                                 const struct ble_gatt_error *error,
                                 const struct ble_gatt_svc *service, void *arg);
typedef int ble_gatt_chr_fn(uint16_t conn_handle,
                            const struct ble_gatt_error *error,
                            const struct ble_gatt_chr *chr, void *arg);
typedef int ble_gatt_dsc_fn(uint16_t conn_handle,
                            const struct ble_gatt_error *error,
                            uint16_t chr_val_handle,
                            const struct ble_gatt_dsc *dsc, void *arg);
typedef int ble_gatt_attr_fn(uint16_t conn_handle,
                             const struct ble_gatt_error *error,
                             struct ble_gatt_attr *attr, void *arg);

int ble_gattc_exchange_mtu(uint16_t conn_handle,
                           ble_gatt_mtu_fn *cb, void *cb_arg);
int ble_gattc_disc_all_svcs(uint16_t conn_handle,
                            ble_gatt_disc_svc_fn *cb, void *cb_arg);
int ble_gattc_disc_svc_by_uuid(uint16_t conn_handle, const ble_uuid_t *uuid,
                               ble_gatt_disc_svc_fn *cb, void *cb_arg);
int ble_gattc_disc_all_chrs(uint16_t conn_handle, uint16_t start_handle,
                            uint16_t end_handle, ble_gatt_chr_fn *cb,
                            void *cb_arg);
int ble_gattc_disc_all_dscs(uint16_t conn_handle, uint16_t start_handle,
                            uint16_t end_handle, ble_gatt_dsc_fn *cb,
                            void *cb_arg);
int ble_gattc_write_flat(uint16_t conn_handle, uint16_t attr_handle,
                         const void *data, uint16_t data_len,
                         ble_gatt_attr_fn *cb, void *cb_arg);
int ble_gattc_write_no_rsp_flat(uint16_t conn_handle, uint16_t attr_handle,
                                const void *data, uint16_t data_len);
int ble_gattc_write(uint16_t conn_handle, uint16_t attr_handle,
                    struct os_mbuf *om, ble_gatt_attr_fn *cb, void *cb_arg);
int ble_att_set_preferred_mtu(uint16_t mtu);
uint16_t ble_att_mtu(uint16_t conn_handle);

// ---- 扫描/生命周期测试注入点(ble_scan.c 用) ----
extern bool test_create_fails;
extern bool test_sem_fails;
extern bool test_mutex_fails;
extern esp_err_t test_nvs_result;
extern unsigned test_nvs_calls;
extern esp_err_t test_nimble_init_result;
extern unsigned test_nimble_inits;
extern esp_err_t test_nimble_deinit_result;
extern int test_nimble_stop_result;
extern unsigned test_nimble_stops;
extern unsigned test_nimble_deinits;
extern int test_ensure_addr_result;
extern int test_infer_addr_result;
extern int test_disc_result;
extern int test_disc_cancel_result;
extern unsigned test_disc_calls;
extern unsigned test_disc_cancel_calls;
extern struct ble_gap_disc_params test_disc_params;
// 最近一次 ble_gap_disc 的 duration_ms(应用期望 20000:开机只扫 20 秒)。
extern int32_t test_disc_duration_ms;
extern uint8_t test_disc_own_addr_type;
// 毫秒时钟(esp_timer_get_time 的返回,单位 us)。
extern int64_t test_time_us;
// 假队列/任务的分配计数(runtime 内部使用,测试可读取)。
extern unsigned test_queue_count;
extern unsigned test_task_deletes;
// 最近一次 xTaskCreate/xTaskCreatePinnedToCore 创建的任务句柄。
extern TaskHandle_t test_last_task;

// 手动触发 NimBLE host 回调(真实固件里由 host 任务在同步完成后调用)。
void test_gap_fire_sync(void);
void test_gap_fire_reset(int reason);
void test_gap_fire_disc(const struct ble_gap_disc_desc *desc);
void test_gap_fire_disc_complete(void);
// 让指定任务执行到"阻塞点"(vTaskSuspend / 空队列读取)停机;可重复调用。
void test_run_worker(TaskHandle_t task);
// 当前是否运行在 test_run_worker() 的任务上下文里。
bool test_in_worker(void);

// ---- central 测试控制:假 GATT 数据库 ----
#define TEST_GATT_MAX_SERVICES 4
#define TEST_GATT_MAX_CHRS 8

// 清空假 GATT 数据库与连接/MTU/写记录(不影响任务/信号量)。
void test_gatt_reset(void);
// 注册服务;start/end 是句柄范围。返回索引,失败 -1。
int test_gatt_add_service(const char *uuid_str, uint16_t start_handle,
                          uint16_t end_handle);
// 在服务里注册特征;cccd_handle 为 0 表示没有 CCCD 描述符。
// properties 用 BLE_GATT_CHR_PROP_* 位。返回索引,失败 -1。
int test_gatt_add_chr(int service_index, const char *uuid_str,
                      uint16_t def_handle, uint16_t val_handle,
                      uint8_t properties, uint16_t cccd_handle,
                      bool cccd_indication);
// 连接行为:0 = 连接成功后同步回调 CONNECT(status=0);
//           1 = 同步回调 CONNECT(status=非 0)失败;2 = 不回调(模拟超时)。
#define TEST_CONNECT_OK 0
#define TEST_CONNECT_FAIL 1
#define TEST_CONNECT_SILENT 2
// 3 = 连接请求成功但 CONNECT 事件在 ble_gap_conn_cancel() 之后才迟到(孤儿连接)。
#define TEST_CONNECT_LATE 3
extern int test_connect_mode;
extern int test_connect_peer_status;
// 连接失败注入:前 test_connect_fail_attempts 次 ble_gap_connect 回调 CONNECT
// status=test_connect_fail_status(用于覆盖"13/574 重试一次"逻辑)。
extern int test_connect_fail_status;
extern unsigned test_connect_fail_attempts;
extern int test_disconnect_result;
extern int test_gap_disconnect_reason;
extern unsigned test_connect_calls;
extern unsigned test_disconnect_calls;
extern ble_addr_t test_connect_peer;
// 最近一次 ble_gap_connect 的连接参数(验证 scan_itvl/window 非 0,防 530)。
extern struct ble_gap_conn_params test_conn_params;
// discovery 注入:test_discovery_error != 0 时首个 discovery 回调返回该错误。
extern int test_discovery_error;
extern int test_mtu_value;              // exchange_mtu 之后的 MTU;默认 247
// 假连接上"已协商"的 ATT MTU(ble_att_mtu 返回)。默认 23;exchange_mtu 成功时
// 更新为 test_mtu_value。可预设它以模拟"MTU 事件丢失但握手已成功"的场景。
extern int test_att_mtu;
extern int test_exchange_mtu_result;    // 非 0 时 exchange_mtu 直接返回错误
extern int test_disc_all_dscs_result;   // 非 0 时 disc_all_dscs 直接返回该错误
extern int test_write_rsp_status;       // write_flat 回调 status;默认 0
extern int test_write_flat_result;      // 非 0 时 write_flat 返回错误
extern int test_write_no_rsp_result;    // 非 0 时 write_no_rsp_flat 返回错误
// 假设备的应答时机:false = 先回调写完成再发 notify(典型 BLE 顺序);
// true = notify 先于写完成到达(用于覆盖客户端"提前到达的业务数据"路径)。
extern bool test_write_notify_before_done;
// 写请求钩子:在 ble_gattc_write_* 内部同步调用,模拟设备立即应答。
// 默认 NULL(只记录不响应)。
typedef void (*test_gatt_write_hook_t)(uint16_t conn_handle,
                                       uint16_t attr_handle,
                                       const uint8_t *data, uint16_t len);
void test_gatt_set_write_hook(test_gatt_write_hook_t hook);
// 触发一次 notify_rx(通过最近一次 ble_gap_connect 注册的 gap 回调)。
void test_gatt_emit_notify(uint16_t attr_handle, const uint8_t *data, size_t len);
// 触发 subscribe 事件(CCCD 写完成)。
void test_gatt_emit_subscribe(uint16_t attr_handle, bool indicate);

struct test_gatt_write_record {
    uint16_t attr_handle;
    bool with_response;
    uint16_t len;
    uint8_t data[600];
};
extern struct test_gatt_write_record test_gatt_writes[32];
extern unsigned test_gatt_write_count;

// 最近一次连接/发现/写调用的计数(便于断言握手顺序)。
extern unsigned test_disc_all_svcs_calls;
extern unsigned test_disc_svc_by_uuid_calls;
extern unsigned test_disc_all_chrs_calls;
extern unsigned test_disc_all_dscs_calls;
extern unsigned test_exchange_mtu_calls;
extern unsigned test_write_flat_calls;
extern unsigned test_write_no_rsp_calls;
