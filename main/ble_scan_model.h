// main/ble_scan_model.h —— BLE 扫描应用的纯逻辑层(不依赖 ESP-IDF / LVGL)。
//
// 这一层只做三件事,全部可在主机上用普通 C 编译与测试:
//   1. 解析广播数据(AD structures),取出设备名、TX power、flags;
//   2. 维护"最近出现的设备表"(按地址合并、过期清理、容量淘汰);
//   3. 把 RSSI 映射为信号等级,并把设备按信号强度排序。
//
// 并发:NimBLE host 任务与 LVGL 任务会同时访问设备表,互斥锁由 main/ble_scan.c
// 负责;本层保持无锁、可重入,方便主机测试。
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// 设备名以 UTF-8 保存,含结尾 NUL 最多 BLE_SCAN_NAME_MAX 字节。
#define BLE_SCAN_NAME_MAX 24
// 设备表容量。8 MB Flash / 无 PSRAM 的板子上,每条约 64 字节,96 条约 6 KB
// 静态内存。开机只扫描 20 秒,要装下这 20 秒里出现的所有设备(室内随机地址设备
// 会不断涌现),48 条偏紧;96 条留足余量。过期条目仍按 15 秒清理。
#define BLE_SCAN_TABLE_CAP 96
// NimBLE 用 127 表示 RSSI / TX power 不可用。
#define BLE_SCAN_RSSI_UNKNOWN 127
// 超过这么久没有新广播的设备视为已离开范围,不再显示。
#define BLE_SCAN_STALE_MS 15000

typedef struct {
    uint8_t addr[6];       // NimBLE 的 ble_addr_t.val 原样保存:addr[0] 是地址最低字节
    uint8_t addr_type;     // 0 = 公共地址,1 = 随机地址(BLE_ADDR_PUBLIC / BLE_ADDR_RANDOM)
    int8_t rssi;           // 最近一次广播的 RSSI(dBm)
    int8_t rssi_ema;       // RSSI 指数平滑值,用于稳定信号等级与排序
    int8_t tx_power;       // 广播里的 TX power(dBm);BLE_SCAN_RSSI_UNKNOWN = 未广播
    uint16_t adv_flags;    // 广播 flags 原始值;0 = 未广播
    bool connectable;      // ADV_IND / ADV_DIRECT_IND 为 true
    char name[BLE_SCAN_NAME_MAX];  // UTF-8,已按字符边界截断,NUL 结尾;空串 = 未命名
    uint32_t first_seen_ms;        // 首次发现时刻(调用方时钟,毫秒)
    uint32_t last_seen_ms;         // 最近一次广播时刻(调用方时钟,毫秒)
    uint32_t seen_count;           // 收到的广播/扫描响应包数
} ble_scan_dev_t;

typedef struct {
    ble_scan_dev_t devs[BLE_SCAN_TABLE_CAP];
    size_t count;
} ble_scan_table_t;

// 信号等级同时也是信号格数(0..4)。
typedef enum {
    BLE_SIGNAL_NONE = 0,    // 无信号 / 无有效 RSSI
    BLE_SIGNAL_WEAK = 1,    // 弱
    BLE_SIGNAL_FAIR = 2,    // 中
    BLE_SIGNAL_GOOD = 3,    // 强
    BLE_SIGNAL_STRONG = 4,  // 极强
} ble_signal_level_t;

// 广播数据解析结果。字段缺失时保持初值(0 / false / BLE_SCAN_RSSI_UNKNOWN)。
typedef struct {
    char name[BLE_SCAN_NAME_MAX];
    bool has_name;
    uint8_t flags;
    bool has_flags;
    int8_t tx_power;
    bool has_tx_power;
} ble_adv_fields_t;

// 解析一段 AD structures(不含长度前缀的整体 payload)。data 为 NULL 或 len 为 0
// 时只把 *out 初始化为"全部缺失"。对畸形长度、截断的类型/数据一律安全停止。
void ble_scan_parse_adv(const uint8_t *data, uint8_t len, ble_adv_fields_t *out);

// 把 UTF-8 文本复制到 dst[cap],在字符边界处截断,并丢弃控制字符(保留可见空格)。
// dst 始终 NUL 结尾。返回写入的字节数(不含 NUL)。用于设备名这种不可信输入。
size_t ble_scan_utf8_copy(char *dst, size_t cap, const char *src, size_t src_len);

// 解码 s 开头的 UTF-8 字符。成功返回消耗的字节数(1..4),*cp 为码点;
// 非法/截断序列返回 0(不推进),控制字符按原样解出由调用方决定是否显示。
size_t ble_scan_utf8_decode(const char *s, size_t len, uint32_t *cp);

// 把码点编码为 UTF-8,返回字节数(1..4);非法码点返回 0 且不写 out。
size_t ble_scan_utf8_encode(uint32_t cp, char *out, size_t cap);

void ble_scan_table_init(ble_scan_table_t *t);
void ble_scan_table_clear(ble_scan_table_t *t);

// 合并一条观测:同地址(addr + addr_type)则更新 RSSI/名称/时间戳并累加计数;
// 新地址追加;表满时淘汰 last_seen_ms 最旧的一条。返回表中对应条目。
// 观测里 first_seen_ms 由本函数决定,调用方可传任意值。
ble_scan_dev_t *ble_scan_table_merge(ble_scan_table_t *t, const ble_scan_dev_t *obs);

// 删除 now_ms - last_seen_ms >= stale_ms 的条目。时间比较用无符号差值,可安全跨回绕。
void ble_scan_table_expire(ble_scan_table_t *t, uint32_t now_ms, uint32_t stale_ms);

// 按 rssi_ema 从强到弱排序,返回写入 order 的条目数(order[i] 是 t->devs 的下标)。
// 未过期的条目最多写 order_cap 个;排序保证确定:同信号时 last_seen 新的靠前,
// 再相同则按地址升序,使 UI 在两次刷新之间不会因排序不稳定而跳动。
size_t ble_scan_table_rank(const ble_scan_table_t *t, uint32_t now_ms,
                           uint32_t stale_ms, size_t *order, size_t order_cap);

// RSSI(dBm)→ 信号等级。BLE_SCAN_RSSI_UNKNOWN 或明显无效值返回 BLE_SIGNAL_NONE。
ble_signal_level_t ble_signal_level(int rssi);

// 把 6 字节地址格式化为 "AA:BB:CC:DD:EE:FF"(大写,out 至少 18 字节)。
// NimBLE 的 val[] 是低字节在前,显示时反转。
void ble_scan_format_addr(const uint8_t addr[6], char out[18]);
