// main/ble_scan_model.c —— BLE 扫描的纯逻辑实现,不依赖 ESP-IDF / LVGL。
//
// 设计说明:
//   * 设备名来自空口广播,属于不可信输入:必须按 UTF-8 字符边界截断,并丢弃
//     控制字符,否则 UI 会画出乱码或占位方块。
//   * 设备表用线性查找合并。容量只有 24 条,线性扫描比哈希更省 RAM 也更好测。
//   * 时间戳都是调用方给的 uint32 毫秒。所有比较都用无符号差值,这样计时器
//     跨 2^32 ms(约 49.7 天)回绕时不会把"刚出现"误判成"很久以前"。
#include "ble_scan_model.h"

#include <stdio.h>
#include <string.h>

// 广播 AD type 定义(Bluetooth Core Specification, Supplement to the ADC)。
#define AD_TYPE_FLAGS          0x01
#define AD_TYPE_NAME_SHORT     0x08
#define AD_TYPE_NAME_COMPLETE  0x09
#define AD_TYPE_TX_POWER       0x0A

// ---------------------------------------------------------------------------
// UTF-8
// ---------------------------------------------------------------------------

size_t ble_scan_utf8_decode(const char *s, size_t len, uint32_t *cp)
{
    if (s == NULL || cp == NULL || len == 0) return 0;

    const uint8_t b0 = (uint8_t)s[0];
    if (b0 < 0x80) {
        *cp = b0;
        return 1;
    }

    size_t need;
    uint32_t value;
    uint32_t min_value;
    if ((b0 & 0xE0) == 0xC0) {
        need = 1;
        value = b0 & 0x1Fu;
        min_value = 0x80;
    } else if ((b0 & 0xF0) == 0xE0) {
        need = 2;
        value = b0 & 0x0Fu;
        min_value = 0x800;
    } else if ((b0 & 0xF8) == 0xF0) {
        need = 3;
        value = b0 & 0x07u;
        min_value = 0x10000;
    } else {
        return 0;  // 续字节/非法首字节
    }
    if (len < need + 1) return 0;

    for (size_t i = 1; i <= need; i++) {
        const uint8_t b = (uint8_t)s[i];
        if ((b & 0xC0) != 0x80) return 0;
        value = (value << 6) | (b & 0x3Fu);
    }
    if (value < min_value || value > 0x10FFFF) return 0;
    if (value >= 0xD800 && value <= 0xDFFF) return 0;  // UTF-16 代理区不允许

    *cp = value;
    return need + 1;
}

size_t ble_scan_utf8_encode(uint32_t cp, char *out, size_t cap)
{
    if (out == NULL) return 0;
    if (cp > 0x10FFFF || (cp >= 0xD800 && cp <= 0xDFFF)) return 0;

    size_t need;
    if (cp < 0x80) need = 1;
    else if (cp < 0x800) need = 2;
    else if (cp < 0x10000) need = 3;
    else need = 4;
    if (cap < need) return 0;

    switch (need) {
    case 1:
        out[0] = (char)cp;
        break;
    case 2:
        out[0] = (char)(0xC0 | (cp >> 6));
        out[1] = (char)(0x80 | (cp & 0x3F));
        break;
    case 3:
        out[0] = (char)(0xE0 | (cp >> 12));
        out[1] = (char)(0x80 | ((cp >> 6) & 0x3F));
        out[2] = (char)(0x80 | (cp & 0x3F));
        break;
    default:
        out[0] = (char)(0xF0 | (cp >> 18));
        out[1] = (char)(0x80 | ((cp >> 12) & 0x3F));
        out[2] = (char)(0x80 | ((cp >> 6) & 0x3F));
        out[3] = (char)(0x80 | (cp & 0x3F));
        break;
    }
    return need;
}

// 显示无关的"可打印"判定:去掉 C0/C1 控制符与 DEL。设备名里混入控制符时,
// 直接丢弃而不是显示成不可见字符,避免 UI 出现空洞。
static bool codepoint_is_printable(uint32_t cp)
{
    if (cp == 0) return false;
    if (cp < 0x20 || cp == 0x7F) return false;
    if (cp >= 0x80 && cp < 0xA0) return false;
    return true;
}

size_t ble_scan_utf8_copy(char *dst, size_t cap, const char *src, size_t src_len)
{
    if (dst == NULL || cap == 0) return 0;
    dst[0] = '\0';
    if (src == NULL) return 0;

    size_t written = 0;
    size_t i = 0;
    while (i < src_len && written + 1 < cap) {
        uint32_t cp = 0;
        const size_t consumed = ble_scan_utf8_decode(src + i, src_len - i, &cp);
        if (consumed == 0) {
            i++;  // 跳过非法字节,继续尝试后面的字符
            continue;
        }
        if (!codepoint_is_printable(cp)) {
            i += consumed;
            continue;
        }
        char encoded[4];
        const size_t encoded_len = ble_scan_utf8_encode(cp, encoded, sizeof(encoded));
        if (encoded_len == 0 || written + encoded_len >= cap) break;
        memcpy(dst + written, encoded, encoded_len);
        written += encoded_len;
        i += consumed;
    }
    dst[written] = '\0';
    return written;
}

// ---------------------------------------------------------------------------
// 广播数据解析
// ---------------------------------------------------------------------------

void ble_scan_parse_adv(const uint8_t *data, uint8_t len, ble_adv_fields_t *out)
{
    if (out == NULL) return;
    memset(out, 0, sizeof(*out));
    out->tx_power = BLE_SCAN_RSSI_UNKNOWN;
    if (data == NULL || len == 0) return;

    size_t i = 0;
    while (i < len) {
        const uint8_t ad_len = data[i];
        if (ad_len == 0) break;  // 后续是填充,结束
        // 每个 AD structure = 1 字节长度 + 1 字节类型 + (ad_len - 1) 字节数据。
        if (i + 1 + ad_len > (size_t)len) break;  // 被截断的尾巴,安全停止

        const uint8_t type = data[i + 1];
        const uint8_t *payload = &data[i + 2];
        const size_t payload_len = (size_t)ad_len - 1;

        switch (type) {
        case AD_TYPE_FLAGS:
            if (payload_len >= 1) {
                out->flags = payload[0];
                out->has_flags = true;
            }
            break;
        case AD_TYPE_NAME_COMPLETE:
        case AD_TYPE_NAME_SHORT: {
            if (payload_len == 0) break;
            // 完整名优先;同一包里短名在前、完整名在后时以完整名为准。
            if (out->has_name && type == AD_TYPE_NAME_SHORT) break;
            const size_t n = ble_scan_utf8_copy(
                out->name, sizeof(out->name), (const char *)payload, payload_len);
            out->has_name = n > 0;
            break;
        }
        case AD_TYPE_TX_POWER:
            if (payload_len >= 1) {
                out->tx_power = (int8_t)payload[0];
                out->has_tx_power = true;
            }
            break;
        default:
            break;  // 服务 UUID、厂商数据等本应用暂不展示
        }
        i += (size_t)ad_len + 1;
    }
}

// ---------------------------------------------------------------------------
// 设备表
// ---------------------------------------------------------------------------

void ble_scan_table_init(ble_scan_table_t *t)
{
    if (t == NULL) return;
    memset(t, 0, sizeof(*t));
}

void ble_scan_table_clear(ble_scan_table_t *t)
{
    if (t == NULL) return;
    memset(t, 0, sizeof(*t));
}

static bool same_device(const ble_scan_dev_t *a, const ble_scan_dev_t *b)
{
    return a->addr_type == b->addr_type && memcmp(a->addr, b->addr, sizeof(a->addr)) == 0;
}

// 指数平滑:alpha = 1/4。BLE 的 RSSI 抖动通常有几 dB,直接显示会让信号格来回跳。
static int8_t smooth_rssi(int8_t previous, int8_t sample)
{
    const int smoothed = ((int)previous * 3 + (int)sample) / 4;
    if (smoothed > 126) return 126;
    if (smoothed < -128) return -128;
    return (int8_t)smoothed;
}

ble_scan_dev_t *ble_scan_table_merge(ble_scan_table_t *t, const ble_scan_dev_t *obs)
{
    if (t == NULL || obs == NULL) return NULL;

    for (size_t i = 0; i < t->count; i++) {
        ble_scan_dev_t *dev = &t->devs[i];
        if (!same_device(dev, obs)) continue;

        if (obs->rssi != BLE_SCAN_RSSI_UNKNOWN) {
            dev->rssi = obs->rssi;
            dev->rssi_ema = dev->seen_count == 0 ? obs->rssi : smooth_rssi(dev->rssi_ema, obs->rssi);
        }
        if (obs->tx_power != BLE_SCAN_RSSI_UNKNOWN) dev->tx_power = obs->tx_power;
        if (obs->adv_flags != 0) dev->adv_flags = obs->adv_flags;
        // 扫描响应里没有连接位时不能把已有的"可连接"覆盖成 false。
        dev->connectable = dev->connectable || obs->connectable;
        // 广播包和扫描响应会交替到达:只有带名字的包才更新名字,避免名字被清空。
        if (obs->name[0] != '\0') {
            memcpy(dev->name, obs->name, sizeof(dev->name));
            dev->name[sizeof(dev->name) - 1] = '\0';
        }
        dev->last_seen_ms = obs->last_seen_ms;
        dev->seen_count++;
        return dev;
    }

    size_t slot;
    if (t->count < BLE_SCAN_TABLE_CAP) {
        slot = t->count++;
    } else {
        // 表满时淘汰最久没出现的设备;用无符号差找最大 age,兼容时间回绕。
        slot = 0;
        uint32_t oldest_age = obs->last_seen_ms - t->devs[0].last_seen_ms;
        for (size_t i = 1; i < t->count; i++) {
            const uint32_t age = obs->last_seen_ms - t->devs[i].last_seen_ms;
            if (age > oldest_age) {
                oldest_age = age;
                slot = i;
            }
        }
    }

    ble_scan_dev_t *dev = &t->devs[slot];
    memset(dev, 0, sizeof(*dev));
    memcpy(dev->addr, obs->addr, sizeof(dev->addr));
    dev->addr_type = obs->addr_type;
    dev->rssi = obs->rssi;
    dev->rssi_ema = obs->rssi == BLE_SCAN_RSSI_UNKNOWN ? BLE_SCAN_RSSI_UNKNOWN : obs->rssi;
    dev->tx_power = obs->tx_power;
    dev->adv_flags = obs->adv_flags;
    dev->connectable = obs->connectable;
    memcpy(dev->name, obs->name, sizeof(dev->name));
    dev->name[sizeof(dev->name) - 1] = '\0';
    dev->first_seen_ms = obs->last_seen_ms;
    dev->last_seen_ms = obs->last_seen_ms;
    dev->seen_count = 1;
    return dev;
}

void ble_scan_table_expire(ble_scan_table_t *t, uint32_t now_ms, uint32_t stale_ms)
{
    if (t == NULL) return;
    size_t i = 0;
    while (i < t->count) {
        if ((uint32_t)(now_ms - t->devs[i].last_seen_ms) >= stale_ms) {
            t->devs[i] = t->devs[t->count - 1];
            t->count--;
        } else {
            i++;
        }
    }
}

static bool device_ranks_before(const ble_scan_dev_t *a, const ble_scan_dev_t *b)
{
    if (a->rssi_ema != b->rssi_ema) return a->rssi_ema > b->rssi_ema;
    const int32_t age_diff = (int32_t)(a->last_seen_ms - b->last_seen_ms);
    if (age_diff != 0) return age_diff > 0;  // 更新的在前,跨回绕安全
    const int addr_cmp = memcmp(a->addr, b->addr, sizeof(a->addr));
    if (addr_cmp != 0) return addr_cmp < 0;
    return a->addr_type < b->addr_type;
}

size_t ble_scan_table_rank(const ble_scan_table_t *t, uint32_t now_ms,
                           uint32_t stale_ms, size_t *order, size_t order_cap)
{
    if (t == NULL || order == NULL || order_cap == 0) return 0;

    size_t count = 0;
    for (size_t i = 0; i < t->count; i++) {
        const ble_scan_dev_t *dev = &t->devs[i];
        if ((uint32_t)(now_ms - dev->last_seen_ms) >= stale_ms) continue;

        const ble_scan_dev_t *candidate = &t->devs[i];  // 仅用于比较,不改写
        if (count < order_cap) {
            size_t pos = count;
            while (pos > 0 && device_ranks_before(candidate, &t->devs[order[pos - 1]])) {
                order[pos] = order[pos - 1];
                pos--;
            }
            order[pos] = i;
            count++;
        } else if (device_ranks_before(candidate, &t->devs[order[order_cap - 1]])) {
            size_t pos = order_cap - 1;
            while (pos > 0 && device_ranks_before(candidate, &t->devs[order[pos - 1]])) {
                order[pos] = order[pos - 1];
                pos--;
            }
            order[pos] = i;
        }
    }
    return count;
}

// ---------------------------------------------------------------------------
// 展示辅助
// ---------------------------------------------------------------------------

ble_signal_level_t ble_signal_level(int rssi)
{
    // 127 = 不可用;>=0 不是合法 RSSI。两者都按"无信号"处理,而不是画成满格。
    if (rssi == BLE_SCAN_RSSI_UNKNOWN || rssi >= 0) return BLE_SIGNAL_NONE;
    if (rssi >= -55) return BLE_SIGNAL_STRONG;
    if (rssi >= -67) return BLE_SIGNAL_GOOD;
    if (rssi >= -78) return BLE_SIGNAL_FAIR;
    return BLE_SIGNAL_WEAK;  // -79 dBm 及以下,直到 -128 都算弱
}

void ble_scan_format_addr(const uint8_t addr[6], char out[18])
{
    if (out == NULL) return;
    if (addr == NULL) {
        out[0] = '\0';
        return;
    }
    // NimBLE 的 val[] 是低字节在前,人读的 MAC 是高位在左,所以反向打印。
    (void)snprintf(out, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
                   addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
}
