// tests/test_ble_scan_model.c —— BLE 扫描纯逻辑层的主机单元测试。
//
// 契约来源:main/ble_scan_model.h。只依赖 C11 标准库与 assert(),不链接
// ESP-IDF / LVGL,由 tools/validate.sh 用普通 cc 编译运行。
//
// 覆盖:广播 AD 解析(含畸形/截断输入)、UTF-8 拷贝/解码/编码、设备表
// 合并/淘汰/过期/排序、RSSI 信号等级、地址格式化。
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ble_scan_model.h"

// 构造一条观测记录。first_seen_ms / seen_count 故意填"垃圾值",用来验证
// ble_scan_table_merge() 自己决定这两个字段(而不是照抄调用方输入)。
static ble_scan_dev_t make_obs(const uint8_t addr[6], uint8_t addr_type, int rssi,
                               int8_t tx_power, uint16_t adv_flags, bool connectable,
                               const char *name, uint32_t last_seen_ms)
{
    ble_scan_dev_t obs;

    memset(&obs, 0, sizeof(obs));
    memcpy(obs.addr, addr, sizeof(obs.addr));
    obs.addr_type = addr_type;
    obs.rssi = (int8_t)rssi;
    obs.rssi_ema = (int8_t)rssi;
    obs.tx_power = tx_power;
    obs.adv_flags = adv_flags;
    obs.connectable = connectable;
    if (name != NULL) {
        size_t n = strlen(name);
        if (n >= sizeof(obs.name)) {
            n = sizeof(obs.name) - 1;
        }
        memcpy(obs.name, name, n);
    }
    obs.first_seen_ms = 0xBEEFu;
    obs.last_seen_ms = last_seen_ms;
    obs.seen_count = 99u;
    return obs;
}

static void addr_with_id(uint8_t addr[6], uint8_t id)
{
    memset(addr, 0, 6);
    addr[0] = id;
}

static bool table_has_addr_id(const ble_scan_table_t *t, uint8_t id)
{
    for (size_t i = 0; i < t->count; i++) {
        if (t->devs[i].addr[0] == id) {
            return true;
        }
    }
    return false;
}

// ---------------------------------------------------------------------------
// UTF-8 编解码
// ---------------------------------------------------------------------------
static void test_utf8_decode(void)
{
    uint32_t cp = 0;

    assert(ble_scan_utf8_decode("A", 1, &cp) == 1 && cp == 0x41);
    assert(ble_scan_utf8_decode("\xC2\xA2", 2, &cp) == 2 && cp == 0xA2);
    assert(ble_scan_utf8_decode("\xE4\xB8\xAD", 3, &cp) == 3 && cp == 0x4E2D);
    assert(ble_scan_utf8_decode("\xF0\x9F\x98\x80", 4, &cp) == 4 && cp == 0x1F600);

    // 从字符串中间解码,只消费一个字符。
    const char mixed[] = "x\xE4\xB8\xADy";
    assert(ble_scan_utf8_decode(mixed + 1, sizeof(mixed) - 2, &cp) == 3 && cp == 0x4E2D);

    // 截断的多字节序列。
    assert(ble_scan_utf8_decode("\xE4\xB8", 2, &cp) == 0);
    assert(ble_scan_utf8_decode("\xF0\x9F\x98", 3, &cp) == 0);

    // 非法首字节 / 续字节当首字节 / 非法续字节。
    assert(ble_scan_utf8_decode("\x80", 1, &cp) == 0);
    assert(ble_scan_utf8_decode("\xFF", 1, &cp) == 0);
    assert(ble_scan_utf8_decode("\xE4\x28\xAD", 3, &cp) == 0);

    // 过长编码、代理区、超出 U+10FFFF 都必须拒绝。
    assert(ble_scan_utf8_decode("\xC0\x80", 2, &cp) == 0);
    assert(ble_scan_utf8_decode("\xE0\x80\x80", 3, &cp) == 0);
    assert(ble_scan_utf8_decode("\xED\xA0\x80", 3, &cp) == 0);       // U+D800
    assert(ble_scan_utf8_decode("\xF4\x90\x80\x80", 4, &cp) == 0);    // U+110000

    // 控制字符按原样解出(是否显示由调用方决定)。
    assert(ble_scan_utf8_decode("\x1F", 1, &cp) == 1 && cp == 0x1F);
    assert(ble_scan_utf8_decode("", 1, &cp) == 1 && cp == 0x00);

    // len 为 0 时不解码。
    assert(ble_scan_utf8_decode("A", 0, &cp) == 0);
}

static void test_utf8_encode(void)
{
    char buf[8];

    memset(buf, 0x7F, sizeof(buf));
    assert(ble_scan_utf8_encode(0x41, buf, sizeof(buf)) == 1);
    assert((uint8_t)buf[0] == 0x41);

    assert(ble_scan_utf8_encode(0x7FF, buf, sizeof(buf)) == 2);
    assert((uint8_t)buf[0] == 0xDF && (uint8_t)buf[1] == 0xBF);

    assert(ble_scan_utf8_encode(0x800, buf, sizeof(buf)) == 3);
    assert((uint8_t)buf[0] == 0xE0 && (uint8_t)buf[1] == 0xA0 && (uint8_t)buf[2] == 0x80);

    assert(ble_scan_utf8_encode(0x4E2D, buf, sizeof(buf)) == 3);
    assert((uint8_t)buf[0] == 0xE4 && (uint8_t)buf[1] == 0xB8 && (uint8_t)buf[2] == 0xAD);

    assert(ble_scan_utf8_encode(0xFFFF, buf, sizeof(buf)) == 3);
    assert((uint8_t)buf[0] == 0xEF && (uint8_t)buf[1] == 0xBF && (uint8_t)buf[2] == 0xBF);

    assert(ble_scan_utf8_encode(0x10000, buf, sizeof(buf)) == 4);
    assert((uint8_t)buf[0] == 0xF0 && (uint8_t)buf[1] == 0x90 &&
           (uint8_t)buf[2] == 0x80 && (uint8_t)buf[3] == 0x80);

    assert(ble_scan_utf8_encode(0x1F600, buf, sizeof(buf)) == 4);
    assert((uint8_t)buf[0] == 0xF0 && (uint8_t)buf[1] == 0x9F &&
           (uint8_t)buf[2] == 0x98 && (uint8_t)buf[3] == 0x80);

    // 非法码点:超出范围、代理区。
    assert(ble_scan_utf8_encode(0x110000, buf, sizeof(buf)) == 0);
    assert(ble_scan_utf8_encode(0xFFFFFFFFu, buf, sizeof(buf)) == 0);
    assert(ble_scan_utf8_encode(0xD800, buf, sizeof(buf)) == 0);
    assert(ble_scan_utf8_encode(0xDFFF, buf, sizeof(buf)) == 0);

    // 输出缓冲不足时不写、返回 0。
    assert(ble_scan_utf8_encode(0x41, buf, 0) == 0);
    assert(ble_scan_utf8_encode(0x4E2D, buf, 2) == 0);

    // 编解码往返。
    const uint32_t cps[] = {0x20, 0x41, 0xA2, 0x7FF, 0x800, 0x4E2D, 0xFFFF, 0x10000, 0x1F600};
    for (size_t i = 0; i < sizeof(cps) / sizeof(cps[0]); i++) {
        const size_t n = ble_scan_utf8_encode(cps[i], buf, sizeof(buf));
        uint32_t decoded = 0;
        assert(n >= 1 && n <= 4);
        assert(ble_scan_utf8_decode(buf, n, &decoded) == n);
        assert(decoded == cps[i]);
    }
}

// ---------------------------------------------------------------------------
// UTF-8 拷贝(不可信设备名)
// ---------------------------------------------------------------------------
static void test_utf8_copy(void)
{
    char dst[8];

    // 正常拷贝。
    memset(dst, 0x7F, sizeof(dst));
    assert(ble_scan_utf8_copy(dst, sizeof(dst), "hello", 5) == 5);
    assert(strcmp(dst, "hello") == 0);
    assert(dst[5] == '\0');

    // 按字节容量截断,不拆分多字节字符。
    assert(ble_scan_utf8_copy(dst, 3, "hello", 5) == 2);
    assert(strcmp(dst, "he") == 0);
    assert(ble_scan_utf8_copy(dst, 3, "A\xE4\xB8\xAD", 4) == 1);
    assert(strcmp(dst, "A") == 0);
    assert(ble_scan_utf8_copy(dst, 5, "A\xE4\xB8\xAD", 4) == 4);
    assert(strcmp(dst, "A\xE4\xB8\xAD") == 0);
    assert(ble_scan_utf8_copy(dst, 4, "\xE4\xB8\xAD\xE4\xB8\xAD", 6) == 3);
    assert(strcmp(dst, "\xE4\xB8\xAD") == 0);

    // 容量 1 只能放 NUL。
    memset(dst, 0x7F, sizeof(dst));
    assert(ble_scan_utf8_copy(dst, 1, "hello", 5) == 0);
    assert(dst[0] == '\0');

    // 丢弃 C0 / DEL / C1 控制字符,保留可见空格。
    const char ctrl[] = "a\tb\nc\x01" "d";
    assert(ble_scan_utf8_copy(dst, sizeof(dst), ctrl, strlen(ctrl)) == 4);
    assert(strcmp(dst, "abcd") == 0);
    assert(ble_scan_utf8_copy(dst, sizeof(dst), "a b", 3) == 3);
    assert(strcmp(dst, "a b") == 0);
    const char del_and_c1[] = "a\x7F b\xC2\x85" "c";  // DEL + U+0085(NEL)
    assert(ble_scan_utf8_copy(dst, sizeof(dst), del_and_c1, strlen(del_and_c1)) == 4);
    assert(strcmp(dst, "a bc") == 0);

    // 非法字节跳过,继续处理后面的字符。
    const char bad[] = {'A', (char)0xFF, 'B'};
    assert(ble_scan_utf8_copy(dst, sizeof(dst), bad, sizeof(bad)) == 2);
    assert(strcmp(dst, "AB") == 0);

    // 截断的尾巴被丢弃,不留下半个字符。
    const char tail[] = {'A', (char)0xE4, (char)0xB8};
    assert(ble_scan_utf8_copy(dst, sizeof(dst), tail, sizeof(tail)) == 1);
    assert(strcmp(dst, "A") == 0);

    // 源里的 NUL 不显示,但后面的字节继续处理。
    const char with_nul[] = {'A', '\0', 'B'};
    assert(ble_scan_utf8_copy(dst, sizeof(dst), with_nul, sizeof(with_nul)) == 2);
    assert(strcmp(dst, "AB") == 0);

    // NULL 源:只写 NUL。
    assert(ble_scan_utf8_copy(dst, sizeof(dst), NULL, 0) == 0);
    assert(dst[0] == '\0');
}

// ---------------------------------------------------------------------------
// 广播数据解析
// ---------------------------------------------------------------------------
static void expect_empty_adv(const ble_adv_fields_t *f)
{
    assert(!f->has_name);
    assert(f->name[0] == '\0');
    assert(!f->has_flags && f->flags == 0);
    assert(!f->has_tx_power);
    assert(f->tx_power == BLE_SCAN_RSSI_UNKNOWN);
}

static void test_parse_adv_basic(void)
{
    ble_adv_fields_t f;
    const uint8_t no_data = 0;

    ble_scan_parse_adv(NULL, 0, &f);
    expect_empty_adv(&f);
    ble_scan_parse_adv(&no_data, 0, &f);
    expect_empty_adv(&f);

    // Flags(AD type 0x01)。
    const uint8_t flags_ad[] = {0x02, 0x01, 0x06};
    ble_scan_parse_adv(flags_ad, sizeof(flags_ad), &f);
    assert(f.has_flags && f.flags == 0x06);
    assert(!f.has_name && f.name[0] == '\0');
    assert(!f.has_tx_power && f.tx_power == BLE_SCAN_RSSI_UNKNOWN);

    // TX power(AD type 0x0A),有符号。
    const uint8_t tx_ad[] = {0x02, 0x0A, 0xF4};  // -12 dBm
    ble_scan_parse_adv(tx_ad, sizeof(tx_ad), &f);
    assert(f.has_tx_power && f.tx_power == -12);
    assert(!f.has_flags && !f.has_name);

    // 完整设备名(AD type 0x09)。
    const uint8_t name_ad[] = {0x06, 0x09, 'E', 'S', 'P', '3', '2'};
    ble_scan_parse_adv(name_ad, sizeof(name_ad), &f);
    assert(f.has_name && strcmp(f.name, "ESP32") == 0);
    assert(!f.has_flags && !f.has_tx_power);

    // 短设备名(AD type 0x08)。
    const uint8_t short_name_ad[] = {0x02, 0x08, 'S'};
    ble_scan_parse_adv(short_name_ad, sizeof(short_name_ad), &f);
    assert(f.has_name && strcmp(f.name, "S") == 0);

    // 未关心的 AD type 跳过,不影响后面的字段。
    const uint8_t unknown_ad[] = {0x03, 0x03, 0x0F, 0x18, 0x02, 0x01, 0x06};
    ble_scan_parse_adv(unknown_ad, sizeof(unknown_ad), &f);
    assert(f.has_flags && f.flags == 0x06 && !f.has_name);

    // 同一 payload 里的多个 AD structure。
    const uint8_t multi_ad[] = {0x02, 0x01, 0x06,
                                0x02, 0x0A, 0xF4,
                                0x06, 0x09, 'E', 'S', 'P', '3', '2'};
    ble_scan_parse_adv(multi_ad, sizeof(multi_ad), &f);
    assert(f.has_flags && f.flags == 0x06);
    assert(f.has_tx_power && f.tx_power == -12);
    assert(f.has_name && strcmp(f.name, "ESP32") == 0);

    // 空名字的 AD(长度为 1)不应阻止后续 AD 的解析。
    const uint8_t empty_name_then_flags[] = {0x01, 0x09, 0x02, 0x01, 0x06};
    ble_scan_parse_adv(empty_name_then_flags, sizeof(empty_name_then_flags), &f);
    assert(f.has_flags && f.flags == 0x06 && !f.has_name);

    // 完整名优先:短名在前、完整名在后时使用完整名。
    const uint8_t short_then_complete[] = {0x02, 0x08, 'S',
                                           0x06, 0x09, 'E', 'S', 'P', '3', '2'};
    ble_scan_parse_adv(short_then_complete, sizeof(short_then_complete), &f);
    assert(f.has_name && strcmp(f.name, "ESP32") == 0);

    // 完整名之后到达的短名不得覆盖完整名。
    const uint8_t complete_then_short[] = {0x06, 0x09, 'E', 'S', 'P', '3', '2',
                                           0x02, 0x08, 'S'};
    ble_scan_parse_adv(complete_then_short, sizeof(complete_then_short), &f);
    assert(f.has_name && strcmp(f.name, "ESP32") == 0);
}

static void test_parse_adv_malformed(void)
{
    ble_adv_fields_t f;

    // AD 长度越过 payload 尾部:安全停止,不读越界。
    const uint8_t truncated_ad[] = {0x10, 0x09, 'A'};
    ble_scan_parse_adv(truncated_ad, sizeof(truncated_ad), &f);
    expect_empty_adv(&f);

    // 只有长度字节、没有类型字节。
    const uint8_t missing_type[] = {0x01};
    ble_scan_parse_adv(missing_type, sizeof(missing_type), &f);
    expect_empty_adv(&f);

    // 长度为 0 表示后续填充,停止解析。
    const uint8_t zero_length[] = {0x02, 0x01, 0x06, 0x00, 0x02, 0x0A, 0xF4};
    ble_scan_parse_adv(zero_length, sizeof(zero_length), &f);
    assert(f.has_flags && f.flags == 0x06);
    assert(!f.has_tx_power && f.tx_power == BLE_SCAN_RSSI_UNKNOWN);

    // 恰好被截断的 TX power:长度声称有数据但少了 1 字节。
    const uint8_t truncated_tx[] = {0x02, 0x0A};
    ble_scan_parse_adv(truncated_tx, sizeof(truncated_tx), &f);
    expect_empty_adv(&f);

    // 超长名字:按 BLE_SCAN_NAME_MAX 截断(含 NUL)。
    uint8_t long_name_ad[2 + 30];
    long_name_ad[0] = 1 + 30;
    long_name_ad[1] = 0x09;
    memset(&long_name_ad[2], 'A', 30);
    ble_scan_parse_adv(long_name_ad, sizeof(long_name_ad), &f);
    assert(f.has_name);
    assert(strlen(f.name) == BLE_SCAN_NAME_MAX - 1);
    for (size_t i = 0; i < strlen(f.name); i++) {
        assert(f.name[i] == 'A');
    }
    assert(f.name[BLE_SCAN_NAME_MAX - 1] == '\0');

    // 多字节字符正好跨在容量边界上时,不得拆出半个 UTF-8 字符。
    uint8_t cjk_ad[2 + 30];
    cjk_ad[0] = 1 + 30;
    cjk_ad[1] = 0x09;
    for (size_t i = 0; i < 10; i++) {
        memcpy(&cjk_ad[2 + i * 3], "\xE4\xB8\xAD", 3);  // U+4E2D
    }
    ble_scan_parse_adv(cjk_ad, sizeof(cjk_ad), &f);
    assert(f.has_name);
    assert(strlen(f.name) == 21);  // 7 个 3 字节字符,23 字节里放不下第 8 个
    for (size_t i = 0; i < 7; i++) {
        assert(memcmp(f.name + i * 3, "\xE4\xB8\xAD", 3) == 0);
    }
    assert(f.name[21] == '\0');

    // 名字里的控制字符被丢弃(0x01 与 DEL 0x7F)。
    const uint8_t ctrl_name_ad[] = {0x06, 0x09, 'A', 0x01, 'B', 0x7F, 'C'};
    ble_scan_parse_adv(ctrl_name_ad, sizeof(ctrl_name_ad), &f);
    assert(f.has_name && strcmp(f.name, "ABC") == 0);

    // 名字里的可见空格保留。
    const uint8_t spaced_name_ad[] = {0x04, 0x09, 'A', ' ', 'B'};
    ble_scan_parse_adv(spaced_name_ad, sizeof(spaced_name_ad), &f);
    assert(f.has_name && strcmp(f.name, "A B") == 0);
}

// ---------------------------------------------------------------------------
// 设备表:初始化 / 合并
// ---------------------------------------------------------------------------
static void test_table_init_clear(void)
{
    ble_scan_table_t t;

    ble_scan_table_init(&t);
    assert(t.count == 0);

    const uint8_t addr[6] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
    ble_scan_dev_t obs = make_obs(addr, 0, -50, BLE_SCAN_RSSI_UNKNOWN, 0, false, "D", 1000);
    assert(ble_scan_table_merge(&t, &obs) != NULL);
    assert(t.count == 1);

    ble_scan_table_clear(&t);
    assert(t.count == 0);
    ble_scan_table_clear(&t);  // 重复清理安全
    assert(t.count == 0);
}

static void test_table_merge(void)
{
    ble_scan_table_t t;
    const uint8_t addr[6] = {0x06, 0x05, 0x04, 0x03, 0x02, 0x01};

    ble_scan_table_init(&t);
    ble_scan_dev_t obs = make_obs(addr, 0, -60, -8, 0x06, true, "DevA", 1000);
    ble_scan_dev_t *dev = ble_scan_table_merge(&t, &obs);
    assert(dev != NULL);
    assert(t.count == 1);
    assert(dev == &t.devs[0]);
    assert(dev->rssi == -60);
    assert(dev->rssi_ema == -60);
    assert(dev->tx_power == -8);
    assert(dev->adv_flags == 0x06);
    assert(dev->connectable);
    assert(strcmp(dev->name, "DevA") == 0);
    // first_seen / seen_count 由合并函数决定,不照抄观测里的垃圾值。
    assert(dev->first_seen_ms == 1000);
    assert(dev->last_seen_ms == 1000);
    assert(dev->seen_count == 1);

    // 同地址 + 同类型:更新 RSSI(EMA = (prev*3 + new)/4)、时间、计数;
    // 名称只有非空才覆盖;tx_power / flags / connectable 缺失时保持旧值。
    obs = make_obs(addr, 0, -40, BLE_SCAN_RSSI_UNKNOWN, 0, false, "", 2000);
    dev = ble_scan_table_merge(&t, &obs);
    assert(dev != NULL && t.count == 1);
    assert(dev->rssi == -40);
    assert(dev->rssi_ema == -55);  // (-60*3 + -40)/4
    assert(dev->first_seen_ms == 1000);
    assert(dev->last_seen_ms == 2000);
    assert(dev->seen_count == 2);
    assert(dev->tx_power == -8);   // 127 = 未广播,不覆盖
    assert(dev->adv_flags == 0x06);  // 0 = 未广播,不覆盖
    assert(dev->connectable);        // 只做 OR,不降级
    assert(strcmp(dev->name, "DevA") == 0);  // 空名字不覆盖

    // 第三次合并:整数除法向零截断,(-55*3 + -50)/4 = -53。
    obs = make_obs(addr, 0, -50, -4, 0x05, false, "DevB", 3000);
    dev = ble_scan_table_merge(&t, &obs);
    assert(dev != NULL && t.count == 1);
    assert(dev->rssi_ema == -53);
    assert(dev->seen_count == 3);
    assert(dev->tx_power == -4);
    assert(dev->adv_flags == 0x05);
    assert(strcmp(dev->name, "DevB") == 0);

    // RSSI 不可用时保留上一次的有效值。
    obs = make_obs(addr, 0, BLE_SCAN_RSSI_UNKNOWN, BLE_SCAN_RSSI_UNKNOWN, 0, false, "", 4000);
    dev = ble_scan_table_merge(&t, &obs);
    assert(dev != NULL);
    assert(dev->rssi == -50 && dev->rssi_ema == -53);
    assert(dev->last_seen_ms == 4000 && dev->seen_count == 4);
    assert(strcmp(dev->name, "DevB") == 0);

    // 相同地址但地址类型不同 = 不同设备。
    obs = make_obs(addr, 1, -70, BLE_SCAN_RSSI_UNKNOWN, 0, false, "DevR", 5000);
    dev = ble_scan_table_merge(&t, &obs);
    assert(dev != NULL);
    assert(t.count == 2);
    assert(t.devs[1].addr_type == 1);
    assert(t.devs[1].rssi == -70);
}

// ---------------------------------------------------------------------------
// 设备表:容量淘汰
// ---------------------------------------------------------------------------
static void test_table_capacity(void)
{
    ble_scan_table_t t;
    uint8_t addr[6];

    ble_scan_table_init(&t);
    for (size_t i = 0; i < BLE_SCAN_TABLE_CAP; i++) {
        addr_with_id(addr, (uint8_t)(i + 1));
        ble_scan_dev_t obs = make_obs(addr, 0, -50 - (int)i, BLE_SCAN_RSSI_UNKNOWN, 0,
                                      false, "D", (uint32_t)(100 + i));
        assert(ble_scan_table_merge(&t, &obs) != NULL);
    }
    assert(t.count == BLE_SCAN_TABLE_CAP);

    // 表满时合并已有设备:不淘汰任何条目。
    addr_with_id(addr, 2);
    ble_scan_dev_t dup = make_obs(addr, 0, -40, BLE_SCAN_RSSI_UNKNOWN, 0, false, "", 2000);
    ble_scan_dev_t *same = ble_scan_table_merge(&t, &dup);
    assert(same == &t.devs[1]);
    assert(t.count == BLE_SCAN_TABLE_CAP);
    assert(same->seen_count == 2 && same->rssi == -40);

    // 表满时新设备淘汰 last_seen 最旧的一条(id=1,last_seen=100)。
    addr_with_id(addr, 0xFF);
    ble_scan_dev_t newcomer = make_obs(addr, 0, -70, BLE_SCAN_RSSI_UNKNOWN, 0, false, "N", 5000);
    ble_scan_dev_t *added = ble_scan_table_merge(&t, &newcomer);
    assert(added != NULL);
    assert(t.count == BLE_SCAN_TABLE_CAP);
    assert(added == &t.devs[0]);  // 淘汰后复用最旧的槽位
    assert(table_has_addr_id(&t, 0xFF));
    assert(!table_has_addr_id(&t, 1));

    // 所有条目 last_seen 相同时,严格大于才替换,平局保留槽位 0。
    ble_scan_table_t tie;
    ble_scan_table_init(&tie);
    for (size_t i = 0; i < BLE_SCAN_TABLE_CAP; i++) {
        addr_with_id(addr, (uint8_t)(i + 1));
        ble_scan_dev_t obs = make_obs(addr, 0, -50, BLE_SCAN_RSSI_UNKNOWN, 0, false, "D", 5000);
        assert(ble_scan_table_merge(&tie, &obs) != NULL);
    }
    addr_with_id(addr, 0xFE);
    ble_scan_dev_t tie_new = make_obs(addr, 0, -50, BLE_SCAN_RSSI_UNKNOWN, 0, false, "N", 5000);
    added = ble_scan_table_merge(&tie, &tie_new);
    assert(added == &tie.devs[0]);
    assert(added->addr[0] == 0xFE);
    assert(tie.count == BLE_SCAN_TABLE_CAP);
}

// ---------------------------------------------------------------------------
// 设备表:过期清理
// ---------------------------------------------------------------------------
static void test_table_expire(void)
{
    ble_scan_table_t t;
    uint8_t addr[6];

    ble_scan_table_init(&t);
    addr_with_id(addr, 0x11);
    ble_scan_dev_t e1 = make_obs(addr, 0, -60, BLE_SCAN_RSSI_UNKNOWN, 0, false, "E1", 1000);
    addr_with_id(addr, 0x22);
    ble_scan_dev_t e2 = make_obs(addr, 0, -60, BLE_SCAN_RSSI_UNKNOWN, 0, false, "E2", 5000);
    addr_with_id(addr, 0x33);
    ble_scan_dev_t e3 = make_obs(addr, 0, -60, BLE_SCAN_RSSI_UNKNOWN, 0, false, "E3", 15000);
    (void)ble_scan_table_merge(&t, &e1);
    (void)ble_scan_table_merge(&t, &e2);
    (void)ble_scan_table_merge(&t, &e3);
    assert(t.count == 3);

    // now-last_seen >= stale:15000 时过期(e1 恰好等于阈值,应删除)。
    ble_scan_table_expire(&t, 16000, BLE_SCAN_STALE_MS);
    assert(t.count == 2);
    assert(!table_has_addr_id(&t, 0x11));
    assert(table_has_addr_id(&t, 0x22));
    assert(table_has_addr_id(&t, 0x33));

    // 没有过期条目时数量不变。
    ble_scan_table_expire(&t, 16000, BLE_SCAN_STALE_MS);
    assert(t.count == 2);

    // 时间戳跨 2^32 回绕时按无符号差值判断。
    ble_scan_table_t w;
    ble_scan_table_init(&w);
    addr_with_id(addr, 0xA1);
    ble_scan_dev_t w1 = make_obs(addr, 0, -60, BLE_SCAN_RSSI_UNKNOWN, 0, false, "W1",
                                 0xFFFFFF00u);  // 4294967040
    addr_with_id(addr, 0xA2);
    ble_scan_dev_t w2 = make_obs(addr, 0, -60, BLE_SCAN_RSSI_UNKNOWN, 0, false, "W2", 100);
    (void)ble_scan_table_merge(&w, &w1);
    (void)ble_scan_table_merge(&w, &w2);
    // now=1000:W1 的差值是 1256,小于 2000,必须保留。
    ble_scan_table_expire(&w, 1000, 2000);
    assert(w.count == 2);
    // stale=1000 时 W1(1256)过期,W2(900)保留。
    ble_scan_table_expire(&w, 1000, 1000);
    assert(w.count == 1);
    assert(w.devs[0].addr[0] == 0xA2);
}

// ---------------------------------------------------------------------------
// 设备表:排序
// ---------------------------------------------------------------------------
static void test_table_rank(void)
{
    ble_scan_table_t t;
    uint8_t addr[6];
    size_t order[BLE_SCAN_TABLE_CAP];

    ble_scan_table_init(&t);
    addr_with_id(addr, 0x01);
    ble_scan_dev_t d1 = make_obs(addr, 0, -60, BLE_SCAN_RSSI_UNKNOWN, 0, false, "D1", 1000);
    addr_with_id(addr, 0x02);
    ble_scan_dev_t d2 = make_obs(addr, 0, -60, BLE_SCAN_RSSI_UNKNOWN, 0, false, "D2", 2000);
    addr_with_id(addr, 0x03);
    ble_scan_dev_t d3 = make_obs(addr, 0, -40, BLE_SCAN_RSSI_UNKNOWN, 0, false, "D3", 500);
    addr_with_id(addr, 0x04);
    ble_scan_dev_t d4 = make_obs(addr, 0, -80, BLE_SCAN_RSSI_UNKNOWN, 0, false, "D4", 1900);
    (void)ble_scan_table_merge(&t, &d1);  // index 0
    (void)ble_scan_table_merge(&t, &d2);  // index 1
    (void)ble_scan_table_merge(&t, &d3);  // index 2
    (void)ble_scan_table_merge(&t, &d4);  // index 3

    // 先按 rssi_ema 降序;同信号时 last_seen 新的在前。
    size_t n = ble_scan_table_rank(&t, 2000, BLE_SCAN_STALE_MS, order, BLE_SCAN_TABLE_CAP);
    assert(n == 4);
    assert(order[0] == 2);  // -40
    assert(order[1] == 1);  // -60 @2000
    assert(order[2] == 0);  // -60 @1000
    assert(order[3] == 3);  // -80
    assert(t.count == 4);   // rank 不修改表

    // order_cap 限制输出条数。
    n = ble_scan_table_rank(&t, 2000, BLE_SCAN_STALE_MS, order, 2);
    assert(n == 2);
    assert(order[0] == 2 && order[1] == 1);

    // 过期条目被排除:now-last_seen >= stale。
    n = ble_scan_table_rank(&t, 2500, 1500, order, BLE_SCAN_TABLE_CAP);
    assert(n == 2);  // d2(500ms) 与 d4(600ms) 未过期
    assert(order[0] == 1 && order[1] == 3);

    // 全部过期时返回 0。
    n = ble_scan_table_rank(&t, 2500, 300, order, BLE_SCAN_TABLE_CAP);
    assert(n == 0);

    // 地址升序(原始 val[] 字节序,addr[0] 先比较)。
    ble_scan_table_t a;
    ble_scan_table_init(&a);
    const uint8_t addr_hi[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x00};
    const uint8_t addr_lo[6] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
    ble_scan_dev_t ahi = make_obs(addr_hi, 0, -50, BLE_SCAN_RSSI_UNKNOWN, 0, false, "HI", 1000);
    ble_scan_dev_t alo = make_obs(addr_lo, 0, -50, BLE_SCAN_RSSI_UNKNOWN, 0, false, "LO", 1000);
    ble_scan_dev_t alo_rand = make_obs(addr_lo, 1, -50, BLE_SCAN_RSSI_UNKNOWN, 0, false, "LR", 1000);
    (void)ble_scan_table_merge(&a, &ahi);      // index 0
    (void)ble_scan_table_merge(&a, &alo);      // index 1
    (void)ble_scan_table_merge(&a, &alo_rand); // index 2
    n = ble_scan_table_rank(&a, 1000, BLE_SCAN_STALE_MS, order, BLE_SCAN_TABLE_CAP);
    assert(n == 3);
    assert(order[0] == 1);  // addr 01
    assert(order[1] == 2);  // 相同地址时公共地址(type 0)在随机地址(type 1)前
    assert(order[2] == 0);  // addr 02

    // NULL 表 / 容量 0 安全返回 0。
    assert(ble_scan_table_rank(NULL, 1000, BLE_SCAN_STALE_MS, order, 4) == 0);
    assert(ble_scan_table_rank(&a, 1000, BLE_SCAN_STALE_MS, NULL, 4) == 0);
    assert(ble_scan_table_rank(&a, 1000, BLE_SCAN_STALE_MS, order, 0) == 0);
}

// ---------------------------------------------------------------------------
// 信号等级 / 地址格式化
// ---------------------------------------------------------------------------
static void test_signal_level(void)
{
    // 信号等级同时是信号格数 0..4。
    assert(BLE_SIGNAL_NONE == 0);
    assert(BLE_SIGNAL_WEAK == 1);
    assert(BLE_SIGNAL_FAIR == 2);
    assert(BLE_SIGNAL_GOOD == 3);
    assert(BLE_SIGNAL_STRONG == 4);

    assert(ble_signal_level(BLE_SCAN_RSSI_UNKNOWN) == BLE_SIGNAL_NONE);
    assert(ble_signal_level(127) == BLE_SIGNAL_NONE);
    assert(ble_signal_level(0) == BLE_SIGNAL_NONE);
    assert(ble_signal_level(1) == BLE_SIGNAL_NONE);

    // 阈值边界:-55 / -67 / -78。
    assert(ble_signal_level(-1) == BLE_SIGNAL_STRONG);
    assert(ble_signal_level(-30) == BLE_SIGNAL_STRONG);
    assert(ble_signal_level(-54) == BLE_SIGNAL_STRONG);
    assert(ble_signal_level(-55) == BLE_SIGNAL_STRONG);
    assert(ble_signal_level(-56) == BLE_SIGNAL_GOOD);
    assert(ble_signal_level(-66) == BLE_SIGNAL_GOOD);
    assert(ble_signal_level(-67) == BLE_SIGNAL_GOOD);
    assert(ble_signal_level(-68) == BLE_SIGNAL_FAIR);
    assert(ble_signal_level(-77) == BLE_SIGNAL_FAIR);
    assert(ble_signal_level(-78) == BLE_SIGNAL_FAIR);
    assert(ble_signal_level(-79) == BLE_SIGNAL_WEAK);
    assert(ble_signal_level(-100) == BLE_SIGNAL_WEAK);
    assert(ble_signal_level(-128) == BLE_SIGNAL_WEAK);

    // 单调性:RSSI 越大,等级不得下降。
    for (int rssi = -119; rssi <= -1; rssi++) {
        assert(ble_signal_level(rssi) >= ble_signal_level(rssi - 1));
    }
}

static void test_format_addr(void)
{
    char out[18];

    // NimBLE val[] 低字节在前,显示时反转并大写。
    const uint8_t addr[6] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
    ble_scan_format_addr(addr, out);
    assert(strcmp(out, "06:05:04:03:02:01") == 0);
    assert(strlen(out) == 17);

    const uint8_t zeros[6] = {0, 0, 0, 0, 0, 0};
    ble_scan_format_addr(zeros, out);
    assert(strcmp(out, "00:00:00:00:00:00") == 0);

    const uint8_t hex[6] = {0xAB, 0xCD, 0xEF, 0x01, 0x23, 0x45};
    ble_scan_format_addr(hex, out);
    assert(strcmp(out, "45:23:01:EF:CD:AB") == 0);
}

int main(void)
{
    test_utf8_decode();
    test_utf8_encode();
    test_utf8_copy();
    test_parse_adv_basic();
    test_parse_adv_malformed();
    test_table_init_clear();
    test_table_merge();
    test_table_capacity();
    test_table_expire();
    test_table_rank();
    test_signal_level();
    test_format_addr();
    puts("BLE scan model tests: PASS");
    return 0;
}
