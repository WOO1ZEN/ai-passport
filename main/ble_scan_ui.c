// main/ble_scan_ui.c —— BLE 信号扫描应用的界面层。
//
// 视觉主题:深色"扫描器/雷达"面板 —— 近黑背景、青绿色强调、细边框卡片、
// 分格信号柱。它不复用 main/ui_pixel.c 的演示外壳、菜单或吉祥物,所有页面
// 都由本文件自己创建。
//
// 页面与按键:
//   * 列表页:标题/电量、扫描状态、选中设备信号卡、5 行窗口式设备列表
//     (UP/DOWN 可浏览全部扫描结果,窗口随选中项滚动)、底部提示;
//     UP/DOWN 短按移动选中项(循环),OK 短按进详情,OK 长按暂停/恢复扫描;
//   * 详情页:标题/电量、设备名与 MAC、大号 RSSI + 5 格信号柱、设备信息行;
//     UP/DOWN 短按切换设备(循环),OK 短按返回列表,OK 长按连接并读取信息;
//   * 设备信息页:NOVA/OMNI/HALO/PACE/AURA/SEEKER 的型号、名称、SN、固件/硬件(系统)版本、状态、
//     电量与读取进度;OK 短按返回详情(读取中忽略),OK 长按在读取中取消、
//     空闲时重读。
//
// 并发约定(见 docs/development/ai-guide.md#runtime-invariants):
//   * 250 ms lv_timer 回调运行在 LVGL 任务中,回调里不重复加锁;
//   * 按键处理运行在应用输入任务中,自己 bsp_lvgl_lock(250);
//   * ble_scan_*() 只取数据快照,不碰 LVGL;电量由 main.c 电源任务在锁外读好后
//     通过 ble_scan_ui_set_battery() 推送,避免把 I2C 读塞进 LVGL 锁;
//   * 设备名是不可信 UTF-8:显示前逐码点检查 app_font_16 的字形覆盖,
//     缺字形替换为 '?'(见 ui_format_device_name)。
//
// 内存(无 PSRAM):整表快照(24 条)+ 三个页面共约 70 个 lv_obj,无每次刷新 malloc,
// 文本都写入静态缓冲区。
#include "ble_scan_ui.h"

#include "ble_client.h"
#include "ble_scan.h"
#include "ble_scan_model.h"
#include "bsp_display.h"
#include "device_proto.h"
#include "esp_timer.h"
#include "lvgl.h"
#include "nova_proto.h"

#include <stdio.h>
#include <string.h>

// 全部中文/西文标签统一使用应用子集字体(U+0020-0x7E + 批准的中文字符清单),
// 大号 RSSI 用 Montserrat 20,MAC/小号纯 ASCII 行用 Montserrat 14。
// 字体绑定只在这里出现:换字体只改这一行。
#include "app_font.h"
#define UI_FONT_TEXT (&app_font_16)
#define UI_FONT_BIG (&lv_font_montserrat_20)
#define UI_FONT_MONO (&lv_font_montserrat_14)

#define UI_W 240
#define UI_MARGIN 8
#define UI_LIST_ROWS 5
// 快照取整个设备表:列表用 UI_LIST_ROWS 行窗口滚动浏览全部,详情页也可遍历全部,
// 也不能把"排名第 5+ 但仍在范围"的设备误判成已离开范围。
#define UI_SNAPSHOT_CAP BLE_SCAN_TABLE_CAP
#define UI_REFRESH_MS 250

// 深色扫描面板配色。
#define UI_COLOR_BG 0x050A0E
#define UI_COLOR_PANEL 0x0B161C
#define UI_COLOR_PANEL_BORDER 0x17333D
#define UI_COLOR_ROW 0x0A1216
#define UI_COLOR_ROW_BORDER 0x13252C
#define UI_COLOR_ROW_SEL 0x0B2B33
#define UI_COLOR_TEXT 0xD8EFF3
#define UI_COLOR_MUTED 0x6E939C
#define UI_COLOR_CYAN 0x3BE3E9
#define UI_COLOR_DIM 0x22424C
#define UI_COLOR_BAR_OFF 0x1C343D
#define UI_COLOR_AMBER 0xFFB454
#define UI_COLOR_RED 0xFF6B6B

typedef enum {
    UI_SCREEN_LIST = 0,
    UI_SCREEN_DETAIL,
    UI_SCREEN_INFO,     // 连接并读取设备信息(第三页)
} ui_screen_t;

// ---- 页面对象 ----
static lv_obj_t *s_list_scr;
static lv_obj_t *s_detail_scr;

// 列表页
static lv_obj_t *s_status_lbl;
static lv_obj_t *s_battery_list;
static lv_obj_t *s_card_name;
static lv_obj_t *s_card_mac;
static lv_obj_t *s_card_rssi;
static lv_obj_t *s_card_quality;
static lv_obj_t *s_card_last;
static lv_obj_t *s_card_bars[5];
static lv_obj_t *s_row[UI_LIST_ROWS];
static lv_obj_t *s_row_name[UI_LIST_ROWS];
static lv_obj_t *s_row_rssi[UI_LIST_ROWS];
static lv_obj_t *s_empty_title;
static lv_obj_t *s_empty_hint;
static lv_obj_t *s_list_footer;

// 详情页
static lv_obj_t *s_dt_battery;
static lv_obj_t *s_dt_name;
static lv_obj_t *s_dt_mac;
static lv_obj_t *s_dt_rssi;
static lv_obj_t *s_dt_quality;
static lv_obj_t *s_dt_away_lbl;
static lv_obj_t *s_dt_bars[5];
static lv_obj_t *s_dt_info[5];

// 设备信息页(详情页 OK 长按进入)
static lv_obj_t *s_info_scr;
static lv_obj_t *s_info_battery;
static lv_obj_t *s_info_row[7];
static lv_obj_t *s_info_hint;
static lv_obj_t *s_info_status;
static lv_obj_t *s_info_detail;   // 失败详情单独一行(Montserrat 14,仅 ASCII)

// ---- 运行时状态 ----
static lv_timer_t *s_timer;
static bool s_created;
static ui_screen_t s_screen;
// 列表过滤:默认只显示广播里带名称的设备。手机/手表/信标这类强信号设备往往
// 完全不广播名称,默认隐藏它们才能让屏幕直接呈现"有名字的设备"。
// 长按上键可临时切换为显示全部设备。
static bool s_named_only = true;

// 定时器最近一次取到的按信号排序快照(整张表;列表用 UI_LIST_ROWS 行窗口浏览全部)。
static ble_scan_dev_t s_devs[UI_SNAPSHOT_CAP];
static size_t s_dev_count;
static uint32_t s_now_ms;

// 列表页选中项:以地址而不是下标记录,重排后不会跳到别的设备。
static uint8_t s_sel_addr[6];
static uint8_t s_sel_type;
static bool s_sel_valid;
static size_t s_sel_index;
// 列表窗口偏移:5 个行控件始终复用,只显示 s_devs[s_list_offset .. +4];
// 选中项移动时窗口跟随,从而可浏览整张设备表(最多 BLE_SCAN_TABLE_CAP 条)。
static size_t s_list_offset;

// 详情页当前设备:保留最后已知值;设备过期后标记 s_dt_away 但仍显示。
static ble_scan_dev_t s_dt_dev;
static uint8_t s_dt_addr[6];
static uint8_t s_dt_type;
static bool s_dt_valid;
static bool s_dt_away;

// 避免每 250 ms 重复写相同文本导致整屏失效。
static int s_list_bars_lit = -1;
static int s_detail_bars_lit = -1;
static int s_battery_pct;
// 每行当前应用的"选中/未选中"样式;0 = 未选中(与创建时的默认样式一致),
// 只在变化时重设样式,避免 250 ms 刷新反复让整行失效重绘。
static int32_t s_row_sel_applied[UI_LIST_ROWS];

// 设备信息页:触发读取时拷下的设备(名称兜底与"长按重读"目标)以及最近一次
// ble_client_get_info() 快照。s_info 只在 LVGL 任务/持锁上下文里读写。
static ble_scan_dev_t s_info_dev;
static ble_client_info_t s_info;
static uint32_t s_info_status_color = 0xFFFFFFFFu;
// 用户已按长按请求取消:客户端 worker 收尾(FAILED)前保持"正在取消..."。
static bool s_info_cancel_requested;
static char s_info_buf[160];
static char s_val_buf[64];

// 复用缓冲区:容量足够容纳 24 字节设备名逐码点替换为 '?' 后的结果。
static char s_name_buf[BLE_SCAN_NAME_MAX * 2 + 1];
static char s_text_buf[64];
static char s_age_buf[24];
static char s_mac_buf[18];

// ---------------------------------------------------------------------------
// 小工具
// ---------------------------------------------------------------------------

// 某个码点在实际字体(含 fallback 链)里是否有真实字形。is_placeholder 为真
// 说明只画占位框,不算覆盖。
static bool ui_font_has_glyph(const lv_font_t *font, uint32_t codepoint)
{
    if (!font) return false;
    lv_font_glyph_dsc_t dsc = { 0 };
    return lv_font_get_glyph_dsc(font, &dsc, codepoint, 0) && !dsc.is_placeholder;
}

// 把不可信设备名转换为可显示文本:非法 UTF-8 与缺字形码点统一替换为 '?'。
// 空格始终保留(子集字体的 U+0020 即使被标成占位符也不影响排版语义)。
// 返回 dst 始终 NUL 结尾;截断只发生在码点边界。
static void ui_format_device_name(char *dst, size_t cap, const char *src)
{
    if (!dst || cap == 0) return;
    dst[0] = '\0';
    if (!src || src[0] == '\0') return;

    const size_t len = strlen(src);
    size_t in = 0;
    size_t out = 0;
    while (in < len) {
        uint32_t cp = 0;
        size_t used = ble_scan_utf8_decode(src + in, len - in, &cp);
        if (used == 0) {
            cp = '?';
            used = 1;
        }
        in += used;
        if (cp != ' ' && !ui_font_has_glyph(UI_FONT_TEXT, cp)) cp = '?';

        char enc[4];
        size_t enc_len = ble_scan_utf8_encode(cp, enc, sizeof(enc));
        if (enc_len == 0) {
            enc[0] = '?';
            enc_len = 1;
        }
        if (out + enc_len + 1 > cap) break;
        memcpy(dst + out, enc, enc_len);
        out += enc_len;
    }
    dst[out] = '\0';
}

static const char *ui_quality_text(ble_signal_level_t level)
{
    switch (level) {
    case BLE_SIGNAL_STRONG: return "极强";
    case BLE_SIGNAL_GOOD:   return "强";
    case BLE_SIGNAL_FAIR:   return "中";
    case BLE_SIGNAL_WEAK:   return "弱";
    default:                return "无信号";
    }
}

// 信号等级 0..4 → 5 格信号柱的点亮格数。NONE=0 格;WEAK/FAIR/GOOD/STRONG
// 依次点亮 2/3/4/5 格,让 5 格柱的每一格都参与显示。
static int ui_bars_lit(ble_signal_level_t level)
{
    if (level <= BLE_SIGNAL_NONE) return 0;
    return (int)level + 1;
}

static void ui_set_text(lv_obj_t *label, const char *text)
{
    if (!label || !text) return;
    const char *old = lv_label_get_text(label);
    if (old && strcmp(old, text) == 0) return;
    lv_label_set_text(label, text);
}

static void ui_set_bars(lv_obj_t *const bars[5], int lit, int *cache)
{
    if (cache && *cache == lit) return;
    if (cache) *cache = lit;
    for (int i = 0; i < 5; i++) {
        if (!bars[i]) continue;
        const uint32_t color = (i < lit) ? UI_COLOR_CYAN : UI_COLOR_BAR_OFF;
        lv_obj_set_style_bg_color(bars[i], lv_color_hex(color), 0);
    }
}

// "刚刚" / "N 秒前"。now_ms/last_ms 都来自 esp_timer,无符号差值可跨回绕。
static void ui_format_age(char *buf, size_t cap, uint32_t last_ms)
{
    const uint32_t age_ms = s_now_ms - last_ms;
    if (age_ms < 1000u) {
        snprintf(buf, cap, "刚刚");
    } else {
        snprintf(buf, cap, "%lu 秒前", (unsigned long)(age_ms / 1000u));
    }
}

static void ui_format_rssi(char *buf, size_t cap, int8_t rssi)
{
    if (ble_signal_level(rssi) == BLE_SIGNAL_NONE) {
        snprintf(buf, cap, "-- dBm");
    } else {
        snprintf(buf, cap, "%d dBm", (int)rssi);
    }
}

// ---------------------------------------------------------------------------
// 控件工厂
// ---------------------------------------------------------------------------

static lv_obj_t *ui_screen_create(void)
{
    lv_obj_t *scr = lv_obj_create(NULL);
    lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(scr, lv_color_hex(UI_COLOR_BG), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(scr, 0, 0);
    lv_obj_set_style_radius(scr, 0, 0);
    lv_obj_set_style_pad_all(scr, 0, 0);
    return scr;
}

static lv_obj_t *ui_label(lv_obj_t *parent, const lv_font_t *font, uint32_t color)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_style_text_font(label, font, 0);
    lv_obj_set_style_text_color(label, lv_color_hex(color), 0);
    return label;
}

static lv_obj_t *ui_box(lv_obj_t *parent, int32_t x, int32_t y, int32_t w, int32_t h,
                        uint32_t bg, uint32_t border, int32_t border_w, int32_t radius)
{
    lv_obj_t *box = lv_obj_create(parent);
    lv_obj_remove_flag(box, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_pos(box, x, y);
    lv_obj_set_size(box, w, h);
    lv_obj_set_style_bg_color(box, lv_color_hex(bg), 0);
    lv_obj_set_style_bg_opa(box, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(box, lv_color_hex(border), 0);
    lv_obj_set_style_border_width(box, border_w, 0);
    lv_obj_set_style_radius(box, radius, 0);
    lv_obj_set_style_pad_all(box, 0, 0);
    return box;
}

// 5 格信号柱:宽度相同、高度从左到右递增,底部对齐(base_y 为柱底 y)。
static const int32_t METER_H_SMALL[5] = { 6, 10, 15, 20, 25 };
static const int32_t METER_H_LARGE[5] = { 8, 14, 20, 26, 32 };

static void ui_create_meter(lv_obj_t *parent, lv_obj_t *bars[5], int32_t x,
                            int32_t base_y, int32_t width, int32_t gap,
                            const int32_t heights[5])
{
    for (int i = 0; i < 5; i++) {
        bars[i] = ui_box(parent, x + i * (width + gap), base_y - heights[i],
                         width, heights[i], UI_COLOR_BAR_OFF, UI_COLOR_BAR_OFF, 0, 1);
    }
}

// ---------------------------------------------------------------------------
// 列表页
// ---------------------------------------------------------------------------

static void ui_build_list_screen(lv_obj_t *scr)
{
    lv_obj_t *title = ui_label(scr, UI_FONT_TEXT, UI_COLOR_CYAN);
    lv_label_set_text(title, "BLE 信号扫描");
    // 屏幕四角有 30 px 圆角遮罩:标题左移会让首字符被裁,故内缩到 x=16。
    lv_obj_set_pos(title, 16, 4);

    s_battery_list = ui_label(scr, UI_FONT_MONO, UI_COLOR_MUTED);
    lv_label_set_text(s_battery_list, "--");
    lv_obj_align(s_battery_list, LV_ALIGN_TOP_RIGHT, -12, 8);

    s_status_lbl = ui_label(scr, UI_FONT_TEXT, UI_COLOR_MUTED);
    lv_label_set_text(s_status_lbl, "正在启动蓝牙...");
    lv_obj_set_pos(s_status_lbl, UI_MARGIN, 26);
    lv_label_set_long_mode(s_status_lbl, LV_LABEL_LONG_MODE_CLIP);
    lv_obj_set_width(s_status_lbl, UI_W - 2 * UI_MARGIN);

    // 选中设备信号卡
    lv_obj_t *card = ui_box(scr, UI_MARGIN, 48, UI_W - 2 * UI_MARGIN, 100,
                            UI_COLOR_PANEL, UI_COLOR_PANEL_BORDER, 1, 10);

    s_card_name = ui_label(card, UI_FONT_TEXT, UI_COLOR_TEXT);
    lv_obj_set_pos(s_card_name, 10, 0);
    lv_obj_set_width(s_card_name, 140);
    lv_label_set_long_mode(s_card_name, LV_LABEL_LONG_MODE_CLIP);

    s_card_mac = ui_label(card, UI_FONT_MONO, UI_COLOR_MUTED);
    lv_obj_set_pos(s_card_mac, 10, 20);
    lv_obj_set_width(s_card_mac, 140);
    lv_label_set_long_mode(s_card_mac, LV_LABEL_LONG_MODE_CLIP);

    s_card_rssi = ui_label(card, UI_FONT_BIG, UI_COLOR_CYAN);
    lv_obj_set_pos(s_card_rssi, 10, 36);
    lv_obj_set_width(s_card_rssi, 140);
    lv_label_set_long_mode(s_card_rssi, LV_LABEL_LONG_MODE_CLIP);

    s_card_quality = ui_label(card, UI_FONT_TEXT, UI_COLOR_TEXT);
    lv_obj_set_pos(s_card_quality, 10, 58);

    // "最近更新: ..." 是中文文案,必须用 app_font_16:Montserrat 14 没有中文字形,
    // 错绑时会整行画成占位方块。app_font_16 行高 20,放在 78(78+20=98<100)。
    s_card_last = ui_label(card, UI_FONT_TEXT, UI_COLOR_MUTED);
    lv_obj_set_pos(s_card_last, 10, 78);

    ui_create_meter(card, s_card_bars, 160, 66, 8, 3, METER_H_SMALL);

    // 5 行设备列表:152..262,行高 22、步进 22。app_font_16 行高 20,
    // 22 - 上下各 1 px 边框 = 20 px 内容区,刚好不裁行高;footer 从 266 开始。
    for (size_t i = 0; i < UI_LIST_ROWS; i++) {
        const int32_t y = 152 + (int32_t)i * 22;
        s_row[i] = ui_box(scr, UI_MARGIN, y, UI_W - 2 * UI_MARGIN, 22,
                          UI_COLOR_ROW, UI_COLOR_ROW_BORDER, 1, 6);
        s_row_name[i] = ui_label(s_row[i], UI_FONT_TEXT, UI_COLOR_TEXT);
        lv_obj_set_pos(s_row_name[i], 8, 0);
        lv_obj_set_width(s_row_name[i], 150);
        lv_label_set_long_mode(s_row_name[i], LV_LABEL_LONG_MODE_CLIP);

        s_row_rssi[i] = ui_label(s_row[i], UI_FONT_MONO, UI_COLOR_MUTED);
        lv_label_set_text(s_row_rssi[i], "-- dBm");
        lv_obj_align(s_row_rssi[i], LV_ALIGN_RIGHT_MID, -8, 0);
        lv_obj_add_flag(s_row[i], LV_OBJ_FLAG_HIDDEN);
    }

    // 空状态:列表区里居中显示两行,发现设备后隐藏。
    s_empty_title = ui_label(scr, UI_FONT_TEXT, UI_COLOR_MUTED);
    lv_label_set_text(s_empty_title, "未发现设备");
    lv_obj_set_pos(s_empty_title, UI_MARGIN, 186);
    lv_obj_set_width(s_empty_title, UI_W - 2 * UI_MARGIN);
    lv_obj_set_style_text_align(s_empty_title, LV_TEXT_ALIGN_CENTER, 0);

    s_empty_hint = ui_label(scr, UI_FONT_TEXT, UI_COLOR_DIM);
    lv_label_set_text(s_empty_hint, "保持扫描中...");
    lv_obj_set_pos(s_empty_hint, UI_MARGIN, 210);
    lv_obj_set_width(s_empty_hint, UI_W - 2 * UI_MARGIN);
    lv_obj_set_style_text_align(s_empty_hint, LV_TEXT_ALIGN_CENTER, 0);

    s_list_footer = ui_label(scr, UI_FONT_TEXT, UI_COLOR_MUTED);
    // 两行固定排版:16 px 字体下整句约 296 px,自然折行可能挤出第三行,
    // 这里用显式换行保证只占 2 行(266..306),不会被底部圆角裁掉。
    lv_label_set_text(s_list_footer, "上下键选择 | 确定查看\n长按暂停/恢复");
    lv_obj_set_pos(s_list_footer, UI_MARGIN, 266);
    lv_obj_set_width(s_list_footer, UI_W - 2 * UI_MARGIN);
    lv_obj_set_style_text_align(s_list_footer, LV_TEXT_ALIGN_CENTER, 0);
}

// ---------------------------------------------------------------------------
// 详情页
// ---------------------------------------------------------------------------

static void ui_build_detail_screen(lv_obj_t *scr)
{
    lv_obj_t *title = ui_label(scr, UI_FONT_TEXT, UI_COLOR_CYAN);
    lv_label_set_text(title, "设备详情");
    lv_obj_set_pos(title, 16, 4);

    s_dt_battery = ui_label(scr, UI_FONT_MONO, UI_COLOR_MUTED);
    lv_label_set_text(s_dt_battery, "--");
    lv_obj_align(s_dt_battery, LV_ALIGN_TOP_RIGHT, -12, 8);

    // 标题下的细分割线,强化"扫描器面板"的视觉语言。
    (void)ui_box(scr, UI_MARGIN, 30, UI_W - 2 * UI_MARGIN, 1,
                 UI_COLOR_PANEL_BORDER, UI_COLOR_PANEL_BORDER, 0, 0);

    s_dt_name = ui_label(scr, UI_FONT_TEXT, UI_COLOR_TEXT);
    lv_label_set_text(s_dt_name, "--");
    lv_obj_set_pos(s_dt_name, UI_MARGIN, 36);
    lv_obj_set_width(s_dt_name, UI_W - 2 * UI_MARGIN);
    lv_label_set_long_mode(s_dt_name, LV_LABEL_LONG_MODE_CLIP);

    s_dt_mac = ui_label(scr, UI_FONT_MONO, UI_COLOR_MUTED);
    lv_label_set_text(s_dt_mac, "--");
    lv_obj_set_pos(s_dt_mac, UI_MARGIN, 58);

    s_dt_rssi = ui_label(scr, UI_FONT_BIG, UI_COLOR_CYAN);
    lv_label_set_text(s_dt_rssi, "-- dBm");
    lv_obj_set_pos(s_dt_rssi, UI_MARGIN, 78);

    ui_create_meter(scr, s_dt_bars, 156, 106, 12, 4, METER_H_LARGE);

    s_dt_quality = ui_label(scr, UI_FONT_TEXT, UI_COLOR_TEXT);
    lv_label_set_text(s_dt_quality, "信号: 无信号");
    lv_obj_set_pos(s_dt_quality, UI_MARGIN, 108);

    s_dt_away_lbl = ui_label(scr, UI_FONT_TEXT, UI_COLOR_AMBER);
    lv_label_set_text(s_dt_away_lbl, "已离开范围");
    lv_obj_align(s_dt_away_lbl, LV_ALIGN_TOP_RIGHT, -UI_MARGIN, 108);
    lv_obj_add_flag(s_dt_away_lbl, LV_OBJ_FLAG_HIDDEN);

    // 设备信息面板:5 行固定信息。
    lv_obj_t *panel = ui_box(scr, UI_MARGIN, 136, UI_W - 2 * UI_MARGIN, 136,
                             UI_COLOR_PANEL, UI_COLOR_PANEL_BORDER, 1, 10);
    for (int i = 0; i < 5; i++) {
        s_dt_info[i] = ui_label(panel, UI_FONT_TEXT, UI_COLOR_TEXT);
        lv_obj_set_pos(s_dt_info[i], 10, 8 + i * 25);
        lv_obj_set_width(s_dt_info[i], UI_W - 2 * UI_MARGIN - 20);
        lv_label_set_long_mode(s_dt_info[i], LV_LABEL_LONG_MODE_CLIP);
    }

    lv_obj_t *footer = ui_label(scr, UI_FONT_TEXT, UI_COLOR_MUTED);
    lv_label_set_text(footer, "确定返回 | 长按读取");
    lv_obj_set_pos(footer, UI_MARGIN, 282);
    lv_obj_set_width(footer, UI_W - 2 * UI_MARGIN);
    lv_obj_set_style_text_align(footer, LV_TEXT_ALIGN_CENTER, 0);
}

// ---------------------------------------------------------------------------
// 设备信息页(OK 长按详情页进入:连接读取 NOVA/OMNI/HALO/PACE/AURA/SEEKER 的信息、状态、版本)
// ---------------------------------------------------------------------------

static void ui_build_info_screen(lv_obj_t *scr)
{
    lv_obj_t *title = ui_label(scr, UI_FONT_TEXT, UI_COLOR_CYAN);
    lv_label_set_text(title, "设备信息");
    lv_obj_set_pos(title, 16, 4);

    s_info_battery = ui_label(scr, UI_FONT_MONO, UI_COLOR_MUTED);
    lv_label_set_text(s_info_battery, "--");
    lv_obj_align(s_info_battery, LV_ALIGN_TOP_RIGHT, -12, 8);

    (void)ui_box(scr, UI_MARGIN, 30, UI_W - 2 * UI_MARGIN, 1,
                 UI_COLOR_PANEL_BORDER, UI_COLOR_PANEL_BORDER, 0, 0);

    // 7 行固定信息:行高 20 + 7 px 行距;值统一 CLIP 到 220 px,避免长 SN /
    // 版本号撑破 240 px 屏幕。文案前缀由 ui_update_info() 每帧拼装。
    for (int i = 0; i < 7; i++) {
        s_info_row[i] = ui_label(scr, UI_FONT_TEXT, UI_COLOR_TEXT);
        lv_obj_set_pos(s_info_row[i], 10, 36 + i * 27);
        lv_obj_set_width(s_info_row[i], UI_W - 20);
        lv_label_set_long_mode(s_info_row[i], LV_LABEL_LONG_MODE_CLIP);
    }

    // 最后一行是连接/读取进度或错误;用"提示"小标题和固定信息行分开。
    s_info_hint = ui_label(scr, UI_FONT_TEXT, UI_COLOR_MUTED);
    lv_label_set_text(s_info_hint, "提示");
    lv_obj_set_pos(s_info_hint, 10, 220);

    s_info_status = ui_label(scr, UI_FONT_TEXT, UI_COLOR_MUTED);
    lv_label_set_text(s_info_status, "正在连接...");
    lv_obj_set_pos(s_info_status, 10, 242);
    lv_obj_set_width(s_info_status, UI_W - 20);
    lv_label_set_long_mode(s_info_status, LV_LABEL_LONG_MODE_CLIP);

    // 失败详情是 ASCII 诊断文本:单独一行用 Montserrat 14 显示,前缀"读取失败: "
    // 留在 16px 字体行;这样 220 px 里能放下约 26 个字符,且不需要把整行缩小。
    s_info_detail = ui_label(scr, UI_FONT_MONO, UI_COLOR_RED);
    lv_label_set_text(s_info_detail, "--");
    lv_obj_set_pos(s_info_detail, 10, 262);
    lv_obj_set_width(s_info_detail, UI_W - 20);
    lv_label_set_long_mode(s_info_detail, LV_LABEL_LONG_MODE_CLIP);
    lv_obj_add_flag(s_info_detail, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t *footer = ui_label(scr, UI_FONT_TEXT, UI_COLOR_MUTED);
    lv_label_set_text(footer, "确定返回 | 长按重读");
    lv_obj_set_pos(footer, UI_MARGIN, 282);
    lv_obj_set_width(footer, UI_W - 2 * UI_MARGIN);
    lv_obj_set_style_text_align(footer, LV_TEXT_ALIGN_CENTER, 0);
}

// ---------------------------------------------------------------------------
// 数据刷新
// ---------------------------------------------------------------------------

static bool ui_addr_matches(const ble_scan_dev_t *dev, const uint8_t addr[6], uint8_t addr_type)
{
    return dev->addr_type == addr_type && memcmp(dev->addr, addr, 6) == 0;
}

static bool ui_find_addr(const ble_scan_dev_t *devs, size_t count,
                         const uint8_t addr[6], uint8_t addr_type, size_t *out_index)
{
    for (size_t i = 0; i < count; i++) {
        if (ui_addr_matches(&devs[i], addr, addr_type)) {
            if (out_index) *out_index = i;
            return true;
        }
    }
    return false;
}

static void ui_select_device(const ble_scan_dev_t *dev)
{
    memcpy(s_sel_addr, dev->addr, 6);
    s_sel_type = dev->addr_type;
    s_sel_valid = true;
}

// 让 5 行窗口包含当前选中项:选中项在窗口上方则窗口贴上去,在下方则下移,
// 并把偏移限制在最后 UI_LIST_ROWS 行。空表时窗口回到顶部。
static void ui_list_ensure_visible(void)
{
    if (s_dev_count == 0) {
        s_list_offset = 0;
        return;
    }
    if (s_sel_index < s_list_offset) {
        s_list_offset = s_sel_index;
    } else if (s_sel_index >= s_list_offset + UI_LIST_ROWS) {
        s_list_offset = s_sel_index - UI_LIST_ROWS + 1;
    }
    const size_t max_offset = (s_dev_count > UI_LIST_ROWS) ? (s_dev_count - UI_LIST_ROWS) : 0;
    if (s_list_offset > max_offset) s_list_offset = max_offset;
}

// 选中项按地址跟随设备(允许任意下标,窗口会跟随);只有设备真正消失时才
// 回落到第一台,避免列表刷新因重排跳动或指向不存在的行。
static void ui_sync_list_selection(void)
{
    if (s_dev_count == 0) {
        s_sel_valid = false;
        s_sel_index = 0;
        s_list_offset = 0;
        return;
    }
    size_t index = 0;
    if (!s_sel_valid) {
        // 首次出现设备时优先选中"有名称"的那台(可能在任何位置),窗口跟随。
        s_sel_index = 0;
        for (size_t i = 0; i < s_dev_count; i++) {
            if (s_devs[i].name[0] != '\0') {
                s_sel_index = i;
                break;
            }
        }
        ui_select_device(&s_devs[s_sel_index]);
        ui_list_ensure_visible();
        return;
    }
    if (ui_find_addr(s_devs, s_dev_count, s_sel_addr, s_sel_type, &index)) {
        s_sel_index = index;
        ui_list_ensure_visible();
        return;
    }
    // 设备已过期或被挤出设备表:回落到第一台,窗口回顶。
    s_sel_index = 0;
    ui_select_device(&s_devs[0]);
    ui_list_ensure_visible();
}

static void ui_update_battery(void)
{
    if (s_battery_pct < 0) {
        ui_set_text(s_battery_list, "--");
        ui_set_text(s_dt_battery, "--");
        ui_set_text(s_info_battery, "--");
        return;
    }
    snprintf(s_text_buf, sizeof(s_text_buf), "%d%%", s_battery_pct);
    ui_set_text(s_battery_list, s_text_buf);
    ui_set_text(s_dt_battery, s_text_buf);
    ui_set_text(s_info_battery, s_text_buf);
}

// 设备名显示:有广播名就显示名字(逐码点做字形检查),没有名字就用 MAC 末两字节
// 做区分,避免整屏都是同一条"未命名设备"。
static void ui_format_display_name(const ble_scan_dev_t *dev)
{
    ui_format_device_name(s_name_buf, sizeof(s_name_buf), dev->name);
    if (s_name_buf[0] == '\0') {
        snprintf(s_name_buf, sizeof(s_name_buf), "未命名 %02X:%02X",
                 dev->addr[1], dev->addr[0]);
    }
}

static void ui_update_card(const ble_scan_dev_t *dev)
{
    if (!dev) {
        ui_set_text(s_card_name, "未发现设备");
        ui_set_text(s_card_mac, "--");
        ui_set_text(s_card_rssi, "-- dBm");
        ui_set_text(s_card_quality, "信号: 无信号");
        ui_set_text(s_card_last, "最近更新: --");
        ui_set_bars(s_card_bars, 0, &s_list_bars_lit);
        return;
    }

    ui_format_display_name(dev);
    ui_set_text(s_card_name, s_name_buf);

    ble_scan_format_addr(dev->addr, s_mac_buf);
    ui_set_text(s_card_mac, s_mac_buf);

    ui_format_rssi(s_text_buf, sizeof(s_text_buf), dev->rssi);
    ui_set_text(s_card_rssi, s_text_buf);

    const ble_signal_level_t level = ble_signal_level(dev->rssi);
    snprintf(s_text_buf, sizeof(s_text_buf), "信号: %s", ui_quality_text(level));
    ui_set_text(s_card_quality, s_text_buf);
    ui_set_bars(s_card_bars, ui_bars_lit(level), &s_list_bars_lit);

    ui_format_age(s_age_buf, sizeof(s_age_buf), dev->last_seen_ms);
    snprintf(s_text_buf, sizeof(s_text_buf), "最近更新: %s", s_age_buf);
    ui_set_text(s_card_last, s_text_buf);
}

// 结果多于 UI_LIST_ROWS 行时,状态行用纯 ASCII 的窗口范围代替"发现 N 台",
// 例如 "1-5/23";不新增中文字符。
static void ui_format_window_range(char *buf, size_t cap, size_t count)
{
    size_t start = s_list_offset + 1;
    size_t end = s_list_offset + UI_LIST_ROWS;
    if (end > count) end = count;
    snprintf(buf, cap, "%u-%u/%u", (unsigned)start, (unsigned)end, (unsigned)count);
}

static void ui_update_list(void)
{
    // 结果数量变化(清空/过滤/新一轮扫描)后自愈:窗口与选中项保持合法。
    ui_list_ensure_visible();
    // 状态行:扫描状态 + (结果少时)发现总数 / (结果多时)窗口范围。
    const size_t total = ble_scan_count();
    const ble_scan_state_t scan_state = ble_scan_state();
    // 窗口是建在 s_devs(过滤后的可浏览结果)上的,范围指示也用同一个计数。
    const bool windowed = s_dev_count > UI_LIST_ROWS;

    const char *state_text = NULL;
    switch (scan_state) {
    case BLE_SCAN_SCANNING: state_text = "扫描中"; break;
    case BLE_SCAN_PAUSED:   state_text = "已暂停"; break;
    case BLE_SCAN_FINISHED: state_text = "扫描完成"; break;
    case BLE_SCAN_FAILED:
        snprintf(s_text_buf, sizeof(s_text_buf), "蓝牙启动失败");
        break;
    default:
        // OFF/STARTING:NimBLE host 还没开始出结果。
        snprintf(s_text_buf, sizeof(s_text_buf), "正在启动蓝牙...");
        break;
    }
    if (state_text != NULL) {
        char range[24];
        if (windowed) {
            ui_format_window_range(range, sizeof(range), s_dev_count);
            snprintf(s_text_buf, sizeof(s_text_buf), "%s%s | %s",
                     state_text, s_named_only ? "-仅看名称" : "", range);
        } else {
            const size_t count = s_named_only ? s_dev_count : total;
            snprintf(s_text_buf, sizeof(s_text_buf), "%s%s | 发现 %u 台",
                     state_text, s_named_only ? "-仅看名称" : "", (unsigned)count);
        }
    }
    ui_set_text(s_status_lbl, s_text_buf);
    // 本轮 20 秒扫描结束后"长按暂停/恢复"已无意义,第二行改为"长按重新扫描";
    // 仍然是两行(266..306),不会挤到屏幕底部圆角。
    ui_set_text(s_list_footer, scan_state == BLE_SCAN_FINISHED
                                   ? "上下键选择 | 确定查看\n长按重新扫描"
                                   : "上下键选择 | 确定查看\n长按暂停/恢复");

    ui_update_card(s_dev_count ? &s_devs[s_sel_index] : NULL);

    // 5 个行控件复用为滑窗:s_devs[s_list_offset + i]。滚动只换文本与高亮,
    // 不创建/删除控件;窗口越界时隐藏多余行。
    for (size_t i = 0; i < UI_LIST_ROWS; i++) {
        const size_t dev_index = s_list_offset + i;
        if (dev_index >= s_dev_count) {
            lv_obj_add_flag(s_row[i], LV_OBJ_FLAG_HIDDEN);
            continue;
        }
        lv_obj_remove_flag(s_row[i], LV_OBJ_FLAG_HIDDEN);

        ui_format_display_name(&s_devs[dev_index]);
        ui_set_text(s_row_name[i], s_name_buf);

        ui_format_rssi(s_text_buf, sizeof(s_text_buf), s_devs[dev_index].rssi);
        ui_set_text(s_row_rssi[i], s_text_buf);

        const int32_t selected = (dev_index == s_sel_index) ? 1 : 0;
        if (s_row_sel_applied[i] != selected) {
            s_row_sel_applied[i] = selected;
            lv_obj_set_style_border_width(s_row[i], selected ? 2 : 1, 0);
            lv_obj_set_style_border_color(
                s_row[i], lv_color_hex(selected ? UI_COLOR_CYAN : UI_COLOR_ROW_BORDER), 0);
            lv_obj_set_style_bg_color(
                s_row[i], lv_color_hex(selected ? UI_COLOR_ROW_SEL : UI_COLOR_ROW), 0);
        }
    }

    if (s_dev_count == 0) {
        lv_obj_remove_flag(s_empty_title, LV_OBJ_FLAG_HIDDEN);
        lv_obj_remove_flag(s_empty_hint, LV_OBJ_FLAG_HIDDEN);
        // 过滤开启时的空状态不是"没扫到",而是"扫到但都没名字",要说明白;
        // 开机 20 秒扫描结束后则显示"扫描完成",避免暗示还在持续扫描。
        const char *hint = s_named_only ? "仅看名称 | 长按上键恢复"
                         : (scan_state == BLE_SCAN_FINISHED ? "扫描完成"
                                                            : "保持扫描中...");
        ui_set_text(s_empty_hint, hint);
    } else {
        lv_obj_add_flag(s_empty_title, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_empty_hint, LV_OBJ_FLAG_HIDDEN);
    }
}

static void ui_render_detail(void)
{
    if (!s_dt_valid) return;

    ui_format_device_name(s_name_buf, sizeof(s_name_buf), s_dt_dev.name);
    ui_set_text(s_dt_name, s_name_buf[0] ? s_name_buf : "未命名设备");

    ble_scan_format_addr(s_dt_dev.addr, s_mac_buf);
    ui_set_text(s_dt_mac, s_mac_buf);

    ui_format_rssi(s_text_buf, sizeof(s_text_buf), s_dt_dev.rssi);
    ui_set_text(s_dt_rssi, s_text_buf);

    const ble_signal_level_t level = ble_signal_level(s_dt_dev.rssi);
    snprintf(s_text_buf, sizeof(s_text_buf), "信号: %s", ui_quality_text(level));
    ui_set_text(s_dt_quality, s_text_buf);
    ui_set_bars(s_dt_bars, ui_bars_lit(level), &s_detail_bars_lit);

    if (s_dt_away) {
        lv_obj_remove_flag(s_dt_away_lbl, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(s_dt_away_lbl, LV_OBJ_FLAG_HIDDEN);
    }

    // 信息行:设备离开范围后继续显示最后一次观测值,方便用户记录。
    if (s_dt_dev.addr_type == 0) {
        ui_set_text(s_dt_info[0], "地址类型: 公共");
    } else {
        ui_set_text(s_dt_info[0], "地址类型: 随机");
    }
    ui_set_text(s_dt_info[1], s_dt_dev.connectable ? "可连接: 是" : "可连接: 否");
    if (s_dt_dev.tx_power == BLE_SCAN_RSSI_UNKNOWN) {
        ui_set_text(s_dt_info[2], "TX 功率: 未广播");
    } else {
        snprintf(s_text_buf, sizeof(s_text_buf), "TX 功率: %d dBm", (int)s_dt_dev.tx_power);
        ui_set_text(s_dt_info[2], s_text_buf);
    }
    snprintf(s_text_buf, sizeof(s_text_buf), "广播次数: %lu",
             (unsigned long)s_dt_dev.seen_count);
    ui_set_text(s_dt_info[3], s_text_buf);

    ui_format_age(s_age_buf, sizeof(s_age_buf), s_dt_dev.last_seen_ms);
    snprintf(s_text_buf, sizeof(s_text_buf), "最近更新: %s", s_age_buf);
    ui_set_text(s_dt_info[4], s_text_buf);
}

static void ui_update_detail(void)
{
    size_t index = 0;
    if (s_dt_valid && ui_find_addr(s_devs, s_dev_count, s_dt_addr, s_dt_type, &index)) {
        // 设备还在快照里:持续更新最新值,清除"已离开范围"。
        s_dt_dev = s_devs[index];
        s_dt_away = false;
    } else if (s_dt_valid) {
        // 设备已过期或不在当前快照里:保留最后已知值,只置离开标记。
        s_dt_away = true;
    }
    ui_render_detail();
}

// NOVA 状态等级 → 中文。映射值来自 nova_proto.h;未列出的等级显示"未知状态"。
static const char *ui_nova_level_text(uint8_t level)
{
    switch (nova_state_from_level(level)) {
    case NOVA_STATE_UNINITIALIZED: return "未初始化";
    case NOVA_STATE_INITIALIZING:  return "初始化中";
    case NOVA_STATE_IDLE:          return "空闲";
    case NOVA_STATE_SERVING:       return "发球中";
    case NOVA_STATE_TRANSITION:    return "切换中";
    case NOVA_STATE_PAUSED:        return "暂停";
    case NOVA_STATE_FAULT:         return "故障";
    case NOVA_STATE_SELF_REPAIR:   return "自修复";
    default:                       return "未知状态";
    }
}

// 使用 nova_level 状态表的型号:NOVA 与 P1/P2 发球机系列(OMNI/HALO/PACE/AURA)
// 共用同一套状态等级/状态码解析,只有 SEEKER 的 hawkeye 字符串是另一条路径。
static bool ui_model_is_ball_machine(device_model_t model)
{
    switch (model) {
    case DEVICE_MODEL_NOVA:
    case DEVICE_MODEL_OMNI:
    case DEVICE_MODEL_HALO:
    case DEVICE_MODEL_PACE:
    case DEVICE_MODEL_AURA:
        return true;
    default:
        return false;
    }
}

// 协议字符串不可信:NUL 安全、逐码点做字形检查(与设备名同一套规则)。
// 空串统一显示 "--"。
static const char *ui_info_value(const char *src)
{
    ui_format_device_name(s_val_buf, sizeof(s_val_buf), src ? src : "");
    return s_val_buf[0] ? s_val_buf : "--";
}

// 把 "前缀 + 值" 写进固定缓冲区后更新标签;标签会复制文本,缓冲区可复用。
static void ui_info_set_row(lv_obj_t *label, const char *prefix, const char *value)
{
    snprintf(s_info_buf, sizeof(s_info_buf), "%s%s", prefix, value);
    ui_set_text(label, s_info_buf);
}

// 失败详情行使用 Montserrat 14(纯 ASCII 更宽裕):先按字体规则清洗,再把非
// 可打印 ASCII 换成 '?',最后在空白处截到 26 个字符并补 "..."。这样 64 字节的
// 诊断文本也不会超出 220 px 或与 footer 重叠。
static void ui_format_failure_detail(char *dst, size_t cap, const char *src)
{
    ui_format_device_name(dst, cap, src ? src : "");
    for (char *p = dst; *p != '\0'; p++) {
        const unsigned char c = (unsigned char)*p;
        if (c < 0x20 || c > 0x7e) *p = '?';
    }
    if (dst[0] == '\0') {
        snprintf(dst, cap, "--");
        return;
    }

    const size_t max_chars = 26;
    const size_t len = strlen(dst);
    if (len <= max_chars) return;

    size_t cut = max_chars;
    // 优先在空格处断开,避免把英文单词截一半;找不到合适位置就硬截断。
    for (size_t i = cut; i > 8; i--) {
        if (dst[i] == ' ') {
            cut = i;
            break;
        }
    }
    if (cut + 4 > cap) cut = cap - 4;
    dst[cut] = '\0';
    strncat(dst, "...", cap - strlen(dst) - 1);
}

static void ui_update_info(void)
{
    ble_client_get_info(&s_info);

    // 名称:客户端读到的广播名优先,触发时刻扫描到的名字兜底。
    const char *raw_name = s_info.device_name[0] ? s_info.device_name : s_info_dev.name;

    ui_info_set_row(s_info_row[0], "型号: ", device_model_name(s_info.model));

    ui_format_device_name(s_val_buf, sizeof(s_val_buf), raw_name);
    ui_info_set_row(s_info_row[1], "名称: ", s_val_buf[0] ? s_val_buf : "--");

    ui_info_set_row(s_info_row[2], "SN: ", ui_info_value(s_info.sn));
    ui_info_set_row(s_info_row[3], "固件版本: ", ui_info_value(s_info.firmware_version));

    // SEEKER 的 hardware_version 字段承载的是系统固件版本,标题也要换掉;
    // 其余型号(NOVA/OMNI/HALO/PACE/AURA)一律显示"硬件版本"。
    if (s_info.model == DEVICE_MODEL_SEEKER) {
        ui_info_set_row(s_info_row[4], "系统版本: ", ui_info_value(s_info.hardware_version));
    } else {
        ui_info_set_row(s_info_row[4], "硬件版本: ", ui_info_value(s_info.hardware_version));
    }

    // 发球机系列共用 nova_level 状态表;SEEKER 仍用原始 hawkeye 字符串;
    // 状态无效(未读到 0x02/status)时显示 "--"。
    if (ui_model_is_ball_machine(s_info.model) && s_info.status_valid) {
        ui_info_set_row(s_info_row[5], "设备状态: ", ui_nova_level_text(s_info.nova_level));
    } else if (s_info.model == DEVICE_MODEL_SEEKER && s_info.seeker_state[0] != '\0') {
        ui_info_set_row(s_info_row[5], "设备状态: ", ui_info_value(s_info.seeker_state));
    } else {
        ui_info_set_row(s_info_row[5], "设备状态: ", "--");
    }

    if (s_info.battery_valid) {
        snprintf(s_val_buf, sizeof(s_val_buf), "%u%%", (unsigned)s_info.battery);
        ui_info_set_row(s_info_row[6], "电量: ", s_val_buf);
    } else {
        ui_info_set_row(s_info_row[6], "电量: ", "--");
    }

    // 进度/错误行:颜色只在状态变化时重设,避免每 250 ms 让整行失效重绘。
    // 用户已长按取消时,worker 真正收尾(FAILED)前保持"正在取消..."。
    uint32_t color = UI_COLOR_MUTED;
    const char *status = NULL;
    const char *detail = NULL;

    if (s_info_cancel_requested) {
        if (s_info.state == BLE_CLIENT_CONNECTING || s_info.state == BLE_CLIENT_DISCOVERING ||
            s_info.state == BLE_CLIENT_SUBSCRIBING ||
            s_info.state == BLE_CLIENT_AUTHENTICATING ||
            s_info.state == BLE_CLIENT_READING) {
            status = "正在取消...";
        } else {
            // DONE/FAILED/UNSUPPORTED/IDLE:取消流程已结束,恢复常规渲染。
            s_info_cancel_requested = false;
        }
    }

    if (status == NULL) {
        switch (s_info.state) {
        case BLE_CLIENT_DONE:
            status = "读取完成";
            color = UI_COLOR_CYAN;
            break;
        case BLE_CLIENT_FAILED:
            status = "读取失败: ";
            ui_format_failure_detail(s_val_buf, sizeof(s_val_buf), s_info.detail);
            detail = s_val_buf;
            color = UI_COLOR_RED;
            break;
        case BLE_CLIENT_UNSUPPORTED:
            status = "暂不支持该设备";
            color = UI_COLOR_AMBER;
            break;
        case BLE_CLIENT_AUTHENTICATING:
            status = "正在验证...";
            break;
        case BLE_CLIENT_READING:
            status = "正在读取...";
            break;
        default:
            // IDLE/CONNECTING/DISCOVERING/SUBSCRIBING:统一显示连接进度。
            status = "正在连接...";
            break;
        }
    }

    ui_set_text(s_info_status, status);
    if (s_info_status_color != color) {
        s_info_status_color = color;
        lv_obj_set_style_text_color(s_info_status, lv_color_hex(color), 0);
    }
    if (detail != NULL && detail[0] != '\0') {
        ui_set_text(s_info_detail, detail);
        lv_obj_remove_flag(s_info_detail, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(s_info_detail, LV_OBJ_FLAG_HIDDEN);
    }
}

static void ui_refresh(void)
{
    s_now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    s_dev_count = ble_scan_snapshot(s_devs, UI_SNAPSHOT_CAP);
    if (s_named_only) {
        // "仅看名称"过滤:挤压掉没有广播名的设备(手机/手表/信标通常如此)。
        size_t keep = 0;
        for (size_t i = 0; i < s_dev_count; i++) {
            if (s_devs[i].name[0] != '\0') s_devs[keep++] = s_devs[i];
        }
        s_dev_count = keep;
    }

    ui_sync_list_selection();
    if (s_screen == UI_SCREEN_LIST) {
        ui_update_list();
    } else if (s_screen == UI_SCREEN_DETAIL) {
        ui_update_detail();
    } else {
        // 信息页:250 ms 拉一次客户端快照,原地更新文本/颜色,不重建控件。
        ui_update_info();
    }
}

// 电量由 main.c 的电源任务在 LVGL 锁之外读取后推送进来,避免在 250 ms 刷新
// 回调里做 I2C 读、把 LVGL 锁持有时间拉长到电池芯片的 100 ms 超时上限。
void ble_scan_ui_set_battery(int percent)
{
    if (!s_created) return;
    if (!bsp_lvgl_lock(250)) return;
    s_battery_pct = percent;
    ui_update_battery();
    bsp_lvgl_unlock();
}

static void ui_refresh_timer_cb(lv_timer_t *timer)
{
    (void)timer;
    // 运行在 LVGL 任务且已由 esp_lvgl_port 持锁,这里不再加锁。
    ui_refresh();
}

// ---------------------------------------------------------------------------
// 页面切换
// ---------------------------------------------------------------------------

static void ui_show_detail(void)
{
    if (s_dev_count == 0) return;
    s_dt_dev = s_devs[s_sel_index];
    memcpy(s_dt_addr, s_dt_dev.addr, 6);
    s_dt_type = s_dt_dev.addr_type;
    s_dt_valid = true;
    s_dt_away = false;
    s_screen = UI_SCREEN_DETAIL;
    ui_render_detail();
    lv_screen_load(s_detail_scr);
}

static void ui_show_list(void)
{
    s_screen = UI_SCREEN_LIST;
    // 返回列表时保持详情页那台设备的选中(可以是任意下标),窗口跟随滚动;
    // 设备已过期/被挤出则回落到第一台。
    if (s_dev_count > 0) {
        size_t index = 0;
        if (s_dt_valid && ui_find_addr(s_devs, s_dev_count, s_dt_addr, s_dt_type, &index)) {
            s_sel_index = index;
            ui_select_device(&s_devs[index]);
        } else {
            s_sel_index = 0;
            ui_select_device(&s_devs[0]);
        }
    }
    ui_update_list();
    lv_screen_load(s_list_scr);
}

// 详情页 OK 长按:固定本次读取目标并切到设备信息页。调用方必须持有 LVGL 锁;
// ble_client_request_read() 只入队,由调用方解锁后再调用(不在锁内做 BLE 工作)。
static void ui_show_info(void)
{
    s_info_dev = s_dt_dev;
    memset(&s_info, 0, sizeof(s_info));
    s_info_status_color = 0xFFFFFFFFu;
    s_info_cancel_requested = false;

    // 先用本地型号分类和已扫描到的名字铺一帧占位内容;真正的读取结果由
    // 250 ms 定时器从 ble_client_get_info() 取回后原地替换。
    const device_model_t model = device_model_from_name(s_info_dev.name);
    const char *hw_prefix = (model == DEVICE_MODEL_SEEKER) ? "系统版本: " : "硬件版本: ";

    ui_info_set_row(s_info_row[0], "型号: ", device_model_name(model));
    ui_format_device_name(s_val_buf, sizeof(s_val_buf), s_info_dev.name);
    ui_info_set_row(s_info_row[1], "名称: ", s_val_buf[0] ? s_val_buf : "--");
    ui_info_set_row(s_info_row[2], "SN: ", "--");
    ui_info_set_row(s_info_row[3], "固件版本: ", "--");
    ui_info_set_row(s_info_row[4], hw_prefix, "--");
    ui_info_set_row(s_info_row[5], "设备状态: ", "--");
    ui_info_set_row(s_info_row[6], "电量: ", "--");

    ui_set_text(s_info_status, "正在连接...");
    lv_obj_set_style_text_color(s_info_status, lv_color_hex(UI_COLOR_MUTED), 0);
    s_info_status_color = UI_COLOR_MUTED;
    lv_obj_add_flag(s_info_detail, LV_OBJ_FLAG_HIDDEN);

    s_screen = UI_SCREEN_INFO;
    lv_screen_load(s_info_scr);
}

// 信息页 OK 短按:回到详情页(读取进行中时由调用方拦截,不切页)。
static void ui_show_detail_from_info(void)
{
    s_screen = UI_SCREEN_DETAIL;
    ui_update_detail();
    lv_screen_load(s_detail_scr);
}

// 详情页 UP/DOWN:在"当前快照"里循环切换,并跳过已消失的设备。
static void ui_detail_step(int direction)
{
    if (s_dev_count == 0) return;
    size_t index = 0;
    if (ui_find_addr(s_devs, s_dev_count, s_dt_addr, s_dt_type, &index)) {
        if (direction > 0) {
            index = (index + 1) % s_dev_count;
        } else {
            index = (index + s_dev_count - 1) % s_dev_count;
        }
    } else {
        index = (direction > 0) ? 0 : s_dev_count - 1;
    }

    s_dt_dev = s_devs[index];
    memcpy(s_dt_addr, s_dt_dev.addr, 6);
    s_dt_type = s_dt_dev.addr_type;
    s_dt_valid = true;
    s_dt_away = false;
    ui_render_detail();
}

// 列表页 UP/DOWN:在全部结果里循环移动选中项(不再限于可见 5 行),
// 窗口由 ui_list_ensure_visible() 跟随,选中状态按地址保存。
static void ui_list_step(int direction)
{
    if (s_dev_count == 0) return;
    size_t index = (s_sel_index < s_dev_count) ? s_sel_index : 0;
    if (direction > 0) {
        index = (index + 1) % s_dev_count;
    } else {
        index = (index + s_dev_count - 1) % s_dev_count;
    }
    s_sel_index = index;
    ui_select_device(&s_devs[index]);
    ui_list_ensure_visible();
    ui_update_list();
}

// ---------------------------------------------------------------------------
// 对外接口
// ---------------------------------------------------------------------------

void ble_scan_ui_create(void)
{
    if (s_created) return;

    s_list_scr = ui_screen_create();
    ui_build_list_screen(s_list_scr);

    s_detail_scr = ui_screen_create();
    ui_build_detail_screen(s_detail_scr);

    s_info_scr = ui_screen_create();
    ui_build_info_screen(s_info_scr);

    s_screen = UI_SCREEN_LIST;
    // 电量由 main.c 的电源任务异步推送:创建阶段先显示 "--",不在 LVGL 锁内做 I2C 读。
    s_battery_pct = -1;
    ui_update_battery();
    s_dev_count = 0;
    s_sel_valid = false;
    s_dt_valid = false;
    s_list_offset = 0;

    lv_screen_load(s_list_scr);
    s_timer = lv_timer_create(ui_refresh_timer_cb, UI_REFRESH_MS, NULL);
    s_created = true;

    // 立即画一帧,避免屏幕在第一个 250 ms 周期里保持占位文本。
    s_now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    s_dev_count = ble_scan_snapshot(s_devs, UI_SNAPSHOT_CAP);
    ui_sync_list_selection();
    ui_update_list();
}

void ble_scan_ui_destroy(void)
{
    if (!s_created) return;

    if (s_timer) {
        lv_timer_delete(s_timer);
        s_timer = NULL;
    }

    // 先回到列表页再删除其余页面,避免删除当前活动屏幕。
    if ((s_screen == UI_SCREEN_DETAIL || s_screen == UI_SCREEN_INFO) && s_list_scr) {
        lv_screen_load(s_list_scr);
    }
    if (s_info_scr) {
        lv_obj_delete(s_info_scr);
        s_info_scr = NULL;
    }
    if (s_detail_scr) {
        lv_obj_delete(s_detail_scr);
        s_detail_scr = NULL;
    }
    if (s_list_scr) {
        lv_obj_delete(s_list_scr);
        s_list_scr = NULL;
    }

    s_created = false;
    s_dt_valid = false;
    s_sel_valid = false;
    s_sel_index = 0;
    s_dev_count = 0;
    s_list_offset = 0;
    s_battery_pct = -1;
    // 重建界面后控件是新对象,清掉"上次样式"缓存,让下一帧重新应用选中样式。
    s_list_bars_lit = -1;
    s_detail_bars_lit = -1;
    for (size_t i = 0; i < UI_LIST_ROWS; i++) s_row_sel_applied[i] = 0;
    memset(&s_info, 0, sizeof(s_info));
    s_info_status_color = 0xFFFFFFFFu;
    s_info_cancel_requested = false;
    s_screen = UI_SCREEN_LIST;
}

void ble_scan_ui_handle_key(bsp_btn_t btn, bsp_btn_ev_t ev)
{
    if (ev != BSP_BTN_CLICK && ev != BSP_BTN_LONG) return;
    if (!s_created) return;

    // 长按确定在各页语义不同:列表=暂停/恢复(或失败重试),详情=连接读取信息,
    // 信息页=读取中取消/空闲时重读。BLE 操作要么只入队、要么必须在 LVGL 锁外调用,
    // 因此先取一次当前页再做决定。
    if (btn == BSP_BTN_OK && ev == BSP_BTN_LONG) {
        if (!bsp_lvgl_lock(250)) return;
        const ui_screen_t screen = s_screen;
        if (screen == UI_SCREEN_LIST) {
            bsp_lvgl_unlock();
            const ble_scan_state_t state = ble_scan_state();
            if (state == BLE_SCAN_SCANNING) {
                (void)ble_scan_pause();
            } else if (state == BLE_SCAN_PAUSED) {
                (void)ble_scan_resume();
            } else if (state == BLE_SCAN_FAILED) {
                // 蓝牙初始化/启动失败后,长按确定重试:ble_scan_start() 在 host
                // 仍存活时只重新发起扫描,在未初始化时会重新走完整启动流程。
                (void)ble_scan_start();
            } else if (state == BLE_SCAN_FINISHED) {
                // 本轮 20 秒扫描已结束:长按确定清空旧结果并重新开始一轮扫描。
                // 与 pause/resume 一样在 LVGL 锁外调用;界面由 250 ms 定时器更新。
                (void)ble_scan_restart();
            }
            return;
        }
        if (screen == UI_SCREEN_DETAIL) {
            // 先在锁内固定读取目标并切页,再在锁外投递请求(只入队,不阻塞)。
            ui_show_info();
            bsp_lvgl_unlock();
            (void)ble_client_request_read(&s_info_dev);
            return;
        }
        // 信息页 OK 长按:读取中 → 请求取消(给用户退出长等待的路径);
        // 空闲 → 重读同一台设备。取消只置标志,真正断开由 client 任务完成。
        const bool busy = ble_client_busy();
        s_info_cancel_requested = busy;
        bsp_lvgl_unlock();
        if (busy) {
            (void)ble_client_cancel();
        } else {
            (void)ble_client_request_read(&s_info_dev);
        }
        return;
    }

    if (!bsp_lvgl_lock(250)) return;

    if (s_screen == UI_SCREEN_LIST) {
        if (btn == BSP_BTN_UP && ev == BSP_BTN_LONG) {
            // 长按上键:切换"仅看名称"过滤。
            s_named_only = !s_named_only;
            s_sel_valid = false;
            s_sel_index = 0;
            ui_refresh();
        } else if (btn == BSP_BTN_UP && ev == BSP_BTN_CLICK) {
            ui_list_step(-1);
        } else if (btn == BSP_BTN_DOWN && ev == BSP_BTN_CLICK) {
            ui_list_step(+1);
        } else if (btn == BSP_BTN_OK && ev == BSP_BTN_CLICK) {
            ui_show_detail();
        }
    } else if (s_screen == UI_SCREEN_DETAIL) {
        if (btn == BSP_BTN_UP && ev == BSP_BTN_CLICK) {
            ui_detail_step(-1);
        } else if (btn == BSP_BTN_DOWN && ev == BSP_BTN_CLICK) {
            ui_detail_step(+1);
        } else if (btn == BSP_BTN_OK && ev == BSP_BTN_CLICK) {
            ui_show_list();
        }
    } else {
        // 信息页:只有"确定短按返回详情";UP/DOWN 忽略,读取进行中不离开页面。
        if (btn == BSP_BTN_OK && ev == BSP_BTN_CLICK && !ble_client_busy()) {
            ui_show_detail_from_info();
        }
    }

    bsp_lvgl_unlock();
}
