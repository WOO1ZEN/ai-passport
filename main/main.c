// main/main.c —— FoloToy AI Passport 应用入口:BLE 信号扫描器。
//
// 这是一个"衍生应用",不是基线硬件测试示例:
//   * 启动后直接进入重新设计的深色扫描界面(列表页 + 设备详情页);
//   * 不进入 main/demo_*.c 的测试菜单,也不使用 main/ui_pixel.* 的演示外壳;
//   * 硬件能力(BSP)复用,应用状态、任务与页面都在本文件/ble_scan_ui.c 里。
//
// 按键语义:
//   上/下 短按   列表:移动选中项;详情:切换上/下一台设备
//   确定  短按   列表:进入设备详情;详情:返回列表
//   确定  长按   列表:暂停/恢复扫描;详情:返回列表
//
// 任务与所有权:
//   * NimBLE host 任务由 ble_scan.c 创建并回收;
//   * 按键回调运行在 button 组件的 esp_timer 任务,只入队;
//   * s_input_task 负责把按键事件交给 UI(UI 自己加 LVGL 锁);
//   * s_power_task 每秒检查空闲时间:30 秒把背光降到 15%,120 秒关屏并暂停扫描
//     (记下"自动暂停"),任意按键恢复全亮并恢复扫描;电量每 5 秒在锁外读一次。
#include "ble_client.h"
#include "ble_scan.h"
#include "ble_scan_ui.h"
#include "bsp_battery.h"
#include "bsp_button.h"
#include "bsp_display.h"
#include "bsp_i2c.h"
#include "bsp_pins.h"
#include "key_guard.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

static const char *TAG = "ble_scanner";

#define INPUT_QUEUE_DEPTH       8
#define BUTTON_TASK_STACK       4096
#define BUTTON_TASK_PRIORITY    5
#define POWER_TASK_STACK        3072
#define POWER_TASK_PRIORITY     3
#define IDLE_DIM_AFTER_MS       30000
// 屏幕全关后继续扫描没有意义:关屏同时暂停扫描,按键唤醒时自动恢复。
#define IDLE_SCREEN_OFF_AFTER_MS 120000
#define BACKLIGHT_FULL          100
#define BACKLIGHT_DIM           15
#define BACKLIGHT_OFF           0
#define UI_LOCK_TIMEOUT_MS      1000

typedef struct {
    bsp_btn_t btn;
    bsp_btn_ev_t ev;
} input_event_t;

static QueueHandle_t s_input_queue;
static TaskHandle_t s_input_task;
static TaskHandle_t s_power_task;
static volatile bool s_input_ready;
static volatile uint32_t s_last_input_ms;
static volatile bool s_dimmed;
static volatile bool s_screen_off;
// 因空闲而自动暂停扫描;用户按键后需要自动恢复(手动暂停不受影响)。
static volatile bool s_auto_paused;
// 深睡按键唤醒守卫。本应用暂不主动进入 deep sleep,所以它通常保持撤防;
// 但按键分发路径保留与基线主程序一致的消费顺序:将来若加入醒睡,唤醒键的
// release 不会被输入任务未就绪的早退吞掉(见 main/key_guard.h)。
static key_guard_t s_key_guard;

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

// 把背光恢复成全亮并记录本次输入时间。可从任意任务调用:bsp_display_backlight()
// 只操作 LEDC,不触碰 LVGL。
static void note_user_activity(void)
{
    s_last_input_ms = now_ms();
    if (s_dimmed || s_screen_off) {
        s_dimmed = false;
        s_screen_off = false;
        bsp_display_backlight(BACKLIGHT_FULL);
    }
}

// 按键回调运行在 button 组件的共享 esp_timer 任务:只做背光恢复与入队,
// 不做任何阻塞操作。
static void on_key(bsp_btn_t btn, bsp_btn_ev_t ev, void *user)
{
    (void)user;
    const bool was_screen_off = s_screen_off;
    note_user_activity();

    // 关屏期间只按下(还没形成点击)也要唤醒输入任务,好让它恢复被自动暂停的扫描;
    // 这个 PRESS 事件不会转成 UI 动作(UI 只处理 CLICK/LONG)。
    if (was_screen_off && ev == BSP_BTN_PRESS && s_input_ready && s_input_queue != NULL) {
        const input_event_t wake = { .btn = btn, .ev = ev };
        (void)xQueueSend(s_input_queue, &wake, 0);
    }

    // 守卫必须先于输入任务就绪检查消费:唤醒键的 release 可能早于 s_input_ready,
    // 若被早退丢弃,守卫将一直拦到 deadline 兜底。
    const bool was_armed = s_key_guard.armed;
    if (key_guard_consume(&s_key_guard, ev == BSP_BTN_RELEASE, esp_timer_get_time())) {
        if (was_armed && !s_key_guard.armed) {
            ESP_LOGI(TAG, "唤醒键已松手,按键恢复响应");
        }
        return;
    }

    if (ev != BSP_BTN_CLICK && ev != BSP_BTN_LONG) return;
    if (!s_input_ready || s_input_queue == NULL) return;
    const input_event_t input = { .btn = btn, .ev = ev };
    (void)xQueueSend(s_input_queue, &input, 0);
}

static void input_task(void *arg)
{
    (void)arg;
    input_event_t input;
    for (;;) {
        if (xQueueReceive(s_input_queue, &input, portMAX_DELAY) != pdTRUE) continue;
        s_last_input_ms = now_ms();
        bool just_resumed = false;
        if (s_auto_paused) {
            s_auto_paused = false;
            if (ble_scan_state() == BLE_SCAN_PAUSED) {
                const esp_err_t err = ble_scan_resume();
                if (err != ESP_OK) ESP_LOGW(TAG, "唤醒后恢复扫描失败: %s", esp_err_to_name(err));
            }
            just_resumed = true;
        }
        if (input.ev != BSP_BTN_CLICK && input.ev != BSP_BTN_LONG) continue;  // 唤醒用的 PRESS
        // 用户用长按唤醒了屏幕:这一次 LONG 已经用于"恢复扫描",不要再把它当成
        // 列表页的"暂停"手势,否则会出现唤醒后立刻又暂停的怪异行为。
        if (just_resumed && input.ev == BSP_BTN_LONG) continue;
        ble_scan_ui_handle_key(input.btn, input.ev);
    }
}

// 低功耗与电量:屏幕是这台设备最耗电的部分。连续 30 秒没有按键时把背光降到
// 15%,扫描继续在后台进行;任意按键立即恢复全亮。电量每 5 秒在 LVGL 锁之外
// 读一次并推送给界面,避免电池芯片的 I2C 读把 LVGL 锁持有时间拉长。
#define BATTERY_READ_TICKS 5
static void power_task(void *arg)
{
    (void)arg;
    uint32_t battery_ticks = BATTERY_READ_TICKS;  // 首次循环立即读一次
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (++battery_ticks >= BATTERY_READ_TICKS) {
            battery_ticks = 0;
            ble_scan_ui_set_battery(bsp_battery_soc());
        }
        if (s_screen_off) {
            // 关屏瞬间 BLE 可能还在 STARTING,稍后才真正开扫:这里补一次暂停,
            // 避免出现"屏幕已关、扫描却一直在跑"的耗电状态。
            if (ble_scan_state() == BLE_SCAN_SCANNING && ble_scan_pause() == ESP_OK) {
                s_auto_paused = true;
            }
            continue;
        }
        const uint32_t idle_ms = (uint32_t)(now_ms() - s_last_input_ms);
        if (idle_ms >= IDLE_SCREEN_OFF_AFTER_MS) {
            // 关屏省电;同时把扫描停掉(如果正在扫描),按键唤醒后自动恢复。
            if (ble_scan_state() == BLE_SCAN_SCANNING) {
                if (ble_scan_pause() == ESP_OK) s_auto_paused = true;
            }
            s_screen_off = true;
            s_dimmed = true;
            bsp_display_backlight(BACKLIGHT_OFF);
            ESP_LOGI(TAG, "空闲 %d s,关屏并暂停扫描", IDLE_SCREEN_OFF_AFTER_MS / 1000);
        } else if (!s_dimmed && idle_ms >= IDLE_DIM_AFTER_MS) {
            s_dimmed = true;
            bsp_display_backlight(BACKLIGHT_DIM);
            ESP_LOGI(TAG, "空闲 %d s,背光降到 %d%%", IDLE_DIM_AFTER_MS / 1000, BACKLIGHT_DIM);
        }
    }
}

static esp_err_t input_start(void)
{
    s_input_queue = xQueueCreate(INPUT_QUEUE_DEPTH, sizeof(input_event_t));
    if (s_input_queue == NULL) return ESP_ERR_NO_MEM;
    if (xTaskCreate(input_task, "app_input", BUTTON_TASK_STACK, NULL,
                    BUTTON_TASK_PRIORITY, &s_input_task) != pdPASS) {
        vQueueDelete(s_input_queue);
        s_input_queue = NULL;
        return ESP_ERR_NO_MEM;
    }
    s_input_ready = true;
    const esp_err_t err = bsp_button_init(on_key, NULL);
    if (err != ESP_OK) {
        s_input_ready = false;
        vTaskDelete(s_input_task);
        s_input_task = NULL;
        vQueueDelete(s_input_queue);
        s_input_queue = NULL;
        return err;
    }
    return ESP_OK;
}

void app_main(void)
{
    ESP_LOGI(TAG, "FoloToy AI Passport BLE 信号扫描器启动");

    bsp_i2c_init();
    bsp_i2c_scan();

    if (bsp_display_init() != ESP_OK || bsp_lvgl_init() == NULL) {
        ESP_LOGE(TAG, "显示/LVGL 初始化失败,应用无法继续。"
                      "检查 SPI 接线(MOSI=%d SCLK=%d CS=%d DC=%d BL=%d)",
                 BSP_LCD_MOSI, BSP_LCD_SCLK, BSP_LCD_CS, BSP_LCD_DC, BSP_LCD_BL);
        return;
    }
    bsp_display_backlight(BACKLIGHT_FULL);
    s_last_input_ms = now_ms();

    // 电量计失败不阻塞扫描功能:UI 上显示 "--"。
    const esp_err_t battery_err = bsp_battery_init();
    if (battery_err != ESP_OK) {
        ESP_LOGW(TAG, "电量计初始化失败: %s(UI 显示 --)", esp_err_to_name(battery_err));
    }

    const esp_err_t input_err = input_start();
    if (input_err != ESP_OK) {
        ESP_LOGE(TAG, "按键/输入任务初始化失败: %s", esp_err_to_name(input_err));
    }

    // UI 只在创建时持锁;之后由 LVGL 定时器与按键任务各自加锁。
    if (!bsp_lvgl_lock(UI_LOCK_TIMEOUT_MS)) {
        ESP_LOGE(TAG, "获取 LVGL 锁失败,无法创建界面");
        return;
    }
    ble_scan_ui_create();
    bsp_lvgl_unlock();

    // 蓝牙失败时 UI 会显示失败状态,应用本身仍可运行。
    const esp_err_t ble_err = ble_scan_start();
    if (ble_err != ESP_OK) {
        ESP_LOGE(TAG, "BLE 扫描启动失败: %s", esp_err_to_name(ble_err));
    } else {
        // 只读设备信息客户端(central GATT)依赖同一个 NimBLE host;任务常驻,
        // 只在用户从设备详情页发起读取时工作。
        const esp_err_t client_err = ble_client_start();
        if (client_err != ESP_OK) {
            ESP_LOGW(TAG, "设备信息读取客户端启动失败: %s", esp_err_to_name(client_err));
        }
    }

    if (xTaskCreate(power_task, "app_power", POWER_TASK_STACK, NULL,
                    POWER_TASK_PRIORITY, &s_power_task) != pdPASS) {
        ESP_LOGW(TAG, "低功耗任务创建失败,屏幕保持常亮");
    }

    // 本板无 PSRAM:记录启动后的空闲堆与最大连续块,便于真机评估新增的
    // BLE central + SEEKER 分包缓冲(约 21KB 静态)与 32KB LVGL 池是否够用。
    ESP_LOGI(TAG, "BLE 信号扫描器已就绪;free heap=%u largest=%u",
             (unsigned)esp_get_free_heap_size(),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
}
