#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/gpio.h"
#include "lvgl.h"
#include "bsp/esp-bsp.h"
#include "gauge_ui.h"
#include "i2c_rx.h"
#include "boot_sound.h"

static const char *TAG = "speedometer";

/* ── Mileage ───────────────────────────────────────────────────────────────── */
#define MILEAGE_NS        "mileage"
#define KEY_TOTAL         "total_mi"
#define KEY_TRIP          "trip_mi"
#define MILEAGE_SAVE_MI   5.0        /* save every N miles */
#define MILEAGE_UPD_MI    0.1        /* redraw every N miles */
#define MILEAGE_TICK_MS   100        /* accumulation period */
#define BLINK_INTERVAL_MS 400
#define SOUND_VOLUME      70         /* speaker volume 0–100 */

/* SD card mileage file — binary, 20 bytes */
#define MILEAGE_FILE      BSP_SD_MOUNT_POINT "/odometer.bin"
#define MILEAGE_MAGIC     0xA55A1234U

typedef struct __attribute__((packed)) {
    uint32_t magic;
    double   total_mi;
    double   trip_mi;
} mileage_file_t;

static bool s_sd_mounted = false;

static double g_total_mi = 0.0;
static double g_trip_mi  = 0.0;
static double g_mi_last_save   = 0.0;
static double g_mi_last_update = 0.0;
static float  g_cur_speed = 0.0f;

/* Dirty flags — set by timer callbacks, cleared by telemetry_task.
 * Keeps the Timer Service task stack usage minimal (no LVGL, no NVS). */
static volatile bool g_mileage_dirty = false;
static volatile bool g_blink_dirty   = false;
static volatile bool g_save_dirty    = false;
static volatile bool g_trip_reset    = false;
static volatile bool    g_play_sound      = false;
static volatile bool    s_sound_playing   = false;
static volatile uint8_t g_sound_volume_req = SOUND_VOLUME;

static void on_trip_reset(void)   { g_trip_reset = true; }
static void on_play_sound(void)   { g_sound_volume_req = SOUND_VOLUME; g_play_sound = true; }
static void on_alert_sound(void)  { g_sound_volume_req = 100;          g_play_sound = true; }
static void boot_sound_task(void *arg);

/* ── Button GPIOs (adjust to match your hardware wiring) ──────────────────── */
#define BTN_UP   GPIO_NUM_32 // 40 pin header pin 26

/* ── Timing ───────────────────────────────────────────────────────────────── */

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  Mileage persistence — SD card primary, NVS fallback                        */
/* ═══════════════════════════════════════════════════════════════════════════ */
static void load_mileage(void)
{
    /* Try SD card first */
    if (s_sd_mounted) {
        FILE *fp = fopen(MILEAGE_FILE, "rb");
        if (fp) {
            mileage_file_t f;
            bool ok = (fread(&f, sizeof(f), 1, fp) == 1)
                      && (f.magic == MILEAGE_MAGIC)
                      && isfinite(f.total_mi) && f.total_mi >= 0.0
                      && isfinite(f.trip_mi)  && f.trip_mi  >= 0.0;
            fclose(fp);
            if (ok) {
                g_total_mi = f.total_mi;
                g_trip_mi  = f.trip_mi;
                ESP_LOGI(TAG, "SD: ODO=%d TRIP=%d.%d",
                         (int)g_total_mi, (int)g_trip_mi,
                         (int)(g_trip_mi * 10) % 10);
                return;
            }
            ESP_LOGW(TAG, "SD: odometer.bin invalid, falling back to NVS");
        }
    }

    /* NVS fallback */
    nvs_handle_t h;
    if (nvs_open(MILEAGE_NS, NVS_READONLY, &h) != ESP_OK) return;
    uint64_t raw_total = 0, raw_trip = 0;
    nvs_get_u64(h, KEY_TOTAL, &raw_total);
    nvs_get_u64(h, KEY_TRIP,  &raw_trip);
    nvs_close(h);
    memcpy(&g_total_mi, &raw_total, sizeof(double));
    memcpy(&g_trip_mi,  &raw_trip,  sizeof(double));
    ESP_LOGI(TAG, "NVS: ODO=%d TRIP=%d.%d",
             (int)g_total_mi, (int)g_trip_mi, (int)(g_trip_mi * 10) % 10);
}

/* SD-only write — no NVS commit, no flash erase.
 * Used for the power-loss (PLP) path where every ms counts. */
static void save_mileage_fast(void)
{
    if (!s_sd_mounted) {
        ESP_LOGW(TAG, "PLP: no SD card, data not saved");
        return;
    }
    mileage_file_t f = { MILEAGE_MAGIC, g_total_mi, g_trip_mi };
    FILE *fp = fopen(MILEAGE_FILE, "wb");
    if (fp) {
        fwrite(&f, sizeof(f), 1, fp);
        fclose(fp);
        ESP_LOGI(TAG, "PLP save: ODO=%d TRIP=%d.%d",
                 (int)g_total_mi, (int)g_trip_mi, (int)(g_trip_mi * 10) % 10);
    } else {
        ESP_LOGW(TAG, "PLP: SD write failed");
    }
}

static void save_mileage(void)
{
    /* SD card — primary */
    if (s_sd_mounted) {
        mileage_file_t f = { MILEAGE_MAGIC, g_total_mi, g_trip_mi };
        FILE *fp = fopen(MILEAGE_FILE, "wb");
        if (fp) {
            fwrite(&f, sizeof(f), 1, fp);
            fclose(fp);
        } else {
            ESP_LOGW(TAG, "SD: failed to write odometer.bin");
        }
    }

    /* NVS backup disabled — SD card is primary storage
    nvs_handle_t h;
    if (nvs_open(MILEAGE_NS, NVS_READWRITE, &h) == ESP_OK) {
        uint64_t raw_total, raw_trip;
        memcpy(&raw_total, &g_total_mi, sizeof(double));
        memcpy(&raw_trip,  &g_trip_mi,  sizeof(double));
        nvs_set_u64(h, KEY_TOTAL, raw_total);
        nvs_set_u64(h, KEY_TRIP,  raw_trip);
        nvs_commit(h);
        nvs_close(h);
    } */

    g_mi_last_save = g_total_mi;
    ESP_LOGI(TAG, "Mileage saved: ODO=%d TRIP=%d.%d",
             (int)g_total_mi, (int)g_trip_mi, (int)(g_trip_mi * 10) % 10);
}

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  Mileage accumulation timer (runs every MILEAGE_TICK_MS)                   */
/*  Keep this minimal — it runs in the Timer Service task (small stack).       */
/* ═══════════════════════════════════════════════════════════════════════════ */
static void mileage_timer_cb(TimerHandle_t xTimer)
{
    if (g_cur_speed > 0.0f) {
        double delta = g_cur_speed * (MILEAGE_TICK_MS / 1000.0 / 3600.0);
        g_total_mi += delta;
        g_trip_mi  += delta;
    }

    if (g_total_mi - g_mi_last_update >= MILEAGE_UPD_MI) {
        g_mi_last_update = g_total_mi;
        g_mileage_dirty = true;
    }

    if (g_total_mi - g_mi_last_save >= MILEAGE_SAVE_MI)
        g_save_dirty = true;
}

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  Blink timer (runs every BLINK_INTERVAL_MS)                                 */
/* ═══════════════════════════════════════════════════════════════════════════ */
static void blink_timer_cb(TimerHandle_t xTimer)
{
    g_blink_dirty = true;
}

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  Telemetry receive task                                                      */
/*  Also drains the dirty flags from timer callbacks — all LVGL and NVS work   */
/*  happens here where the stack (4096 bytes) is large enough.                 */
/* ═══════════════════════════════════════════════════════════════════════════ */
static void telemetry_task(void *arg)
{
    telemetry_t t;
    TickType_t last_ui = 0;
    float prev_speed = 0.0f;

    while (1) {
        bool need_telemetry = false;
        if (i2c_rx_get(&t)) {
            float new_speed = (t.speed_mph < 0) ? 0 :
                              (t.speed_mph > 100) ? 100 : t.speed_mph;

            /* Save when speed drops below 1 mph (car just stopped) */
            if (new_speed < 1.0f && prev_speed >= 1.0f)
                g_save_dirty = true;

            g_cur_speed = new_speed;
            prev_speed  = new_speed;
            TickType_t now = xTaskGetTickCount();
            if ((now - last_ui) >= pdMS_TO_TICKS(50)) {
                need_telemetry = true;
                last_ui = now;
            }
        }

        /* Acquire the lock once per cycle for all pending LVGL work.
         * 500 ms timeout rides out the adapter's lv_timer_handler() render. */
        if (need_telemetry || g_mileage_dirty || g_blink_dirty) {
            if (bsp_display_lock(500) == ESP_OK) {
                if (need_telemetry)      gauge_ui_update_telemetry(&t);
                if (g_mileage_dirty)   { g_mileage_dirty = false; gauge_ui_update_mileage(g_total_mi, g_trip_mi); }
                if (g_blink_dirty)     { g_blink_dirty   = false; gauge_ui_blink_tick(); }
                bsp_display_unlock();
            }
        }

        if (g_trip_reset) {
            g_trip_reset = false;
            g_trip_mi = 0.0;
            g_mileage_dirty = true;
        }

        if (g_save_dirty) {
            g_save_dirty = false;
            save_mileage();
        }

        if (g_play_sound && !s_sound_playing) {
            g_play_sound = false;
            s_sound_playing = true;
            xTaskCreate(boot_sound_task, "snd", 32768,
                        (void *)(uintptr_t)g_sound_volume_req, 3, NULL);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  Button handling                                                             */
/* ═══════════════════════════════════════════════════════════════════════════ */
/*  Button — GPIO interrupt driven                                              */
/* ═══════════════════════════════════════════════════════════════════════════ */
static void IRAM_ATTR btn_isr(void *arg)
{
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR((TaskHandle_t)arg, &woken);
    portYIELD_FROM_ISR(woken);
}

static void button_task(void *arg)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << BTN_UP),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&io);

    /* gpio_install_isr_service may already be called by BSP; ignore if so */
    esp_err_t r = gpio_install_isr_service(0);
    if (r != ESP_OK && r != ESP_ERR_INVALID_STATE)
        ESP_ERROR_CHECK(r);

    gpio_isr_handler_add(BTN_UP, btn_isr, xTaskGetCurrentTaskHandle());

    TickType_t last_trigger = xTaskGetTickCount() - pdMS_TO_TICKS(1000);

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);   /* sleep until ISR fires */
        vTaskDelay(pdMS_TO_TICKS(10));              /* debounce */

        TickType_t now = xTaskGetTickCount();
        if (gpio_get_level(BTN_UP) == 0 &&
            (now - last_trigger) >= pdMS_TO_TICKS(1000)) {
            last_trigger = now;
            save_mileage_fast();   /* SD only — no NVS flash erase */
        }
    }
}

static void boot_sound_task(void *arg)
{
    boot_sound_play((uint8_t)(uintptr_t)arg);
    s_sound_playing = false;
    vTaskDelete(NULL);
}

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  app_main                                                                    */
/* ═══════════════════════════════════════════════════════════════════════════ */
void app_main(void)
{
    /* NVS for mileage */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* Mount SD card before loading mileage so the file is preferred over NVS */
    if (bsp_sdcard_mount() == ESP_OK) {
        s_sd_mounted = true;
        ESP_LOGI(TAG, "SD card mounted");
    } else {
        ESP_LOGW(TAG, "SD card not available — using NVS");
    }

    load_mileage();
    g_mi_last_save   = g_total_mi;
    g_mi_last_update = g_total_mi;

    /* Display + LVGL */
    bsp_display_cfg_t cfg = {
        .lv_adapter_cfg   = ESP_LV_ADAPTER_DEFAULT_CONFIG(),
        .rotation         = ESP_LV_ADAPTER_ROTATE_0,
        .tear_avoid_mode  = ESP_LV_ADAPTER_TEAR_AVOID_MODE_TRIPLE_PARTIAL,
        .touch_flags      = {.swap_xy = 0, .mirror_x = 0, .mirror_y = 0},
    };
    bsp_display_start_with_config(&cfg);
    bsp_display_backlight_on();

    /* Build gauge UI */
    bsp_display_lock(-1);
    gauge_ui_init(lv_screen_active());
    gauge_ui_update_mileage(g_total_mi, g_trip_mi);
    gauge_ui_set_trip_cb(on_trip_reset);
    gauge_ui_set_sound_cb(on_play_sound);
    gauge_ui_set_alert_sound_cb(on_alert_sound);
    bsp_display_unlock();

    /* minimp3's decode scratch uses ~15 KB of stack — run in its own task */
    s_sound_playing = true;
    xTaskCreate(boot_sound_task, "boot_snd", 32768,
                (void *)(uintptr_t)SOUND_VOLUME, 3, NULL);

    /* ESP-NOW receiver (also inits WiFi + NVS event loop) */
    ESP_ERROR_CHECK(i2c_rx_init());

    /* Periodic timers */
    TimerHandle_t t_mile = xTimerCreate("mile", pdMS_TO_TICKS(MILEAGE_TICK_MS),
                                        pdTRUE, NULL, mileage_timer_cb);
    TimerHandle_t t_blink = xTimerCreate("blink", pdMS_TO_TICKS(BLINK_INTERVAL_MS),
                                         pdTRUE, NULL, blink_timer_cb);
    xTimerStart(t_mile,  0);
    xTimerStart(t_blink, 0);

    /* Tasks */
    xTaskCreate(telemetry_task, "telemetry", 4096, NULL, 5, NULL);
    xTaskCreate(button_task,    "buttons",   2048, NULL, 3, NULL);
}
