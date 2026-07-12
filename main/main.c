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
#define MILEAGE_SAVE_MI   0.1        /* save every N miles 0.1 is hard on flash drive fix PLP ciruit and change later */
#define MILEAGE_UPD_MI    0.1        /* redraw every N miles */
#define MILEAGE_TICK_MS   100        /* accumulation period */

/* g_cur_speed is a HELD value — it only changes when a telemetry frame arrives.
 * If frames stop (I2C wedged, sender rebooting) the last speed would otherwise
 * be integrated forever, inventing distance while the car sits still: 10 min of
 * a stale 20 mph = 3.3 phantom miles. Only accumulate while the reading is
 * fresh; a gap longer than this means "speed unknown", not "speed unchanged". */
#define SPEED_STALE_MS    500
#define BLINK_INTERVAL_MS 400
#define SOUND_VOLUME      100         /* speaker volume 0–100 */

/* SD card mileage file. Two on-disk layouts share MILEAGE_MAGIC:
 *   legacy (20 bytes): magic + total_mi + trip_mi
 *   current (52 bytes): the above + charge_mi + total_wh + trip_wh + charge_wh
 * load_mileage() reads whichever length is on disk, so a card written by the
 * pre-energy firmware upgrades cleanly (the new counters start at 0, odometer
 * preserved). */
#define MILEAGE_FILE          BSP_SD_MOUNT_POINT "/odometer.bin"
#define MILEAGE_MAGIC         0xA55A1234U
#define MILEAGE_LEGACY_BYTES  20           /* sizeof the pre-energy layout */

/* Reject an implausible frame-to-frame jump in the IVT energy counter (a shunt
 * reset, a dropped/garbled frame). Well above one telemetry tick of real drive
 * energy (~110 kW * 40 ms ≈ 1.2 Wh) but below a counter wrap/reset. */
#define ENERGY_MAX_STEP_WH    5000.0

typedef struct __attribute__((packed)) {
    uint32_t magic;
    double   total_mi;    /* lifetime odometer, miles                     */
    double   trip_mi;     /* miles since last trip reset                  */
    double   charge_mi;   /* miles since last transition into Charge      */
    double   total_wh;    /* lifetime energy, Wh (accumulated IVT deltas) */
    double   trip_wh;     /* energy since last trip reset, Wh             */
    double   charge_wh;   /* energy since last transition into Charge, Wh */
} mileage_file_t;

/* Intermediate layout that shipped briefly between the legacy file and the
 * current one: it had the three Wh counters but no charge_mi. A card written by
 * that firmware is 44 bytes; without this migration it matches neither the
 * legacy nor the current length and would be rejected outright — silently
 * resetting the odometer. Field order here must match that build exactly. */
typedef struct __attribute__((packed)) {
    uint32_t magic;
    double   total_mi;
    double   trip_mi;
    double   total_wh;
    double   trip_wh;
    double   charge_wh;
} mileage_file_v2_t;
#define MILEAGE_V2_BYTES  44

static bool s_sd_mounted = false;

static double g_total_mi = 0.0;
static double g_trip_mi  = 0.0;
static double g_charge_mi = 0.0;   /* miles since last transition into Charge */
static double g_mi_last_save   = 0.0;
static double g_mi_last_update = 0.0;
static float  g_cur_speed = 0.0f;
static volatile TickType_t g_last_frame_tick = 0;   /* when g_cur_speed last came from a real frame */
/* Debug: seconds the odometer actually integrated this trip. Compare against the
 * real drive time — if it far exceeds it, the odometer accumulated while parked
 * (stale speed), which is the signature of an inflated trip reading. */
static double g_trip_moving_s = 0.0;
static bool   s_was_run_mode   = false;  /* tracks CAR_MODE_RUN edge for start sound */

/* Energy counters — integrated from the IVT 0x528 cumulative counter forwarded
 * in telemetry_t.energy_wh. We accumulate deltas (rather than mirror the raw
 * counter) so these track the odometer and survive a shunt counter reset. */
static double g_total_wh  = 0.0;
static double g_trip_wh   = 0.0;
static double g_charge_wh = 0.0;
static double g_energy_prev      = 0.0;   /* last raw counter value seen        */
static bool   g_energy_have_prev = false; /* false until the first valid frame  */
static bool   s_was_charge_mode  = false; /* tracks CAR_MODE_CHARGE edge        */

/* Dirty flags — set by timer callbacks, cleared by telemetry_task.
 * Keeps the Timer Service task stack usage minimal (no LVGL, no NVS). */
static volatile bool g_mileage_dirty    = false;
static volatile bool g_blink_dirty      = false;
static volatile bool g_save_dirty       = false;
static volatile bool g_trip_reset       = false;
static volatile bool g_play_sound       = false;
static volatile bool s_sound_playing    = false;
static volatile uint8_t g_sound_volume_req = SOUND_VOLUME;
static volatile bool g_brightness_dirty = false;
static volatile bool g_energy_dirty     = false;

/* Brightness bounce: 100 → 95 → … → 50 → 55 → … → 100, repeat */
static int g_brightness      = 70;
static int g_brightness_step = -10;

static void on_trip_reset(void)       { g_trip_reset = true; }
static void on_play_sound(void)       { g_sound_volume_req = SOUND_VOLUME; g_play_sound = true; }
static void on_alert_sound(void)      { g_sound_volume_req = 100;          g_play_sound = true; }
static void on_brightness_step(void)  { g_brightness_dirty = true; }
static void boot_sound_task(void *arg);

/* ── Button GPIOs (adjust to match your hardware wiring) ──────────────────── */
#define BTN_UP   GPIO_NUM_32 // 40 pin header pin 26

/* ── Timing ───────────────────────────────────────────────────────────────── */

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  Mileage persistence — SD card primary, NVS fallback                        */
/* ═══════════════════════════════════════════════════════════════════════════ */

/* Dump every persisted counter, plus the derived Wh/mi and the raw IVT counter
 * we're differencing. Full float precision — the older logs cast to int, which
 * made a 12.97 mi odometer read as "12" and looked like data loss. */
static void log_counters(const char *what)
{
    double trip_whmi = (g_trip_mi   >= 0.1) ? fabs(g_trip_wh)   / g_trip_mi   : 0.0;
    double chg_whmi  = (g_charge_mi >= 0.1) ? fabs(g_charge_wh) / g_charge_mi : 0.0;

    /* Implied average speed over the time we actually integrated. This MUST match
     * your real average moving speed. If it does but the miles are too high, the
     * odometer ran while parked (integration window too long); if it doesn't, the
     * speed source is wrong. One number separates the two failure modes. */
    double moving_min = g_trip_moving_s / 60.0;
    double implied_mph = (g_trip_moving_s > 1.0)
                       ? g_trip_mi / (g_trip_moving_s / 3600.0) : 0.0;

    ESP_LOGI(TAG, "%s | MI  total=%.4f trip=%.4f charge=%.4f | integrated %.1f min -> implied avg %.1f mph",
             what, g_total_mi, g_trip_mi, g_charge_mi, moving_min, implied_mph);
    ESP_LOGI(TAG, "%s | WH  total=%.2f trip=%.2f charge=%.2f", what,
             g_total_wh, g_trip_wh, g_charge_wh);
    uint32_t age_ms = (uint32_t)((xTaskGetTickCount() - g_last_frame_tick) * portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "%s | EFF trip=%.1f Wh/mi  charge=%.1f Wh/mi | IVT raw=%.2f Wh (have_prev=%d) "
                  "speed=%.2f mph (frame age=%lu ms%s)",
             what, trip_whmi, chg_whmi, g_energy_prev, (int)g_energy_have_prev,
             (double)g_cur_speed, (unsigned long)age_ms,
             (age_ms >= SPEED_STALE_MS) ? ", STALE - odo frozen" : "");
}

static void load_mileage(void)
{
    /* Try SD card first */
    if (s_sd_mounted) {
        FILE *fp = fopen(MILEAGE_FILE, "rb");
        if (fp) {
            /* Read whichever layout is on disk. Three are accepted:
             *   20 bytes — legacy (pre-energy): odometer only
             *   44 bytes — intermediate (energy, no charge_mi)
             *   52 bytes — current
             * Anything else is rejected. The missing fields default to 0. */
            fseek(fp, 0, SEEK_END);
            long sz = ftell(fp);
            fseek(fp, 0, SEEK_SET);

            mileage_file_t f;
            memset(&f, 0, sizeof(f));
            const char *layout = "unknown";
            bool read_ok = false;

            if (sz >= (long)sizeof(f)) {
                layout  = "current(52)";
                read_ok = (fread(&f, sizeof(f), 1, fp) == 1);
            } else if (sz == MILEAGE_V2_BYTES) {
                /* Migrate the intermediate layout: same fields but no charge_mi,
                 * so the Wh values sit 8 bytes earlier than they do now. Read it
                 * in its own shape and map across rather than reinterpreting. */
                mileage_file_v2_t v2;
                memset(&v2, 0, sizeof(v2));
                layout  = "intermediate(44)";
                read_ok = (fread(&v2, sizeof(v2), 1, fp) == 1);
                if (read_ok) {
                    f.magic     = v2.magic;
                    f.total_mi  = v2.total_mi;
                    f.trip_mi   = v2.trip_mi;
                    f.charge_mi = 0.0;          /* field didn't exist yet */
                    f.total_wh  = v2.total_wh;
                    f.trip_wh   = v2.trip_wh;
                    f.charge_wh = v2.charge_wh;
                }
            } else if (sz == MILEAGE_LEGACY_BYTES) {
                layout  = "legacy(20)";
                read_ok = (fread(&f, MILEAGE_LEGACY_BYTES, 1, fp) == 1);
            }
            fclose(fp);

            bool magic_ok = read_ok && (f.magic == MILEAGE_MAGIC);
            bool vals_ok  = magic_ok
                            && isfinite(f.total_mi)  && f.total_mi  >= 0.0
                            && isfinite(f.trip_mi)   && f.trip_mi   >= 0.0
                            && isfinite(f.charge_mi) && f.charge_mi >= 0.0
                            && isfinite(f.total_wh)  && isfinite(f.trip_wh)
                            && isfinite(f.charge_wh);

            ESP_LOGI(TAG, "SD: odometer.bin %ld bytes, layout=%s, read=%d magic=%d(0x%08lX) vals=%d",
                     sz, layout, (int)read_ok, (int)magic_ok,
                     (unsigned long)f.magic, (int)vals_ok);

            if (vals_ok) {
                g_total_mi  = f.total_mi;
                g_trip_mi   = f.trip_mi;
                g_charge_mi = f.charge_mi;
                g_total_wh  = f.total_wh;
                g_trip_wh   = f.trip_wh;
                g_charge_wh = f.charge_wh;
                log_counters("LOAD");
                return;
            }
            /* Loud, because the NVS fallback is effectively empty (NVS writes are
             * disabled) — landing here means the odometer resets to zero, and the
             * next save overwrites the SD file with those zeros. */
            ESP_LOGE(TAG, "SD: odometer.bin REJECTED (%ld bytes) — counters will start at 0 "
                          "and the next save will OVERWRITE the file. Back it up now if it matters.",
                     sz);
        } else {
            ESP_LOGW(TAG, "SD: no odometer.bin yet — starting all counters at 0");
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
    mileage_file_t f = { MILEAGE_MAGIC, g_total_mi, g_trip_mi, g_charge_mi,
                         g_total_wh, g_trip_wh, g_charge_wh };
    FILE *fp = fopen(MILEAGE_FILE, "wb");
    if (fp) {
        size_t n = fwrite(&f, 1, sizeof(f), fp);
        fclose(fp);
        ESP_LOGI(TAG, "SAVE(PLP) wrote %u/%u bytes", (unsigned)n, (unsigned)sizeof(f));
        log_counters("SAVE(PLP)");
    } else {
        ESP_LOGW(TAG, "PLP: SD write failed");
    }
}

static void save_mileage(void)
{
    /* SD card — primary */
    if (s_sd_mounted) {
        mileage_file_t f = { MILEAGE_MAGIC, g_total_mi, g_trip_mi, g_charge_mi,
                             g_total_wh, g_trip_wh, g_charge_wh };
        FILE *fp = fopen(MILEAGE_FILE, "wb");
        if (fp) {
            size_t n = fwrite(&f, 1, sizeof(f), fp);
            fclose(fp);
            ESP_LOGI(TAG, "SAVE wrote %u/%u bytes", (unsigned)n, (unsigned)sizeof(f));
            log_counters("SAVE");
        } else {
            ESP_LOGW(TAG, "SD: failed to write odometer.bin");
        }
    } else {
        ESP_LOGW(TAG, "SAVE skipped — no SD card mounted");
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
}

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  Mileage accumulation timer (runs every MILEAGE_TICK_MS)                   */
/*  Keep this minimal — it runs in the Timer Service task (small stack).       */
/* ═══════════════════════════════════════════════════════════════════════════ */
static void mileage_timer_cb(TimerHandle_t xTimer)
{
    /* Never integrate a stale speed — see SPEED_STALE_MS. A dropped telemetry
     * link must freeze the odometer, not keep extrapolating the last reading. */
    bool fresh = (xTaskGetTickCount() - g_last_frame_tick) < pdMS_TO_TICKS(SPEED_STALE_MS);

    if (fresh && g_cur_speed > 0.0f) {
        double delta = g_cur_speed * (MILEAGE_TICK_MS / 1000.0 / 3600.0);
        g_total_mi  += delta;
        g_trip_mi   += delta;
        g_charge_mi += delta;
        g_trip_moving_s += MILEAGE_TICK_MS / 1000.0;
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

            /* Play the start sound on the rising edge into Run mode */
            bool is_run_mode = (t.opmode == CAR_MODE_RUN);
            if (is_run_mode && !s_was_run_mode) {
                g_sound_volume_req = SOUND_VOLUME;
                g_play_sound = true;
            }
            s_was_run_mode = is_run_mode;

            /* Integrate the IVT cumulative energy counter into our trip/lifetime/
             * charge totals. The first frame only latches the baseline (no delta
             * yet); a jump beyond ENERGY_MAX_STEP_WH (shunt reset / bad frame) is
             * discarded rather than accumulated. */
            static uint32_t s_energy_rejects = 0;
            double e = (double)t.energy_wh;
            double d = 0.0;
            bool   rejected = false;
            if (g_energy_have_prev) {
                d = e - g_energy_prev;
                if (fabs(d) < ENERGY_MAX_STEP_WH) {
                    g_total_wh  += d;
                    g_trip_wh   += d;
                    g_charge_wh += d;
                    g_energy_dirty = true;
                } else {
                    rejected = true;
                    s_energy_rejects++;
                }
            } else {
                ESP_LOGI(TAG, "IVT: baseline latched at %.2f Wh", e);
            }
            g_energy_prev = e;
            g_energy_have_prev = true;

            /* A rejected delta means we re-baselined and dropped that interval's
             * energy — never silent, it directly explains a low Wh total. */
            if (rejected)
                ESP_LOGW(TAG, "IVT: delta %.2f Wh exceeds +/-%.0f limit — DISCARDED, "
                              "re-baselined to %.2f Wh (rejects=%lu)",
                         d, ENERGY_MAX_STEP_WH, e, (unsigned long)s_energy_rejects);

            /* Periodic snapshot so miles vs Wh can be watched while driving. */
            static TickType_t last_dbg = 0;
            TickType_t now_dbg = xTaskGetTickCount();
            if (now_dbg - last_dbg >= pdMS_TO_TICKS(5000)) {
                last_dbg = now_dbg;
                log_counters("TICK");
            }

            /* Reset the "since last charge" meters on the rising edge into Charge. */
            bool is_charge_mode = (t.opmode == CAR_MODE_CHARGE);
            if (is_charge_mode && !s_was_charge_mode) {
                g_charge_wh = 0.0;
                g_charge_mi = 0.0;
                g_save_dirty = true;   /* persist the reset promptly */
            }
            s_was_charge_mode = is_charge_mode;

            g_cur_speed = new_speed;
            g_last_frame_tick = xTaskGetTickCount();   /* marks g_cur_speed as fresh */
            prev_speed  = new_speed;
            TickType_t now = xTaskGetTickCount();
            if ((now - last_ui) >= pdMS_TO_TICKS(50)) {
                need_telemetry = true;
                last_ui = now;
            }
        }

        /* Acquire the lock once per cycle for all pending LVGL work.
         * 500 ms timeout rides out the adapter's lv_timer_handler() render. */
        if (need_telemetry || g_mileage_dirty || g_blink_dirty || g_energy_dirty) {
            if (bsp_display_lock(500) == ESP_OK) {
                if (need_telemetry)      gauge_ui_update_telemetry(&t);
                if (g_mileage_dirty)   { g_mileage_dirty = false; gauge_ui_update_mileage(g_total_mi, g_trip_mi); }
                if (g_energy_dirty)    { g_energy_dirty  = false; gauge_ui_update_energy(g_trip_wh, g_trip_mi, g_charge_wh, g_charge_mi); }
                if (g_blink_dirty)     { g_blink_dirty   = false; gauge_ui_blink_tick(); }
                bsp_display_unlock();
            }
        }

        if (g_trip_reset) {
            g_trip_reset = false;
            g_trip_mi = 0.0;
            g_trip_wh = 0.0;
            g_trip_moving_s = 0.0;
            g_mileage_dirty = true;
            g_energy_dirty  = true;
            ESP_LOGI(TAG, "TRIP RESET");
        }

        if (g_save_dirty) {
            g_save_dirty = false;
            save_mileage();
        }

        if (g_brightness_dirty) {
            g_brightness_dirty = false;
            int prev = g_brightness;
            g_brightness += g_brightness_step;
            if (g_brightness >= 100) { g_brightness = 100; g_brightness_step = -10; }
            else if (g_brightness <= 30) { g_brightness = 30;  g_brightness_step =  10; }
            esp_err_t err = bsp_display_brightness_set(g_brightness);
            ESP_LOGI(TAG, "Brightness: %d%% -> %d%% (err=%d)", prev, g_brightness, err);
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
    gauge_ui_update_energy(g_trip_wh, g_trip_mi, g_charge_wh, g_charge_mi);
    gauge_ui_set_trip_cb(on_trip_reset);
    gauge_ui_set_sound_cb(on_play_sound);
    gauge_ui_set_alert_sound_cb(on_alert_sound);
    gauge_ui_set_brightness_cb(on_brightness_step);
    bsp_display_unlock();

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
    /* 4096, not 2048: this task calls save_mileage_fast() -> log_counters(), and
     * newlib's full (non-nano) float formatting burns ~1-1.5 kB of stack on the
     * %f conversions alone, on top of fopen/fwrite. */
    xTaskCreate(button_task,    "buttons",   4096, NULL, 3, NULL);
}
