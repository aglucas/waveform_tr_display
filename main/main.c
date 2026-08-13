#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/gpio.h"
#include "lvgl.h"
#include "bsp/esp-bsp.h"
#include "gauge_ui.h"
#include "rs485_rx.h"
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

/* SD card mileage file. Every layout shares MILEAGE_MAGIC and is a prefix of the
 * next EXCEPT v2, whose field order differs (it predates charge_mi):
 *   legacy (20 bytes): magic + total_mi + trip_mi
 *   v2     (44 bytes): + total_wh + trip_wh + charge_wh   [no charge_mi]
 *   v3     (52 bytes): + charge_mi, Wh fields shifted     [no last_soc]
 *   current(60 bytes): + last_soc
 * load_mileage() reads whichever length is on disk. Because current is a strict
 * append onto v3, a v3 card is migrated by simply reading its first 52 bytes —
 * the fields land in the right place and last_soc defaults to 0. Only v2 needs a
 * real remap (see mileage_file_v2_t). */
#define MILEAGE_FILE          BSP_SD_MOUNT_POINT "/odometer.bin"
#define MILEAGE_MAGIC         0xA55A1234U
#define MILEAGE_LEGACY_BYTES  20           /* sizeof the pre-energy layout */
#define MILEAGE_V3_BYTES      52           /* pre-last_soc layout           */

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
    double   last_soc;    /* last SOC seen with the contactors closed, %  */
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

/* Last SOC read with the contactors closed. Outside Run/Charge the sender only
 * has a placeholder to send (DEFAULT_SOC_PCT), so we show this remembered value
 * instead — and persist it, so it survives a power cycle and the gauge still
 * reads something meaningful when you walk up to a parked car. */
static double g_last_soc = 0.0;
static bool   s_was_bus_live = false;   /* tracks the Run/Charge -> off edge */

/* ── Rolling efficiency window ────────────────────────────────────────────────
 * Wh/mile over the last ROLL_WINDOW_MI of TRIP distance, so it resets with the
 * trip. We snapshot (trip_mi, trip_wh) into a ring every ROLL_BUCKET_MI; once the
 * ring is full its oldest entry is exactly one window back, so the rolling value
 * is just the delta between now and that entry — no per-sample decay or IIR to
 * tune, and it's exact rather than approximate.
 *
 * Before the ring fills, the oldest entry is the trip origin, so it reads as the
 * trip average until 10 miles are on the clock — which is the sensible warm-up.
 *
 * RAM-only (~1.6 kB): a reboot restarts the window even though the trip counters
 * themselves persist, so after a power cycle M reads as "since reboot" until it
 * refills. Persisting the ring would mean carrying it in odometer.bin. */
#define ROLL_WINDOW_MI  10.0
#define ROLL_BUCKET_MI  0.1
/* = ROLL_WINDOW_MI / ROLL_BUCKET_MI + 1. Spelled as a literal because a float
 * division is not an integer constant expression in C — using it as an array
 * size makes g_roll a VLA at file scope ("variably modified" warning). */
#define ROLL_N          101

typedef struct { double mi, wh; } roll_sample_t;
static roll_sample_t g_roll[ROLL_N];
static int    g_roll_head    = 0;   /* next write slot */
static int    g_roll_count   = 0;
static double g_roll_last_mi = 0.0; /* trip_mi at the last push */

/* Seed the window at the current trip point (called at boot and on trip reset). */
static void roll_reset(void)
{
    g_roll[0].mi   = g_trip_mi;
    g_roll[0].wh   = g_trip_wh;
    g_roll_head    = 1 % ROLL_N;
    g_roll_count   = 1;
    g_roll_last_mi = g_trip_mi;
}

/* Push a snapshot once the trip has advanced another bucket. */
static void roll_update(void)
{
    if (g_roll_count == 0)                      { roll_reset(); return; }
    if (g_trip_mi - g_roll_last_mi < ROLL_BUCKET_MI) return;

    g_roll[g_roll_head].mi = g_trip_mi;
    g_roll[g_roll_head].wh = g_trip_wh;
    g_roll_head = (g_roll_head + 1) % ROLL_N;
    if (g_roll_count < ROLL_N) g_roll_count++;
    g_roll_last_mi = g_trip_mi;
}

/* Window deltas. Handed to the UI as a (wh, mi) pair so it derives Wh/mi exactly
 * the same way it does for the trip and charge meters. */
static void roll_window(double *wh, double *mi)
{
    if (g_roll_count == 0) { *wh = 0.0; *mi = 0.0; return; }
    int oldest = (g_roll_head - g_roll_count + ROLL_N) % ROLL_N;
    *mi = g_trip_mi - g_roll[oldest].mi;
    *wh = g_trip_wh - g_roll[oldest].wh;
}

/* Dirty flags — set by timer callbacks, cleared by telemetry_task.
 * Keeps the Timer Service task stack usage minimal (no LVGL, no NVS). */
static volatile bool g_mileage_dirty    = false;
static volatile bool g_blink_dirty      = false;
static volatile bool g_save_dirty       = false;
static volatile bool g_trip_reset       = false;
/* Which sound to play. The fault alarm is a different sound from the chime (not
 * just the same clip louder), and it preempts — see the dispatcher below. */
typedef enum {
    SND_NONE = 0,
    SND_CHIME,   /* boot jingle / tap feedback (the embedded MP3) */
    SND_ERROR,   /* synthesised two-tone fault alarm              */
} sound_id_t;

static volatile sound_id_t g_sound_req     = SND_NONE;   /* pending request */
static volatile sound_id_t s_sound_current = SND_NONE;   /* what's playing  */
static volatile bool       s_sound_playing = false;
static volatile uint8_t g_sound_volume_req = SOUND_VOLUME;
static volatile bool g_brightness_dirty = false;
static volatile bool g_energy_dirty     = false;

/* Brightness bounce: 100 → 95 → … → 50 → 55 → … → 100, repeat */
static int g_brightness      = 70;
static int g_brightness_step = -10;

static void on_trip_reset(void)       { g_trip_reset = true; }
static void on_play_sound(void)       { g_sound_volume_req = SOUND_VOLUME; g_sound_req = SND_CHIME; }
static void on_alert_sound(void)      { g_sound_volume_req = 100;          g_sound_req = SND_ERROR; }
static void on_brightness_step(void)  { g_brightness_dirty = true; }
static void sound_task(void *arg);

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  Task watchdog                                                               */
/*                                                                              */
/*  CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0/1 only watches the IDLE tasks, so  */
/*  the watchdog never fired on the lockups: when the LVGL task blocks inside    */
/*  the touch driver on a wedged I2C bus, it blocks — it does not spin — so idle */
/*  keeps running and happily feeds the watchdog while the screen is frozen.     */
/*                                                                              */
/*  Subscribing the two tasks that can actually stall makes a hang observable:   */
/*    - LVGL task: holds the display lock and reads the GT911 over the SHARED    */
/*      I2C bus. This is the one that freezes the UI.                            */
/*    - telemetry_task: drives LVGL, writes odometer.bin through FATFS.          */
/*  With CONFIG_ESP_TASK_WDT_PANIC=y, either one stalling >5 s now panics and     */
/*  reboots instead of leaving a dead dash.                                      */
/*                                                                              */
/*  Deliberately NOT subscribed: the sound task (legitimately blocks for minutes */
/*  streaming the MP3) and the timer service.                                    */
/* ═══════════════════════════════════════════════════════════════════════════ */
#define LVGL_WDT_FEED_MS  1000   /* well under CONFIG_ESP_TASK_WDT_TIMEOUT_S (5 s) */

/* lv_timer callbacks are dispatched from inside lv_timer_handler(), i.e. in the
 * LVGL task's own context — so esp_task_wdt_add(NULL) here subscribes THAT task,
 * without needing the BSP/adapter to expose its handle. It must also be the thing
 * that feeds it: esp_task_wdt_reset() only ever resets the CALLING task's entry,
 * so subscribing the LVGL task from anywhere else would just trip the watchdog. */
static void lvgl_wdt_cb(lv_timer_t *timer)
{
    static bool subscribed = false;

    if (!subscribed) {
        esp_err_t err = esp_task_wdt_add(NULL);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "LVGL task TWDT subscribe failed: %s", esp_err_to_name(err));
            return;   /* don't reset an entry we don't own */
        }
        subscribed = true;
        ESP_LOGI(TAG, "LVGL task subscribed to TWDT (feeding every %d ms)", LVGL_WDT_FEED_MS);
    }

    esp_task_wdt_reset();
}

/* ── Button GPIOs (adjust to match your hardware wiring) ──────────────────── */
#define BTN_PFAIL   GPIO_NUM_32 // 40 pin header pin 26

/* ── Timing ───────────────────────────────────────────────────────────────── */

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  Boot / reset-reason log                                                     */
/*                                                                              */
/*  The dash lives in a car with no serial monitor attached, so a lockup or a   */
/*  spontaneous reboot otherwise leaves no trace at all. Append the reset reason */
/*  to the SD card every boot: TASK_WDT means a task hung (most likely blocked   */
/*  on a wedged I2C bus), PANIC means a crash or stack overflow, BROWNOUT means  */
/*  the supply dipped. That one word tells us which failure we're chasing.       */
/* ═══════════════════════════════════════════════════════════════════════════ */
#define BOOTLOG_FILE  BSP_SD_MOUNT_POINT "/bootlog.txt"
#define BOOT_NS       "boot"
#define KEY_BOOTCNT   "count"

static const char *reset_reason_str(esp_reset_reason_t r)
{
    switch (r) {
        case ESP_RST_POWERON:   return "POWERON";    /* normal cold start        */
        case ESP_RST_EXT:       return "EXT";        /* external reset pin       */
        case ESP_RST_SW:        return "SW";         /* esp_restart()            */
        case ESP_RST_PANIC:     return "PANIC";      /* crash / stack overflow   */
        case ESP_RST_INT_WDT:   return "INT_WDT";    /* interrupts blocked       */
        case ESP_RST_TASK_WDT:  return "TASK_WDT";   /* a task hung  <-- lockup  */
        case ESP_RST_WDT:       return "OTHER_WDT";
        case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
        case ESP_RST_BROWNOUT:  return "BROWNOUT";   /* supply dipped            */
        case ESP_RST_SDIO:      return "SDIO";
        default:                return "UNKNOWN";
    }
}

static void log_boot_reason(void)
{
    esp_reset_reason_t r = esp_reset_reason();
    const char *why = reset_reason_str(r);

    /* Boot counter in NVS — boots are rare, so the flash wear is negligible, and
     * it makes the SD log readable at a glance ("we're on boot 37"). */
    uint32_t n = 0;
    nvs_handle_t h;
    if (nvs_open(BOOT_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_get_u32(h, KEY_BOOTCNT, &n);
        n++;
        nvs_set_u32(h, KEY_BOOTCNT, n);
        nvs_commit(h);
        nvs_close(h);
    }

    /* An unexpected reason is the interesting case — shout about it. */
    if (r == ESP_RST_POWERON || r == ESP_RST_EXT || r == ESP_RST_SW)
        ESP_LOGI(TAG, "Boot #%lu, reset reason: %s", (unsigned long)n, why);
    else
        ESP_LOGE(TAG, "Boot #%lu after an ABNORMAL reset: %s", (unsigned long)n, why);

    if (!s_sd_mounted) return;
    FILE *fp = fopen(BOOTLOG_FILE, "a");
    if (!fp) {
        ESP_LOGW(TAG, "bootlog: could not open %s", BOOTLOG_FILE);
        return;
    }
    fprintf(fp, "boot %lu  reset=%s\n", (unsigned long)n, why);
    fclose(fp);
}

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
    ESP_LOGI(TAG, "%s | WH  total=%.2f trip=%.2f charge=%.2f | last_soc=%.1f%%", what,
             g_total_wh, g_trip_wh, g_charge_wh, g_last_soc);
    uint32_t age_ms = (uint32_t)((xTaskGetTickCount() - g_last_frame_tick) * portTICK_PERIOD_MS);
    double rwh, rmi;
    roll_window(&rwh, &rmi);
    double roll_whmi = (rmi >= 0.1) ? fabs(rwh) / rmi : 0.0;
    ESP_LOGI(TAG, "%s | EFF roll(M)=%.1f Wh/mi over %.2f mi (%d/%d buckets)",
             what, roll_whmi, rmi, g_roll_count, ROLL_N);

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
                layout  = "current(60)";
                read_ok = (fread(&f, sizeof(f), 1, fp) == 1);
            } else if (sz == MILEAGE_V3_BYTES) {
                /* Strict prefix of the current layout — the fields land where
                 * they belong; only last_soc is missing (stays 0 from memset). */
                layout  = "v3(52)";
                read_ok = (fread(&f, MILEAGE_V3_BYTES, 1, fp) == 1);
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
                            && isfinite(f.charge_wh)
                            && isfinite(f.last_soc)  && f.last_soc >= 0.0
                                                     && f.last_soc <= 100.0;

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
                g_last_soc  = f.last_soc;
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
                         g_total_wh, g_trip_wh, g_charge_wh, g_last_soc };
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
                             g_total_wh, g_trip_wh, g_charge_wh, g_last_soc };
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

    /* Watched by the TWDT — this task must complete a loop within the 5 s
     * timeout. It never blocks unboundedly: bsp_display_lock() has a 500 ms
     * timeout and rs485_rx_get() is non-blocking, so the only long pole is the
     * FATFS write in save_mileage(). */
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    while (1) {
        esp_task_wdt_reset();

        bool need_telemetry = false;
        if (rs485_rx_get(&t)) {
            float new_speed = (t.speed_mph < 0) ? 0 :
                              (t.speed_mph > 100) ? 100 : t.speed_mph;

            /* Save when speed drops below 1 mph (car just stopped) */
            if (new_speed < 1.0f && prev_speed >= 1.0f)
                g_save_dirty = true;

            /* Play the start sound on the rising edge into Run mode */
            bool is_run_mode = (t.opmode == CAR_MODE_RUN);
            if (is_run_mode && !s_was_run_mode) {
                g_sound_volume_req = SOUND_VOLUME;
                g_sound_req = SND_CHIME;
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

            /* Advance the rolling-efficiency window (no-op until the trip crosses
             * the next 0.1 mi bucket). */
            roll_update();

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

            /* SOC is only real with the contactors closed. While they are, keep
             * remembering it; while they aren't, the sender can only send a
             * placeholder — so substitute the last real reading into the frame
             * before it reaches the UI. Saving on the falling edge (the moment we
             * stop getting real data) is what makes the value survive shutdown.
             * The UI still suppresses the SOC warning outside Run/Charge, so this
             * is displayed but never graded. */
            bool bus_live = is_run_mode || is_charge_mode;
            if (bus_live) {
                g_last_soc = t.soc_pct;
            } else {
                if (s_was_bus_live) g_save_dirty = true;   /* just went dark — persist it */
                t.soc_pct = (float)g_last_soc;
            }
            s_was_bus_live = bus_live;

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
                if (g_energy_dirty)    {
                    g_energy_dirty = false;
                    double rwh, rmi;
                    roll_window(&rwh, &rmi);
                    gauge_ui_update_energy(g_trip_wh, g_trip_mi,
                                           g_charge_wh, g_charge_mi, rwh, rmi);
                }
                if (g_blink_dirty)     { g_blink_dirty   = false; gauge_ui_blink_tick(); }
                bsp_display_unlock();
            }
        }

        if (g_trip_reset) {
            g_trip_reset = false;
            g_trip_mi = 0.0;
            g_trip_wh = 0.0;
            g_trip_moving_s = 0.0;
            roll_reset();           /* M is trip-based, so it resets with the trip */
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

        /* Sound dispatcher. A fault alarm must not be swallowed by a jingle that
         * happens to be playing — the boot clip is a ~10 minute song, so the old
         * "drop it if busy" rule meant an over-temp could go unheard entirely. */
        if (g_sound_req != SND_NONE) {
            if (!s_sound_playing) {
                sound_id_t id = g_sound_req;
                g_sound_req     = SND_NONE;
                s_sound_current = id;
                s_sound_playing = true;
                uint32_t arg = ((uint32_t)id << 8) | g_sound_volume_req;
                xTaskCreate(sound_task, "snd", 32768,
                            (void *)(uintptr_t)arg, 3, NULL);
            } else if (g_sound_req == SND_ERROR && s_sound_current != SND_ERROR) {
                /* Preempt: cut the current sound short. The request stays pending
                 * and starts on a later pass, once that task has exited. */
                sound_request_stop();
            } else {
                /* Can't preempt (a chime while something plays, or an alarm over
                 * an alarm) — drop it rather than queue a sound that fires late. */
                g_sound_req = SND_NONE;
            }
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
        .pin_bit_mask = (1ULL << BTN_PFAIL),
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

    gpio_isr_handler_add(BTN_PFAIL, btn_isr, xTaskGetCurrentTaskHandle());

    TickType_t last_trigger = xTaskGetTickCount() - pdMS_TO_TICKS(1000);

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);   /* sleep until ISR fires */
        vTaskDelay(pdMS_TO_TICKS(10));              /* debounce */

        TickType_t now = xTaskGetTickCount();
        if (gpio_get_level(BTN_PFAIL) == 0 &&
            (now - last_trigger) >= pdMS_TO_TICKS(1000)) {
            last_trigger = now;
            save_mileage_fast();   /* SD only — no NVS flash erase */
        }
    }
}

/* arg packs the sound id and volume: (id << 8) | volume. The 32 kB stack is sized
 * for the MP3 decoder (mp3dec_t alone is ~6.5 kB); the tone path needs far less
 * but shares the task rather than duplicating it. */
static void sound_task(void *arg)
{
    uint32_t   a   = (uint32_t)(uintptr_t)arg;
    sound_id_t id  = (sound_id_t)(a >> 8);
    uint8_t    vol = (uint8_t)(a & 0xFF);

    if (id == SND_ERROR) alert_sound_play(vol);
    else                 boot_sound_play(vol);

    s_sound_current = SND_NONE;
    s_sound_playing = false;
    vTaskDelete(NULL);
}

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  app_main                                                                    */
/* ═══════════════════════════════════════════════════════════════════════════ */
void app_main(void)
{
    /* esp_codec_dev's I2S backend unconditionally disables the output channel
     * on every esp_codec_dev_open() to safely reconfigure it, and ignores the
     * result — so every play after the first (which closes and leaves the
     * channel disabled) hits an already-disabled channel here. Harmless; the
     * channel is re-enabled right after as part of the same open sequence. */
    esp_log_level_set("i2s_common", ESP_LOG_NONE);

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

    /* Straight after the mount, before anything else can crash — this is the only
     * record of WHY we restarted when no serial monitor is attached. */
    log_boot_reason();

    load_mileage();
    g_mi_last_save   = g_total_mi;
    g_mi_last_update = g_total_mi;
    /* Anchor the rolling window at the restored trip point — the ring itself
     * isn't persisted, so M measures from here until it refills. */
    roll_reset();

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
    {
        double rwh, rmi;
        roll_window(&rwh, &rmi);
        gauge_ui_update_energy(g_trip_wh, g_trip_mi,
                               g_charge_wh, g_charge_mi, rwh, rmi);
    }
    gauge_ui_set_trip_cb(on_trip_reset);
    gauge_ui_set_sound_cb(on_play_sound);
    gauge_ui_set_alert_sound_cb(on_alert_sound);
    gauge_ui_set_brightness_cb(on_brightness_step);

    /* Runs in the LVGL task, so it both subscribes that task to the TWDT and
     * feeds it. If the touch read ever blocks on a wedged I2C bus, this timer
     * stops firing and the watchdog reboots us instead of leaving a dead dash. */
    lv_timer_create(lvgl_wdt_cb, LVGL_WDT_FEED_MS, NULL);

    bsp_display_unlock();

    /* RS-485 telemetry receiver — dedicated UART, so a telemetry fault can
     * never touch the I2C bus the touch controller lives on. */
    ESP_ERROR_CHECK(rs485_rx_init());

    /* Periodic timers */
    TimerHandle_t t_mile = xTimerCreate("mile", pdMS_TO_TICKS(MILEAGE_TICK_MS),
                                        pdTRUE, NULL, mileage_timer_cb);
    TimerHandle_t t_blink = xTimerCreate("blink", pdMS_TO_TICKS(BLINK_INTERVAL_MS),
                                         pdTRUE, NULL, blink_timer_cb);
    xTimerStart(t_mile,  0);
    xTimerStart(t_blink, 0);

    /* Tasks */
    /* 8192, not 4096: this task drives LVGL, writes odometer.bin through FATFS,
     * and calls log_counters() — whose %f conversions burn ~1-1.5 kB on their own
     * under newlib's full (non-nano) float formatting. 4 kB was too tight. */
    xTaskCreate(telemetry_task, "telemetry", 8192, NULL, 5, NULL);
    /* 4096, not 2048: this task calls save_mileage_fast() -> log_counters(), and
     * newlib's full (non-nano) float formatting burns ~1-1.5 kB of stack on the
     * %f conversions alone, on top of fopen/fwrite. */
    xTaskCreate(button_task,    "buttons",   4096, NULL, 3, NULL);
}
