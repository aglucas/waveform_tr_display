#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "bsp/esp-bsp.h"
#include "i2c_rx.h"

/* Must match the I2C slave address configured in the sender firmware */
#define TELEMETRY_I2C_ADDR   0x42
#define TELEMETRY_POLL_MS    20
#define I2C_TIMEOUT_MS       10

/* Framing — must match the sender (speed_test_driver main.cpp).
 * The sender wraps each telemetry snapshot as:
 *     [SYNC0][SYNC1][ telemetry_t payload ][CRC8 over payload]
 * The sender's byte-stream FIFO has no message boundaries, and it refreshes far
 * slower than we poll, so a raw read can land on a torn/underrun window. We read
 * a multi-frame window, scan for the sync marker, and accept the payload only if
 * the CRC checks out — so shifted/underrun reads are dropped, not displayed. */
#define I2C_SYNC0            0xAA
#define I2C_SYNC1            0x55
#define I2C_FRAME_LEN        (2 + (int)sizeof(telemetry_t) + 1)   /* sync(2)+payload+crc(1) */
#define I2C_READ_LEN         (I2C_FRAME_LEN * 2)                  /* window to resync on */

/* Bus recovery. A noisy/long run can leave the bus wedged — most often the slave
 * mid-byte, holding SDA low, waiting for clocks that never come. i2c_master_bus_reset()
 * resets the master FSM and (on P4, via s_i2c_master_clear_bus) pulses SCL to free a
 * held SDA. Losing a few frames is fine: the sender re-frames continuously and every
 * frame is CRC-checked, so a dropped one is simply not displayed. */
#define I2C_RECOVER_AFTER_ERRS   5      /* consecutive failed reads before resetting */
#define I2C_RECOVER_MIN_GAP_MS   1000   /* don't thrash the bus with back-to-back resets */

static const char *TAG = "i2c_rx";
static QueueHandle_t s_queue;
static i2c_master_bus_handle_t s_bus;   /* kept for bus recovery */

/* Dallas/Maxim CRC8 (poly 0x8C, LSB-first) — must match sender's ow_crc8(). */
static uint8_t crc8(const uint8_t *data, int len)
{
    uint8_t crc = 0;
    for (int i = 0; i < len; i++) {
        uint8_t inbyte = data[i];
        for (int b = 0; b < 8; b++) {
            uint8_t mix = (crc ^ inbyte) & 0x01;
            crc >>= 1;
            if (mix) crc ^= 0x8C;
            inbyte >>= 1;
        }
    }
    return crc;
}

static bool telemetry_valid(const telemetry_t *t)
{
    if (t->ready != TELEMETRY_READY_MAGIC) return false;
    if (!isfinite(t->speed_mph)  || t->speed_mph  <    0.0f || t->speed_mph  >  250.0f) return false;
    if (!isfinite(t->temp_bat_c) || t->temp_bat_c <  -40.0f || t->temp_bat_c >  150.0f) return false;
    if (!isfinite(t->temp_mot_c) || t->temp_mot_c <  -40.0f || t->temp_mot_c >  300.0f) return false;
    if (!isfinite(t->temp_inv_c) || t->temp_inv_c <  -40.0f || t->temp_inv_c >  200.0f) return false;
    if (!isfinite(t->temp_isa_c) || t->temp_isa_c <  -40.0f || t->temp_isa_c >  200.0f) return false;
    if (!isfinite(t->soc_pct)    || t->soc_pct    <    0.0f || t->soc_pct    >  100.0f) return false;
    if (!isfinite(t->udc_v)      || t->udc_v      <    0.0f || t->udc_v      > 1000.0f) return false;
    /* Cumulative energy in Wh; ±100,000 kWh brackets any plausible counter value. */
    if (!isfinite(t->energy_wh)  || t->energy_wh  < -1e8f  || t->energy_wh  >  1e8f)   return false;
    if (t->opmode >= CAR_MODE_LAST) return false;
    return true;
}

static void poll_task(void *arg)
{
    i2c_master_dev_handle_t dev = (i2c_master_dev_handle_t)arg;
    uint8_t buf[I2C_READ_LEN];
    /* Bus-health counters. A failed receive means we never clocked the slave,
     * which is what backs up its TX ring buffer — so surface it here rather
     * than letting the sender be the only place the fault is visible. */
    uint32_t   err_count = 0, ok_count = 0;
    TickType_t last_report = 0;
    esp_err_t  last_err = ESP_OK;
    uint32_t   consec_err = 0;
    TickType_t last_reset = 0;

    while (1) {
        esp_err_t err = i2c_master_receive(dev, buf, sizeof(buf), I2C_TIMEOUT_MS);
        if (err == ESP_OK) {
            ok_count++;
            consec_err = 0;
            /* Scan the window for a valid frame, newest first (highest offset),
             * so we display the freshest snapshot the sender has queued. */
            for (int i = I2C_READ_LEN - I2C_FRAME_LEN; i >= 0; i--) {
                if (buf[i] != I2C_SYNC0 || buf[i + 1] != I2C_SYNC1) continue;
                const uint8_t *payload = &buf[i + 2];
                if (crc8(payload, sizeof(telemetry_t)) != payload[sizeof(telemetry_t)]) continue;
                telemetry_t t;
                memcpy(&t, payload, sizeof(t));
                if (telemetry_valid(&t)) {
                    xQueueOverwrite(s_queue, &t);
                    ESP_LOGD(TAG, "rx: spd=%d soc=%d", (int)t.speed_mph, (int)t.soc_pct);
                    break;
                }
            }
        } else {
            err_count++;
            last_err = err;

            /* A run of failures means the bus is wedged, not merely noisy —
             * a single corrupted transaction recovers on the next poll. Reset
             * the master FSM and pulse SCL to release a slave holding SDA low. */
            if (++consec_err >= I2C_RECOVER_AFTER_ERRS &&
                xTaskGetTickCount() - last_reset >= pdMS_TO_TICKS(I2C_RECOVER_MIN_GAP_MS)) {
                esp_err_t r = i2c_master_bus_reset(s_bus);
                ESP_LOGW(TAG, "I2C bus wedged after %lu consecutive failures (%s) — bus reset: %s",
                         (unsigned long)consec_err, esp_err_to_name(last_err), esp_err_to_name(r));
                last_reset = xTaskGetTickCount();
                consec_err = 0;
            }
        }

        /* Report bus health once a second, only when something is failing. */
        TickType_t now = xTaskGetTickCount();
        if (now - last_report >= pdMS_TO_TICKS(1000)) {
            if (err_count)
                ESP_LOGW(TAG, "I2C reads failing: %lu err / %lu ok in last 1s (last: %s)",
                         (unsigned long)err_count, (unsigned long)ok_count,
                         esp_err_to_name(last_err));
            last_report = now;
            err_count = ok_count = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(TELEMETRY_POLL_MS));
    }
}

esp_err_t i2c_rx_init(void)
{
    s_queue = xQueueCreate(1, sizeof(telemetry_t));
    if (!s_queue) return ESP_ERR_NO_MEM;

    /* The BSP initialises I2C (GPIO7=SDA, GPIO8=SCL) for the touch controller
     * and codec. Reuse that bus handle — no second init needed. */
    i2c_master_bus_handle_t bus = bsp_i2c_get_handle();
    if (!bus) {
        ESP_LOGE(TAG, "BSP I2C bus not ready — call after bsp_display_start");
        return ESP_ERR_INVALID_STATE;
    }
    s_bus = bus;   /* kept so poll_task can recover a wedged bus */

    /* 100 kHz, not 400 kHz. The telemetry slave sits at the end of a ~50 cm
     * unshielded run inside an EV — far outside I2C's intended same-PCB reach
     * (400 pF total bus capacitance, no differential signalling, no link-layer
     * error detection). At 400 kHz, inverter noise corrupts transactions; the
     * master then stops clocking, which is the ONE condition that lets the
     * slave's TX ring buffer fill ("no space in ringbuffer" on the sender).
     * Standard mode gives 4x the bit period and much better noise margin.
     * scl_speed_hz is per-device (the driver re-applies bus timing before each
     * transaction), so the touch controller and codec keep their own clocks. */
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = TELEMETRY_I2C_ADDR,
        .scl_speed_hz    = 100000,
    };
    i2c_master_dev_handle_t dev;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_cfg, &dev));

    xTaskCreate(poll_task, "i2c_poll", 4096, (void *)dev, 5, NULL);

    ESP_LOGI(TAG, "I2C telemetry receiver ready (addr=0x%02x, SDA=GPIO7, SCL=GPIO8)",
             TELEMETRY_I2C_ADDR);
    return ESP_OK;
}

bool i2c_rx_get(telemetry_t *out)
{
    return xQueueReceive(s_queue, out, 0) == pdTRUE;
}
