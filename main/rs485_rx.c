#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "rs485_rx.h"

/* ── RS-485 telemetry receiver ────────────────────────────────────────────────
 * Listens to the sender's 25 Hz broadcast on a dedicated UART. This replaces
 * the I2C link that shared a bus with the touch controller — a fault on this
 * wire can corrupt a frame (which the CRC rejects) but can never wedge a bus,
 * block the touch read, or freeze the UI.
 *
 * Receive-only: the transceiver's DE is tied low (10k pulldown), so this node
 * physically cannot drive the pair. If the sender is off, the fail-safe
 * receiver idles the line and no frames arrive — telemetry_task's staleness
 * guard (SPEED_STALE_MS) then freezes the odometer rather than integrating a
 * stale speed.
 *
 * Frame format (must match speed_test_driver/main/main.cpp):
 *     [0xAA][0x55][ telemetry_t payload ][CRC8 over payload]
 * A UART byte stream has no message boundaries, so we accumulate bytes, scan
 * for the sync marker, and accept a payload only when the CRC checks out. */

#define RS485_UART        UART_NUM_1
#define RS485_BAUD        115200     /* must match the sender */

/* PLACEHOLDER — set to the 40-pin-header GPIO wired to the transceiver's RO
 * before flashing. Must be free of BSP assignments (I2C=7/8, I2S=9-13,
 * LCD=23/26/27, SD=39-44, amp=53) and of BTN_UP (GPIO32 in main.c). Any
 * input-capable pin works — the UART routes through the GPIO matrix. */
#define RS485_RX_PIN      GPIO_NUM_33

#define TELEM_SYNC0       0xAA
#define TELEM_SYNC1       0x55
#define TELEM_FRAME_LEN   (2 + (int)sizeof(telemetry_t) + 1)   /* sync(2)+payload+crc(1) */

/* Accumulator sized for a few frames: at 25 Hz the scan drains it every pass,
 * so this only needs headroom for a burst after a stall, not sustained input. */
#define RX_ACC_LEN        (TELEM_FRAME_LEN * 6)

static const char *TAG = "rs485_rx";
static QueueHandle_t s_queue;

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

/* Range checks: the CRC proves the bytes arrived intact; this proves the sender
 * itself wasn't fed garbage (SDO glitch, uninitialised field). */
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
    if (!isfinite(t->power_kw)   || t->power_kw   < -300.0f || t->power_kw  >  300.0f) return false;
    if (t->opmode >= CAR_MODE_LAST) return false;
    return true;
}

static void rx_task(void *arg)
{
    static uint8_t acc[RX_ACC_LEN];
    int fill = 0;

    /* Watched by the TWDT for symmetry with telemetry_task. uart_read_bytes()
     * always returns within its 100 ms timeout, so a trip here means the UART
     * driver itself is broken — reboot is the right answer. */
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    /* Log link health once a second, only while it's bad. */
    uint32_t   ok_count = 0, rej_count = 0;
    TickType_t last_report = xTaskGetTickCount();

    while (1) {
        esp_task_wdt_reset();

        int n = uart_read_bytes(RS485_UART, acc + fill, RX_ACC_LEN - fill,
                                pdMS_TO_TICKS(100));
        if (n > 0) fill += n;

        /* Scan for complete frames; keep whatever tail remains for next pass. */
        int i = 0;
        while (fill - i >= TELEM_FRAME_LEN) {
            if (acc[i] != TELEM_SYNC0 || acc[i + 1] != TELEM_SYNC1) { i++; continue; }
            const uint8_t *payload = &acc[i + 2];
            if (crc8(payload, sizeof(telemetry_t)) != payload[sizeof(telemetry_t)]) {
                /* Sync bytes can occur inside a payload; a CRC miss here is not
                 * necessarily corruption. Step one byte and keep scanning. */
                i++;
                rej_count++;
                continue;
            }
            telemetry_t t;
            memcpy(&t, payload, sizeof(t));
            if (telemetry_valid(&t)) {
                xQueueOverwrite(s_queue, &t);
                ok_count++;
            } else {
                rej_count++;
            }
            i += TELEM_FRAME_LEN;
        }
        if (i > 0) {
            memmove(acc, acc + i, fill - i);
            fill -= i;
        }
        /* A full accumulator with no frame found means sustained garbage
         * (wrong baud, floating line). Start over rather than wedging. */
        if (fill >= RX_ACC_LEN) fill = 0;

        TickType_t now = xTaskGetTickCount();
        if (now - last_report >= pdMS_TO_TICKS(1000)) {
            if (rej_count)
                ESP_LOGW(TAG, "RS-485 rejects: %lu bad / %lu ok in last 1s (noise or baud mismatch?)",
                         (unsigned long)rej_count, (unsigned long)ok_count);
            last_report = now;
            ok_count = rej_count = 0;
        }
    }
}

esp_err_t rs485_rx_init(void)
{
    s_queue = xQueueCreate(1, sizeof(telemetry_t));
    if (!s_queue) return ESP_ERR_NO_MEM;

    uart_config_t cfg = {
        .baud_rate  = RS485_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(RS485_UART, 1024, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(RS485_UART, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(RS485_UART, UART_PIN_NO_CHANGE, RS485_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    xTaskCreate(rx_task, "rs485_rx", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "RS-485 telemetry receiver ready (RX GPIO %d, %d baud, %d-byte frames)",
             RS485_RX_PIN, RS485_BAUD, TELEM_FRAME_LEN);
    return ESP_OK;
}

bool rs485_rx_get(telemetry_t *out)
{
    return xQueueReceive(s_queue, out, 0) == pdTRUE;
}
