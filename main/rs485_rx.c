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
 * Receive-only: the transceiver's combined DE/RE line is held low by this
 * driver at init and never toggled, so this node physically cannot drive the
 * pair. If the sender is off, the fail-safe receiver idles the line and no
 * frames arrive — telemetry_task's staleness guard (SPEED_STALE_MS) then
 * freezes the odometer rather than integrating a stale speed.
 *
 * Frame format (must match speed_test_driver/main/main.cpp):
 *     [0xAA][0x55][ telemetry_t payload ][CRC8 over payload]
 * A UART byte stream has no message boundaries, so we accumulate bytes, scan
 * for the sync marker, and accept a payload only when the CRC checks out. */

#define RS485_UART        UART_NUM_1
#define RS485_BAUD        115200     /* must match the sender */

/* Wired to the transceiver's TXD (its UART output — the data it recovered off
 * the RS-485 bus). NOTE: swapped from GPIO21 to GPIO20 — the module's TXD/RXD
 * pins were originally straight-matched to the ESP32's RX/TX instead of
 * crossed, which left this receiver listening to the module's input pin.
 * GPIO21 is wired to the module's RXD (its UART input, unused in this
 * receive-only build — nothing here needs to drive it). */
#define RS485_RX_PIN      GPIO_NUM_20

/* Combined DE/RE line. Held low permanently — never driven high, since this
 * build has no TX path wired to the UART. Low disables the driver (DE) and
 * enables the receiver output (RE), i.e. permanent receive mode. */
#define RS485_DE_PIN      GPIO_NUM_2

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
    if (!isfinite(t->max_throttle_pct) || t->max_throttle_pct < 0.0f || t->max_throttle_pct > 100.0f) return false;
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
    uint32_t   ok_count = 0, rej_count = 0, byte_count = 0;
    TickType_t last_report = xTaskGetTickCount();

    while (1) {
        esp_task_wdt_reset();

        int n = uart_read_bytes(RS485_UART, acc + fill, RX_ACC_LEN - fill,
                                pdMS_TO_TICKS(100));
        if (n > 0) { fill += n; byte_count += n; }

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
            if (byte_count == 0)
                ESP_LOGW(TAG, "RS-485: no bytes received in last 1s (check wiring/power/baud)");
            else if (rej_count)
                ESP_LOGW(TAG, "RS-485 rejects: %lu bad / %lu ok in last 1s, %lu bytes in (noise or baud mismatch?)",
                         (unsigned long)rej_count, (unsigned long)ok_count, (unsigned long)byte_count);
            last_report = now;
            ok_count = rej_count = byte_count = 0;
        }
    }
}

esp_err_t rs485_rx_init(void)
{
    s_queue = xQueueCreate(1, sizeof(telemetry_t));
    if (!s_queue) return ESP_ERR_NO_MEM;

    /* Hold the transceiver in receive mode before the UART (and thus the
     * sender's line) is live, so no window exists where DE/RE floats. */
    gpio_config_t de_io = {
        .pin_bit_mask = (1ULL << RS485_DE_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&de_io));
    ESP_ERROR_CHECK(gpio_set_level(RS485_DE_PIN, 0));

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
