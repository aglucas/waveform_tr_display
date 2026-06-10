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

static const char *TAG = "i2c_rx";
static QueueHandle_t s_queue;

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
    return true;
}

static void poll_task(void *arg)
{
    i2c_master_dev_handle_t dev = (i2c_master_dev_handle_t)arg;
    uint8_t buf[sizeof(telemetry_t)];

    while (1) {
        if (i2c_master_receive(dev, buf, sizeof(buf), I2C_TIMEOUT_MS) == ESP_OK) {
            telemetry_t t;
            memcpy(&t, buf, sizeof(t));
            if (telemetry_valid(&t)) {
                xQueueOverwrite(s_queue, &t);
                ESP_LOGD(TAG, "rx: spd=%d soc=%d", (int)t.speed_mph, (int)t.soc_pct);
            }
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

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = TELEMETRY_I2C_ADDR,
        .scl_speed_hz    = 400000,
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
