#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "esp_log.h"
#include "espnow_rx.h"

#define PACKET_LEN 28

/* Fixed locally-administered MAC advertised to the ESP-NOW sender.
 * Update receiverMAC[] in the Arduino sketch to match. */
static const uint8_t RECEIVER_MAC[6] = {0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x01};

static const char *TAG = "espnow_rx";
static QueueHandle_t s_queue;

static void on_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    ESP_LOGI(TAG, "on_recv: %d bytes from %02x:%02x:%02x:%02x:%02x:%02x",
             len,
             info->src_addr[0], info->src_addr[1], info->src_addr[2],
             info->src_addr[3], info->src_addr[4], info->src_addr[5]);
    if (len != PACKET_LEN) return;
    telemetry_t t;
    memcpy(&t, data, PACKET_LEN);
    xQueueOverwrite(s_queue, &t);
}

esp_err_t espnow_rx_init(void)
{
    s_queue = xQueueCreate(1, sizeof(telemetry_t));
    if (!s_queue) return ESP_ERR_NO_MEM;

    /* Event loop and NVS already initialised by app_main before this call */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_mac(WIFI_IF_STA, RECEIVER_MAC));

    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));

    ESP_LOGI(TAG, "Receiver MAC: %02x:%02x:%02x:%02x:%02x:%02x",
             RECEIVER_MAC[0], RECEIVER_MAC[1], RECEIVER_MAC[2],
             RECEIVER_MAC[3], RECEIVER_MAC[4], RECEIVER_MAC[5]);

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_recv));

    uint32_t espnow_ver;
    esp_now_get_version(&espnow_ver);
    ESP_LOGI(TAG, "ESP-NOW version: %"PRIu32, espnow_ver);
    ESP_LOGI(TAG, "ESP-NOW receiver ready");
    return ESP_OK;
}

bool espnow_rx_get(telemetry_t *out)
{
    return xQueueReceive(s_queue, out, 0) == pdTRUE;
}
