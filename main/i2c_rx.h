#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* Sender must write this value into telemetry_t.ready before each I2C transaction.
 * Any other value means the slave is not ready and the packet is discarded. */
#define TELEMETRY_READY_MAGIC  0xA5

/* Telemetry packet — must match sender struct layout exactly (7 × float + 1 byte = 29 bytes) */
typedef struct __attribute__((packed)) {
    float speed_mph;
    float temp_bat_c;
    float temp_mot_c;
    float temp_inv_c;
    float temp_isa_c;
    float soc_pct;
    float udc_v;
    uint8_t ready;       /* set to TELEMETRY_READY_MAGIC by sender when data is valid */
} telemetry_t;

esp_err_t i2c_rx_init(void);
bool      i2c_rx_get(telemetry_t *out);
