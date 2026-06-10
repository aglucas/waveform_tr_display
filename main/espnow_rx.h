#pragma once
#include "esp_err.h"

/* Telemetry packet — 28 bytes, matches CircuitPython sender:
 *   struct.pack('<fffffff', speed_mph, t_bat, t_mot, t_inv, t_isa, soc, udc)
 */
typedef struct {
    float speed_mph;
    float temp_bat_c;
    float temp_mot_c;
    float temp_inv_c;
    float temp_isa_c;
    float soc_pct;
    float udc_v;
} telemetry_t;

esp_err_t espnow_rx_init(void);

/* Returns true and fills *out if a new packet arrived since last call. */
bool espnow_rx_get(telemetry_t *out);
