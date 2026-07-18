#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* Sender writes this value into telemetry_t.ready before each frame.
 * Any other value means the frame is not valid and is discarded. */
#define TELEMETRY_READY_MAGIC  0xA5

/* ZombieVerter opmode, forwarded verbatim by the sender (param_prj.h "modes" enum). */
typedef enum {
    CAR_MODE_OFF       = 0,
    CAR_MODE_RUN       = 1,
    CAR_MODE_PRECHARGE = 2,
    CAR_MODE_PCHFAIL   = 3,
    CAR_MODE_CHARGE    = 4,
    CAR_MODE_PREHEAT   = 5,
    CAR_MODE_LAST                /* first invalid value — used to range-check */
} car_mode_t;

/* Telemetry packet — one frame for the whole dashboard, broadcast over RS-485.
 * Three-way contract, change all together:
 *   - TelemetryPacket in speed_test_driver/main/main.cpp
 *   - this struct
 *   - telemetry_t in waveform_power_display/main/rs485_rx.h
 * (9 × float + 2 bytes = 38 bytes) */
typedef struct __attribute__((packed)) {
    float speed_mph;
    float temp_bat_c;
    float temp_mot_c;
    float temp_inv_c;
    float temp_isa_c;
    float soc_pct;
    float udc_v;
    float energy_wh;     /* cumulative energy counter, Wh (signed; ZombieVerter KWh spot value ×1000) */
    float power_kw;      /* DC bus power, kW; regen is negative (used by the power display) */
    uint8_t opmode;      /* car_mode_t — Off / Run / Precharge / PchFail / Charge / Preheat */
    uint8_t ready;       /* set to TELEMETRY_READY_MAGIC by sender when data is valid */
} telemetry_t;

esp_err_t rs485_rx_init(void);
bool      rs485_rx_get(telemetry_t *out);
