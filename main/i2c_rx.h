#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* Sender must write this value into telemetry_t.ready before each I2C transaction.
 * Any other value means the slave is not ready and the packet is discarded. */
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

/* Telemetry packet — must match sender struct layout exactly (8 × float + 2 bytes = 34 bytes) */
typedef struct __attribute__((packed)) {
    float speed_mph;
    float temp_bat_c;
    float temp_mot_c;
    float temp_inv_c;
    float temp_isa_c;
    float soc_pct;
    float udc_v;
    float energy_wh;     /* IVT 0x528 cumulative energy counter, Wh (signed) — snooped off CAN by the sender */
    uint8_t opmode;      /* car_mode_t — Off / Run / Precharge / PchFail / Charge / Preheat */
    uint8_t ready;       /* set to TELEMETRY_READY_MAGIC by sender when data is valid */
} telemetry_t;

esp_err_t i2c_rx_init(void);
bool      i2c_rx_get(telemetry_t *out);
