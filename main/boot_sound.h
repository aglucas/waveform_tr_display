#pragma once
#include "esp_err.h"

/* Play the boot jingle via I2S/ES8311 at the given volume (0–100).
 * Blocks until playback is complete.
 * Safe to call once after display initialisation. */
esp_err_t boot_sound_play(uint8_t volume);
