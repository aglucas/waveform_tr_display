#pragma once
#include <stdint.h>
#include "esp_err.h"

/* Play the boot jingle via I2S/ES8311 at the given volume (0–100), streaming the
 * MP3 from the SD card (see BOOT_SOUND_FILE in boot_sound.c) rather than flash.
 * Returns ESP_ERR_NOT_FOUND if the card isn't mounted or the file is missing.
 * Blocks until playback is complete, or returns early if sound_request_stop()
 * is called. Safe to call once display initialisation is done. */
esp_err_t boot_sound_play(uint8_t volume);

/* Play the fault alarm at the given volume (0–100). This is synthesised in code
 * (a two-tone alternating alarm), not decoded from an asset — so it costs no
 * flash, is instantly retunable, and reads as an alarm rather than a music clip.
 * Blocks until complete, or returns early on sound_request_stop(). */
esp_err_t alert_sound_play(uint8_t volume);

/* Ask any in-progress playback to stop early. The flag is cleared when the next
 * sound starts, so this never wedges playback. Safe to call from any task or
 * from an LVGL event callback — it only sets a flag. */
void sound_request_stop(void);
