#define MINIMP3_IMPLEMENTATION
#include "minimp3.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

#include "boot_sound.h"
#include "bsp/esp32_p4_wifi6_touch_lcd_xc.h"
#include "esp_codec_dev.h"
#include "driver/i2s_std.h"
#include "esp_log.h"

static const char *TAG = "boot_sound";

/* Boot jingle streams from the SD card instead of living in flash — drop an MP3
 * at this path to change it, no firmware rebuild required. */
#define BOOT_SOUND_FILE      BSP_SD_MOUNT_POINT "/car_jam.mp3"

/* Read buffer for streaming the MP3 off the card. Refilled once fewer than
 * MP3_REFILL_MARGIN bytes remain — comfortably more than the largest possible
 * MPEG frame (~2.9 KB), so mp3dec_decode_frame() is never starved mid-frame. */
#define MP3_BUF_SIZE         (16 * 1024)
#define MP3_REFILL_MARGIN    (8 * 1024)
static uint8_t s_mp3_buf[MP3_BUF_SIZE];

/* Slide any unconsumed bytes to the front of the buffer and top it back up from
 * fp. Safe to call at EOF — fread() just returns 0 and remaining stops growing. */
static void mp3_refill(FILE *fp, uint8_t **ptr, int *remaining)
{
    if (*remaining > 0 && *ptr != s_mp3_buf)
        memmove(s_mp3_buf, *ptr, (size_t)*remaining);
    *ptr = s_mp3_buf;

    size_t space = MP3_BUF_SIZE - (size_t)*remaining;
    size_t got   = fread(s_mp3_buf + *remaining, 1, space, fp);
    *remaining += (int)got;
}

/* Set by sound_request_stop(), polled by the playback loops so a fault alarm can
 * cut the boot jingle short instead of waiting ~10 minutes for a song to finish.
 * Cleared at the start of each play, so a stale request can't kill the next sound. */
static volatile bool s_stop_req = false;

void sound_request_stop(void)
{
    s_stop_req = true;
}

/* ── Fault alarm — synthesised, no MP3 asset ─────────────────────────────────
 * A two-tone alternating alarm. Deliberately harsh and periodic so it reads as a
 * fault, not a notification. Retune by editing these five numbers. */
#define ALERT_SR         22050   /* Hz — matches the BSP's own default I2S rate */
#define ALERT_HI_HZ      1200.0f
#define ALERT_LO_HZ       800.0f
#define ALERT_TONE_MS     160    /* length of each tone in the pair            */
#define ALERT_CYCLES        3    /* hi/lo pairs per alarm                      */
#define ALERT_GAP_MS       70    /* silence between pairs                      */
#define ALERT_AMPLITUDE 26000.0f /* of 32767; leaves headroom to avoid clipping */

#define ALERT_CHUNK       256    /* samples per codec write                    */
static int16_t s_alert_buf[ALERT_CHUNK];

/* Emit one tone. A short linear fade in/out kills the click you'd otherwise get
 * from starting and ending a sine mid-cycle at full amplitude. */
static void alert_tone(esp_codec_dev_handle_t spk, float freq, int ms)
{
    const int total  = ALERT_SR * ms / 1000;
    const int fade   = ALERT_SR / 200;          /* 5 ms */
    const float dphi = 2.0f * (float)M_PI * freq / (float)ALERT_SR;
    float phi = 0.0f;

    for (int done = 0; done < total && !s_stop_req; ) {
        int n = total - done;
        if (n > ALERT_CHUNK) n = ALERT_CHUNK;

        for (int i = 0; i < n; i++) {
            const int idx = done + i;
            float env = 1.0f;
            if (idx < fade)                 env = (float)idx / (float)fade;
            else if (idx > total - fade)    env = (float)(total - idx) / (float)fade;
            if (env < 0.0f) env = 0.0f;

            s_alert_buf[i] = (int16_t)(sinf(phi) * ALERT_AMPLITUDE * env);
            phi += dphi;
            if (phi > 2.0f * (float)M_PI) phi -= 2.0f * (float)M_PI;
        }
        esp_codec_dev_write(spk, s_alert_buf, n * (int)sizeof(int16_t));
        done += n;
    }
}

static void alert_silence(esp_codec_dev_handle_t spk, int ms)
{
    int total = ALERT_SR * ms / 1000;
    memset(s_alert_buf, 0, sizeof(s_alert_buf));
    for (int done = 0; done < total && !s_stop_req; ) {
        int n = total - done;
        if (n > ALERT_CHUNK) n = ALERT_CHUNK;
        esp_codec_dev_write(spk, s_alert_buf, n * (int)sizeof(int16_t));
        done += n;
    }
}

esp_err_t alert_sound_play(uint8_t volume)
{
    s_stop_req = false;

    /* bsp_audio_init() is a no-op once audio is up (it returns early without
     * re-applying the config), so the format we actually get is whatever
     * esp_codec_dev_open() below reconfigures the I2S clock/slot to. */
    i2s_std_config_t i2s_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(ALERT_SR),
        .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                       I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = BSP_I2S_MCLK,
            .bclk = BSP_I2S_SCLK,
            .ws   = BSP_I2S_LCLK,
            .dout = BSP_I2S_DOUT,
            .din  = BSP_I2S_DSIN,
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };
    ESP_ERROR_CHECK(bsp_audio_init(&i2s_cfg));

    esp_codec_dev_handle_t spk = bsp_audio_codec_speaker_init();
    if (spk == NULL) {
        ESP_LOGE(TAG, "Speaker init failed");
        return ESP_FAIL;
    }

    esp_codec_dev_sample_info_t fs = {
        .sample_rate     = ALERT_SR,
        .channel         = 1,
        .bits_per_sample = 16,
    };
    ESP_ERROR_CHECK(esp_codec_dev_set_out_vol(spk, volume));
    ESP_ERROR_CHECK(esp_codec_dev_open(spk, &fs));

    for (int c = 0; c < ALERT_CYCLES && !s_stop_req; c++) {
        alert_tone(spk, ALERT_HI_HZ, ALERT_TONE_MS);
        alert_tone(spk, ALERT_LO_HZ, ALERT_TONE_MS);
        if (c < ALERT_CYCLES - 1) alert_silence(spk, ALERT_GAP_MS);
    }

    esp_codec_dev_close(spk);
    ESP_LOGI(TAG, "Alert tone complete%s", s_stop_req ? " (stopped early)" : "");
    return ESP_OK;
}

esp_err_t boot_sound_play(uint8_t volume)
{
    s_stop_req = false;

    FILE *fp = fopen(BOOT_SOUND_FILE, "rb");
    if (!fp) {
        ESP_LOGW(TAG, "Boot jingle skipped — %s not found on SD card", BOOT_SOUND_FILE);
        return ESP_ERR_NOT_FOUND;
    }

    /* Peek at the first valid frame to learn sample-rate and channel count
     * before initialising I2S so the clock is correct. */
    /* Keep these in BSS, not on the stack — mp3dec_t alone is ~6.5 KB and
     * app_main's stack is only 10 KB, which would overflow immediately. */
    static mp3dec_t dec;
    static mp3d_sample_t pcm[MINIMP3_MAX_SAMPLES_PER_FRAME];

    mp3dec_init(&dec);

    mp3dec_frame_info_t info = {0};

    uint8_t *ptr       = s_mp3_buf;
    int      remaining = 0;
    mp3_refill(fp, &ptr, &remaining);

    /* Advance past ID3 tags / garbage until we find a real sync frame.
     * mp3d_find_frame() reports "no frame" by consuming its entire input
     * buffer, so a tag bigger than one refill (an embedded cover image in an
     * ID3v2 header routinely runs to hundreds of KB, vs. MP3_BUF_SIZE's
     * 16 KB) can drain `remaining` to exactly 0 and fail the while condition
     * before the loop ever gets back around to refilling — wrongly reported
     * as "no valid MP3 frame" on a perfectly good file. Refill right after
     * that happens too, and only give up once a refill actually yields no
     * new bytes (true EOF). */
    int first_samples = 0;
    for (;;) {
        if (remaining < MP3_REFILL_MARGIN) {
            int before = remaining;
            mp3_refill(fp, &ptr, &remaining);
            /* No new bytes AND nothing left buffered either — true EOF.
             * (A refill can legitimately add 0 bytes while `remaining` stays
             * positive, e.g. one last leftover chunk right at EOF — that
             * still needs a decode attempt, so don't bail out on it here.) */
            if (remaining == before && remaining == 0) break;
        }
        if (remaining <= 0) break;
        first_samples = mp3dec_decode_frame(&dec, ptr, remaining, pcm, &info);
        if (info.frame_bytes == 0) break;     /* decoder made zero progress */
        ptr       += info.frame_bytes;
        remaining -= info.frame_bytes;
        if (first_samples > 0) break;         /* found real audio */
    }

    if (info.hz == 0 || info.frame_bytes == 0) {
        ESP_LOGE(TAG, "Could not find valid MP3 frame in %s", BOOT_SOUND_FILE);
        fclose(fp);
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "MP3: %d Hz, %d ch", info.hz, info.channels);

    /* Build I2S config using the public BSP pin macros. */
    i2s_std_config_t i2s_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG((uint32_t)info.hz),
        .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(
                        I2S_DATA_BIT_WIDTH_16BIT,
                        info.channels == 2 ? I2S_SLOT_MODE_STEREO
                                           : I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = BSP_I2S_MCLK,
            .bclk = BSP_I2S_SCLK,
            .ws   = BSP_I2S_LCLK,
            .dout = BSP_I2S_DOUT,
            .din  = BSP_I2S_DSIN,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };

    ESP_ERROR_CHECK(bsp_audio_init(&i2s_cfg));

    esp_codec_dev_handle_t spk = bsp_audio_codec_speaker_init();
    if (spk == NULL) {
        ESP_LOGE(TAG, "Speaker init failed");
        fclose(fp);
        return ESP_FAIL;
    }

    esp_codec_dev_sample_info_t fs = {
        .sample_rate    = (uint32_t)info.hz,
        .channel        = info.channels,
        .bits_per_sample = 16,
    };
    ESP_ERROR_CHECK(esp_codec_dev_set_out_vol(spk, volume));
    ESP_ERROR_CHECK(esp_codec_dev_open(spk, &fs));

    /* Write the first decoded frame we already have. */
    if (first_samples > 0) {
        esp_codec_dev_write(spk, pcm,
                            first_samples * info.channels * sizeof(int16_t));
    }

    /* Decode and stream the rest. Bail out on a stop request — without this, a
     * fault alarm raised during the boot jingle would have to wait out the whole
     * song before it could be heard. */
    while (remaining > 0 && !s_stop_req) {
        if (remaining < MP3_REFILL_MARGIN) mp3_refill(fp, &ptr, &remaining);
        int samples = mp3dec_decode_frame(&dec, ptr, remaining, pcm, &info);
        if (info.frame_bytes == 0) break;
        ptr       += info.frame_bytes;
        remaining -= info.frame_bytes;
        if (samples > 0) {
            esp_codec_dev_write(spk, pcm,
                                samples * info.channels * sizeof(int16_t));
        }
    }

    esp_codec_dev_close(spk);
    fclose(fp);
    ESP_LOGI(TAG, "Playback complete%s", s_stop_req ? " (stopped early)" : "");
    return ESP_OK;
}
