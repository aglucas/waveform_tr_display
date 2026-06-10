#define MINIMP3_IMPLEMENTATION
#include "minimp3.h"

#include "boot_sound.h"
#include "boot_sound_data.h"
#include "bsp/esp32_p4_wifi6_touch_lcd_xc.h"
#include "esp_codec_dev.h"
#include "driver/i2s_std.h"
#include "esp_log.h"

static const char *TAG = "boot_sound";

esp_err_t boot_sound_play(uint8_t volume)
{
    /* Peek at the first valid frame to learn sample-rate and channel count
     * before initialising I2S so the clock is correct. */
    /* Keep these in BSS, not on the stack — mp3dec_t alone is ~6.5 KB and
     * app_main's stack is only 10 KB, which would overflow immediately. */
    static mp3dec_t dec;
    static mp3d_sample_t pcm[MINIMP3_MAX_SAMPLES_PER_FRAME];

    mp3dec_init(&dec);

    mp3dec_frame_info_t info = {0};

    const uint8_t *ptr       = boot_sound_mp3;
    int            remaining = (int)boot_sound_mp3_len;

    /* Advance past ID3 tags / garbage until we find a real sync frame. */
    int first_samples = 0;
    while (remaining > 0) {
        first_samples = mp3dec_decode_frame(&dec, ptr, remaining, pcm, &info);
        if (info.frame_bytes == 0) break;
        ptr       += info.frame_bytes;
        remaining -= info.frame_bytes;
        if (first_samples > 0) break;
    }

    if (info.hz == 0 || info.frame_bytes == 0) {
        ESP_LOGE(TAG, "Could not find valid MP3 frame");
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

    /* Decode and stream the rest. */
    while (remaining > 0) {
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
    ESP_LOGI(TAG, "Playback complete");
    return ESP_OK;
}
