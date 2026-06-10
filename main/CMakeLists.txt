idf_component_register(
    SRCS
        "main.c"
        "gauge_ui.c"
        "i2c_rx.c"
        "triumph_logo.c"
        "boot_sound.c"
        "boot_sound_data.c"
    INCLUDE_DIRS "."
    REQUIRES
        nvs_flash
        driver
        esp_event
        waveshare__esp32_p4_wifi6_touch_lcd_xc)
