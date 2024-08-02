idf_component_register(
    SRCS    "src/misc.c"
            "src/pca-driver.c"
            "src/sbc-i2c-interaction.c"
            "src/int-brain.c"
            "src/four-wheel-bot.c"
            "src/motor-current-sense.c"
            "src/motor-encoder.c"
    INCLUDE_DIRS "include"
    REQUIRES 
        driver
        esp_adc
)