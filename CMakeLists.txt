idf_component_register(SRCS "misc.c" "pca-driver.c" "sbc-i2c-interaction.c" "int-brain.c" "four-wheel-bot.c" "motor-current-sense.c" "motor-encoder.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver esp_adc)