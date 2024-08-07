idf_component_register(
        "src/int-brain.c"
        "src/analog-inputs.c"
        "src/motor-commands.c"
        "src/pca-driver.c"
        "src/pcnt-encoder.c"
        "src/sbc-interaction.c"
    INCLUDE_DIRS "include"
    REQUIRES 
        driver
        esp_adc
)