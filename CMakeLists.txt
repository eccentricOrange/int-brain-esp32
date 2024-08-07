idf_component_register(
        "src/int-brain.c"
        "src/pca-driver.c"
        "src/motor-commands.c"
    INCLUDE_DIRS "include"
    REQUIRES 
        driver
        esp_adc
)