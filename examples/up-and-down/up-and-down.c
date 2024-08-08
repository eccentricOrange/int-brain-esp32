#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "int-brain.h"

const uint8_t PCA_DRIVER_ADDRESS = PCA_DRIVER_I2C0_DEFAULT_ADDRESS;
const int ENCODER_LIMIT = DEFAULT_ENCODER_LIMIT;
const int ENCODER_GLITCH_PERIOD = DEFAULT_ENCODER_GLITCH_PERIOD;

#define STANDARD_DELAY 10
#define TASK_PRIORITY 1

i2c_master_bus_handle_t i2c0_master_bus_handle;

void int_brain_init(void) {
    const gpio_config_t LED_config = {
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = 1 << DEFAULT_LED_PIN,
    };

    ESP_ERROR_CHECK(gpio_config(&LED_config));

    ESP_ERROR_CHECK(i2c_new_master_bus(&DEFAULT_I2C0_MASTER_CONFIG, &i2c0_master_bus_handle));

    ESP_ERROR_CHECK(PCA_init(i2c0_master_bus_handle));

    ESP_ERROR_CHECK(ADC1_motors_init(DEFAULT_MOTORS_PIN_OUTS, NUMBER_OF_MOTORS));

    ESP_ERROR_CHECK(encoder_all_init(DEFAULT_MOTORS_PIN_OUTS, NUMBER_OF_MOTORS));

    ESP_ERROR_CHECK(set_motor_mode_register(PROTECT_DISCONNECT, COMMON, true));

    return;
}

void update_sensor_registers_task() {
    while (1) {
        ESP_ERROR_CHECK(update_motor_current_data_register(DEFAULT_MOTORS_PIN_OUTS));
        ESP_ERROR_CHECK(update_motor_disconnect_status());

        vTaskDelay(pdMS_TO_TICKS(STANDARD_DELAY));
    }
}

void command_bot_task() {
    ESP_ERROR_CHECK(set_bot_direction(FRONT));
    ESP_ERROR_CHECK(publish_motor_command(DEFAULT_MOTORS_PIN_OUTS));

    bool LED_state = false;

    while (1) {
        for (size_t i = 0; i < PCA_MAX_PWM_DUTY; i++) {
            ESP_ERROR_CHECK(set_common_motor_speed(i));
            ESP_ERROR_CHECK(publish_motor_command(DEFAULT_MOTORS_PIN_OUTS));
            vTaskDelay(pdMS_TO_TICKS(5));
        }

        LED_state = !LED_state;
        ESP_ERROR_CHECK(gpio_set_level(DEFAULT_LED_PIN, LED_state));

        for (size_t i = PCA_MAX_PWM_DUTY; i > 0; i--) {
            ESP_ERROR_CHECK(set_common_motor_speed(i));
            ESP_ERROR_CHECK(publish_motor_command(DEFAULT_MOTORS_PIN_OUTS));
            vTaskDelay(pdMS_TO_TICKS(5));
        }

        LED_state = !LED_state;
        ESP_ERROR_CHECK(gpio_set_level(DEFAULT_LED_PIN, LED_state));

        vTaskDelay(pdMS_TO_TICKS(STANDARD_DELAY));

    }
}

void app_main(void) {
    int_brain_init();
    xTaskCreate(update_sensor_registers_task, "update_sensor_registers", 4096, NULL, TASK_PRIORITY, NULL);
    xTaskCreate(command_bot_task, "command_bot", 4096, NULL, TASK_PRIORITY, NULL);

    return;
}