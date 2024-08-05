#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "int-brain.h"

const uint8_t PCA_DRIVER_ADDRESS = PCA_DRIVER_I2C0_DEFAULT_ADDRESS;

#define STANDARD_DELAY 10
#define LOOP_DELAY 5
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

    ESP_ERROR_CHECK(ADC_motors_init(DEFAULT_MOTORS_PIN_OUTS, NUMBER_OF_MOTORS));

    ESP_ERROR_CHECK(encoder_all_init(DEFAULT_MOTORS_PIN_OUTS, NUMBER_OF_MOTORS));

    PCA_enable();

    return;
}

void update_sensor_registers_task() {
    while (1) {
        ESP_ERROR_CHECK(update_motor_current_data(DEFAULT_MOTORS_PIN_OUTS));
        ESP_ERROR_CHECK(update_motor_disconnect_status());

        vTaskDelay(pdMS_TO_TICKS(STANDARD_DELAY));
    }
}

void command_bot_task() {
    for (size_t i = 0; i < NUMBER_OF_MOTORS; i++) {
        _motor_directions[i] = FORWARD;
    }

    bool LED_state = false;

    while (1) {
        for (size_t duty = 0; duty < PCA_MAX_PWM_DUTY; duty++) {
            for (size_t motor = 0; motor < NUMBER_OF_MOTORS; motor++) {
                _motor_speeds[motor] = duty;
            }

            ESP_ERROR_CHECK(command_all_motors_safe(
                DEFAULT_MOTORS_PIN_OUTS,
                PROTECT_DISCONNECT,
                COMMAND,
                _motor_speeds,
                _motor_directions));

            vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY));
        }

        LED_state = !LED_state;
        ESP_ERROR_CHECK(gpio_set_level(DEFAULT_LED_PIN, LED_state));

        for (size_t duty = PCA_MAX_PWM_DUTY; duty > 0; duty--) {
            for (size_t motor = 0; motor < NUMBER_OF_MOTORS; motor++) {
                _motor_speeds[motor] = duty;
            }

            ESP_ERROR_CHECK(command_all_motors_safe(
                DEFAULT_MOTORS_PIN_OUTS,
                PROTECT_DISCONNECT,
                COMMAND,
                _motor_speeds,
                _motor_directions));

            vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY));
        }

        LED_state = !LED_state;
        ESP_ERROR_CHECK(gpio_set_level(DEFAULT_LED_PIN, LED_state));
    }
}

void app_main(void) {
    int_brain_init();

    xTaskCreate(update_sensor_registers_task, "update_sensor_registers", 4096, NULL, TASK_PRIORITY, NULL);
    xTaskCreate(command_bot_task, "command_bot", 4096, NULL, TASK_PRIORITY, NULL);

    return;
}