#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "int-brain.h"

const uint8_t PCA_DRIVER_ADDRESS = PCA_DRIVER_I2C0_DEFAULT_ADDRESS;
const int ENCODER_GLITCH_PERIOD = DEFAULT_ENCODER_GLITCH_PERIOD;
const int ENCODER_LIMIT = DEFAULT_ENCODER_LIMIT;

bool command_received = false;

#define STANDARD_DELAY 10
#define TASK_PRIORITY 1

i2c_master_bus_handle_t i2c0_master_bus_handle;
i2c_slave_dev_handle_t i2c1_slave_device_handle;

void reset_motors(void) {
    set_motor_mode_register(UNSAFE, COMMON, false);
    set_bot_direction(FORWARD);
    set_common_motor_speed(0);
    ESP_ERROR_CHECK(publish_motor_command(DEFAULT_MOTORS_PIN_OUTS));
}

void int_brain_init(void) {
    const gpio_config_t LED_config = {
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = 1 << DEFAULT_LED_PIN,
    };
    ESP_ERROR_CHECK(gpio_config(&LED_config));

    ESP_ERROR_CHECK(i2c_new_master_bus(&DEFAULT_I2C0_MASTER_CONFIG, &i2c0_master_bus_handle));

    ESP_ERROR_CHECK(i2c_new_slave_device(&DEFAULT_I2C1_SLAVE_CONFIG, &i2c1_slave_device_handle));

    ESP_ERROR_CHECK(PCA_init(i2c0_master_bus_handle));

    ESP_ERROR_CHECK(ADC1_motors_init(DEFAULT_MOTORS_PIN_OUTS, NUMBER_OF_MOTORS));

    ESP_ERROR_CHECK(ADC2_battery_init());

    ESP_ERROR_CHECK(encoder_all_init(DEFAULT_MOTORS_PIN_OUTS, NUMBER_OF_MOTORS));

    ESP_ERROR_CHECK(sbc_i2c1_register_callback(i2c1_slave_device_handle));

    reset_motors();

    return;
}

void i2c1_communicate_with_sbc_task() {
    while (1) {
        ESP_ERROR_CHECK(sbc_i2c1_read_data(i2c1_slave_device_handle, sbc_i2c_parse_data));

        if (_sbc_i2c1_register != 0x00) {
            if (_sbc_i2c1_register <= LAST_SLAVE_REQUEST_ADDRESS) {
                ESP_ERROR_CHECK(i2c_slave_transmit(i2c1_slave_device_handle, _sbc_i2c1_send_buffer, SBC_I2C1_SEND_DATA_BUFFER_SIZE, pdMS_TO_TICKS(I2C_STANDARD_TIMEOUT_MS)));
            } else {
                command_received = true;
            }

            _sbc_i2c1_register = 0;
        }
    }
}

void update_sensor_registers_task() {
    while (1) {
        ESP_ERROR_CHECK(update_encoder_data_register(DEFAULT_MOTORS_PIN_OUTS));
        ESP_ERROR_CHECK(update_motor_current_data_register(DEFAULT_MOTORS_PIN_OUTS));
        ESP_ERROR_CHECK(update_motor_disconnect_status());
        ESP_ERROR_CHECK(update_battery_voltage_register());

        vTaskDelay(pdMS_TO_TICKS(STANDARD_DELAY));
    }
}

void command_bot_task() {
    while (1) {
        if (command_received) {
            ESP_ERROR_CHECK(publish_motor_command(DEFAULT_MOTORS_PIN_OUTS));
            command_received = false;
        }

        vTaskDelay(pdMS_TO_TICKS(STANDARD_DELAY));
    }
}

void app_main(void) {
    int_brain_init();

    xTaskCreate(i2c1_communicate_with_sbc_task, "i2c1_communicate_with_sbc", 4096, NULL, TASK_PRIORITY, NULL);
    xTaskCreate(update_sensor_registers_task, "update_sensor_registers", 4096, NULL, TASK_PRIORITY, NULL);
    xTaskCreate(command_bot_task, "command_bot", 4096, NULL, TASK_PRIORITY, NULL);

    return;
}