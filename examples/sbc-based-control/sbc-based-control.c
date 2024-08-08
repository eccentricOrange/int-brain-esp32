/** @file sbc-based-control.c
 *  @brief Example to control the robot using an SBC.
 *  @details This example demonstrates how to control the robot using an SBC. The SBC communicates with the robot using I2C1. In general, the SBC is expected to be the master and the robot the slave. The SBC sends commands to the robot, and the robot sends sensor data back to the SBC.
 *  @details In this example, we first initialize the robot. We then create RTOS tasks to communicate with the SBC, update sensor registers, and command the robot. These tasks run indefinitely. Keep the "pseudo register" concept in mind when reading the code.
 *  @author @eccentricOrange
 */

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "int-brain.h"

/// @brief Set the PCA driver address. If you solder just the A6 pin, the address is 0x40, which is the default.
const uint8_t PCA_DRIVER_ADDRESS = PCA_DRIVER_I2C0_DEFAULT_ADDRESS;

/// @brief Set a debouncing period for the encoder.
const int ENCODER_GLITCH_PERIOD = DEFAULT_ENCODER_GLITCH_PERIOD;

/// @brief Set the encoder limit, the maximum value the encoder can reach (PPR).
const int ENCODER_LIMIT = DEFAULT_ENCODER_LIMIT;

/// @brief Flag to check if a command has been received. Here, a "command" means a write from the SBC for the motors, as opposed to request for sensor or other telemetry data.
bool command_received = false;

/// @brief Delay between execution of RTOS tasks.
#define STANDARD_DELAY 10

/// @brief Priority of RTOS tasks.
#define TASK_PRIORITY 1

/// @brief I2C0 master bus handle (ESP32 as master).
i2c_master_bus_handle_t i2c0_master_bus_handle;

/// @brief I2C1 slave device handle (ESP32 as slave).
i2c_slave_dev_handle_t i2c1_slave_device_handle;

/** @brief Initialize the motors.
 *  @details Set the motor mode register, set the bot direction and set an initial common motor speed. Then send that data to the PCA driver over I2C.
 */
void initialize_motors(void) {
    set_motor_mode_register(UNSAFE, COMMON, false);
    set_bot_direction(FORWARD);
    set_common_motor_speed(0);
    ESP_ERROR_CHECK(publish_motor_command(DEFAULT_MOTORS_PIN_OUTS));
}

/** @brief Initialize the robot brain.
 *  @details Initialize the LED, I2C buses, PCA driver, motors, battery voltage sense, and encoders. Register the I2C1 slave device with the SBC. Then initialize the motors.
 */
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

    initialize_motors();

    return;
}

/** @brief Communicate with the SBC over I2C1.
 *  @details Read data from the SBC I2C1 and dump it to `_sbc_i2c1_receive_buffer`. If data was received from the SBC, it will be written to the relevant variable; if data was requested, it will be written to the send buffer `_sbc_i2c1_send_buffer`. Further, the data parsed to make sense of the identifying "pseudo register" address.
 */
void i2c1_communicate_with_sbc_task() {
    while (1) {
        ESP_ERROR_CHECK(sbc_i2c1_read_data(i2c1_slave_device_handle, sbc_i2c_parse_data));

        // Make sure that a glitch read hasn't been triggered
        if (_sbc_i2c1_register != 0x00) {

            // Make sure that we don't unnecessarily send data to the I2C buses, since this will fill up their FIFO ring buffers
            if (_sbc_i2c1_register <= LAST_SLAVE_REQUEST_ADDRESS) {
                ESP_ERROR_CHECK(i2c_slave_transmit(i2c1_slave_device_handle, _sbc_i2c1_send_buffer, SBC_I2C1_SEND_DATA_BUFFER_SIZE, pdMS_TO_TICKS(I2C_STANDARD_TIMEOUT_MS)));
            } else {
                command_received = true;
            }

            // Reset the identifying register
            _sbc_i2c1_register = 0;
        }
    }
}

/** @brief Update sensor registers.
 *  @details Update the encoder data register, motor current data register, motor disconnect status, and battery voltage register. This essentially meas placing the data from the actual peripherals into the pseudo registers. This task runs indefinitely.
 */
void update_sensor_registers_task() {
    while (1) {
        ESP_ERROR_CHECK(update_encoder_data_register(DEFAULT_MOTORS_PIN_OUTS));
        ESP_ERROR_CHECK(update_motor_current_data_register(DEFAULT_MOTORS_PIN_OUTS));
        ESP_ERROR_CHECK(update_motor_disconnect_status());
        ESP_ERROR_CHECK(update_battery_voltage_register());

        vTaskDelay(pdMS_TO_TICKS(STANDARD_DELAY));
    }
}

/** @brief Command the bot.
 *  @details If a command has been received, publish the motor command to the PCA driver. This task runs indefinitely.
 */
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