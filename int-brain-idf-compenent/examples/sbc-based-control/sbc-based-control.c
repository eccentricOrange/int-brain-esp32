/** @file sbc-based-control.c
 *  @brief Example to control the robot using an SBC.
 *  @details This example demonstrates how to control the robot using an SBC. The SBC communicates with the robot using I2C1 or UART0. In general, the SBC is expected to be the master and the robot the slave. The SBC sends commands to the robot, and the robot sends sensor data back to the SBC.
 *  @details In this example, we first initialize the robot. We then create RTOS tasks to communicate with the SBC, update sensor registers, and command the robot. These tasks run indefinitely. Keep the "pseudo register" concept in mind when reading the code.
 *  @author @eccentricOrange
 */

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "int-brain.h"

/// @brief Choose the mode of communication with the SBC.
#define SBC_UART_MODE
// #define SBC_I2C_MODE

/// @brief Set the PCA driver address. If you solder just the A6 pin, the address is 0x40, which is the default.
const uint8_t PCA_DRIVER_ADDRESS = PCA_DRIVER_I2C0_DEFAULT_ADDRESS;

/// @brief Set a debouncing period for the encoder.
const int ENCODER_GLITCH_PERIOD = DEFAULT_ENCODER_GLITCH_PERIOD;

/// @brief Set the maximum possible RPM of the motor.
const int MAX_MOTOR_RPM = 600;

/// @brief Set the gear ratio of the motor. This is the ratio of the motor's input speed to the output speed.
const int MOTOR_GEAR_RATIO = 30;

/// @brief Set the encoder CPR (counts per revolution), according to the internal encoder.
const int ENCODER_CPR = 52;

/// @brief Set the encoder limit, the maximum value the encoder can reach (PPR). This is calculated by multiplying the CPR by the gear ratio.
const int ENCODER_LIMIT = DEFAULT_ENCODER_LIMIT;

/// @brief Delay between execution of RTOS tasks.
#define STANDARD_DELAY 10

/// @brief Priority of RTOS tasks.
#define TASK_PRIORITY 1

/// @brief I2C0 master bus handle (ESP32 as master).
i2c_master_bus_handle_t i2c0_master_bus_handle;

#ifdef SBC_I2C_MODE
/// @brief I2C1 slave device handle (ESP32 as slave).
i2c_slave_dev_handle_t i2c1_slave_device_handle;
#endif

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

#ifdef SBC_I2C_MODE
    ESP_ERROR_CHECK(i2c_new_slave_device(&DEFAULT_I2C1_SLAVE_CONFIG, &i2c1_slave_device_handle));

    ESP_ERROR_CHECK(sbc_i2c1_register_callback(i2c1_slave_device_handle));
#endif

#ifdef SBC_UART_MODE
    ESP_ERROR_CHECK(sbc_uart_init(DEFAULT_UART0_CONFIG));
#endif

    ESP_ERROR_CHECK(PCA_init(i2c0_master_bus_handle));

    ESP_ERROR_CHECK(ADC1_motors_init(DEFAULT_MOTORS_PIN_OUTS, NUMBER_OF_MOTORS));

    ESP_ERROR_CHECK(ADC2_battery_init());

    ESP_ERROR_CHECK(encoder_all_init(DEFAULT_MOTORS_PIN_OUTS, NUMBER_OF_MOTORS));

    initialize_motors();

    return;
}

#ifdef SBC_I2C_MODE
/** @brief Communicate with the SBC over I2C1.
 *  @details Read data from the SBC I2C1 and dump it to `sbc_i2c1_receive_buffer`. If data was received from the SBC, it will be written to the relevant variable; if data was requested, it will be written to the send buffer `_sbc_i2c1_send_buffer`. Further, the data parsed to make sense of the identifying "pseudo register" address.
 */
void i2c1_communicate_with_sbc_task() {
    uint8_t data_type;
    uint8_t received_data_count;
    uint8_t transmit_data_count;
    int received_data[NUMBER_OF_MOTORS];
    int data_to_send[NUMBER_OF_MOTORS];
    int transmit_length;

    while (1) {
        ESP_ERROR_CHECK(sbc_i2c1_read_data(i2c1_slave_device_handle));

        if (sbc_data_received_flag) {
            ESP_ERROR_CHECK(parse_i2c_frame(sbc_i2c1_receive_buffer, SBC_I2C1_RECEIVE_MAX_DATA_LENGTH, &data_type, &received_data_count, received_data));

            ESP_ERROR_CHECK(parse_sbc_data(data_type, received_data_count, received_data, &transmit_data_count, data_to_send));
            if (transmit_data_count > 0) {
                ESP_ERROR_CHECK(package_i2c_frame(data_to_send, transmit_data_count, data_type, &transmit_length, sbc_i2c1_send_buffer));
                ESP_ERROR_CHECK(i2c_slave_transmit(i2c1_slave_device_handle, sbc_i2c1_send_buffer, transmit_length, pdMS_TO_TICKS(I2C_STANDARD_TIMEOUT_MS)));
            }
        }
    }
}
#endif

#ifdef SBC_UART_MODE
/** @brief Communicate with the SBC over UART0.
 *  @details Read data from the SBC UART0 and dump it to `_sbc_uart0_receive_buffer`. If data was received from the SBC, it will be written to the relevant variable; if data was requested, it will be written to the send buffer `_sbc_uart0_send_buffer`. Further, the data parsed to make sense of the identifying "pseudo register" address.
 */
void uart0_communicate_with_sbc_task() {
    uint8_t data_type;
    uint8_t received_data_count;
    uint8_t transmit_data_count;
    size_t transmit_length;
    int received_data[NUMBER_OF_MOTORS];
    int data_to_send[NUMBER_OF_MOTORS];

    while (1) {
        ESP_ERROR_CHECK(uart_event_task());

        if (sbc_data_received_flag) {

            ESP_ERROR_CHECK(parse_uart_frame(sbc_uart0_receive_buffer, uart_received_data_length, &data_type, &received_data_count, received_data));
            ESP_ERROR_CHECK(parse_sbc_data(data_type, received_data_count, received_data, &transmit_data_count, data_to_send));
            
            if (transmit_data_count > 0) {
                ESP_ERROR_CHECK(package_uart_frame(data_to_send, transmit_data_count, data_type, &transmit_length, sbc_uart0_send_buffer));
                uart_tx_chars(UART_CHOSEN_PORT, sbc_uart0_send_buffer, transmit_length);
                ESP_ERROR_CHECK(uart_wait_tx_done(UART_CHOSEN_PORT, pdMS_TO_TICKS(1000)));
            }
        }
    }
}
#endif

/** @brief Update sensor registers.
 *  @details Update the encoder data register, motor current data register, motor disconnect status, and battery voltage register. This essentially meas placing the data from the actual peripherals into the pseudo registers. This task runs indefinitely.
 */
void update_sensor_registers_task() {
    while (1) {
        ESP_ERROR_CHECK(update_encoder_data_register(DEFAULT_MOTORS_PIN_OUTS));
        ESP_ERROR_CHECK(update_motor_current_data_register(DEFAULT_MOTORS_PIN_OUTS));
        ESP_ERROR_CHECK(update_motor_disconnect_status());
        ESP_ERROR_CHECK(update_battery_voltage_register());
        ESP_ERROR_CHECK(calculate_rpm_task());

        vTaskDelay(pdMS_TO_TICKS(STANDARD_DELAY));
    }
}

/** @brief Command the bot.
 *  @details If a command has been received, publish the motor command to the PCA driver. This task runs indefinitely.
 */
void command_bot_task() {
    while (1) {
        if (sbc_data_received_flag) {
            ESP_ERROR_CHECK(publish_motor_command(DEFAULT_MOTORS_PIN_OUTS));
            sbc_data_received_flag = false;
        }

        vTaskDelay(pdMS_TO_TICKS(STANDARD_DELAY));
    }
}

void app_main(void) {
    int_brain_init();

#ifdef SBC_I2C_MODE
    xTaskCreatePinnedToCore(i2c1_communicate_with_sbc_task, "i2c1_communicate_with_sbc", 4096, NULL, TASK_PRIORITY, NULL, APP_CPU_NUM);
#endif

#ifdef SBC_UART_MODE
    xTaskCreatePinnedToCore(uart0_communicate_with_sbc_task, "uart0_communicate_with_sbc", 4096, NULL, TASK_PRIORITY, NULL, APP_CPU_NUM);
#endif

    xTaskCreatePinnedToCore(update_sensor_registers_task, "update_sensor_registers", 4096, NULL, TASK_PRIORITY, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(command_bot_task, "command_bot", 4096, NULL, TASK_PRIORITY, NULL, APP_CPU_NUM);

    return;
}