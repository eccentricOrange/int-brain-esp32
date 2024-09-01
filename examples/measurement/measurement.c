/** @file measurement.c
 *  @brief Similar to up-and-down.c, but with the addition of measuring the RPM and current of a motor.
 *  @details This example demonstrates a simple ramp up and ramp down movement of the bot. The bot moves forward and backward in a loop. The LEDs of all motors show the speed by brightness.
 *  @details The  RPM and current of a motor are measured and printed.
 *  @details The "unsafe" mode is used to control the motors.
 *  @author @eccentricOrange
 */

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "int-brain.h"

/// @brief Set the PCA driver address. If you solder just the A6 pin, the address is 0x40, which is the default.

const uint8_t PCA_DRIVER_ADDRESS = PCA_DRIVER_I2C0_DEFAULT_ADDRESS;
/// @brief Set the encoder limit, the maximum value the encoder can reach (PPR).

/// @brief Set the maximum possible RPM of the motor.
const int MAX_MOTOR_RPM = 600;

/// @brief Set the gear ratio of the motor. This is the ratio of the motor's input speed to the output speed.
const int MOTOR_GEAR_RATIO = 30;

/// @brief Set the encoder CPR (counts per revolution), according to the internal encoder.
const int ENCODER_CPR = 52;

/// @brief Set the encoder limit, the maximum value the encoder can reach (PPR). This is calculated by multiplying the CPR by the gear ratio.
const int ENCODER_LIMIT = DEFAULT_ENCODER_LIMIT;

/// @brief Set a debouncing period for the encoder.
const int ENCODER_GLITCH_PERIOD = DEFAULT_ENCODER_GLITCH_PERIOD;

/// @brief Delay between execution of RTOS tasks.
#define STANDARD_DELAY 10

/// @brief Priority of RTOS tasks.
#define TASK_PRIORITY 1

/// @brief I2C0 master bus handle (ESP32 as master).
i2c_master_bus_handle_t i2c0_master_bus_handle;

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

    ESP_ERROR_CHECK(PCA_init(i2c0_master_bus_handle));

    ESP_ERROR_CHECK(ADC1_motors_init(DEFAULT_MOTORS_PIN_OUTS, NUMBER_OF_MOTORS));

    ESP_ERROR_CHECK(encoder_all_init(DEFAULT_MOTORS_PIN_OUTS, NUMBER_OF_MOTORS));

    ESP_ERROR_CHECK(set_motor_mode_register(UNSAFE, COMMON, true));

    return;
}

/** @brief Update sensor registers.
 *  @details Update the encoder data register, motor current data register, motor disconnect status, and battery voltage register. This essentially meas placing the data from the actual peripherals into the pseudo registers. This task runs indefinitely.
 */
void update_sensor_registers_task() {
    while (1) {
        ESP_ERROR_CHECK(update_motor_current_data_register(DEFAULT_MOTORS_PIN_OUTS));
        ESP_ERROR_CHECK(update_motor_disconnect_status());
        ESP_ERROR_CHECK(update_encoder_data_register(DEFAULT_MOTORS_PIN_OUTS));
        ESP_ERROR_CHECK(calculate_rpm_task());

        vTaskDelay(pdMS_TO_TICKS(STANDARD_DELAY));
    }
}

/** @brief Measure motor data.
 *  @details Measure the RPM and current of a motor and print it.
 *  @param motor_index The index of the motor whose data is to be measured.
 */
void measure_motor_data(uint8_t motor_index) {
    printf("Speed: %5d, Current: %4d\n", get_rpm_reading(motor_index), get_current_reading(motor_index));
}

/** @brief Command the bot.
 *  @details Send tasks to the bot to ramp up and ramp down its motors, alternately forwards and backwards. The LEDs of all motors show the speed by brightness. This task runs indefinitely.
 */
void command_bot_task() {
    /// Initialise the bot to move forward. We'll flip this later in the loop.
    bot_direction_t bot_direction = FRONT;

    /// Set the bot to move forward initially.
    ESP_ERROR_CHECK(set_bot_direction(FRONT));

    /// We'll use this to toggle the LED, showing ramp up and ramp down.
    bool LED_state = false;

    /// Choose one of the motors for measurement.
    const uint8_t MOTOR_INDEX = 2;

    while (1) {
        /// Ramp up the motors.
        for (size_t i = 0; i < PCA_MAX_PWM_DUTY; i++) {
            ESP_ERROR_CHECK(set_common_motor_speed(i));
            ESP_ERROR_CHECK(publish_motor_command(DEFAULT_MOTORS_PIN_OUTS));
            measure_motor_data(MOTOR_INDEX);
            vTaskDelay(pdMS_TO_TICKS(5));
        }

        /// Hold the motors at full speed for a while while printing measured RPM and current.
        for (size_t i = 0; i < 500; i++) {
            measure_motor_data(MOTOR_INDEX);
            vTaskDelay(pdMS_TO_TICKS(5));
        }

        LED_state = !LED_state;
        ESP_ERROR_CHECK(gpio_set_level(DEFAULT_LED_PIN, LED_state));

        /// Ramp down the motors.
        for (size_t i = PCA_MAX_PWM_DUTY; i > 0; i--) {
            ESP_ERROR_CHECK(set_common_motor_speed(i));
            ESP_ERROR_CHECK(publish_motor_command(DEFAULT_MOTORS_PIN_OUTS));
            measure_motor_data(MOTOR_INDEX);
            vTaskDelay(pdMS_TO_TICKS(5));
        }

        /// Hold the motors stopped for a while while printing measured RPM and current.
        for (size_t i = 0; i < 500; i++) {
            measure_motor_data(MOTOR_INDEX);
            vTaskDelay(pdMS_TO_TICKS(5));
        }

        LED_state = !LED_state;
        ESP_ERROR_CHECK(gpio_set_level(DEFAULT_LED_PIN, LED_state));

        /// Toggle the bot direction.
        bot_direction = (bot_direction == FRONT) ? BACK : FRONT;
        ESP_ERROR_CHECK(set_bot_direction(bot_direction));

        vTaskDelay(pdMS_TO_TICKS(STANDARD_DELAY));
    }
}

void app_main(void) {
    int_brain_init();
    xTaskCreate(update_sensor_registers_task, "update_sensor_registers", 4096, NULL, TASK_PRIORITY, NULL);
    xTaskCreate(command_bot_task, "command_bot", 4096, NULL, TASK_PRIORITY, NULL);

    return;
}