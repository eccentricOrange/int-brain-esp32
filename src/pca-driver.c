#include "int-brain.h"

/** @file PCA Driver
 *  @brief This file contains the functions to handle the PCA9635 PWM demux.
 */

// for a high PWM: the motor is fast but the LED is dim
// for a low PWM: the motor is slow but the LED is bright
// correct this by reversing the PWM for the LED
// also remap the LED's PWM to leverage logarithmic perception
#define calculate_LED_PWM(PWM) (-(PCA_MAX_PWM_DUTY / PCA_MAX_LED_PWM) * (PWM - PCA_MAX_PWM_DUTY))

/** @brief Initialize the PCA9635 driver as an I2C device.
 *  @param bus_handle I2C bus handle.
 *  @return `ESP_OK` if successful.
 */
esp_err_t _PCA_i2c_init(i2c_master_bus_handle_t bus_handle) {
    const i2c_device_config_t PCA_I2C_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = PCA_DRIVER_ADDRESS,
        .scl_speed_hz = PCA_I2C0_SPEED};

    return i2c_master_bus_add_device(bus_handle, &PCA_I2C_config, &_PCA_I2C0_device_handle);
}

/** @brief Set the register of the PCA9635 driver.
 *  @param register_address Register address to write to.
 *  @param data Data to write to the register.
 *  @return `ESP_OK` if successful.
 */
esp_err_t _PCA_set_register(uint8_t register_address, uint8_t data) {
    const uint8_t data_array[] = {register_address, data};
    return i2c_master_transmit(_PCA_I2C0_device_handle, &data_array, 2, PCA_I2C0_WRITE_TIMEOUT);
}

/** @brief Enable the LED drivers on the PCA9635 driver byt setting their registers.
 *  @return `ESP_OK` if successful.
 */
esp_err_t _PCA_enable_drivers() {
    for (uint8_t address_index = 0; address_index < PCA_NUMBER_OF_LED_DRIVERS; address_index++) {
        esp_err_t status = _PCA_set_register(PCA_LED_DRIVER_ADDRESS + address_index, PCA_ENABLE_DRIVERS);
        if (status != ESP_OK) {
            return status;
        }
    }
    return ESP_OK;
}

/** @brief Command a motor and an LED with a PWM value. LED PWMs are re-scaled and motor PWMs are converted based on direction.
 *  @param motor Motor to command.
 *  @param direction Direction to move the motor.
 *  @param PWM PWM value for the motor.
 *  @return `ESP_OK` if successful.
 */
esp_err_t PCA_command_motor_with_LED(motor_t motor, motor_direction_t direction, uint8_t PWM) {
    uint8_t PWM_1, PWM_2;
    esp_err_t status;

    direction_to_PWMs(direction, PWM, &PWM_1, &PWM_2);

    const uint8_t PWM1_data[] = {PWM0_REGISTER_ADDRESS + motor.output_pin_1, PWM_1};
    const uint8_t PWM2_data[] = {PWM0_REGISTER_ADDRESS + motor.output_pin_2, PWM_2};
    const uint8_t LED_data[] = {PWM0_REGISTER_ADDRESS + motor.LED_pin, calculate_LED_PWM(PWM)};

    status = i2c_master_transmit(_PCA_I2C0_device_handle, &PWM1_data, 2, PCA_I2C0_WRITE_TIMEOUT);
    if (status != ESP_OK) {
        return status;
    }

    status = i2c_master_transmit(_PCA_I2C0_device_handle, &PWM2_data, 2, PCA_I2C0_WRITE_TIMEOUT);
    if (status != ESP_OK) {
        return status;
    }

    status = i2c_master_transmit(_PCA_I2C0_device_handle, &LED_data, 2, PCA_I2C0_WRITE_TIMEOUT);
    if (status != ESP_OK) {
        return status;
    }

    return ESP_OK;
}

/** @brief Command a motor with a PWM value, ignoring the LED. Motor PWMs are converted based on direction.
 *  @param motor Motor to command.
 *  @param direction Direction to move the motor.
 *  @param PWM PWM value for the motor.
 *  @return `ESP_OK` if successful.
 */
esp_err_t PCA_command_motor(motor_t motor, motor_direction_t direction, uint8_t PWM) {
    uint8_t PWM_1, PWM_2;
    esp_err_t status;

    direction_to_PWMs(direction, PWM, &PWM_1, &PWM_2);

    const uint8_t PWM1_data[] = {PWM0_REGISTER_ADDRESS + motor.output_pin_1, PWM_1};
    const uint8_t PWM2_data[] = {PWM0_REGISTER_ADDRESS + motor.output_pin_2, PWM_2};

    status = i2c_master_transmit(_PCA_I2C0_device_handle, &PWM1_data, 2, PCA_I2C0_WRITE_TIMEOUT);
    if (status != ESP_OK) {
        return status;
    }

    status = i2c_master_transmit(_PCA_I2C0_device_handle, &PWM2_data, 2, PCA_I2C0_WRITE_TIMEOUT);
    if (status != ESP_OK) {
        return status;
    }

    return ESP_OK;
}

/** @brief Command an LED with a PWM value.
 *  @param motor Motor the LED indicates for.
 *  @param PWM PWM value for the LED.
 *  @return `ESP_OK` if successful.
 */
esp_err_t PCA_command_LED(motor_t motor, uint8_t PWM) {
    const uint8_t LED_data[] = {PWM0_REGISTER_ADDRESS + motor.LED_pin, calculate_LED_PWM(PWM)};
    return i2c_master_transmit(_PCA_I2C0_device_handle, &LED_data, 2, PCA_I2C0_WRITE_TIMEOUT);
}

/** @brief Initialize the motor output enable (MOE) pin as an output. This enables/disables the PCA9635 driver.
 *  @return `ESP_OK` if successful.
 */
esp_err_t _PCA_MOE_init() {
    const gpio_config_t MOE_config = {
        .pin_bit_mask = 1 << PCA_MOTOR_ENABLE_PIN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    return gpio_config(&MOE_config);
}

/** @brief Enable the PCA9635 driver by setting the motor output enable (MOE) pin low.
 *  @return `ESP_OK` if successful.
 */
esp_err_t PCA_enable() {
    return gpio_set_level(PCA_MOTOR_ENABLE_PIN, 0);
}

/** @brief Disable the PCA9635 driver by setting the motor output enable (MOE) pin high.
 *  @return `ESP_OK` if successful.
 */
esp_err_t PCA_disable() {
    return gpio_set_level(PCA_MOTOR_ENABLE_PIN, 1);
}

/** @brief Set the motor output enable (MOE) pin based on the motor mode register.
 *  @return `ESP_OK` if successful.
 */
esp_err_t PCA_set_moe_by_register() {
    if ((_motor_mode & 0b00110000) >> 4) {
        return PCA_enable();
    } else {
        return PCA_disable();
    }
}

/** @brief Initialize the PCA9635 driver, by collecting many of the initialization functions defined in this file.
 *  @param bus_handle I2C bus handle.
 *  @return `ESP_OK` if successful.
 */
esp_err_t PCA_init(i2c_master_bus_handle_t bus_handle) {
    esp_err_t status = _PCA_i2c_init(bus_handle);
    if (status != ESP_OK) {
        return status;
    }

    status = _PCA_MOE_init();
    if (status != ESP_OK) {
        return status;
    }

    status = _PCA_enable_drivers();
    if (status != ESP_OK) {
        return status;
    }

    status = _PCA_set_register(PCA_MODE1_ADDRESS, PCA_MODE1);
    if (status != ESP_OK) {
        return status;
    }

    return ESP_OK;
}