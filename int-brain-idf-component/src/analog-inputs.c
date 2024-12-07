#include "int-brain.h"

/** @file analog-inputs.c
 *  @brief Implement the functions to read the motor current sense pins and battery voltage sense pin.
 *  @author @eccentricOrange
 */

/** @brief Initialize the ADC for the motor current sense pins. Calls `_ADC1_register_configs` and `_ADC1_config_oneshot_channels`.
 *  @attention Direct task function
 *  @param motors Array of `motor_t` structures, the pin-outs.
 *  @param number_of_motors Number of motors.
 *  @return `ESP_OK` if successful.
 */
esp_err_t ADC1_motors_init(motor_t* motors, size_t number_of_motors) {
    esp_err_t status;

    status = _ADC1_register_configs();
    if (status != ESP_OK) {
        return status;
    }

    status = _ADC1_config_oneshot_channels(motors, number_of_motors);
    if (status != ESP_OK) {
        return status;
    }

    return ESP_OK;
}

/** @brief Initialize ADC1 by setting up a new unit and its calibration scheme.
 *  @attention Direct task function
 *  @return `ESP_OK` if successful.
 */
esp_err_t _ADC1_register_configs() {
    const adc_oneshot_unit_init_cfg_t adc1_config = {
        .unit_id = ADC_UNIT_1,
    };

    esp_err_t status;

    status = adc_oneshot_new_unit(&adc1_config, &_adc1_unit_handle);
    if (status != ESP_OK) {
        return status;
    }

    adc_cali_line_fitting_config_t adc1_cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_CHOSEN_ATTEN,
        .bitwidth = ADC_CHHOSEN_BITWIDTH,
    };

    status = adc_cali_create_scheme_line_fitting(&adc1_cali_config, &_adc1_cali_handle);
    if (status != ESP_OK) {
        return status;
    }

    return ESP_OK;
}

/** @brief Configure the ADC1 channels for the motor current sense pins, using GPIO pin-outs from the pin-outs array.
 *  @attention Direct task function
 *  @param motors Array of `motor_t` structures, the pin-outs.
 *  @param number_of_motors Number of motors.
 *  @return `ESP_OK` if successful.
 */
esp_err_t _ADC1_config_oneshot_channels(motor_t* motors, size_t number_of_motors) {
    const adc_oneshot_chan_cfg_t adc_chan_config = {
        .atten = ADC_CHOSEN_ATTEN,
        .bitwidth = ADC_CHHOSEN_BITWIDTH,
    };

    adc_channel_t adc_channel;

    esp_err_t status;

    for (size_t i = 0; i < number_of_motors; i++) {
        status = adc_oneshot_io_to_channel(motors[i].current_sense_pin, _adc1_unit_handle, &adc_channel);

        if (status != ESP_OK) {
            return status;
        }

        status = adc_oneshot_config_channel(_adc1_unit_handle, adc_channel, &adc_chan_config);

        if (status != ESP_OK) {
            return status;
        }
    }

    return ESP_OK;
}

/** @brief Read the motor current sense pin and return the value.
 *  @attention Direct task function
 *  @param motor Motor to read the current from.
 *  @return The current reading.
 */
int _ADC_read_motor_current(motor_t motor) {
    int voltageReading;
    adc_channel_t channel;

    ESP_ERROR_CHECK(adc_oneshot_io_to_channel(motor.current_sense_pin, _adc1_unit_handle, &channel));
    ESP_ERROR_CHECK(adc_oneshot_get_calibrated_result(_adc1_unit_handle, _adc1_cali_handle, channel, &voltageReading));

    return voltageReading;
}

/** @brief Read the actual motor current values and update them in the relevant array.
 *  @attention Pseudo register-based function
 *  @param motors Array of `motor_t` structures, the pin-outs.
 *  @return `ESP_OK` if successful.
 */
esp_err_t update_motor_current_data_register(motor_t* motors) {
    for (size_t i = 0; i < NUMBER_OF_MOTORS; i++) {
        _motor_currents[i] = _ADC_read_motor_current(motors[i]);
    }

    return ESP_OK;
}

/** @brief Get the current reading of a motor.
 *  @attention Pseudo register-based function
 *  @param motor_number Motor number.
 *  @return The current reading.
 */
int get_current_reading(uint8_t motor_number) {
    if (motor_number < NUMBER_OF_MOTORS) {
        return _motor_currents[motor_number];
    } else {
        return -1;
    }
}

/** @brief Initialize ADC2 for the battery voltage sense pin by setting up a new unit, its calibration scheme, and configuring the GPIO channel.
 *  @attention Direct task function
 *  @attention **DO NOT USE IF YOU CANNOT USE ADC2** (such as when using Wi-Fi)
 *  @return `ESP_OK` if successful.
 */
esp_err_t ADC2_battery_init() {
    const adc_oneshot_unit_init_cfg_t adc2_config = {
        .unit_id = ADC_UNIT_2,
    };
    const adc_oneshot_chan_cfg_t adc_chan_config = {
        .atten = ADC_CHOSEN_ATTEN,
        .bitwidth = ADC_CHHOSEN_BITWIDTH,
    };
    const adc_cali_line_fitting_config_t adc2_cali_config = {
        .unit_id = ADC_UNIT_2,
        .atten = ADC_CHOSEN_ATTEN,
        .bitwidth = ADC_CHHOSEN_BITWIDTH,
    };

    esp_err_t status;
    adc_channel_t adc_channel;

    status = adc_oneshot_new_unit(&adc2_config, &_adc2_unit_handle);
    if (status != ESP_OK) {
        return status;
    }

    status = adc_cali_create_scheme_line_fitting(&adc2_cali_config, &_adc2_cali_handle);
    if (status != ESP_OK) {
        return status;
    }

    status = adc_oneshot_io_to_channel(BATTERY_VOLTAGE_SENSE_PIN, _adc2_unit_handle, &adc_channel);
    if (status != ESP_OK) {
        return status;
    }

    status = adc_oneshot_config_channel(_adc2_unit_handle, adc_channel, &adc_chan_config);
    if (status != ESP_OK) {
        return status;
    }

    return ESP_OK;
}

/** @brief Read the battery voltage sense pin and return the value.
 *  @attention Direct task function
 *  @attention **DO NOT USE IF YOU CANNOT USE ADC2** (such as when using Wi-Fi)
 *  @return The voltage reading.
 */
int ADC2_read_battery_voltage() {
    int voltageReading;
    adc_channel_t channel;

    ESP_ERROR_CHECK(adc_oneshot_io_to_channel(BATTERY_VOLTAGE_SENSE_PIN, _adc2_unit_handle, &channel));
    ESP_ERROR_CHECK(adc_oneshot_get_calibrated_result(_adc2_unit_handle, _adc2_cali_handle, channel, &voltageReading));

    return voltageReading;
}

/** @brief Update the battery voltage register with the current battery voltage reading.
 *  @attention Direct task function
 *  @return `ESP_OK` if successful.
 */
esp_err_t update_battery_voltage_register() {
    _battery_voltage = ADC2_read_battery_voltage();
    return ESP_OK;
}