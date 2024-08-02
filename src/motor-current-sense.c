#include "int-brain.h"

/** @file Motor current sense
 *  @brief This file contains the functions to read the current of the motors.
 */

/** @brief Configure the ADC channels for the motor current sense pins.
 * @param motors Array of `motor_t` structures, the pin-outs.
 * @param number_of_motors Number of motors.
 * @return `ESP_OK` if successful.
 */
esp_err_t _ADC_channels_config(motor_t* motors, size_t number_of_motors) {
    const adc_oneshot_chan_cfg_t adc_chan_config = {
        .atten = ADC_CHOSEN_ATTEN,
        .bitwidth = ADC_CHHOSEN_BITWIDTH,
    };

    adc_channel_t adc_channel;

    esp_err_t status;

    for (size_t i = 0; i < number_of_motors; i++) {
        status = adc_oneshot_io_to_channel(motors[i].current_sense_pin, _adc_motors_handle, &adc_channel);

        if (status != ESP_OK) {
            return status;
        }

        status = adc_oneshot_config_channel(_adc_motors_handle, adc_channel, &adc_chan_config);

        if (status != ESP_OK) {
            return status;
        }
    }

    return ESP_OK;
}

/** @brief Initialize the ADC for the motor current sense pins. Also calibrate the ADC.
 * @param motors Array of `motor_t` structures, the pin-outs.
 * @param number_of_motors Number of motors.
 * @return `ESP_OK` if successful.
 */
esp_err_t ADC_motors_init(motor_t* motors, size_t number_of_motors) {
    const adc_oneshot_unit_init_cfg_t adc1_config = {
        .unit_id = ADC_CHOSEN_UNIT,
    };

    esp_err_t status;
    
    status = adc_oneshot_new_unit(&adc1_config, &_adc_motors_handle);
    if (status != ESP_OK) {
        return status;
    }

    adc_cali_line_fitting_config_t adc1_cali_config = {
        .unit_id = ADC_CHOSEN_UNIT,
        .atten = ADC_CHOSEN_ATTEN,
        .bitwidth = ADC_CHHOSEN_BITWIDTH,
    };

    status = adc_cali_create_scheme_line_fitting(&adc1_cali_config, &_adc_motors_cali_handle);
    if (status != ESP_OK) {
        return status;
    }

    return _ADC_channels_config(motors, number_of_motors);
}

/** @brief Read the motor current sense pin and return the value.
 * @param motor Motor to read the current from.
 * @return The current reading.
 */
int ADC_read_motor_current(motor_t motor) {
    int voltageReading;
    adc_channel_t channel;

    ESP_ERROR_CHECK(adc_oneshot_io_to_channel(motor.current_sense_pin, _adc_motors_handle, &channel));
    ESP_ERROR_CHECK(adc_oneshot_get_calibrated_result(_adc_motors_handle, _adc_motors_cali_handle, channel, &voltageReading));

    return voltageReading;
}