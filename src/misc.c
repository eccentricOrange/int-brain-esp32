#include "int-brain.h"

/** @file misc.c
 *  @brief Mostly deals with updating sensor registers and converting data between formats.
 */

/** @brief Read the actual encoder values and update them in the relevant array.
 *  @attention Pseudo register-based function
 *  @param motors Array of `motor_t` structures, the pin-outs.
 *  @return `ESP_OK` if successful.
 */
esp_err_t update_encoder_data(motor_t* motors) {
    for (size_t i = 0; i < NUMBER_OF_MOTORS; i++) {
        _encoder_positions[i] = encoder_read(motors[i]);
    }

    return ESP_OK;
}

/** @brief Read the actual motor current values and update them in the relevant array.
 *  @attention Pseudo register-based function
 *  @param motors Array of `motor_t` structures, the pin-outs.
 *  @return `ESP_OK` if successful.
 */
esp_err_t update_motor_current_data(motor_t* motors) {
    for (size_t i = 0; i < NUMBER_OF_MOTORS; i++) {
        _motor_currents[i] = _ADC_read_motor_current(motors[i]);
    }

    return ESP_OK;
}

/** @brief Update the `_motor_disconnect_status` variable. A motor is disconnected if it doesn't draw a minimum current but has a minimum PWM.
 *  @attention Pseudo register-based function
 *  @return `ESP_OK` if successful.
 */
esp_err_t update_motor_disconnect_status() {
    uint8_t mask = 0b01;
    _motor_disconnect_status = 0;

    for (size_t i = 0; i < NUMBER_OF_MOTORS; i++) {
        if (_motor_speeds[i] >= MOTOR_MINIMUM_DISCONNECT_PWM && _motor_currents[i] < MOTOR_MINIMUM_CONNECTED_CURRENT) {
            _motor_disconnect_status |= mask;
        }

        mask <<= MOTOR_NUMBER_OF_BITS_PER_MOTOR;
    }

    return ESP_OK;
}

/** @brief Update the `_motor_directions` array from the `_motor_direction_register`.
 *  @attention Pseudo register-based function
 *  @return `ESP_OK` if successful.
 */
esp_err_t update_motor_directions_from_register() {
    for (size_t i = 0; i < NUMBER_OF_MOTORS; i++) {
        _motor_directions[i] = (_motor_direction_register >> (i * MOTOR_NUMBER_OF_BITS_PER_MOTOR)) & 0b11;
    }

    return ESP_OK;
}

/** @brief Convert the `motor_direction_t` style direction and the PWM for a motor into PWM values for each terminal of the motor.
 *  @attention Direct task function
 *  @param direction Direction to move the motor.
 *  @param PWM_in PWM value for the motor.
 *  @param PWM_1_out Pointer to the PWM value for terminal 1.
 *  @param PWM_2_out Pointer to the PWM value for terminal 2.
 */
void direction_to_PWMs(motor_direction_t direction, uint8_t PWM_in, uint8_t* PWM_1_out, uint8_t* PWM_2_out) {
    switch (direction) {
        // Idle
        case IDLE:
            *PWM_1_out = 0;
            *PWM_2_out = 0;
            break;

        // Reverse
        case REVERSE:
            *PWM_1_out = 0;
            *PWM_2_out = PWM_in;
            break;

        // Brake
        case BRAKE:
            *PWM_1_out = 0;
            *PWM_2_out = 0;
            break;

        // Forward
        case FORWARD:
            *PWM_1_out = PWM_in;
            *PWM_2_out = 0;
            break;

        // Error, brake and exit
        default:
            *PWM_1_out = 0;
            *PWM_2_out = 0;
            break;
    }

    return;
}

/** @brief Choose the PWM value based on the motor mode.
 *  @attention Direct task function
 *  @param command_PWM PWM value to filter.
 *  @return Filtered PWM value.
 */
uint8_t motor_PWM_mode_filter(uint8_t command_PWM, motor_speed_mode_t speed_mode) {
    switch (speed_mode) {
        case STOP:
            return 0;

        case MAX:
            return PCA_MAX_PWM_DUTY;

        case COMMAND:
            return command_PWM;

        case COMMON:
            return _common_speed;

        default:
            return command_PWM;
    }
}

/** @brief Initialize the ADC for the battery voltage sense pin. Also calibrate the ADC.
 *  @attention Direct task function
 *  @attention **DO NOT USE IF YOU CANNOT USE ADC2** (such as when using Wi-Fi)
 *  @return `ESP_OK` if successful.
 */
esp_err_t ADC_battery_init() {
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
 *  @return The voltage reading.
 */
int ADC_read_battery_voltage() {
    int voltageReading;
    adc_channel_t channel;

    ESP_ERROR_CHECK(adc_oneshot_io_to_channel(BATTERY_VOLTAGE_SENSE_PIN, _adc2_unit_handle, &channel));
    ESP_ERROR_CHECK(adc_oneshot_get_calibrated_result(_adc2_unit_handle, _adc2_cali_handle, channel, &voltageReading));

    return voltageReading;
}

/** @brief Update the battery voltage.
 *  @attention Direct task function
 *  @return `ESP_OK` if successful.
 */
esp_err_t update_battery_voltage() {
    _battery_voltage = ADC_read_battery_voltage();
    return ESP_OK;
}