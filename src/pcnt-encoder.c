#include "int-brain.h"

/** @file pcnt-encoder.c
 *  @brief Initialize the encoder for the motors and read the encoder values.
 *  @author @eccentricOrange
 */

/** @brief Initialize the encoder for a single motor.
 *  @attention Direct task function
 *  @param motor Motor to initialize the encoder for.
 *  @param pcnt_unit_handle Pointer to the PCNT unit handle.
 *  @return `ESP_OK` if successful.
 */
esp_err_t _encoder_individual_init(motor_t motor, pcnt_unit_handle_t* pcnt_unit_handle) {
    esp_err_t status;

    const pcnt_unit_config_t pcnt_unit_config = {
        .high_limit = ENCODER_LIMIT,
        .low_limit = -ENCODER_LIMIT};

    const pcnt_glitch_filter_config_t glitch_filter_config = {
        .max_glitch_ns = ENCODER_GLITCH_PERIOD,
    };

    // Configure the PCNT unit
    status = pcnt_new_unit(&pcnt_unit_config, pcnt_unit_handle);
    if (status != ESP_OK) {
        return status;
    }

    // Take care of bouncing and other glitches
    status = pcnt_unit_set_glitch_filter(*pcnt_unit_handle, &glitch_filter_config);
    if (status != ESP_OK) {
        return status;
    }

    // Actually configure what a rising and falling edge does
    pcnt_chan_config_t pcnt_chan_config_a = {
        .edge_gpio_num = motor.encoder_pin_1,
        .level_gpio_num = motor.encoder_pin_2,
    };
    pcnt_channel_handle_t pcnt_chan_handle_a;
    status = pcnt_new_channel(*pcnt_unit_handle, &pcnt_chan_config_a, &pcnt_chan_handle_a);
    if (status != ESP_OK) {
        return status;
    }

    pcnt_chan_config_t pcnt_chan_config_b = {
        .edge_gpio_num = motor.encoder_pin_2,
        .level_gpio_num = motor.encoder_pin_1,
    };
    pcnt_channel_handle_t pcnt_chan_handle_b;
    status = pcnt_new_channel(*pcnt_unit_handle, &pcnt_chan_config_b, &pcnt_chan_handle_b);
    if (status != ESP_OK) {
        return status;
    }

    // Register the increment and decrement actions
    status = pcnt_channel_set_edge_action(pcnt_chan_handle_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    if (status != ESP_OK) {
        return status;
    }

    status = pcnt_channel_set_edge_action(pcnt_chan_handle_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    if (status != ESP_OK) {
        return status;
    }

    status = pcnt_channel_set_level_action(pcnt_chan_handle_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    if (status != ESP_OK) {
        return status;
    }

    status = pcnt_channel_set_level_action(pcnt_chan_handle_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    if (status != ESP_OK) {
        return status;
    }

    // Enable the PCNT unit, clear the count, and start counting
    status = pcnt_unit_enable(*pcnt_unit_handle);
    if (status != ESP_OK) {
        return status;
    }

    status = pcnt_unit_clear_count(*pcnt_unit_handle);
    if (status != ESP_OK) {
        return status;
    }

    status = pcnt_unit_start(*pcnt_unit_handle);
    if (status != ESP_OK) {
        return status;
    }

    return ESP_OK;
}

/** @brief Initialize the encoder for all motors.
 *  @attention Direct task function
 *  @param motors Array of `motor_t` structures, the pin-outs.
 *  @param number_of_motors Number of motors.
 *  @return `ESP_OK` if successful.
 */
esp_err_t encoder_all_init(motor_t* motors, size_t number_of_motors) {
    esp_err_t status;

    for (size_t i = 0; i < number_of_motors; i++) {

        status = _encoder_individual_init(motors[i], &motors[i].encoder_unit_handle);
        if (status != ESP_OK) {
            return status;
        }
        
    }
    
    return ESP_OK;
}

/** @brief Read the encoder count for a motor.
 *  @attention Direct task function
 *  @param motor Motor to read the encoder count for.
 *  @return The encoder count.
 */
int encoder_read(motor_t motor) {
    int count;
    ESP_ERROR_CHECK(pcnt_unit_get_count(motor.encoder_unit_handle, &count));
    return count;
}

/** @brief Read the actual encoder values and update them in the relevant array `_encoder_positions`.
 *  @attention Pseudo register-based function
 *  @param motors Array of `motor_t` structures, the pin-outs.
 *  @return `ESP_OK` if successful.
 */
esp_err_t update_encoder_data_register(motor_t* motors) {
    for (size_t i = 0; i < NUMBER_OF_MOTORS; i++) {
        _encoder_positions[i] = encoder_read(motors[i]);
    }

    return ESP_OK;
}