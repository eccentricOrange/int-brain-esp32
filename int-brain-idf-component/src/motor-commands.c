#include "int-brain.h"

/** @file motor-commands.c
 *  @brief Implement the functions to control the motors.
 *  @author @eccentricOrange
 */

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

/** @brief Choose the PWM value based on the motor mode as defined in `motor_speed_mode_t`.
 *  @attention Direct task function
 *  @param command_PWM Per-motor command PWM
 *  @param common_speed Common PWM speed for all motors
 *  @param speed_mode Speed mode
 *  @return Chosen PWM value.
 */
uint8_t motor_PWM_mode_filter(uint8_t command_PWM, uint8_t common_speed, motor_speed_mode_t speed_mode) {
    switch (speed_mode) {
        case STOP:
            return 0;

        case MAX:
            return PCA_MAX_PWM_DUTY;

        case COMMAND:
            return command_PWM;

        case COMMON:
            return common_speed;

        default:
            return command_PWM;
    }
}

/** @brief Update the motor mode register `_motor_mode_register` and the motor mode variables.
 *  @attention Pseudo-register task function
 *  @param safety_mode Safety mode
 *  @param speed_mode Speed mode
 *  @param enable Enable motor output?
 *  @return `ESP_OK` if successful.
 */
esp_err_t set_motor_mode_register(motor_safety_mode_t safety_mode, motor_speed_mode_t speed_mode, bool enable) {
    _motor_mode_register = 0;

    _motor_mode_register |= safety_mode << MOTOR_MODE_SAFETY_BIT_0_POSITION;
    _motor_mode_register |= speed_mode << MOTOR_MODE_SPEED_BIT_0_POSITION;
    _motor_mode_register |= enable << MOTOR_MODE_ENABLE_BIT_POSITION;

    _motor_safety_mode = safety_mode;
    _motor_speed_mode = speed_mode;
    _motor_output_enabled = enable;

    return ESP_OK;
}

/** @brief Update the motor safety mode register `_motor_safety_mode` and the motor mode variables.
 *  @attention Pseudo-register task function
 *  @param safety_mode Safety mode
 *  @return `ESP_OK` if successful.
 */
esp_err_t set_motor_safety_mode(motor_safety_mode_t safety_mode) {
    _motor_safety_mode = safety_mode;
    return set_motor_mode_register(_motor_safety_mode, _motor_speed_mode, _motor_output_enabled);
}

/** @brief Update the motor speed mode register `_motor_speed_mode` and the motor mode variables.
 *  @attention Pseudo-register task function
 *  @param speed_mode Speed mode
 *  @return `ESP_OK` if successful.
 */
esp_err_t set_motor_speed_mode(motor_speed_mode_t speed_mode) {
    _motor_speed_mode = speed_mode;
    return set_motor_mode_register(_motor_safety_mode, _motor_speed_mode, _motor_output_enabled);
}

/** @brief Update the motor output register `_motor_output_enabled` and the motor mode variables. Sets it to true.
 *  @attention Pseudo-register task function
 *  @return `ESP_OK` if successful.
 */
esp_err_t enable_motor_output() {
    _motor_output_enabled = true;
    return set_motor_mode_register(_motor_safety_mode, _motor_speed_mode, _motor_output_enabled);
}

/** @brief Update the motor output register `_motor_output_enabled` and the motor mode variables. Sets it to false.
 *  @attention Pseudo-register task function
 *  @return `ESP_OK` if successful.
 */
esp_err_t disable_motor_output() {
    _motor_output_enabled = false;
    return set_motor_mode_register(_motor_safety_mode, _motor_speed_mode, _motor_output_enabled);
}

/** @brief Update the motor speeds `_raw_motor_speeds` pseudo-register.
 *  @attention Pseudo-register task function
 *  @param speeds Array of motor speeds.
 *  @return `ESP_OK` if successful.
 */
esp_err_t set_motor_speeds(uint8_t* speeds) {
    for (size_t i = 0; i < NUMBER_OF_MOTORS; i++) {
        _raw_motor_speeds[i] = speeds[i];
    }

    return ESP_OK;
}

/** @brief Update the common motor speed `_common_speed` pseudo-register.
 *  @attention Pseudo-register task function
 *  @param speed Common motor speed.
 *  @return `ESP_OK` if successful.
 */
esp_err_t set_common_motor_speed(uint8_t speed) {
    _common_speed = speed;
    return ESP_OK;
}

/** @brief Update the motor directions `_motor_directions` pseudo-register.
 *  @attention Pseudo-register task function
 *  @param directions Array of motor directions.
 *  @return `ESP_OK` if successful.
 */
esp_err_t set_motor_directions(motor_direction_t* directions) {
    for (size_t i = 0; i < NUMBER_OF_MOTORS; i++) {
        _motor_directions[i] = directions[i];
    }

    return ESP_OK;
}

/** @brief Update the motor direction `_bot_direction` pseudo-register, and parse the direction into motor directions `_motor_directions`.
 *  @attention Pseudo-register task function
 *  @param direction Direction of the robot.
 *  @return `ESP_OK` if successful.
 */
esp_err_t set_bot_direction(bot_direction_t direction) {
    _bot_direction = direction;

    switch (_bot_direction) {
        case FRONT:
            _motor_directions[0] = FORWARD;
            _motor_directions[1] = FORWARD;
            _motor_directions[2] = FORWARD;
            _motor_directions[3] = FORWARD;
            break;

        case BACK:
            _motor_directions[0] = REVERSE;
            _motor_directions[1] = REVERSE;
            _motor_directions[2] = REVERSE;
            _motor_directions[3] = REVERSE;
            break;

        case ROTATE_CLOCKWISE:
            _motor_directions[0] = FORWARD;
            _motor_directions[1] = REVERSE;
            _motor_directions[2] = FORWARD;
            _motor_directions[3] = REVERSE;
            break;

        case ROTATE_COUNTERCLOCKWISE:
            _motor_directions[0] = REVERSE;
            _motor_directions[1] = FORWARD;
            _motor_directions[2] = REVERSE;
            _motor_directions[3] = FORWARD;
            break;

        case FRONT_LEFT:
            _motor_directions[0] = FORWARD;
            _motor_directions[1] = BRAKE;
            _motor_directions[2] = FORWARD;
            _motor_directions[3] = BRAKE;
            break;

        case FRONT_RIGHT:
            _motor_directions[0] = BRAKE;
            _motor_directions[1] = FORWARD;
            _motor_directions[2] = BRAKE;
            _motor_directions[3] = FORWARD;
            break;

        case BACK_LEFT:
            _motor_directions[0] = REVERSE;
            _motor_directions[1] = BRAKE;
            _motor_directions[2] = REVERSE;
            _motor_directions[3] = BRAKE;
            break;

        case BACK_RIGHT:
            _motor_directions[0] = BRAKE;
            _motor_directions[1] = REVERSE;
            _motor_directions[2] = BRAKE;
            _motor_directions[3] = REVERSE;
            break;

        default:
            for (size_t i = 0; i < NUMBER_OF_MOTORS; i++) {
                _motor_directions[i] = BRAKE;
            }
            break;
    }

    return ESP_OK;
}

/** @brief Write all motor-related pseudo-registers to the PCA9685 over I2C0.
 *  @attention Pseudo-register task function
 *  @param motors Array of `motor_t` structures, the pin-outs.
 *  @return `ESP_OK` if successful.
 */
esp_err_t publish_motor_command(motor_t* motors) {
    esp_err_t status;

    status = gpio_set_level(PCA_MOTOR_ENABLE_PIN, !_motor_output_enabled);
    if (status != ESP_OK) {
        return status;
    }

    for (size_t i = 0; i < NUMBER_OF_MOTORS; i++) {
        _filtered_motor_speeds[i] = motor_PWM_mode_filter(_raw_motor_speeds[i], _common_speed, _motor_speed_mode);
    }

    switch (_motor_safety_mode) {
        case UNSAFE:
            for (size_t i = 0; i < NUMBER_OF_MOTORS; i++) {
                status = PCA_command_motor_with_LED(motors[i], _motor_directions[i], _filtered_motor_speeds[i]);
            }

            break;

        case PROTECT_DISCONNECT:
            for (size_t i = 0; i < NUMBER_OF_MOTORS; i++) {
                uint8_t disconnected_status = _motor_disconnect_status & (0b11 << (i * MOTOR_NUMBER_OF_BITS_PER_MOTOR));

                if (disconnected_status == 0b00) {
                    esp_err_t status = PCA_command_motor_with_LED(motors[i], _motor_directions[i], _filtered_motor_speeds[i]);
                    if (status != ESP_OK) {
                        return status;
                    }
                } else {
                    esp_err_t status = PCA_command_motor_with_LED(motors[i], BRAKE, 0);
                    if (status != ESP_OK) {
                        return status;
                    }
                }
            }

            break;

        default:
            for (size_t i = 0; i < NUMBER_OF_MOTORS; i++) {
                esp_err_t status = PCA_command_motor_with_LED(motors[i], _motor_directions[i], _filtered_motor_speeds[i]);

                if (status != ESP_OK) {
                    return status;
                }
            }

            break;
    }

    return ESP_OK;
}

/** @brief Update the motor stall status `_motor_stall_status` pseudo-register.
 *  @attention Pseudo-register task function
 *  @return `ESP_OK` if successful.
 */
esp_err_t update_motor_disconnect_status() {
    uint8_t mask = 0b01;
    _motor_disconnect_status = 0;

    for (size_t i = 0; i < NUMBER_OF_MOTORS; i++) {
        if (_filtered_motor_speeds[i] >= MOTOR_MINIMUM_DISCONNECT_PWM && _motor_currents[i] < MOTOR_MINIMUM_CONNECTED_CURRENT) {
            _motor_disconnect_status |= mask;
        }

        mask <<= MOTOR_NUMBER_OF_BITS_PER_MOTOR;
    }

    return ESP_OK;
}