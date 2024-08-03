#include "int-brain.h"

/** @file Four-wheel bot
 *  @brief This file contains the functions to control a four-wheel bot (instead of dealing motor-by-motor).
 */

/** @brief Command the bot to move in a certain direction. No protection or registers are used.
 *  @param motors Array of `motor_t` structures, the pin-outs.
 *  @param bot_direction Direction to move the bot.
 *  @param PWM Array of PWM values for each motor.
 *  @return `ESP_OK` if successful.
 */
esp_err_t command_bot(const motor_t motors[NUMBER_OF_MOTORS], bot_direction_t bot_direction, uint8_t PWM[NUMBER_OF_MOTORS]) {
    motor_direction_t motor_directions[NUMBER_OF_MOTORS];
    esp_err_t status;

    switch (bot_direction) {
        case FRONT:
            motor_directions[0] = FORWARD;
            motor_directions[1] = FORWARD;
            motor_directions[2] = FORWARD;
            motor_directions[3] = FORWARD;
            break;

        case BACK:
            motor_directions[0] = REVERSE;
            motor_directions[1] = REVERSE;
            motor_directions[2] = REVERSE;
            motor_directions[3] = REVERSE;
            break;

        case ROTATE_CLOCKWISE:
            motor_directions[0] = FORWARD;
            motor_directions[1] = REVERSE;
            motor_directions[2] = FORWARD;
            motor_directions[3] = REVERSE;
            break;

        case ROTATE_COUNTERCLOCKWISE:
            motor_directions[0] = REVERSE;
            motor_directions[1] = FORWARD;
            motor_directions[2] = REVERSE;
            motor_directions[3] = FORWARD;
            break;

        case FRONT_LEFT:
            motor_directions[0] = FORWARD;
            motor_directions[1] = BRAKE;
            motor_directions[2] = FORWARD;
            motor_directions[3] = BRAKE;
            break;

        case FRONT_RIGHT:
            motor_directions[0] = BRAKE;
            motor_directions[1] = FORWARD;
            motor_directions[2] = BRAKE;
            motor_directions[3] = FORWARD;
            break;

        case BACK_LEFT:
            motor_directions[0] = REVERSE;
            motor_directions[1] = BRAKE;
            motor_directions[2] = REVERSE;
            motor_directions[3] = BRAKE;
            break;

        case BACK_RIGHT:
            motor_directions[0] = BRAKE;
            motor_directions[1] = REVERSE;
            motor_directions[2] = BRAKE;
            motor_directions[3] = REVERSE;
            break;

        default:
            for (size_t i = 0; i < NUMBER_OF_MOTORS; i++) {
                motor_directions[i] = BRAKE;
            }
            break;
    }

    for (size_t i = 0; i < NUMBER_OF_MOTORS; i++) {
        status = PCA_command_motor_with_LED(motors[i], motor_directions[i], PWM[i]);

        if (status != ESP_OK) {
            return status;
        }
    }

    return ESP_OK;
}

/** @brief Command the bot to move in a certain direction with the same speed, no protection or registers are used.
 *  @param motors Array of `motor_t` structures, the pin-outs.
 *  @param bot_direction Direction to move the bot.
 *  @param PWM PWM value for all motors.
 *  @return `ESP_OK` if successful.
 */
esp_err_t command_bot_same_speed(const motor_t motors[NUMBER_OF_MOTORS], bot_direction_t bot_direction, uint8_t PWM) {
    const uint8_t PWMs[NUMBER_OF_MOTORS] = {PWM, PWM, PWM, PWM};
    return command_bot(motors, bot_direction, PWMs);
}

/** @brief Command the bot to move in a certain direction with the same speed, no protection or registers are used.
 *  @param motors Array of `motor_t` structures, the pin-outs.
 *  @param data Array of data for each motor, `[direction, PWM]`.
 *  @return `ESP_OK` if successful.
 */
esp_err_t command_bot_array(const motor_t motors[NUMBER_OF_MOTORS], uint8_t* data) {
    for (size_t i = 0; i < NUMBER_OF_MOTORS * 2; i += 2) {
        esp_err_t status = PCA_command_motor_with_LED(motors[i / 2], data[i], data[i + 1]);

        if (status != ESP_OK) {
            return status;
        }
    }

    return ESP_OK;
}

/** @brief Command the bot to move in a certain direction with the same speed, with protection and registers.
 *  @param motors Array of `motor_t` structures, the pin-outs.
 *  @return `ESP_OK` if successful.
 */
esp_err_t command_bot_registers_safe(const motor_t motors[NUMBER_OF_MOTORS]) {
    const uint8_t mask = 0b11;
    uint8_t filtered_motor_speeds[NUMBER_OF_MOTORS];
    for (size_t i = 0; i < NUMBER_OF_MOTORS; i++) {
        filtered_motor_speeds[i] = motor_PWM_mode_filter(_motor_speeds[i], (_motor_mode & 0b00001100) >> 2);
    }

    motor_safety_mode_t safety_mode = _motor_mode & 0b00000011;

    switch (safety_mode) {
        case UNSAFE:

            // unsafe
            for (size_t i = 0; i < NUMBER_OF_MOTORS; i++) {
                esp_err_t status = PCA_command_motor_with_LED(motors[i], _motor_directions[i], filtered_motor_speeds[i]);

                if (status != ESP_OK) {
                    return status;
                }
            }

            break;

        case PROTECT_DISCONNECT:

            // Protect disconnect
            for (size_t i = 0; i < NUMBER_OF_MOTORS; i++) {
                uint8_t disconnected_status = _motor_disconnect_status & (mask << (i * MOTOR_NUMBER_OF_BITS_PER_MOTOR));

                if (disconnected_status == 0b00) {
                    esp_err_t status = PCA_command_motor_with_LED(motors[i], _motor_directions[i], filtered_motor_speeds[i]);

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
                esp_err_t status = PCA_command_motor_with_LED(motors[i], _motor_directions[i], filtered_motor_speeds[i]);

                if (status != ESP_OK) {
                    return status;
                }
            }

            break;
    }

    return ESP_OK;
}

/** @brief Command all motors with individual speeds and directions, but you can choose the safety mode and speed mode.
 *  @param motors Array of `motor_t` structures, the pin-outs.
 *  @param safety_mode Safety mode to use.
 *  @param speed_mode Speed mode to use.
 *  @param motor_speeds Array of PWM values for each motor.
 *  @param motor_directions Array of directions for each motor.
 *  @return `ESP_OK` if successful.
 */
esp_err_t command_all_motors_safe(
    const motor_t motors[NUMBER_OF_MOTORS],
    motor_safety_mode_t safety_mode,
    motor_speed_mode_t speed_mode,
    uint8_t* motor_speeds,
    motor_direction_t* motor_directions) {
    const uint8_t mask = 0b11;
    uint8_t filtered_motor_speeds[NUMBER_OF_MOTORS];

    for (size_t i = 0; i < NUMBER_OF_MOTORS; i++) {
        filtered_motor_speeds[i] = motor_PWM_mode_filter(motor_speeds[i], speed_mode);
    }

    switch (safety_mode) {
        case UNSAFE:

            // unsafe
            for (size_t i = 0; i < NUMBER_OF_MOTORS; i++) {
                esp_err_t status = PCA_command_motor_with_LED(motors[i], motor_directions[i], filtered_motor_speeds[i]);

                if (status != ESP_OK) {
                    return status;
                }
            }

            break;

        case PROTECT_DISCONNECT:

            // Protect disconnect
            for (size_t i = 0; i < NUMBER_OF_MOTORS; i++) {
                uint8_t disconnected_status = _motor_disconnect_status & (mask << (i * MOTOR_NUMBER_OF_BITS_PER_MOTOR));

                if (disconnected_status == 0b00) {
                    esp_err_t status = PCA_command_motor_with_LED(motors[i], motor_directions[i], filtered_motor_speeds[i]);

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
                esp_err_t status = PCA_command_motor_with_LED(motors[i], motor_directions[i], filtered_motor_speeds[i]);

                if (status != ESP_OK) {
                    return status;
                }
            }

            break;
    }

    return ESP_OK;
}