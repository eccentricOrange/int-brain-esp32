#include "int-brain.h"

/** @file SBC I2C Interaction
 *  @brief This file contains all the necessary functions to interact with the Single Board Computer (SBC) via I2C1.
 */

/** @brief I2C Receive callback for the SBC.
 *  @attention Pseudo register-based function
 *  @param device_handle Handle of the I2C device.
 *  @param edata Event data of the I2C event.
 *  @param user_data User data to pass to the callback.
 *  @return `pdTRUE` if successful.
 */
IRAM_ATTR bool _sbc_i2c1_receive_callback(i2c_slave_dev_handle_t device_handle, i2c_slave_rx_done_event_data_t* edata, void* user_data) {
    QueueHandle_t receive_queue = (QueueHandle_t)user_data;
    return xQueueSendFromISR(receive_queue, edata, NULL);
}

/** @brief Register the callback for the SBC I2C1.
 *  @attention Pseudo register-based function
 *  @param device_handle Handle of the I2C device.
 *  @return `ESP_OK` if successful.
 */
esp_err_t sbc_i2c1_register_callback(i2c_slave_dev_handle_t device_handle) {
    i2c_slave_event_callbacks_t callback_config = {
        .on_recv_done = _sbc_i2c1_receive_callback,
    };
    _sbc_i2c1_receive_queue = xQueueCreate(1, sizeof(i2c_slave_rx_done_event_data_t));

    return i2c_slave_register_event_callbacks(device_handle, &callback_config, _sbc_i2c1_receive_queue);
}

/** @brief Read data from the SBC I2C1 and dump it to `_sbc_i2c1_receive_buffer`.
 *  @attention Pseudo register-based function
 *  @param device_handle Handle of the I2C device.
 *  @param on_receive_callback Callback to call when data is received.
 *  @return `ESP_OK` if successful.
 */
esp_err_t sbc_i2c1_read_data(i2c_slave_dev_handle_t device_handle, esp_err_t* on_receive_callback()) {
    i2c_slave_rx_done_event_data_t rx_data;

    i2c_slave_receive(device_handle, &_sbc_i2c1_receive_buffer, SBC_I2C1_RECEIVE_DATA_BUFFER_SIZE);

    if (xQueueReceive(_sbc_i2c1_receive_queue, &rx_data, 0) == pdPASS) {
        return on_receive_callback();
    }

    return ESP_OK;
}

/** @brief Split an int (4 bytes) into 4 `uint8_t` bytes and write it to a buffer.
 *  @attention Direct task function
 *  @param buffer Buffer to write to.
 *  @param value Value to write.
 *  @param index Buffer index to write to.
 */
inline void _write_int_to_i2c_buffer(uint8_t* buffer, int value, size_t index) {
    for (size_t i = 0; i < MAX_INT_SIZE; i++) {
        buffer[index + i] = (value >> (i * 8)) & 0xFF;
    }
}

/** @brief Parse the data received from the SBC I2C1. Will check the first element for the "register". If data ws received from the SBC, it will be written to the relevant variable; if data was requested, it will be written to the send buffer `_sbc_i2c1_send_buffer`.
 *  @attention Pseudo register-based function
 *  @return `ESP_OK` if successful.
 */
esp_err_t sbc_i2c_parse_data() {
    _sbc_i2c1_register = _sbc_i2c1_receive_buffer[0];

    if (_sbc_i2c1_register == 0x00) {
        return ESP_OK;
    }

    uint8_t motor_index;

    if (_sbc_i2c1_register <= LAST_SLAVE_REQUEST_ADDRESS) {
        for (size_t i = 0; i < SBC_I2C1_SEND_DATA_BUFFER_SIZE; i++) {
            _sbc_i2c1_send_buffer[i] = 0;
        }

        switch (_sbc_i2c1_register) {
            case REQUEST_ALL_ENCODER_ADDRESS:
                for (size_t i = 0; i < MAX_INT_SIZE * NUMBER_OF_MOTORS; i += MAX_INT_SIZE) {
                    _write_int_to_i2c_buffer(_sbc_i2c1_send_buffer, _encoder_positions[i / MAX_INT_SIZE], i);
                }

                break;

            case REQUEST_INDIVIDUAL_ENCODER_FIRST_ADDRESS ...(REQUEST_INDIVIDUAL_ENCODER_FIRST_ADDRESS + NUMBER_OF_MOTORS - 1):
                motor_index = _sbc_i2c1_register - REQUEST_INDIVIDUAL_ENCODER_FIRST_ADDRESS;
                _write_int_to_i2c_buffer(_sbc_i2c1_send_buffer, _encoder_positions[motor_index], 0);
                break;

            case REQUEST_ALL_MOTOR_CURRENT_ADDRESS:
                for (size_t i = 0; i < MAX_INT_SIZE * NUMBER_OF_MOTORS; i += MAX_INT_SIZE) {
                    _write_int_to_i2c_buffer(_sbc_i2c1_send_buffer, _motor_currents[i / MAX_INT_SIZE], i);
                }

                break;

            case REQUEST_INDIVIDUAL_MOTOR_CURRENT_FIRST_ADDRESS ...(REQUEST_INDIVIDUAL_MOTOR_CURRENT_FIRST_ADDRESS + NUMBER_OF_MOTORS - 1):
                motor_index = _sbc_i2c1_register - REQUEST_INDIVIDUAL_MOTOR_CURRENT_FIRST_ADDRESS;
                _write_int_to_i2c_buffer(_sbc_i2c1_send_buffer, _motor_currents[motor_index], 0);
                break;

            case REQUEST_ALL_MOTOR_STALL_ADDRESS:
                _sbc_i2c1_send_buffer[0] = _motor_stall_status;
                break;

            case REQUEST_ALL_MOTOR_DISCONNECT_ADDRESS:
                _sbc_i2c1_send_buffer[0] = _motor_disconnect_status;
                break;

            case REQUEST_BATTERY_VOLTAGE_ADDRESS:
                _write_int_to_i2c_buffer(_sbc_i2c1_send_buffer, _battery_voltage, 0);
                break;

            default:
                break;
        }

    } else {
        switch (_sbc_i2c1_register) {
            case SET_MOTOR_MODE_ADDRESS:
                _motor_mode = _sbc_i2c1_receive_buffer[1];

                break;

            case SET_MOTOR_STANDARD_DATA_ADDRESS:
                _motor_direction_register = _sbc_i2c1_receive_buffer[1];

                for (size_t i = 0; i < NUMBER_OF_MOTORS; i++) {
                    _motor_speeds[i] = _sbc_i2c1_receive_buffer[i + 2];
                }

                break;

            case SET_MOTOR_INDIVIDUAL_STANDARD_DATA_FIRST_ADDRESS ...(SET_MOTOR_INDIVIDUAL_STANDARD_DATA_FIRST_ADDRESS + NUMBER_OF_MOTORS - 1):
                motor_index = _sbc_i2c1_register - SET_MOTOR_INDIVIDUAL_STANDARD_DATA_FIRST_ADDRESS;
                _motor_directions[motor_index] = _sbc_i2c1_receive_buffer[1];
                _motor_speeds[motor_index] = _sbc_i2c1_receive_buffer[2];
                break;

            case SET_MOTOR_DIRECTION_ADDRESS:
                _motor_direction_register = _sbc_i2c1_receive_buffer[1];
                break;

            case SET_MOTOR_INDIVIDUAL_DIRECTION_FIRST_ADDRESS ...(SET_MOTOR_INDIVIDUAL_DIRECTION_FIRST_ADDRESS + NUMBER_OF_MOTORS - 1):
                motor_index = _sbc_i2c1_register - SET_MOTOR_INDIVIDUAL_DIRECTION_FIRST_ADDRESS;
                _motor_directions[motor_index] = _sbc_i2c1_receive_buffer[1];
                break;

            case SET_MOTOR_SPEED_ADDRESS:
                for (size_t i = 0; i < NUMBER_OF_MOTORS; i++) {
                    _motor_speeds[i] = _sbc_i2c1_receive_buffer[i + 1];
                }

                break;

            case SET_MOTOR_INDIVIDUAL_SPEED_FIRST_ADDRESS ...(SET_MOTOR_INDIVIDUAL_SPEED_FIRST_ADDRESS + NUMBER_OF_MOTORS - 1):
                motor_index = _sbc_i2c1_register - SET_MOTOR_INDIVIDUAL_SPEED_FIRST_ADDRESS;
                _motor_speeds[motor_index] = _sbc_i2c1_receive_buffer[1];
                break;

            case SET_ROBOT_DIRECTION_ADDRESS:
                _bot_direction = _sbc_i2c1_receive_buffer[1];
                break;

            case SET_ROBOT_COMMON_SPEED_ADDRESS:
                _common_speed = _sbc_i2c1_receive_buffer[1];
                break;

            default:
                break;
        }

        for (size_t i = 0; i < SBC_I2C1_RECEIVE_DATA_BUFFER_SIZE; i++) {
            _sbc_i2c1_receive_buffer[i] = 0;
        }
    }

    return ESP_OK;
}