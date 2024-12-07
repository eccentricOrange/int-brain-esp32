#include "int-brain.h"

/** @file sbc-interaction.c
 *  @brief Implement the functions to interact with the SBC, using the I2C1 and UART communication interfaces used to control the robot.
 *  @author @eccentricOrange
 */

/// @section Common data parsing

/** @brief Parse the data received from the SBC. The data is actually interpreted and acted upon here.
 *  @attention Pseudo register-based function
 *  @param data_type Type of data to parse, as defined in int-brain-sbc-registers.h.
 *  @param number_of_values Number of values to parse.
 *  @param received_values Array of values received.
 *  @param number_of_values_to_transmit Number of values to transmit.
 *  @param values_to_transmit Array of values to transmit.
 *  @return `ESP_OK` if successful.
 */
esp_err_t parse_sbc_data(uint8_t data_type, uint8_t number_of_values, int* received_values, uint8_t* number_of_values_to_transmit, int* values_to_transmit) {
    *number_of_values_to_transmit = 0;
    uint8_t motor_index;

    switch (data_type) {
        case REQUEST_ALL_ENCODER_ADDRESS:
            for (size_t i = 0; i < NUMBER_OF_MOTORS; i++) {
                values_to_transmit[i] = _encoder_positions[i];
            }
            *number_of_values_to_transmit = NUMBER_OF_MOTORS;
            break;

        case REQUEST_INDIVIDUAL_ENCODER_FIRST_ADDRESS ...(REQUEST_INDIVIDUAL_ENCODER_FIRST_ADDRESS + NUMBER_OF_MOTORS - 1):
            motor_index = data_type - REQUEST_INDIVIDUAL_ENCODER_FIRST_ADDRESS;
            values_to_transmit[0] = _encoder_positions[motor_index];
            number_of_values_to_transmit = 1;
            break;

        case REQUEST_ALL_RPM_ADDRESS:
            for (size_t i = 0; i < NUMBER_OF_MOTORS; i++) {
                values_to_transmit[i] = _motor_rpms[i];
            }
            *number_of_values_to_transmit = NUMBER_OF_MOTORS;
            break;

        case REQUEST_INDIVIDUAL_RPM_FIRST_ADDRESS ...(REQUEST_INDIVIDUAL_RPM_FIRST_ADDRESS + NUMBER_OF_MOTORS - 1):
            motor_index = data_type - REQUEST_INDIVIDUAL_RPM_FIRST_ADDRESS;
            values_to_transmit[0] = _motor_rpms[motor_index];
            number_of_values_to_transmit = 1;
            break;

        case REQUEST_ALL_MOTOR_CURRENT_ADDRESS:
            for (size_t i = 0; i < NUMBER_OF_MOTORS; i++) {
                values_to_transmit[i] = _motor_currents[i];
            }
            *number_of_values_to_transmit = NUMBER_OF_MOTORS;
            break;

        case REQUEST_INDIVIDUAL_MOTOR_CURRENT_FIRST_ADDRESS ...(REQUEST_INDIVIDUAL_MOTOR_CURRENT_FIRST_ADDRESS + NUMBER_OF_MOTORS - 1):
            motor_index = data_type - REQUEST_INDIVIDUAL_MOTOR_CURRENT_FIRST_ADDRESS;
            values_to_transmit[0] = _motor_currents[motor_index];
            *number_of_values_to_transmit = 1;
            break;

        case REQUEST_ALL_MOTOR_STALL_ADDRESS:
            values_to_transmit[0] = _motor_stall_status;
            *number_of_values_to_transmit = 1;
            break;

        case REQUEST_ALL_MOTOR_DISCONNECT_ADDRESS:
            values_to_transmit[0] = _motor_disconnect_status;
            *number_of_values_to_transmit = 1;
            break;

        case REQUEST_BATTERY_VOLTAGE_ADDRESS:
            values_to_transmit[0] = _battery_voltage;
            *number_of_values_to_transmit = 1;
            break;

        case SET_MOTOR_MODE_ADDRESS:
            _motor_mode_register = (uint8_t)received_values[0];

            _motor_safety_mode = _motor_mode_register & 0b11;
            _motor_speed_mode = (_motor_mode_register >> 2) & 0b11;
            _motor_output_enabled = (_motor_mode_register >> 4) & 0b11;

            break;

        case SET_MOTOR_STANDARD_DATA_ADDRESS:
            _motor_direction_register = (uint8_t)received_values[0];

            for (size_t i = 0; i < NUMBER_OF_MOTORS; i++) {
                _raw_motor_speeds[i] = (uint8_t)received_values[i + 1];
                _motor_directions[i] = (_motor_direction_register >> (i * 2)) & 0b11;
            }

            break;

        case SET_MOTOR_INDIVIDUAL_STANDARD_DATA_FIRST_ADDRESS ...(SET_MOTOR_INDIVIDUAL_STANDARD_DATA_FIRST_ADDRESS + NUMBER_OF_MOTORS - 1):
            motor_index = data_type - SET_MOTOR_INDIVIDUAL_STANDARD_DATA_FIRST_ADDRESS;
            _motor_directions[motor_index] = (uint8_t)received_values[0];
            _raw_motor_speeds[motor_index] = (uint8_t)received_values[1];
            break;

        case SET_MOTOR_DIRECTION_ADDRESS:
            _motor_direction_register = (uint8_t)received_values[0];

            for (size_t i = 0; i < NUMBER_OF_MOTORS; i++) {
                _motor_directions[i] = (_motor_direction_register >> (i * 2)) & 0b11;
            }
            break;

        case SET_MOTOR_INDIVIDUAL_DIRECTION_FIRST_ADDRESS ...(SET_MOTOR_INDIVIDUAL_DIRECTION_FIRST_ADDRESS + NUMBER_OF_MOTORS - 1):
            motor_index = data_type - SET_MOTOR_INDIVIDUAL_DIRECTION_FIRST_ADDRESS;
            _motor_directions[motor_index] = (uint8_t)received_values[0];
            break;

        case SET_MOTOR_SPEED_ADDRESS:
            for (size_t i = 0; i < NUMBER_OF_MOTORS; i++) {
                _raw_motor_speeds[i] = (uint8_t)received_values[i];
            }
            break;

        case SET_MOTOR_INDIVIDUAL_SPEED_FIRST_ADDRESS ...(SET_MOTOR_INDIVIDUAL_SPEED_FIRST_ADDRESS + NUMBER_OF_MOTORS - 1):
            motor_index = data_type - SET_MOTOR_INDIVIDUAL_SPEED_FIRST_ADDRESS;
            _raw_motor_speeds[motor_index] = (uint8_t)received_values[0];
            break;

        case SET_ROBOT_DIRECTION_ADDRESS:
            _bot_direction = (uint8_t)received_values[0];
            set_bot_direction(_bot_direction);
            break;

        case SET_ROBOT_COMMON_SPEED_ADDRESS:
            _common_speed = (uint8_t)received_values[0];
            break;

        default:
            break;
    }

    return ESP_OK;
}

/// @section I2C1 communication

/** @brief I2C Receive callback for the SBC.
 *  @attention Direct task function
 *  @param device_handle Handle of the I2C device.
 *  @param edata Event data of the I2C event.
 *  @param user_data User data to pass to the callback.
 *  @return `pdTRUE` if successful.
 */
IRAM_ATTR bool _sbc_i2c1_receive_callback(i2c_slave_dev_handle_t device_handle, i2c_slave_rx_done_event_data_t* edata, void* user_data) {
    return xQueueSendFromISR(user_data, edata, NULL);
}

/** @brief Register the callback for the SBC I2C1.
 *  @attention Direct task function
 *  @param device_handle Handle of the I2C device.
 *  @return `ESP_OK` if successful.
 */
esp_err_t sbc_i2c1_register_callback(i2c_slave_dev_handle_t device_handle) {
    i2c_slave_event_callbacks_t callback_config = {
        .on_recv_done = _sbc_i2c1_receive_callback,
    };
    _sbc_i2c1_receive_queue = xQueueCreate(2, sizeof(i2c_slave_rx_done_event_data_t));

    return i2c_slave_register_event_callbacks(device_handle, &callback_config, _sbc_i2c1_receive_queue);
}

/** @brief Read data from the SBC I2C1 and dump it to `sbc_i2c1_receive_buffer`.
 *  @attention Direct task function
 *  @param device_handle Handle of the I2C device.
 *  @param on_receive_callback Callback to call when data is received.
 *  @return `ESP_OK` if successful.
 */
esp_err_t sbc_i2c1_read_data(i2c_slave_dev_handle_t device_handle) {
    i2c_slave_rx_done_event_data_t rx_data;

    ESP_ERROR_CHECK(i2c_slave_receive(device_handle, &sbc_i2c1_receive_buffer, SBC_I2C1_RECEIVE_DATA_BUFFER_SIZE));

    if (xQueueReceive(_sbc_i2c1_receive_queue, &rx_data, 0) == pdPASS) {
        sbc_data_received_flag = true;
    }

    return ESP_OK;
}

/** @brief Parse the data received from the SBC I2C1. Does not actually interpret the data.
 *  @attention Direct task function
 *  @param received_data Data received from the SBC, in the form of an array of bytes.
 *  @param data_length Length of the data received.
 *  @param value_type Pointer to the type of the value received, as defined in int-brain-sbc-registers.h.
 *  @param number_of_values Number of values received.
 *  @param returned_values Array of values received.
 *  @return `ESP_OK` if successful.
 */
esp_err_t parse_i2c_frame(uint8_t* received_data, size_t data_length, uint8_t* value_type, uint8_t* number_of_values, int* returned_values) {
    *value_type = received_data[0];
    *number_of_values = SBC_I2C1_RECEIVE_MAX_DATA_LENGTH;

    for (size_t i = 1; i < data_length; i++) {
        returned_values[i] = received_data[i];
    }

    return ESP_OK;
}

/** @brief Package the data to be sent to the SBC I2C1.
 *  @attention Direct task function
 *  @param data Data to be sent.
 *  @param data_length Length of the data to be sent.
 *  @param value_type Type of the data to be sent.
 *  @param return_buffer_length Length of the buffer to return.
 *  @param return_buffer Buffer to return.
 *  @return `ESP_OK` if successful.
 */
esp_err_t package_i2c_frame(int* data, uint8_t data_length, uint8_t value_type, int* return_buffer_length, uint8_t* return_buffer) {
    for (size_t i = 0; i < data_length; i++) {
        for (size_t j = 0; j < MAX_INT_SIZE; j++) {
            return_buffer[i * MAX_INT_SIZE + j] = (data[i] >> (j * 8)) & 0xFF;
        }
    }

    *return_buffer_length = data_length * MAX_INT_SIZE;

    return ESP_OK;
}

/// @section UART communication

/// @subsection Transmission control character definitions
#define END_OF_TRANSMISSION 0x04
#define ENQUIRY 0x05
#define ACKNOWLEDGE 0x06
#define LINE_FEED '\n'
#define DEVICE_CONTROL_1 0x11
#define END_OF_TRANSMISSION_BLOCK 0x17
#define GROUP_SEPARATOR 0x1D
#define UNIT_SEPARATOR 0x1F

/**
 * @subsection UART frame structure
 * @details This is the structure of any uart transmission
 * 1. A transmission begins with either an ENQUIRY or a DEVICE_CONTROL_1 character.
 * 2. The second byte defines what the transmission is about, as defined in int-brain-sbc-registers.h.
 * 3. The third byte is the length of the data to be transmitted, in number of values. The control bytes (first three and last one) are not counted.
 * 4. The actual data is then transmitted as groups of characters, separated by a GROUP_SEPARATOR character.
 * 5. The transmission ends with an END_OF_TRANSMISSION character.
 *
 * If no data was requested by the SBC, the ESP32 will send back an ACKNOWLEDGE character. If data was requested, the ESP32 will send back the data as a whole frame.
 */

/** @brief Initialize the UART.
 *  @attention Direct task function
 *  @param uart_config Configuration of the UART.
 *  @details Initialize the UART with the given configuration.
 */
esp_err_t sbc_uart_init(uart_config_t uart_config) {
    esp_err_t status;

    status = uart_driver_install(UART_CHOSEN_PORT, UART0_RX_BUFFER_SIZE * 2, UART0_TX_BUFFER_SIZE * 2, 20, &_sbc_uart0_receive_queue, 0);
    if (status != ESP_OK) {
        return status;
    }

    status = uart_param_config(UART_CHOSEN_PORT, &uart_config);
    if (status != ESP_OK) {
        return status;
    }

    status = uart_set_pin(UART_CHOSEN_PORT, DEFAULT_UART0_TX_PIN, DEFAULT_UART0_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (status != ESP_OK) {
        return status;
    }

    status = uart_enable_pattern_det_baud_intr(UART_CHOSEN_PORT, END_OF_TRANSMISSION, 1, 9, 0, 0);
    if (status != ESP_OK) {
        return status;
    }

    status = uart_pattern_queue_reset(UART_CHOSEN_PORT, 20);
    if (status != ESP_OK) {
        return status;
    }

    return ESP_OK;
}

/** @brief UART event task.
 *  @attention Direct task function
 *  @details Task to handle the UART events.
 */
esp_err_t uart_event_task() {
    uart_event_t event;

    if (xQueueReceive(_sbc_uart0_receive_queue, (void*)&event, (TickType_t)portMAX_DELAY)) {
        switch (event.type) {
            case UART_PATTERN_DET:
                if (uart_pattern_get_pos(UART_CHOSEN_PORT) <= 0) {
                    ESP_ERROR_CHECK(uart_flush_input(UART_CHOSEN_PORT));
                    xQueueReset(_sbc_uart0_receive_queue);
                } else {
                    ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_CHOSEN_PORT, &uart_received_data_length));
                    uart_read_bytes(UART_CHOSEN_PORT, sbc_uart0_receive_buffer, uart_received_data_length, portMAX_DELAY);
                }
                sbc_data_received_flag = true;
                break;

            case UART_FIFO_OVF:
                ESP_ERROR_CHECK(uart_flush_input(UART_CHOSEN_PORT));
                xQueueReset(_sbc_uart0_receive_queue);
                break;

            case UART_BUFFER_FULL:
                ESP_ERROR_CHECK(uart_flush_input(UART_CHOSEN_PORT));
                xQueueReset(_sbc_uart0_receive_queue);
                break;

            default:
                break;
        }
    }

    return ESP_OK;
}

/** @brief Parse the data received from the SBC UART0.
 *  @attention Direct task function
 *  @param received_data Data received from the SBC, in the form of an array of characters.
 *  @param data_length Length of the data received.
 *  @param value_type Pointer to the type of the value received, as defined in int-brain-sbc-registers.h.
 *  @param number_of_values Number of values received.
 *  @param returned_values Array of values received.
 *  @return `ESP_OK` if successful.
 */
esp_err_t parse_uart_frame(char* received_data, size_t data_length, uint8_t* value_type, uint8_t* number_of_values, int* returned_values) {
    size_t index = -1;
    uint8_t values_identifier;
    uint8_t count;

    // Find the start of the transmission frame
    for (size_t i = 0; i < data_length; i++) {
        char current_char = received_data[i];
        if ((current_char == ENQUIRY) || (current_char == DEVICE_CONTROL_1)) {
            index = i;
            break;
        }
    }
    if (index == -1) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    // Identify the register
    index++;
    values_identifier = received_data[index];

    // Get the number of returned_values
    index++;
    count = received_data[index] - FIRST_SLAVE_REQUEST_ADDRESS;

    // example data: 323<UNIT_SEPARATOR>4222<UNIT_SEPARATOR>42245<UNIT_SEPARATOR>3422342<UNIT_SEPARATOR>
    index++;
    for (size_t i = 0; i < count; i++) {
        char* end;
        returned_values[i] = strtol(&received_data[index], &end, 10);
        index += end - &received_data[index] + 1;
    }

    // Make sure we've reached the end of the transmission
    if (received_data[index] != END_OF_TRANSMISSION) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    // Now that we have the data, we can write it to the relevant variables
    *value_type = values_identifier;
    *number_of_values = count;
    return ESP_OK;
}

/** @brief Package the data to be sent to the SBC UART0.
 *  @attention Direct task function
 *  @param data Data to be sent.
 *  @param data_length Length of the data to be sent.
 *  @param value_type Type of the data to be sent.
 *  @param return_buffer_length Length of the return_buffer to return.
 *  @param return_buffer Buffer to return.
 *  @return `ESP_OK` if successful.
 */
esp_err_t package_uart_frame(int* data, uint8_t data_length, uint8_t value_type, size_t* return_buffer_length, char* return_buffer) {
    size_t index = 0;

    return_buffer[index] = ENQUIRY;
    index++;

    return_buffer[index] = value_type;
    index++;

    return_buffer[index] = data_length + FIRST_SLAVE_REQUEST_ADDRESS;
    index++;

    for (size_t i = 0; i < data_length; i++) {
        index += sprintf(&return_buffer[index], "%d", data[i]);
        return_buffer[index] = UNIT_SEPARATOR;
        index++;
    }

    return_buffer[index] = END_OF_TRANSMISSION;

    *return_buffer_length = index+1;

    return ESP_OK;
}