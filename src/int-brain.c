#include "int-brain.h"

/** @file int-brain.c
 * @brief Declare and initialize variables to 0.
 */

/**
 *  @section State variables
 */
uint8_t _sbc_i2c1_receive_buffer[SBC_I2C1_RECEIVE_DATA_BUFFER_SIZE] = {0};
uint8_t _sbc_i2c1_send_buffer[SBC_I2C1_SEND_DATA_BUFFER_SIZE] = {0};
uint8_t _sbc_i2c1_register = 0;

int _encoder_positions[NUMBER_OF_MOTORS] = {0};
int _motor_currents[NUMBER_OF_MOTORS] = {0};
uint8_t _motor_stall_status = 0;
uint8_t _motor_disconnect_status = 0;

uint8_t _motor_mode = 0;

uint8_t _motor_speeds[NUMBER_OF_MOTORS] = {0};
uint8_t _motor_direction_register = 0;
motor_direction_t _motor_directions[NUMBER_OF_MOTORS];

bot_direction_t _bot_direction;
uint8_t _common_speed = 0;

int _battery_voltage = 0;

/**
 *  @section Local handles
 */
i2c_master_dev_handle_t _PCA_I2C0_device_handle;
adc_oneshot_unit_handle_t _adc_motors_handle;
adc_cali_handle_t _adc_motors_cali_handle;
