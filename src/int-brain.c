#include "int-brain.h"

/** @file int-brain.c
 *  @brief Initialize the state variables and local handles.
 *  @author @eccentricOrange
 */

/**
 *  @section State variables
 */
uint8_t _sbc_i2c1_receive_buffer[SBC_I2C1_RECEIVE_DATA_BUFFER_SIZE];
uint8_t _sbc_i2c1_send_buffer[SBC_I2C1_SEND_DATA_BUFFER_SIZE];
uint8_t _sbc_i2c1_register;

int _encoder_positions[NUMBER_OF_MOTORS];
int _motor_currents[NUMBER_OF_MOTORS];
uint8_t _motor_stall_status;
uint8_t _motor_disconnect_status;

uint8_t _motor_mode_register;
uint8_t _motor_direction_register;

uint8_t _raw_motor_speeds[NUMBER_OF_MOTORS];
uint8_t _filtered_motor_speeds[NUMBER_OF_MOTORS];
motor_direction_t _motor_directions[NUMBER_OF_MOTORS];
motor_safety_mode_t _motor_safety_mode;
motor_speed_mode_t _motor_speed_mode;
bool _motor_output_enabled;

bot_direction_t _bot_direction;
uint8_t _common_speed;

int _battery_voltage;

/**
 *  @section Local handles
 */
i2c_master_dev_handle_t _PCA_I2C0_device_handle;
adc_oneshot_unit_handle_t _adc1_unit_handle;
adc_cali_handle_t _adc1_cali_handle;
adc_oneshot_unit_handle_t _adc2_unit_handle;
adc_cali_handle_t _adc2_cali_handle;