#pragma once

/**
 *  @section Raspberry Pi request from ESP32
 */
#define FIRST_SLAVE_REQUEST_ADDRESS 0x00
#define LAST_SLAVE_REQUEST_ADDRESS 0x9F

// Encoder "registers"
#define REQUEST_ALL_ENCODER_ADDRESS 0x10
#define REQUEST_INDIVIDUAL_ENCODER_FIRST_ADDRESS 0x11

// Motor current sense "registers"
#define REQUEST_ALL_MOTOR_CURRENT_ADDRESS 0x20
#define REQUEST_INDIVIDUAL_MOTOR_CURRENT_FIRST_ADDRESS 0x21

// Motor stall status "registers"
#define REQUEST_ALL_MOTOR_STALL_ADDRESS 0x30

// Motor disconnect status "registers"
#define REQUEST_ALL_MOTOR_DISCONNECT_ADDRESS 0x31

// IMU "registers"
#define REQUEST_IMU_ACCELERATION_ADDRESS 0x40
#define REQUEST_IMU_GYROSCOPE_ADDRESS 0x41
#define REQUEST_IMU_MAGNETIC_ADDRESS 0x42
#define REQUEST_IMU_QUATERNION_ADDRESS 0x43

// Battery "registers"
#define REQUEST_BATTERY_VOLTAGE_ADDRESS 0x50


/**
 *  @section Raspberry Pi send command to ESP32
 */
#define FIRST_SLAVE_COMMAND_ADDRESS 0xA0
#define LAST_SLAVE_COMMAND_ADDRESS 0xFF

// Motor operation mode "registers"
#define SET_MOTOR_MODE_ADDRESS 0xA0

// Standard motor data "registers"
#define SET_MOTOR_STANDARD_DATA_ADDRESS 0xA1
#define SET_MOTOR_INDIVIDUAL_STANDARD_DATA_FIRST_ADDRESS 0xA2

// Motor direction "registers"
#define SET_MOTOR_DIRECTION_ADDRESS 0xA6
#define SET_MOTOR_INDIVIDUAL_DIRECTION_FIRST_ADDRESS 0xA7

// Motor speed "registers"
#define SET_MOTOR_SPEED_ADDRESS 0xAB
#define SET_MOTOR_INDIVIDUAL_SPEED_FIRST_ADDRESS 0xAC

// Robot "registers"
#define SET_ROBOT_DIRECTION_ADDRESS 0xB0
#define SET_ROBOT_COMMON_SPEED_ADDRESS 0xB2


/**
 *  @section Bit positions
 */

// Motor operation mode
#define MOTOR_MODE_SAFETY_BIT_0_POSITION 0
#define MOTOR_MODE_SAFETY_BIT_1_POSITION 1

#define MOTOR_MODE_SPEED_BIT_0_POSITION 2
#define MOTOR_MODE_SPEED_BIT_1_POSITION 3

#define MOTOR_MODE_ENABLE_BIT_POSITION 4

// Other motor registers
#define MOTOR_NUMBER_OF_BITS_PER_MOTOR 2