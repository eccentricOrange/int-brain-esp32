#pragma once

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/i2c_slave.h"
#include "driver/pulse_cnt.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_task.h"
#include "freertos/queue.h"

/** @headerfile Integrated Brain Header
 *  @brief This header file contains all the necessary definitions and function prototypes to include in the main program.
 */

#include "int-brain-sbc-registers.h"

/**
 *  @section General constants
 */
#define NUMBER_OF_MOTORS 4
// Motor definitions:
// Motor 1: Front left
// Motor 2: Front right
// Motor 3: Back left
// Motor 4: Back right

extern const uint8_t PCA_DRIVER_ADDRESS;

#define ENCODER_LIMIT 1024         // pulses
#define ENCODER_GLITCH_PERIOD 1e3  // ns

#define ADC_CHOSEN_ATTEN ADC_ATTEN_DB_0
#define ADC_CHHOSEN_BITWIDTH ADC_BITWIDTH_DEFAULT
#define ADC_CHOSEN_UNIT ADC_UNIT_1

#define MOTOR_MAXIMUM_STALL_CURRENT 750
#define MOTOR_MINIMUM_DISCONNECT_PWM 50
#define MOTOR_MINIMUM_CONNECTED_CURRENT 100
#define MOTOR_STOPPED_CURRENT 75

#define I2C_STANDARD_TIMEOUT_MS 3e3

/**
 *  @section Types
 */
enum motor_direction_t {
    IDLE = 0b00,
    FORWARD = 0b01,
    REVERSE = 0b10,
    BRAKE = 0b11,
} typedef motor_direction_t;

enum motor_safety_mode_t {
    UNSAFE = 0b00,
    PROTECT_STALL = 0b01,
    PROTECT_DISCONNECT = 0b10,
    PROTECT_STALL_AND_DISCONNECT = 0b11,
} typedef motor_safety_mode_t;

enum motor_speed_mode_t {
    STOP = 0b00,
    MAX = 0b01,
    COMMAND = 0b10,
    COMMON = 0b11,
} typedef motor_t;

enum bot_direction_t {
    FRONT = 0,
    BACK = 1,
    ROTATE_CLOCKWISE = 2,
    ROTATE_COUNTERCLOCKWISE = 3,
    FRONT_LEFT = 4,
    FRONT_RIGHT = 5,
    BACK_LEFT = 6,
    BACK_RIGHT = 7,
} typedef bot_direction_t;

/**
 *  @section Pin-outs
 */

static motor_t DEFAULT_MOTORS_PIN_OUTS[NUMBER_OF_MOTORS] = {
    {
        .LED_pin = 11,
        .output_pin_1 = 12,
        .output_pin_2 = 13,
        .encoder_pin_1 = 18,
        .encoder_pin_2 = 19,
        .current_sense_pin = 35,
    },
    {
        .LED_pin = 10,
        .output_pin_1 = 8,
        .output_pin_2 = 9,
        .encoder_pin_1 = 23,
        .encoder_pin_2 = 25,
        .current_sense_pin = 34,
    },
    {
        .LED_pin = 2,
        .output_pin_1 = 3,
        .output_pin_2 = 4,
        .encoder_pin_1 = 5,
        .encoder_pin_2 = 17,
        .current_sense_pin = 39,
    },
    {
        .LED_pin = 7,
        .output_pin_1 = 5,
        .output_pin_2 = 6,
        .encoder_pin_1 = 14,
        .encoder_pin_2 = 12,
        .current_sense_pin = 36,
    }};

// PCA enable pin
#define PCA_MOTOR_ENABLE_PIN 4

// Built-in LED pin
#define DEFAULT_LED_PIN 2

// I2C pins
#define I2C0_MASTER_SCL_PIN 22
#define I2C0_MASTER_SDA_PIN 21
#define I2C1_SLAVE_SCL_PIN 15
#define I2C1_SLAVE_SDA_PIN 13

// Battery voltage sense pin
#define BATTERY_VOLTAGE_SENSE_PIN 25

/**
 *  @section I2C constants
 */
#define ESP32_I2C1_SLAVE_ADDRESS 0x30
#define PCA_DRIVER_I2C0_DEFAULT_ADDRESS 0x40
#define IMU_MPU9250_I2C0_DEFAULT_ADDRESS 0x68

#define PCA_I2C0_SPEED 100e3       // Hz
#define PCA_I2C0_WRITE_TIMEOUT 50  // ms

#define MAX_INT_SIZE sizeof(int)
#define SBC_I2C1_RECEIVE_MAX_DATA_LENGTH 8
#define SBC_I2C1_RECEIVE_DATA_BUFFER_SIZE (SBC_I2C1_RECEIVE_MAX_DATA_LENGTH + 1)  // +1 for the command byte
#define SBC_I2C1_SEND_MAX_DATA_LENGTH (NUMBER_OF_MOTORS * MAX_INT_SIZE)
#define SBC_I2C1_SEND_DATA_BUFFER_SIZE (SBC_I2C1_SEND_MAX_DATA_LENGTH)  // +1 for the command byte

static const i2c_master_bus_config_t DEFAULT_I2C0_MASTER_CONFIG = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_NUM_0,
    .sda_io_num = I2C0_MASTER_SDA_PIN,
    .scl_io_num = I2C0_MASTER_SCL_PIN,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = false,  // We have external pull-ups
};

static const i2c_slave_config_t DEFAULT_I2C1_SLAVE_CONFIG = {
    .addr_bit_len = I2C_ADDR_BIT_7,
    .clk_source = I2S_CLK_SRC_DEFAULT,
    .i2c_port = I2C_NUM_1,
    .send_buf_depth = 256,
    .sda_io_num = I2C1_SLAVE_SDA_PIN,
    .scl_io_num = I2C1_SLAVE_SCL_PIN,
    .slave_addr = ESP32_I2C1_SLAVE_ADDRESS,
};

/**
 *  @section PCA Driver Registers
 */
// PCA registers

#define PCA_MODE1_ADDRESS 0x00
#define PCA_MODE1 0b10000001
// (MSB)
// 1: Register Auto-Increment enabled.
// 0: Register Auto-Increment bit 1 = 0
// 0: Register Auto-Increment bit 0 = 0
// 0: Normal mode
// 0: does not respond to I2C-bus subaddress 1
// 0: does not respond to I2C-bus subaddress 2
// 0: does not respond to I2C-bus subaddress 3
// 1: responds to LED All Call I2C-bus address
// (LSB)

#define PCA_LED_DRIVER_ADDRESS 0x14
#define PCA_ENABLE_DRIVERS 0b10101010
// 10: LED driver x individual brightness can be controlled through its PWMx register
// Each pair of bits corresponds to one driver

#define PWM0_REGISTER_ADDRESS 0x02

// Standard constants
#define PCA_NUMBER_OF_LED_DRIVERS 4
#define PCA_MAX_PWM_DUTY 255
#define PCA_MAX_LED_PWM 155

/**
 *  @section State variables
 */
extern uint8_t _sbc_i2c1_receive_buffer[SBC_I2C1_RECEIVE_DATA_BUFFER_SIZE];
extern uint8_t _sbc_i2c1_send_buffer[SBC_I2C1_SEND_DATA_BUFFER_SIZE];
extern uint8_t _sbc_i2c1_register;

extern int _encoder_positions[NUMBER_OF_MOTORS];
extern int _motor_currents[NUMBER_OF_MOTORS];
extern uint8_t _motor_stall_status;
extern uint8_t _motor_disconnect_status;

extern uint8_t _motor_mode;

extern uint8_t _motor_speeds[NUMBER_OF_MOTORS];
extern uint8_t _motor_direction_register;
extern motor_direction_t _motor_directions[NUMBER_OF_MOTORS];

extern bot_direction_t _bot_direction;
extern uint8_t _common_speed;

extern int _battery_voltage;

/**
 *  @section Local handles
 */
extern i2c_master_dev_handle_t _PCA_I2C0_device_handle;
extern adc_oneshot_unit_handle_t _adc_motors_handle;
extern adc_cali_handle_t _adc_motors_cali_handle;

/**
 *  @section PCA Function prototypes (motor command)
 */
extern esp_err_t _PCA_set_register(uint8_t register_address, uint8_t data);

extern esp_err_t _PCA_i2c_init(i2c_master_bus_handle_t bus_handle);
extern esp_err_t _PCA_enable_drivers();
extern esp_err_t _PCA_MOE_init();
extern esp_err_t PCA_enable();
extern esp_err_t PCA_disable();
extern esp_err_t PCA_set_moe_by_register();

extern esp_err_t PCA_init(i2c_master_bus_handle_t bus_handle);

extern void direction_to_PWMs(motor_direction_t direction, uint8_t PWM_in, uint8_t* PWM_1_out, uint8_t* PWM_2_out);
extern uint8_t motor_PWM_mode_filter(uint8_t command_PWM, motor_speed_mode_t speed_mode);

extern esp_err_t PCA_command_LED(motor_t motor, uint8_t PWM);
extern esp_err_t PCA_command_motor_with_LED(motor_t motor, motor_direction_t direction, uint8_t PWM);
extern esp_err_t PCA_command_motor(motor_t motor, motor_direction_t direction, uint8_t PWM);

/**
 *  @section ADC Function prototypes (motor current sense)
 */
extern esp_err_t ADC_motors_init(motor_t* motors, size_t number_of_motors);
extern esp_err_t _ADC_channels_config(motor_t* motors, size_t number_of_motors);
extern int ADC_read_motor_current(motor_t motor);

/**
 *  @section PCNT Function prototypes (motor encoder)
 */
extern esp_err_t _encoder_init(motor_t motor, pcnt_unit_handle_t* pcnt_unit_handle);
extern esp_err_t encoder_all_init(motor_t* motors, size_t number_of_motors);
extern int encoder_read(motor_t motor);

/**
 *  @section Four-wheel bot Function prototypes
 */
extern esp_err_t command_bot(const motor_t motors[NUMBER_OF_MOTORS], bot_direction_t bot_direction, uint8_t PWM[NUMBER_OF_MOTORS]);
extern esp_err_t command_bot_same_speed(const motor_t motors[NUMBER_OF_MOTORS], bot_direction_t bot_direction, uint8_t PWM);
extern esp_err_t command_bot_array(const motor_t motors[NUMBER_OF_MOTORS], uint8_t* data);
extern esp_err_t command_bot_registers_safe(const motor_t motors[NUMBER_OF_MOTORS]);
extern esp_err_t command_all_motors_safe(
    const motor_t motors[NUMBER_OF_MOTORS],
    motor_safety_mode_t safety_mode,
    motor_speed_mode_t speed_mode,
    uint8_t* motor_speeds,
    motor_direction_t* motor_directions
);

/**
 *  @section SBC I2C Function prototypes
 */
static QueueHandle_t _sbc_i2c1_receive_queue;

extern IRAM_ATTR bool _sbc_i2c1_receive_callback(i2c_slave_dev_handle_t device_handle, i2c_slave_rx_done_event_data_t *edata, void* user_data);
extern esp_err_t sbc_i2c1_register_callback(i2c_slave_dev_handle_t device_handle);
extern esp_err_t sbc_i2c1_read_data(i2c_slave_dev_handle_t device_handle, esp_err_t* on_receive_callback());
extern esp_err_t sbc_i2c_parse_data();

extern esp_err_t update_encoder_data(motor_t* motors);
extern esp_err_t update_motor_current_data(motor_t* motors);
extern esp_err_t update_motor_disconnect_status();
extern esp_err_t update_motor_directions_from_register();