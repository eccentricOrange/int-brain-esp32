from enum import Enum


class MotorSafetyMode(Enum):
    UNSAFE = 0b00
    PROTECT_STALL = 0b01
    PROTECT_DISCONNECT = 0b10
    PROTECT_STALL_AND_DISCONNECT = 0b11


class MotorSpeedMode(Enum):

    STOP = 0b00
    """
    Stops the bot immediately by disabling the PCA driver.
    """

    MAX = 0b01
    """
    Sets the speed of the bot to the maximum value (usually 255).
    """

    COMMAND_SPEED = 0b10
    """
    Allows you to set speeds for individual motors separately.
    """

    COMMON_SPEED = 0b11
    """
    Allows you to set a common speed for all motors.
    """


class MotorDirection(Enum):
    IDLE = 0b00
    FORWARD = 0b01
    BACKWARD = 0b10
    BRAKE = 0b11


class BotQueries(Enum):
    """
    All the types of data that can be sent or received from the bot.
    - Each transmission may either be a command to the robot or a request from the robot.
    - Each transmission may be a request for all the motors or for a specific motor.
    - The value of each enum is the code used to identify the type of transmission.
    """

    """Raspberry Pi request from ESP32"""

    # Encoder "registers"
    REQUEST_ALL_ENCODER_ADDRESS = 0x20
    REQUEST_INDIVIDUAL_ENCODER_FIRST_ADDRESS = 0x21
    REQUEST_INDIVIDUAL_ENCODER_SECOND_ADDRESS = 0x22
    REQUEST_INDIVIDUAL_ENCODER_THIRD_ADDRESS = 0x23
    REQUEST_INDIVIDUAL_ENCODER_FOURTH_ADDRESS = 0x24

    # RPM "registers"
    REQUEST_ALL_RPM_ADDRESS = 0x25
    REQUEST_INDIVIDUAL_RPM_FIRST_ADDRESS = 0x26
    REQUEST_INDIVIDUAL_RPM_SECOND_ADDRESS = 0x27
    REQUEST_INDIVIDUAL_RPM_THIRD_ADDRESS = 0x28
    REQUEST_INDIVIDUAL_RPM_FOURTH_ADDRESS = 0x29

    # Motor current sense "registers"
    REQUEST_ALL_MOTOR_CURRENT_ADDRESS = 0x30
    REQUEST_INDIVIDUAL_MOTOR_CURRENT_FIRST_ADDRESS = 0x31
    REQUEST_INDIVIDUAL_MOTOR_CURRENT_SECOND_ADDRESS = 0x32
    REQUEST_INDIVIDUAL_MOTOR_CURRENT_THIRD_ADDRESS = 0x33
    REQUEST_INDIVIDUAL_MOTOR_CURRENT_FOURTH_ADDRESS = 0x34

    # Motor stall status "registers"
    REQUEST_ALL_MOTOR_STALL_ADDRESS = 0x40

    # Motor disconnect status "registers"
    REQUEST_ALL_MOTOR_DISCONNECT_ADDRESS = 0x41

    # IMU "registers"
    REQUEST_IMU_ACCELERATION_ADDRESS = 0x50
    REQUEST_IMU_GYROSCOPE_ADDRESS = 0x51
    REQUEST_IMU_MAGNETIC_ADDRESS = 0x52
    REQUEST_IMU_QUATERNION_ADDRESS = 0x53

    # Battery "registers"
    REQUEST_BATTERY_VOLTAGE_ADDRESS = 0x60

    """Raspberry Pi send command to ESP32"""

    # Motor operation mode "registers"
    SET_MOTOR_MODE_ADDRESS = 0xA0

    # Standard motor data "registers"
    SET_MOTOR_STANDARD_DATA_ADDRESS = 0xA1
    SET_MOTOR_INDIVIDUAL_STANDARD_DATA_FIRST_ADDRESS = 0xA2
    SET_MOTOR_INDIVIDUAL_STANDARD_DATA_SECOND_ADDRESS = 0xA3
    SET_MOTOR_INDIVIDUAL_STANDARD_DATA_THIRD_ADDRESS = 0xA4
    SET_MOTOR_INDIVIDUAL_STANDARD_DATA_FOURTH_ADDRESS = 0xA5

    # Motor direction "registers"
    SET_MOTOR_DIRECTION_ADDRESS = 0xA6
    SET_MOTOR_INDIVIDUAL_DIRECTION_FIRST_ADDRESS = 0xA7
    SET_MOTOR_INDIVIDUAL_DIRECTION_SECOND_ADDRESS = 0xA8
    SET_MOTOR_INDIVIDUAL_DIRECTION_THIRD_ADDRESS = 0xA9
    SET_MOTOR_INDIVIDUAL_DIRECTION_FOURTH_ADDRESS = 0xAA

    # Motor speed "registers"
    SET_MOTOR_SPEED_ADDRESS = 0xAB
    SET_MOTOR_INDIVIDUAL_SPEED_FIRST_ADDRESS = 0xAC
    SET_MOTOR_INDIVIDUAL_SPEED_SECOND_ADDRESS = 0xAD
    SET_MOTOR_INDIVIDUAL_SPEED_THIRD_ADDRESS = 0xAE
    SET_MOTOR_INDIVIDUAL_SPEED_FOURTH_ADDRESS = 0xAF

    # Robot "registers"
    SET_ROBOT_DIRECTION_ADDRESS = 0xB0
    SET_ROBOT_COMMON_SPEED_ADDRESS = 0xB2


class BotQueriesLimits(Enum):
    FIRST_SLAVE_REQUEST_ADDRESS = 0x20
    LAST_SLAVE_REQUEST_ADDRESS = 0x9F
    FIRST_SLAVE_COMMAND_ADDRESS = 0xA0
    LAST_SLAVE_COMMAND_ADDRESS = 0xFF

    NUMBER_OF_MOTORS = 4
    MAX_PWM = 255


class BotDirections(Enum):
    FRONT = 0
    BACK = 1
    ROTATE_CLOCKWISE = 2
    ROTATE_COUNTERCLOCKWISE = 3
    FRONT_LEFT = 4
    FRONT_RIGHT = 5
    BACK_LEFT = 6
    BACK_RIGHT = 7


class BitPositions(Enum):
    """Bit positions"""

    # Motor operation mode
    MOTOR_MODE_SAFETY_BIT_0_POSITION = 0
    MOTOR_MODE_SAFETY_BIT_1_POSITION = 1

    MOTOR_MODE_SPEED_BIT_0_POSITION = 2
    MOTOR_MODE_SPEED_BIT_1_POSITION = 3

    MOTOR_MODE_ENABLE_BIT_POSITION = 4

    # Other motor registers
    MOTOR_NUMBER_OF_BITS_PER_MOTOR = 2