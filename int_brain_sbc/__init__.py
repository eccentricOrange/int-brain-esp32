import serial
import smbus2

from . import enums, protocols


class IntBrain:
    def __init__(self, protocol_object: serial.Serial | smbus2.SMBus, address: int = None):
        if isinstance(protocol_object, serial.Serial):
            self.communication = protocols.uart.UARTCommunication(
                protocol_object)

        elif isinstance(protocol_object, smbus2.SMBus) and address is not None:
            self.communication = protocols.i2c.I2CCommunication(
                protocol_object, address)

        else:
            raise ValueError(
                f"Invalid protocol object {protocol_object} and/or address {address}")
        
        self.individual_motor = motor(self)
        self.all_motors = all_motors(self)
        self.bot = bot(self)
        self.imu = IMU(self)


class motor:
    def __init__(self, int_brain: IntBrain):
        self.int_brain = int_brain

    def set_speed(self, speed: int, motor_index: int):
        self.int_brain.communication.send_command(
            enums.BotQueries.SET_MOTOR_INDIVIDUAL_SPEED_FIRST_ADDRESS, speed, motor_index)

    def set_direction(self, direction: enums.MotorDirection, motor_index: int):
        self.int_brain.communication.send_command(
            enums.BotQueries.SET_MOTOR_INDIVIDUAL_DIRECTION_FIRST_ADDRESS, direction.value, motor_index)

    def set_standard_data(self, speed: int, direction: enums.MotorDirection, motor_index: int):
        self.int_brain.communication.send_command(
            enums.BotQueries.SET_MOTOR_INDIVIDUAL_STANDARD_DATA_FIRST_ADDRESS, [speed, direction.value], motor_index)

    def get_speed(self, motor_index: int):
        return self.int_brain.communication.request_data(enums.BotQueries.REQUEST_INDIVIDUAL_RPM_FIRST_ADDRESS, motor_index)[0]

    def get_encoder_data(self, motor_index: int):
        return self.int_brain.communication.request_data(enums.BotQueries.REQUEST_INDIVIDUAL_ENCODER_FIRST_ADDRESS, motor_index)[0]

    def get_motor_current(self, motor_index: int):
        return self.int_brain.communication.request_data(enums.BotQueries.REQUEST_INDIVIDUAL_MOTOR_CURRENT_FIRST_ADDRESS, motor_index)[0]


class all_motors:
    def __init__(self, int_brain: IntBrain):
        self.int_brain = int_brain

    def set_motor_mode(self, safety_mode: enums.MotorSafetyMode, speed_mode: enums.MotorSpeedMode, enable: bool):
        motor_mode = 0

        motor_mode |= safety_mode.value
        motor_mode |= speed_mode.value << 2
        motor_mode |= enable << 4

        self.int_brain.communication.send_command(
            enums.BotQueries.SET_MOTOR_MODE_ADDRESS, motor_mode)

    def set_speeds(self, speeds: list[int]):
        self.int_brain.communication.send_command(
            enums.BotQueries.SET_MOTOR_SPEED_ADDRESS, speeds)
        
    def set_common_speed(self, speed: int):
        self.int_brain.communication.send_command(
            enums.BotQueries.SET_ROBOT_COMMON_SPEED_ADDRESS, speed)

    def set_directions(self, directions: list[enums.MotorDirection]):
        self.int_brain.communication.send_command(enums.BotQueries.SET_MOTOR_DIRECTION_ADDRESS, [
                                                  direction.value for direction in directions])

    def set_standard_data(self, speeds: list[int], directions: list[enums.MotorDirection]):
        self.int_brain.communication.send_command(enums.BotQueries.SET_MOTOR_STANDARD_DATA_ADDRESS, [
                                                  speeds, [direction.value for direction in directions]])

    def get_speeds(self):
        return self.int_brain.communication.request_data(enums.BotQueries.REQUEST_ALL_RPM_ADDRESS)

    def get_encoder_data(self):
        return self.int_brain.communication.request_data(enums.BotQueries.REQUEST_ALL_ENCODER_ADDRESS)

    def get_motor_current(self):
        return self.int_brain.communication.request_data(enums.BotQueries.REQUEST_ALL_MOTOR_CURRENT_ADDRESS)


class bot:
    def __init__(self, int_brain: IntBrain):
        self.int_brain = int_brain

    def set_robot_direction(self, direction: enums.BotDirections):
        self.int_brain.communication.send_command(
            enums.BotQueries.SET_ROBOT_DIRECTION_ADDRESS, direction.value)

    def get_battery_voltage(self):
        return self.int_brain.communication.request_data(enums.BotQueries.REQUEST_BATTERY_VOLTAGE_ADDRESS)[0]

    def get_motor_stall_status(self):
        status_register = self.int_brain.communication.request_data(
            enums.BotQueries.REQUEST_ALL_MOTOR_STALL_ADDRESS)[0]
        stall_data = []

        for i in range(enums.BotQueriesLimits.NUMBER_OF_MOTORS.value):
            stall_data.append(bool(status_register & (
                1 << (i*enums.BitPositions.MOTOR_NUMBER_OF_BITS_PER_MOTOR))))

        return stall_data

    def get_motor_disconnect_status(self):
        status_register = self.int_brain.communication.request_data(
            enums.BotQueries.REQUEST_ALL_MOTOR_DISCONNECT_ADDRESS)[0]
        disconnect_data = []

        for i in range(enums.BotQueriesLimits.NUMBER_OF_MOTORS.value):
            disconnect_data.append(bool(status_register & (
                1 << (i*enums.BitPositions.MOTOR_NUMBER_OF_BITS_PER_MOTOR))))

        return disconnect_data


class IMU:
    def __init__(self, int_brain: IntBrain):
        self.int_brain = int_brain

    def get_imu_acceleration(self):
        return self.int_brain.communication.request_data(enums.BotQueries.REQUEST_IMU_ACCELERATION_ADDRESS)

    def get_imu_gyroscope(self):
        return self.int_brain.communication.request_data(enums.BotQueries.REQUEST_IMU_GYROSCOPE_ADDRESS)

    def get_imu_magnetic(self):
        return self.int_brain.communication.request_data(enums.BotQueries.REQUEST_IMU_MAGNETIC_ADDRESS)

    def get_imu_quaternion(self):
        return self.int_brain.communication.request_data(enums.BotQueries.REQUEST_IMU_QUATERNION_ADDRESS)
