"""Integrated Brain Core
- define few default constants
- provide the main class `IntBrain`
- handle I2C initialization and communication
- parse received data
"""

import smbus2
from . import registers
from . import enums

ESP32_SLAVE_ADDRESS = 0x30
STANDARD_SBC_RECEIVE_DATA_LENGTH = 16
STANDARD_SBC_SEND_DATA_LENGTH = 8
NUMBER_OF_MOTORS = 4


class IntBrain():
    """Main class for the Integrated Brain SBC"""

    motor_stall_status = [False] * NUMBER_OF_MOTORS
    motor_disconnect_status = [False] * NUMBER_OF_MOTORS

    def __init__(self, bus: smbus2.SMBus, address: int = ESP32_SLAVE_ADDRESS):
        self.bus = bus
        self.address = address

    def __convert_bytes_to_ints(self, data: list[int], length: int = 4 * NUMBER_OF_MOTORS) -> list[int]:
        """
        Convert a list of bytes to a list of integers
        - data: the list of bytes to convert. each integer should be 4 bytes long
        - length: the number of bytes to convert to an integer (default `4 * NUMBER_OF_MOTORS`)

        Returns a list of integers
        """

        return [
            int.from_bytes(data[i:i+4], byteorder='little', signed=True)
            for i in range(0, length, 4)
        ]

    def request_raw_data(self, register: int, length: int = STANDARD_SBC_RECEIVE_DATA_LENGTH) -> list[int]:
        """
        Request raw data from the ESP32
        - register: the pseudo register to read from
        - length: the number of bytes to read (default `STANDARD_SBC_RECEIVE_DATA_LENGTH`)
        
        Does not process the data in any way, returns an array of bytes (integers)
        """

        return self.bus.read_i2c_block_data(self.address, register, length)

    def send_raw_data(self, register: int, data: list[int]):
        """
        Send raw data to the ESP32
        - register: the pseudo register to write to
        - data: the list of bytes to write

        Pads the data with zeros to `STANDARD_SBC_SEND_DATA_LENGTH` if necessary
        """

        padded_data = data + [0] * (STANDARD_SBC_SEND_DATA_LENGTH - len(data))
        self.bus.write_i2c_block_data(self.address, register, padded_data)
    
    def request_data(self, request_type: enums.BotQueries, motor_index: int = None):
        """
        Request processed data from the ESP32
        - request_type: the type of data to request
        - motor_index: the index of the motor to request data from (default `None`)

        Returns the requested data in a processed form.
        """

        if request_type == enums.BotQueries.ALL_ENCODER_DATA:
            raw_encoder_data = self.request_raw_data(registers.REQUEST_ALL_ENCODER_ADDRESS)

            return self.__convert_bytes_to_ints(raw_encoder_data)
        
        elif request_type == enums.BotQueries.SPECIFIC_ENCODER_DATA:

            if motor_index == None:
                raise ValueError("Motor index not provided")
            
            if 0 < motor_index < (NUMBER_OF_MOTORS - 1):
                raw_encoder_data = self.request_raw_data(registers.REQUEST_INDIVIDUAL_ENCODER_FIRST_ADDRESS + motor_index)

                return self.__convert_bytes_to_ints(raw_encoder_data, 4)[0]
            
            else:
                raise ValueError("Motor index out of range")
            
        elif request_type == enums.BotQueries.ALL_MOTOR_CURRENT:
            raw_current_data = self.request_raw_data(registers.REQUEST_ALL_MOTOR_CURRENT_ADDRESS)

            return self.__convert_bytes_to_ints(raw_current_data)
        
        elif request_type == enums.BotQueries.SPECIFIC_MOTOR_CURRENT:
                
                if motor_index == None:
                    raise ValueError("Motor index not provided")
                
                if 0 < motor_index < (NUMBER_OF_MOTORS - 1):
                    raw_current_data = self.request_raw_data(registers.REQUEST_INDIVIDUAL_MOTOR_CURRENT_FIRST_ADDRESS + motor_index)

                    return self.__convert_bytes_to_ints(raw_current_data, 4)[0]
                
                else:
                    raise ValueError("Motor index out of range")
                
        elif request_type == enums.BotQueries.MOTOR_STALL_STATUS:
            raw_stall_data = self.request_raw_data(registers.REQUEST_ALL_MOTOR_STALL_ADDRESS)[0]

            for i in range(NUMBER_OF_MOTORS):
                self.motor_stall_status[i] = bool(raw_stall_data & (1 << i))

            return self.motor_stall_status
        
        elif request_type == enums.BotQueries.MOTOR_DISCONNECT_STATUS:
            raw_disconnect_data = self.request_raw_data(registers.REQUEST_ALL_MOTOR_DISCONNECT_ADDRESS)[0]

            for i in range(NUMBER_OF_MOTORS):
                self.motor_disconnect_status[i] = bool(raw_disconnect_data & (1 << i))

            return self.motor_disconnect_status
        
        elif request_type == enums.BotQueries.BATTERY_VOLTAGE:
            raw_voltage_data = self.request_raw_data(registers.REQUEST_BATTERY_VOLTAGE_ADDRESS)

            return self.__convert_bytes_to_ints(raw_voltage_data, 4)[0]
        
        else:
            raise ValueError("Invalid request type")
        
    def set_motor_mode(
            self,
            safety: enums.MotorSafetyMode = enums.MotorSafetyMode.UNSAFE,
            speed: enums.MotorSpeedMode = enums.MotorSpeedMode.COMMAND_SPEED,
            enable: bool = True
    ):
        motor_mode = 0

        motor_mode |= safety.value
        motor_mode |= speed.value << 2
        motor_mode |= enable << 4

        self.send_raw_data(registers.SET_MOTOR_MODE_ADDRESS, [motor_mode])

    def set_motor_speeds(self, speeds: list[int]):
        if len(speeds) != NUMBER_OF_MOTORS:
            raise ValueError(f"Invalid number of motor speeds {len(speeds)}")
        
        for speed in speeds:
            if speed < 0 or speed > 255:
                raise ValueError(f"Invalid motor speed value {speed}")
            
        self.send_raw_data(registers.SET_MOTOR_SPEED_ADDRESS, speeds)

    def set_individual_motor_speed(self, motor_index: int, speed: int):
        if 0 < motor_index < (NUMBER_OF_MOTORS - 1):
            if speed < 0 or speed > 255:
                raise ValueError(f"Invalid motor speed value {speed}")
            
            self.send_raw_data(registers.SET_MOTOR_INDIVIDUAL_SPEED_FIRST_ADDRESS + motor_index, [speed])
        
        else:
            raise ValueError("Motor index out of range")
        
    def set_motor_directions(self, directions: list[enums.MotorDirection]):
        if len(directions) != NUMBER_OF_MOTORS:
            raise ValueError(f"Invalid number of motor directions {len(directions)}")
        
        motor_directions = 0

        for i, direction in enumerate(directions):
            motor_directions |= direction.value << (i * 2)
        
        self.send_raw_data(registers.SET_MOTOR_DIRECTION_ADDRESS, [motor_directions])

    def set_individual_motor_direction(self, motor_index: int, direction: enums.MotorDirection):
        if 0 < motor_index < (NUMBER_OF_MOTORS - 1):
            self.bus.write_byte_data(self.address, registers.SET_MOTOR_INDIVIDUAL_DIRECTION_FIRST_ADDRESS + motor_index, direction.value)
        
        else:
            raise ValueError("Motor index out of range")
        
    def send_standard_motor_data(self, directions: list[enums.MotorDirection], speeds: list[int]):
        motor_directions = 0

        for i, direction in enumerate(directions):
            motor_directions |= direction.value << (i * 2)

        self.send_raw_data(registers.SET_MOTOR_STANDARD_DATA_ADDRESS, [motor_directions] + speeds)

    def send_individual_standard_motor_data(self, motor_index: int, direction: enums.MotorDirection, speed: int):
        if 0 < motor_index < (NUMBER_OF_MOTORS - 1):
            self.bus.write_i2c_block_data(self.address, registers.SET_MOTOR_INDIVIDUAL_STANDARD_DATA_FIRST_ADDRESS + motor_index, [direction.value, speed])
        
        else:
            raise ValueError("Motor index out of range")
        
    def set_robot_direction(self, direction: enums.BotDirections):
        self.send_raw_data(self.address, registers.SET_ROBOT_DIRECTION_ADDRESS, [direction.value])

    def set_common_speed(self, speed: int):
        if speed < 0 or speed > 255:
            raise ValueError(f"Invalid common speed value {speed}")
        
        self.send_raw_data(registers.SET_ROBOT_COMMON_SPEED_ADDRESS, [speed])