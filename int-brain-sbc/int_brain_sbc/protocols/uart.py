from enum import Enum

from serial import Serial

from int_brain_sbc import enums
from int_brain_sbc.communication import Communication, DataFrame

ENCODING_TYPE = 'ascii'
TEST_DATA = b'\x11\xb2\x2410\x1F10\x1F4320\x1F433\x1F\x04'


class UARTControlCharacters(Enum):
    END_OF_TRANSMISSION = 0x04
    ENQUIRY = 0x05
    ACKNOWLEDGE = 0x06
    LINE_FEED = 0x0A
    DEVICE_CONTROL_1 = 0x11
    END_OF_TRANSMISSION_BLOCK = 0x17
    GROUP_SEPARATOR = 0x1D
    UNIT_SEPARATOR = 0x1F


def get_nature(message_type: enums.BotQueries) -> UARTControlCharacters:
    if enums.BotQueriesLimits.FIRST_SLAVE_COMMAND_ADDRESS.value <= message_type.value <= enums.BotQueriesLimits.LAST_SLAVE_COMMAND_ADDRESS.value:
        return UARTControlCharacters.DEVICE_CONTROL_1

    elif enums.BotQueriesLimits.FIRST_SLAVE_REQUEST_ADDRESS.value <= message_type.value <= enums.BotQueriesLimits.LAST_SLAVE_REQUEST_ADDRESS.value:
        return UARTControlCharacters.ENQUIRY

    raise ValueError(f"Invalid message type: {message_type}")


class UARTMessage:
    """
    ## UART frame structure
    This is the structure of any UART transmission
    1. A transmission begins with either an `ENQUIRY` or a `DEVICE_CONTROL_1` character.
    2. The second byte defines what the transmission is about, as defined in the `BotQueries` enum.
    3. The third byte is the length of the data to be transmitted, in number of values. The control bytes (first three and last one) are not counted.
    4. The actual data is then transmitted as groups of characters, separated by a `UNIT_SEPARATOR` character.
    5. The transmission ends with an `END_OF_TRANSMISSION` character.

    If no data was requested by the SBC, the ESP32 will send back an `ACKNOWLEDGE` character. If data was requested, the ESP32 will send back the data as a whole frame.
    """

    message: bytearray

    def __init__(self, message: bytearray = None):
        self.message = message

    def encode(self, data_frame: DataFrame) -> bytearray:
        self.message = bytearray()

        self.message.append(get_nature(data_frame.message_type).value)
        self.message.append(data_frame.message_type.value)
        self.message.append(data_frame.number_of_values +
                            enums.BotQueriesLimits.FIRST_SLAVE_REQUEST_ADDRESS.value)

        if data_frame.number_of_values != 0:
            for value in data_frame.values:
                self.message.extend(str(value).encode(ENCODING_TYPE))
                self.message.append(UARTControlCharacters.UNIT_SEPARATOR.value)

        self.message.append(UARTControlCharacters.END_OF_TRANSMISSION.value)

        return self.message

    def decode(self, message: bytearray = None) -> DataFrame:
        message_to_decode = bytes(message if message else self.message)

        data_frame = DataFrame(enums.BotQueries(message_to_decode[1]))
        data_frame.number_of_values = message_to_decode[2] - \
            enums.BotQueriesLimits.FIRST_SLAVE_REQUEST_ADDRESS.value

        unparsed_values = message_to_decode[3:]
        unparsed_values = unparsed_values.split(
            UARTControlCharacters.UNIT_SEPARATOR.value.to_bytes())
        unparsed_values = list(
            map(lambda x: x.decode(ENCODING_TYPE), unparsed_values))

        # Remove the last element if it is the end of transmission character. Raise an error if it is not.
        if ord(unparsed_values[-1]) == UARTControlCharacters.END_OF_TRANSMISSION.value:
            unparsed_values.pop()

        else:
            raise ValueError(f"No terminating character: {unparsed_values}")

        # Convert the values to integers. Raise an error if any value cannot be converted.
        try:
            parsed_values = list(map(int, unparsed_values))

        except ValueError as e:
            raise ValueError(
                f"Cannot convert some value to int: {unparsed_values}") from e

        data_frame.values = parsed_values

        return data_frame

    def __eq__(self, value: object) -> bool:
        if not isinstance(value, UARTMessage):
            return False

        return self.message == value.message

    def __repr__(self) -> str:
        return f"UARTMessage({self.message.hex(' ')})"

    def __len__(self) -> int:
        return len(self.message)

    def __iter__(self):
        return iter(self.message)


class UARTCommunication(Communication):

    def __init__(self, port: Serial):
        self.port = port
        self.encode = UARTMessage().encode
        self.decode = UARTMessage().decode

    def read(self, request: DataFrame) -> DataFrame:
        self.port.write(self.encode(request))
        return self.decode(self.port.read_until(expected=UARTControlCharacters.END_OF_TRANSMISSION.value.to_bytes()))

    def write(self, message: DataFrame) -> None:
        self.port.write(self.encode(message))
