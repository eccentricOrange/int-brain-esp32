import itertools

from smbus2 import SMBus

from int_brain_sbc import enums
from int_brain_sbc.communication import Communication, DataFrame


class I2CMessage:
    """
    ## I2C frame structure
    This is the structure of any I2C transmission
    1. The first byte defines what the transmission is about, as defined in the `BotQueries` enum.
    2. The actual data is then transmitted as bytes. Since an `int` is 4 bytes long, the data is transmitted as multiples of 4 bytes.
    """

    message: bytearray

    def __init__(self, message: bytearray = None):
        self.message = message

    def encode(self, data_frame: DataFrame) -> bytearray:
        self.message = bytearray()

        for value in data_frame.values:
            self.message.extend(value.to_bytes())

        return self.message

    def decode(self, message: bytearray = None) -> DataFrame:
        message_to_decode = bytes(message if message else self.message)

        data_frame = DataFrame(enums.BotQueries(0x20))

        unparsed_values = message_to_decode

        if len(unparsed_values) % 4 != 0:
            raise ValueError(
                f"Invalid number of bytes: {len(unparsed_values)}")

        unparsed_values = itertools.zip_longest(
            *[iter(unparsed_values)] * 4, fillvalue=0)
        parsed_values = list(
            map(lambda x: int.from_bytes(x, 'little'), unparsed_values))

        data_frame.number_of_values = len(parsed_values)
        data_frame.values = parsed_values

        return data_frame

    def __eq__(self, value: object) -> bool:
        if not isinstance(value, I2CMessage):
            return False

        return self.message == value.message

    def __repr__(self) -> str:
        return f"I2CMessage({self.message.hex(' ')})"

    def __len__(self) -> int:
        return len(self.message)

    def __iter__(self):
        return iter(self.message)


class I2CCommunication(Communication):

    def __init__(self, bus: SMBus, address: int):
        self.bus = bus
        self.address = address
        self.encode = I2CMessage().encode
        self.decode = I2CMessage().decode

    def read(self, request: DataFrame) -> DataFrame:
        return self.decode(self.bus.read_i2c_block_data(self.address, request.message_type.value, 16))

    def write(self, message: DataFrame) -> None:
        self.bus.write_i2c_block_data(
            self.address, message.message_type.value, self.encode(message))
