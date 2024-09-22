from int_brain_sbc import enums


class DataFrame:
    message_type: enums.BotQueries
    number_of_values: int
    values: list[int]

    def __init__(self, message_type: enums.BotQueries, number_of_values: int = None, values: list[int] = None):
        self.message_type = message_type
        self.number_of_values = number_of_values
        self.values = values

    def __repr__(self) -> str:
        return f"DataFrame({self.message_type=}, {self.number_of_values=}, {self.values=})"

    # raise an error if the number of values is not equal to the length of the values
    def __setattr__(self, name, value):
        if name == "values" and hasattr(self, "number_of_values") and value is not None and len(value) != self.number_of_values:
            raise ValueError(
                f"Expected {self.number_of_values} values, got {len(value)}")

        super().__setattr__(name, value)

    def __eq__(self, value: object) -> bool:
        if not isinstance(value, DataFrame):
            return False

        return self.message_type == value.message_type and self.number_of_values == value.number_of_values and self.values == value.values

    def __len__(self) -> int:
        return self.number_of_values

    def __iter__(self):
        return iter(self.values)


class Communication:
    def encode(self, data_frame: DataFrame) -> bytearray:
        ...

    def decode(self, message: bytearray) -> DataFrame:
        ...

    def read(self, request: DataFrame) -> DataFrame:
        ...

    def write(self, message: DataFrame) -> None:
        ...

    def request_data(self, request_type: enums.BotQueries, motor_index: int = None) -> list[int]:
        if request_type in (
            enums.BotQueries.REQUEST_INDIVIDUAL_ENCODER_FIRST_ADDRESS,
            enums.BotQueries.REQUEST_INDIVIDUAL_RPM_FIRST_ADDRESS,
            enums.BotQueries.REQUEST_INDIVIDUAL_MOTOR_CURRENT_FIRST_ADDRESS
        ):
            if motor_index is None or motor_index > enums.BotQueriesLimits.NUMBER_OF_MOTORS.value:
                raise ValueError(
                    f"Motor index must be provided for {request_type}")

            request_type = enums.BotQueries(motor_index + request_type.value)

        response = self.read(DataFrame(request_type, number_of_values=0))
        return response.values

    def send_command(self, command_type: enums.BotQueries, values: list[int] | int, motor_index: int = None) -> None:
        if command_type in (
            enums.BotQueries.SET_MOTOR_INDIVIDUAL_STANDARD_DATA_FIRST_ADDRESS,
            enums.BotQueries.SET_MOTOR_INDIVIDUAL_DIRECTION_FIRST_ADDRESS,
            enums.BotQueries.SET_MOTOR_INDIVIDUAL_SPEED_FIRST_ADDRESS
        ):
            if motor_index is None or motor_index > enums.BotQueriesLimits.NUMBER_OF_MOTORS.value:
                raise ValueError(
                    f"Motor index must be provided for {command_type}")

            command_type = enums.BotQueries(motor_index + command_type.value)

        array = values if isinstance(values, list) else [values]

        self.write(
            DataFrame(command_type, number_of_values=len(array), values=array))
