import int_brain.core
import smbus2

import time

import int_brain.enums

with smbus2.SMBus(1) as bus:

    bot = int_brain.core.IntBrain(bus)

    bot.set_motor_mode(
        enable=True,
        speed=int_brain.enums.MotorSpeedMode.COMMAND_SPEED,
        safety=int_brain.enums.MotorSafetyMode.UNSAFE
    )

    time.sleep(0.1)

    bot.set_motor_speeds([0, 0, 0, 0])

    time.sleep(0.1)

    bot.set_motor_directions([int_brain.enums.MotorDirection.FORWARD] * 4)

    time.sleep(0.1)

    print(bot.request_data(request_type=int_brain.enums.BotQueries.ALL_MOTOR_CURRENT))

    while 1:
        for i in range(0, 255, 5):
            bot.set_motor_speeds([i, i, i, i])
            time.sleep(0.1)
            print(bot.request_data(request_type=int_brain.enums.BotQueries.ALL_MOTOR_CURRENT))
            time.sleep(0.1)

        for i in range(255, 0, -5):
            bot.set_motor_speeds([i, i, i, i])
            time.sleep(0.1)
            print(bot.request_data(request_type=int_brain.enums.BotQueries.ALL_MOTOR_CURRENT))
            time.sleep(0.1)
