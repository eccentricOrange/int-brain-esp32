import time
import smbus2

import int_brain_sbc.core
import int_brain_sbc.enums

DELAY = 0

with smbus2.SMBus(1) as bus:

    bot = int_brain_sbc.core.IntBrain(bus)

    bot.set_motor_mode(
        enable=True,
        speed=int_brain_sbc.enums.MotorSpeedMode.COMMAND_SPEED,
        safety=int_brain_sbc.enums.MotorSafetyMode.PROTECT_DISCONNECT
    )

    time.sleep(0.1)

    bot.set_motor_speeds([0, 0, 0, 0])

    time.sleep(0.1)

    bot.set_motor_directions([int_brain_sbc.enums.MotorDirection.FORWARD] * 4)

    time.sleep(0.1)

    print(bot.request_data(request_type=int_brain_sbc.enums.BotQueries.ALL_MOTOR_CURRENT))

    while 1:
        for i in range(0, 255, 5):
            bot.set_motor_speeds([i, i, i, i])
            time.sleep(DELAY)
            print(f"{i:03}: {bot.request_data(request_type=int_brain_sbc.enums.BotQueries.ALL_MOTOR_CURRENT)}")
            time.sleep(DELAY)

        for i in range(255, -1, -5):
            bot.set_motor_speeds([i, i, i, i])
            time.sleep(DELAY)
            print(f"{i:03}: {bot.request_data(request_type=int_brain_sbc.enums.BotQueries.ALL_MOTOR_CURRENT)}")
            time.sleep(DELAY)
