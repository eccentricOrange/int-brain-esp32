import time
import smbus2

import int_brain_sbc.core
import int_brain_sbc.enums

DELAY = 0.01
STEP = 5

def write_common_speed_and_print_current(bot: int_brain_sbc.core.IntBrain, i: int):
    bot.set_common_speed(i)
    time.sleep(DELAY)
    print(f"{i:03}: {bot.request_data(request_type=int_brain_sbc.enums.BotQueries.ALL_MOTOR_CURRENT)}")
    time.sleep(DELAY)

def initialize_motor_system(bot: int_brain_sbc.core.IntBrain):
    bot.set_motor_mode(
            enable=True,
            speed=int_brain_sbc.enums.MotorSpeedMode.COMMON_SPEED,
            safety=int_brain_sbc.enums.MotorSafetyMode.PROTECT_DISCONNECT
        )

    bot.set_common_speed(0)
    bot.set_motor_directions([int_brain_sbc.enums.MotorDirection.FORWARD] * 4)

def main():
    
    with smbus2.SMBus(1) as bus:

        bot = int_brain_sbc.core.IntBrain(bus)
        initialize_motor_system(bot)

        while 1:
            for i in range(0, 256, STEP):
                try:
                    write_common_speed_and_print_current(bot, i)
                except OSError:
                    bot = re_open_bot(bot, bus)

            for i in range(255, -1, -STEP):
                try:
                    write_common_speed_and_print_current(bot, i)
                except OSError:
                    bot = re_open_bot(bot, bus)

def re_open_bot(bot: int_brain_sbc.core.IntBrain, bus: smbus2.SMBus):
    while True:
        try:
            bot = int_brain_sbc.core.IntBrain(bus)
            initialize_motor_system(bot)
            return bot
        except OSError:
            time.sleep(1)


if __name__ == "__main__":
    main()