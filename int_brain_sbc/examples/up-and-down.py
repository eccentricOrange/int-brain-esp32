import time

import serial
import smbus2

import int_brain_sbc

DELAY = 1e-3

with serial.Serial("/dev/serial0", baudrate=115200, timeout=1) as port:
# with smbus2.SMBus(1) as port:

    bot = int_brain_sbc.IntBrain(port, 0x30)

    bot.all_motors.set_motor_mode(
        safety_mode=int_brain_sbc.enums.MotorSafetyMode.UNSAFE,
        speed_mode=int_brain_sbc.enums.MotorSpeedMode.COMMAND_SPEED,
        enable=True
    )
    time.sleep(DELAY)

    bot.bot.set_robot_direction(direction=int_brain_sbc.enums.BotDirections.FRONT)
    time.sleep(DELAY)

    for motor_index in range(int_brain_sbc.enums.BotQueriesLimits.NUMBER_OF_MOTORS.value):
        bot.individual_motor.set_speed(0, motor_index)
        time.sleep(DELAY)

    try:
        while(1):
            for motor_index in range(int_brain_sbc.enums.BotQueriesLimits.NUMBER_OF_MOTORS.value):
                for duty_cycle in range(int_brain_sbc.enums.BotQueriesLimits.MAX_PWM.value + 1):
                    bot.individual_motor.set_speed(duty_cycle, motor_index)
                    time.sleep(DELAY)
                    print(f"[{motor_index}][{duty_cycle:03}] Currents: {bot.all_motors.get_motor_current()}, Encoder: {bot.all_motors.get_encoder_data()}, Speeds: {bot.all_motors.get_speeds()}, Battery level: {bot.bot.get_battery_voltage()}")

                for duty_cycle in range(int_brain_sbc.enums.BotQueriesLimits.MAX_PWM.value, -1, -1):
                    bot.individual_motor.set_speed(duty_cycle, motor_index)
                    time.sleep(DELAY)
                    print(f"[{motor_index}][{duty_cycle:03}] Currents: {bot.all_motors.get_motor_current()}, Encoder: {bot.all_motors.get_encoder_data()}, Speeds: {bot.all_motors.get_speeds()}, Battery level: {bot.bot.get_battery_voltage()}")

    except KeyboardInterrupt:
        for motor_index in range(int_brain_sbc.enums.BotQueriesLimits.NUMBER_OF_MOTORS.value):
            bot.individual_motor.set_speed(0, motor_index)
            time.sleep(DELAY)