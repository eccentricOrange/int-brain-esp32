# Integrated Brain
The Integrated Brain is a custom PCB designed to control, supply power to, and monitor a four-wheeled robot. It's built on a master-slave architecture, with an ESP32 as the slave and an SBC (such as an NVIDIA Jetson or Raspberry Pi) as the master.

## Features
| Feature | Description | Specification |
| --- | --- | --- |
| ESP32 Microcontroller | Header for external ESP32 | **ESP32-WROOM-32** 38-pin dev board |
| Motor Drivers | Built on to the PCB | 4x **DRV8251A**. While the 8251A can provide up to **4.1 A**, the PCB design is compatible with other DRV series drivers too. |
| Motor Indicator LEDs | Built on to the PCB | 4x, user-controllable through the PWM demux. |
| Motor Encoder | Support for external quadrature encoders | Connect through JST-4 connectors, directly to ESP32 GPIO. |
| Motor Current Sense | Built on to the PCB | Using the 8251A's feature, connected to the ESP32 ADCs. Additional support for shunt resistor and current-sense amplifier in case of different motor driver IC. |
| IMU | Headers for external IMU | **MPU9250** connected over I2C0 to the ESP32 |
| Internal power supply | Built on to the PCB | **5V (5A)** (TPS5450 buck) and **3.3V** (AMS1117) power supplies for the ESP32 and other peripherals. |
| SBC Power supply | Built on to the PCB | **5V (5A)** (TPS5450 buck) power supply for the SBC., via USB-A. |
| PWM Demux | Built on to the PCB | **PCA9635** connected over I2C0 to the ESP32. Spare PWM channels are *not* exposed. |

## Overall system architecture
![block](assets/int-brain-block/signals/signals-white.png)

The ESP32 microcontroller handles the interaction between all the different peripherals, along with low-level control loops and signal processing. Telemetry and IMU data is fed directly to this ESP32.

Motors are driven using full H-bridge drivers, with built-in braking and current-sense capabilities. These motor drivers are controlled through a PWM demux, owing to the requirement for a large number of PWM pins.

Signals on-board are handled over I2C0, where the ESP32 acts as the master. It is configured as a slave on the I2C1 bus, which is used to communicate with an SBC. I2C0, I2C1, as well as an additional UART port are all exposed with JST connectors.

## Responsibilities of the ESP32 microcontroller
*   Receive commands from the SBC over I2C1
*   Control the motor drivers and the motor indicator LEDs, using the PCA9635 PWM mux over I2C0
*   Read the motor encoders and the motor current sense
*   Read the IMU data, over I2C0
*   Read the battery voltage (using a voltage divider)
*   Run the internal PID loop for motor control (RPM using encoder data)
*   Handle motor protection in case of stall (detected by encoder data) or disconnect (detected by current sense)
*   Filter IMU data
*   Send data to the SBC over I2C1 when requested

## Motor driver design
The system is designed to use the Texas Instruments DRV8251A motor driver, which is capable of driving up to 4.1 A.

Provided that DRV8251A (or any other DRV82\*\*A) drivers are used, we can leverage both the current-sense and current-limiting capabilities of the chip. Since the driver handles current attenuation, we may use a low-power high-resistance shunt resistor to generate a voltage drop. Additionally, using this as a voltage divider, we may supply a current limit; beyond this limit, the driver will automatically throttle voltage.

However, if an alternate motor driver is used (such as DRV8251), the system is designed to support a high-power low-resistance shunt resistor and a current-sense amplifier. The current-limiting feature of the driver will not be used in this case.

The equations to calculate these resistances were derived from the DRV8251A data sheet, and are provided in the schematics.

## Power supply design
![power](assets/int-brain-block/power/power-white.png)

The system is designed to take either 12 V or 24 V DC power. This can be given by either a DC barrel jack or an XT60 connector, designed with the intention of using a LiPo battery. This power must go through two diodes (hooked up in parallel) before being used for any purpose.

Conversion to 5 V and 3.3 V is handled on-board. The buck converter responsible for supplying power to the SBC is isolated from the rest of the system for increased stability.

## Pin-outs

### IMU MPU9250
| MPU9250 | ESP32 GPIO |
| --- | --- |
| INT | 33 |
| FSYNC | 32 |

Use I2C0 for communication with the IMU.

### Motor Drivers
| Motor | DRV8251A | PCA9685 |
| --- | --- | --- |
| 1A | IN 1 | 12 |
| 1A | IN 2 | 13 |
| 1B | IN 1 | 8 |
| 1B | IN 2 | 9 |
| 2A | IN 1 | 3 |
| 2A | IN 2 | 4 |
| 2B | IN 1 | 5 |
| 2B | IN 2 | 6 |

Use I2C0 for communication with the PCA9685.

### Motor indicator LEDs
| Motor | PCA9685 |
| --- | --- |
| 1A | 11 |
| 1B | 10 |
| 2A | 2 |
| 2B | 7 |

Use I2C0 for communication with the PCA9685.

### Encoders
| Encoder | Pin | ESP32 GPIO |
| --- | --- | --- |
| 1A | 1 | 18 |
| 1A | 2 | 19 |
| 1B | 1 | 23 |
| 1B | 2 | 25 |
| 2A | 1 | 5 |
| 2A | 2 | 17 |
| 2B | 1 | 14 |
| 2B | 2 | 12 |

### Motor current sense
| Motor | ESP32 GPIO |
| --- | --- |
| 1A | 35 |
| 1B | 34 |
| 2A | 39 |
| 2B | 36 |

### Protocols
| Protocol | Pin | ESP32 GPIO | Built-in Pull-up |
| --- | --- | --- | --- |
| I2C0 | SDA | 21 | Yes, 4.7k to 3.3V |
| I2C0 | SCL | 22 | Yes, 4.7k to 3.3V |
| I2C1 | SDA | 13 | Yes, 4.7k to 3.3V |
| I2C1 | SCL | 15 | Yes, 4.7k to 3.3V |
| UART0 | TX | 1 | No |
| UART0 | RX | 3 | No |

Use I2C0 (as master) for communication with the IMU and PCA9685. Use I2C1 (as slave) for communication with an SBC such as a Raspberry Pi or NVIDIA Jetson.

### Miscellaneous
| Function | ESP32 GPIO |
| --- | --- |
| Battery Voltage | 25 |
| Motor PCA Enable | 4 |

## Screenshots

PCB Front
![pcb-front](assets/pcb-front.png)

PCB Back
![pcb-back](assets/pcb-back.png)

3D Front
![3d-front](assets/3d-front.png)

3D Back
![3d-back](assets/3d-back.png)

3D Angled
![3d-angled](assets/3d-angled.png)