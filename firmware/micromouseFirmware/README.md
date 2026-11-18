# Micromouse firmware

STM32CubeIDE project for the STM32F746VET6 micromouse PCB.

## Safety defaults

- Both MP6550 motor drivers remain asleep after boot/reset.
- No motor moves until a Bluetooth command is received.
- Physical PWM is capped at 71%, keeping the average at or below about 6 V
  even when a two-cell battery is fully charged to 8.4 V.
- A moving command must be refreshed within 500 ms or the firmware stops and
  sleeps both drivers.
- Motor 2 uses the opposite polarity because the right motor is mirrored.

## HC-05 Bluetooth

- USART1 TX: PA9 / schematic net `MCU_TX`
- USART1 RX: PA10 / schematic net `MCU_RX`
- KEY/EN: PB4 / schematic net `KEY`; driven low for normal data mode
- STATE: PB5 / schematic net `STATE`; GPIO input
- UART format: 9600 baud, 8 data bits, no parity, 1 stop bit

Commands are single ASCII characters:

- `f` or `F`: forward at 50% PWM
- `b` or `B`: backward at 50% PWM
- `s` or `S`: stop immediately
- `?`: report `MICROMOUSE READY MAX_PWM=71`

Forward/backward commands must be repeated at least every 500 ms. Responses are
sent through the HC-05 so the host can confirm each command.

## KiCad-verified hardware map

The assignments below were extracted from
`Micromouse/Micromouse_main_project/MicromouseSchematic.kicad_sch`.

| Function | STM32 pin | Peripheral |
|---|---|---|
| VISEN 1 | PA2 | ADC1 IN2 |
| VISEN 2 | PA3 | ADC2 IN3 |
| Encoder 1 A/B | PE9 / PE11 | TIM1 CH1/CH2 |
| Encoder 2 A/B | PA6 / PA7 | TIM3 CH1/CH2 |
| Motor 1 IN1/IN2 | PD12 / PD13 | TIM4 CH1/CH2 |
| Motor 2 IN1/IN2 | PD14 / PD15 | TIM4 CH3/CH4 |
| Motor 1/2 nSLEEP | PE7 / PE10 | GPIO outputs |
| IMU CS | PB12 | GPIO output |
| IMU SCK/MISO/MOSI | PB13 / PB14 / PB15 | SPI2 |
| IMU INT1/INT2 | PC6 / PC7 | GPIO EXTI6/EXTI7 |
| HC-05 TX/RX | PA9 / PA10 | USART1 |
| HC-05 KEY/STATE | PB4 / PB5 | GPIO output/input |
| Test LED | PB0 | GPIO output |

The motor, IMU, PID, and Kalman modules are present but intentionally not yet
called by `main.c`; they are staged for later integration.
