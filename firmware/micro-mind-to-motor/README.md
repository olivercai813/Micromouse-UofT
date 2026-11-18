# micro-mind-to-motor

Self-contained Muse-to-Micromouse project. This folder contains both:

- the STM32CubeIDE/CubeMX firmware for the STM32F746 and MP6550 drivers;
- the Python laptop controller that reads the Muse over BLE and sends wheel
  commands through the HC-05 Bluetooth Classic COM port.

The complete data path is:

```text
Muse --BLE--> laptop Python --Bluetooth SPP--> HC-05 --USART1--> STM32F746
     --> TIM4 PWM --> MP6550 drivers --> motors
```

The Muse never connects directly to the HC-05. The laptop bridges the two
Bluetooth connections.

## Folder layout

```text
Core/                    STM32 application source
Drivers/                 STM32 HAL and CMSIS
micro-mind-to-motor.ioc  CubeMX configuration
mind2motor/              laptop EEG and robot-control package
tests/                   Python tests
pyproject.toml           Python package and dependencies
```

## One-time laptop setup

Open Command Prompt in this folder:

```cmd
cd /d C:\Users\olive\OneDrive\Documents\GitHub\Micromouse-UofT\firmware\micro-mind-to-motor
```

Create the local environment and install this project:

```cmd
"C:\Users\olive\AppData\Local\Programs\Python\Python313\python.exe" -m venv .venv
.\.venv\Scripts\python.exe -m pip install --upgrade pip
.\.venv\Scripts\python.exe -m pip install -e ".[ble]"
```

## Firmware

Import this folder as an existing STM32CubeIDE project, build the Debug
configuration, and flash `Debug/micro-mind-to-motor.elf`.

Hardware configuration:

- USART1 PA9/PA10, 9600 baud, to the HC-05, with interrupt-driven reception;
- TIM4 PD12-PD15 for the two MP6550 drivers;
- PE7 and PE10 for the driver nSLEEP inputs;
- Motor 1/J8 is treated as left; Motor 2/J7 is treated as right.

## Wheel protocol

The laptop sends a line at least every 400 ms:

```text
W<left>,<right>\n
```

The protocol accepts signed PWM percentages from -100 through 100. The
firmware clamps each wheel to -71 through 71 so a fully charged 8.4 V battery
applies no more than about 6 V average to the motors. Positive is forward and
negative is backward:

```text
W60,60    forward
W-60,-60  backward
W30,60    turn left
W60,30    turn right
W0,0      stop
```

The board reports status every 100 ms:

```text
S pc 60 60 0
```

The final number counts rejected input lines. If no valid wheel frame arrives
for 400 ms, the firmware stops both motors and reports `S idle 0 0 ...`.

## Manual motor tests

Pair the HC-05 with Windows, find its outgoing COM port, close PuTTY, and run
these commands from this folder. Replace `COM7` if needed. The HC-05 baud
defaults to 9600, so `--baud` is optional.

The `--seconds` timer starts only after the Bluetooth link has identified the
board, so connection time does not shorten the requested motor-test duration.

```cmd
.\.venv\Scripts\python.exe -m mind2motor wheels 60 60 --seconds 3 --car COM7
.\.venv\Scripts\python.exe -m mind2motor wheels -60 -60 --seconds 1 --car COM7
.\.venv\Scripts\python.exe -m mind2motor wheels 30 60 --seconds 3 --car COM7
.\.venv\Scripts\python.exe -m mind2motor wheels 60 30 --seconds 3 --car COM7
.\.venv\Scripts\python.exe -m mind2motor wheels 0 0 --seconds 1 --car COM7
```

Test with the wheels raised and the motor supply connected.

## Muse control

The HC-05 should be paired with Windows. The Muse should be on, worn with good
electrode contact, disconnected from phones, and normally not paired in
Windows. Start a session with:

```cmd
.\.venv\Scripts\python.exe -m mind2motor drive --car COM7
```

The program calibrates relaxed versus focused EEG, then uses focus to move,
relaxation to stop, and calibrated head tilt to steer.
