# Micromouse firmware

STM32CubeIDE project for the STM32F746VET6 micromouse PCB.

## Safety defaults

- Both MP6550 motor drivers remain asleep after boot/reset.
- No motor moves until a Bluetooth command is received.
- Physical PWM is capped at 71%, keeping the average at or below about 6 V
  even when a two-cell battery is fully charged to 8.4 V.
- Every moving command includes its own duration (maximum 10 seconds). The
  firmware stops and sleeps both drivers when that duration expires.
- Motor 2 uses the opposite polarity because the right motor is mirrored.

## HC-05 Bluetooth

- USART1 TX: PA9 / schematic net `MCU_TX`
- USART1 RX: PA10 / schematic net `MCU_RX`
- Module enable: PB4 / schematic net `KEY`; driven high to keep the installed
  HC-05 breakout enabled
- STATE: PB5 / schematic net `STATE`; GPIO input
- UART format: 9600 baud, 8 data bits, no parity, 1 stop bit

Commands are newline-terminated ASCII lines:

- `M10 <left_tenths> <right_tenths> <milliseconds>`: independently drive each
  wheel with 0.1% command resolution, clamped from -71.0% to +71.0%, for at
  most 10000 ms
- `M <left> <right> <milliseconds>`: legacy whole-percent command
- `STOP`: stop immediately and sleep the drivers
- `ZERO_ENC`: reset both accumulated encoder counts
- `TELEM <milliseconds>`: set telemetry interval (200-2000 ms)
- `HELLO` or `?`: report protocol identity and the PWM limit

Responses and telemetry are sent through the HC-05 so the host can confirm each
command and show live hardware readings.

## Bluetooth test hub

The main hardware test application is in [`test_hub`](test_hub). It belongs to
this `micromouseFirmware` project, not the separate `micro-mind-to-motor`
project. It provides independent timed motor tests, press-and-hold drive
presets, emergency stop, encoder rotations, live sensor panels, a protocol log,
and CSV telemetry recording. See [`test_hub/README.md`](test_hub/README.md).

The hub sensor pins are initialized in the preserved `USER CODE` section of
`Core/Src/main.c`: I2C1 PB8/PB9, IR receivers PC0-PC3, IR emitters PE2-PE5,
and ToF XSHUT PE13-PE15. Do not assign those pins elsewhere in CubeMX. An
unplugged LIS2MDL breakout is expected and is shown as `not detected`.

## Operating the Bluetooth test hub

Flash the current `micromouseFirmware` project to the STM32 before starting the
hub, and lift the wheels off the table for the first motor test.

### Pair the HC-05 and find its COM port

1. Power the PCB and confirm that the HC-05 status LED is blinking.
2. In Windows, open **Settings > Bluetooth & devices**, turn Bluetooth on, and
   choose **Add device > Bluetooth**.
3. Select **HC-05**. Enter `1234` when Windows requests a PIN. Some modules use
   `0000` instead.
4. Open **More Bluetooth settings > COM Ports** and find the HC-05 entry whose
   direction is **Outgoing**. Record its COM number, such as `COM7`. The test hub
   must use the outgoing port, not the incoming port.
5. You can also open **Device Manager > Ports (COM & LPT)** to confirm that a
   **Standard Serial over Bluetooth link (COMx)** exists. If several Bluetooth
   ports are listed, use the COM Ports page from the previous step to identify
   which one is the HC-05 outgoing port.

The paired device may appear disconnected in Windows until the test hub opens
the serial port; this is normal. Close PuTTY, Arduino Serial Monitor, and any
other serial program before opening the hub because only one program can use the
outgoing COM port at a time.

To list the serial ports from PowerShell, you can run:

```powershell
Get-CimInstance Win32_SerialPort | Select-Object DeviceID, Name
```

Open **Command Prompt** and enter:

```bat
cd C:\Users\olive\OneDrive\Documents\GitHub\Micromouse-UofT\firmware\micromouseFirmware\test_hub
.\.venv\Scripts\python.exe run_hub.py
```

If `.venv` has not been created on that computer yet, run this one-time setup
from the same `test_hub` folder:

```bat
py -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe run_hub.py
```

In the hub:

1. Click **Refresh**, select the HC-05 outgoing COM port found above, and click
   **Connect**. The protocol log should begin receiving telemetry.
2. Use **Run timed test** for a fixed-duration motor test. Positive PWM is
   forward and negative PWM is backward. Both the hub and firmware enforce the
   `-71.0%` to `+71.0%` safety limit.
3. The Forward, Backward, Spin left, and Spin right controls run only while held.
   Releasing the button or W/S/A/D key stops the motors.
4. Press **EMERGENCY STOP** or the space bar whenever immediate stopping is
   required.
5. Click **Zero encoder totals**, then rotate or drive the wheels to check ticks
   and wheel rotations.
6. Check the live sensor panel for IMU, magnetometer, IR, and ToF readings.
   Unplugged or failed optional sensors are shown as `not detected`.
7. Use **Start CSV recording** to save incoming telemetry for later analysis.

If the COM port reports access denied, close PuTTY and any other serial program.
Only one program can own the Bluetooth COM port at a time. If the GUI connects
but receives no telemetry, confirm that the updated firmware was flashed, the
HC-05 is connected at 9600 baud, and the outgoing COM port was selected.

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
| HC-05 EN/STATE | PB4 / PB5 | GPIO output/input |
| Test LED | PB0 | GPIO output |
| I2C sensor bus SCL/SDA | PB8 / PB9 | I2C1 |
| IR receivers 1-4 | PC0 / PC1 / PC2 / PC3 | ADC3 IN10-IN13 |
| IR emitters 1-4 | PE2 / PE3 / PE4 / PE5 | GPIO outputs |
| ToF XSHUT 1-3 | PE13 / PE14 / PE15 | GPIO outputs |
| ToF interrupt 1-3 | PC9 / PC10 / PC11 | GPIO inputs (reserved) |
| Magnetometer interrupt | PC8 | GPIO input (reserved) |

The PID and Kalman modules remain staged for later closed-loop navigation work.
