# Micromouse Bluetooth Test Hub

This desktop GUI controls and monitors the **micromouseFirmware** project over the HC-05 serial link. It is intentionally separate from `micro-mind-to-motor`.

## Safety model

- Both the GUI and STM32 firmware clamp each wheel to **-71% through +71% PWM**.
- Typed PWM values are rounded to one decimal place and visibly clamped in the
  controls; positive means forward and negative means backward for that wheel.
- Every `M` command is timed. The longest accepted run is 10 seconds.
- Press-and-hold driving refreshes a 400 ms command; releasing the key/button sends `STOP`.
- Space, the red button, disconnect, and closing the window all send `STOP`.
- The firmware is the final authority: it stops when the duration expires even if the laptop freezes.

Lift the wheels off the table for the first test. Do not operate the motors without the motor battery/power path connected correctly.

## Run on Windows

From `firmware\micromouseFirmware\test_hub`:

```bat
py -m venv .venv
.venv\Scripts\python.exe -m pip install -r requirements.txt
.venv\Scripts\python.exe run_hub.py
```

Pair the HC-05 in Windows first, then choose its **outgoing** COM port in the GUI. PuTTY and the GUI cannot own the same COM port at the same time, so close PuTTY before connecting.

The four hold controls use the adjustable **Hold-control power** value:

- Forward sends `(+power, +power)`.
- Backward sends `(-power, -power)`.
- Spin left sends `(-power, +power)`.
- Spin right sends `(+power, -power)`.

They only keep moving while held. The same mappings are available on W/S/A/D,
except while typing in an input field. Emergency stop cancels the refresh timer
before sending `STOP`, so a held command cannot restart the motors afterward.

## Commands sent to the firmware

- `HELLO` — identify the firmware/protocol.
- `M10 <left_tenths> <right_tenths> <milliseconds>` — signed independent wheel
  commands in tenths of a percent. For example, `M10 501 -355 1000` means
  +50.1% left and -35.5% right for one second.
- `M <left> <right> <milliseconds>` — legacy whole-percent form, retained for
  PuTTY and older scripts.
- `STOP` — immediate coast/driver sleep.
- `ZERO_ENC` — zero both accumulated encoder counts.
- `TELEM <milliseconds>` — telemetry interval; GUI requests 250 ms.

Telemetry is one compact comma-separated line:

```text
T,time_ms,left_pwm_tenths,right_pwm_tenths,left_ticks,right_ticks,ax_mg,ay_mg,az_mg,gx_mdps,gy_mdps,gz_mdps,mx_mgauss,my_mgauss,mz_mgauss,ir1,ir2,ir3,ir4,tof1_mm,tof2_mm,tof3_mm,flags
```

The validity flags prevent an unplugged breakout or failed sensor from looking like a real zero reading. Encoder rotation defaults to 360 quadrature ticks per wheel revolution (12 CPR × 30:1). The value is editable directly in the encoder panel so you can correct it after measuring one physical wheel revolution.
