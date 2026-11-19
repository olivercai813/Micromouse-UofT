from __future__ import annotations

from dataclasses import dataclass


MAX_PWM = 71.0
TICKS_PER_WHEEL_REV = 360.0
MISSING = -32768

FLAG_IMU = 1 << 0
FLAG_MAG = 1 << 1
FLAG_IR = 1 << 2
FLAG_TOF_1 = 1 << 3
FLAG_TOF_2 = 1 << 4
FLAG_TOF_3 = 1 << 5
FLAG_BT_CONNECTED = 1 << 6
FLAG_MOTORS_ACTIVE = 1 << 7


@dataclass(frozen=True)
class Telemetry:
    milliseconds: int
    left_pwm_tenths: int
    right_pwm_tenths: int
    encoder_left: int
    encoder_right: int
    accel_x_mg: int
    accel_y_mg: int
    accel_z_mg: int
    gyro_x_mdps: int
    gyro_y_mdps: int
    gyro_z_mdps: int
    mag_x_mgauss: int
    mag_y_mgauss: int
    mag_z_mgauss: int
    ir_1: int
    ir_2: int
    ir_3: int
    ir_4: int
    tof_1_mm: int
    tof_2_mm: int
    tof_3_mm: int
    flags: int

    @property
    def left_rotations(self) -> float:
        return self.encoder_left / TICKS_PER_WHEEL_REV

    @property
    def right_rotations(self) -> float:
        return self.encoder_right / TICKS_PER_WHEEL_REV

    @property
    def left_pwm(self) -> float:
        return self.left_pwm_tenths / 10.0

    @property
    def right_pwm(self) -> float:
        return self.right_pwm_tenths / 10.0


def clamp_pwm(value: float) -> float:
    return round(max(-MAX_PWM, min(MAX_PWM, float(value))), 1)


def motor_command(left: float, right: float, duration_ms: int) -> str:
    """Build a safe timed motor command. Zero duration means stop."""
    left = clamp_pwm(left)
    right = clamp_pwm(right)
    duration_ms = max(0, min(10_000, int(duration_ms)))
    return f"M10 {round(left * 10):d} {round(right * 10):d} {duration_ms}"


def parse_line(line: str) -> Telemetry | tuple[str, str] | None:
    line = line.strip()
    if not line:
        return None
    if not line.startswith("T,"):
        head, _, rest = line.partition(" ")
        return head, rest

    parts = line.split(",")
    if len(parts) != 23:
        raise ValueError(f"expected 23 telemetry fields, received {len(parts)}")
    values = [int(value, 0) for value in parts[1:]]
    return Telemetry(*values)
