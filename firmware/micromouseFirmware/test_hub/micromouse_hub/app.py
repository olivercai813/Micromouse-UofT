from __future__ import annotations

import csv
import queue
import time
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

from .protocol import (
    FLAG_BT_CONNECTED,
    FLAG_IMU,
    FLAG_IR,
    FLAG_MAG,
    FLAG_MOTORS_ACTIVE,
    FLAG_TOF_1,
    FLAG_TOF_2,
    FLAG_TOF_3,
    MAX_PWM,
    MISSING,
    Telemetry,
    motor_command,
    parse_line,
)
from .serial_link import LinkEvent, SerialLink, available_ports


class HubApp(tk.Tk):
    BG = "#10151c"
    PANEL = "#19212b"
    TEXT = "#e8edf3"
    MUTED = "#8fa1b3"
    GREEN = "#48d597"
    RED = "#ff5f67"
    PORT_BG = "#ffd166"
    PORT_TEXT = "#10151c"

    def __init__(self) -> None:
        super().__init__()
        self.title("Micromouse Bluetooth Test Hub")
        self.geometry("1180x820")
        self.minsize(1000, 760)
        self.configure(bg=self.BG)
        self.events: queue.Queue[LinkEvent] = queue.Queue()
        self.link = SerialLink(self.events)
        self.last_telemetry: Telemetry | None = None
        self.last_packet_time = 0.0
        self.csv_file = None
        self.csv_writer = None
        self._drive_job: str | None = None
        self._held_drive = (0, 0)
        self._pressed_drive_keys: list[str] = []
        self._make_style()
        self._build()
        self.refresh_ports()
        self.protocol("WM_DELETE_WINDOW", self.close)
        self.bind_all("<FocusOut>", self._on_focus_out, add="+")
        self.bind("<Unmap>", lambda _event: self._stop_active_hold(), add="+")
        self.after(40, self._poll_events)
        self.after(250, self._update_age)

    def _make_style(self) -> None:
        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure(".", background=self.BG, foreground=self.TEXT, fieldbackground=self.PANEL)
        style.configure("TFrame", background=self.BG)
        style.configure("Panel.TFrame", background=self.PANEL)
        style.configure("TLabel", background=self.BG, foreground=self.TEXT)
        style.configure("Panel.TLabel", background=self.PANEL, foreground=self.TEXT)
        style.configure("Muted.Panel.TLabel", background=self.PANEL, foreground=self.MUTED)
        style.configure("Title.TLabel", font=("Segoe UI Semibold", 18), background=self.BG)
        style.configure("Value.Panel.TLabel", font=("Consolas", 13), background=self.PANEL)
        style.configure("TButton", padding=(9, 6))
        style.configure("Stop.TButton", background=self.RED, foreground="white", font=("Segoe UI Semibold", 12))
        style.map("Stop.TButton", background=[("active", "#df434d")])
        style.configure("Run.TButton", background=self.GREEN, foreground="#07130e")
        style.map("Run.TButton", background=[("active", "#35bd82")])
        style.configure(
            "Port.TCombobox",
            fieldbackground=self.PORT_BG,
            background=self.PORT_BG,
            foreground=self.PORT_TEXT,
            arrowcolor=self.PORT_TEXT,
            bordercolor=self.PORT_BG,
            lightcolor=self.PORT_BG,
            darkcolor=self.PORT_BG,
        )
        style.map(
            "Port.TCombobox",
            fieldbackground=[("readonly", self.PORT_BG)],
            foreground=[("readonly", self.PORT_TEXT)],
            selectbackground=[("readonly", self.PORT_BG)],
            selectforeground=[("readonly", self.PORT_TEXT)],
        )
        self.option_add("*TCombobox*Listbox.background", self.PORT_BG)
        self.option_add("*TCombobox*Listbox.foreground", self.PORT_TEXT)
        self.option_add("*TCombobox*Listbox.selectBackground", "#f0b83f")
        self.option_add("*TCombobox*Listbox.selectForeground", self.PORT_TEXT)

    def _build(self) -> None:
        header = ttk.Frame(self, padding=(18, 14))
        header.pack(fill="x")
        ttk.Label(header, text="Micromouse Test Hub", style="Title.TLabel").pack(side="left")
        self.connection_text = tk.StringVar(value="Disconnected")
        ttk.Label(header, textvariable=self.connection_text).pack(side="right")

        body = ttk.Panedwindow(self, orient="horizontal")
        body.pack(fill="both", expand=True, padx=16, pady=(0, 16))
        left = ttk.Frame(body)
        right = ttk.Frame(body)
        body.add(left, weight=2)
        body.add(right, weight=3)
        self._build_connection(left)
        self._build_motor(left)
        self._build_encoder(left)
        self._build_sensors(right)
        self._build_log(right)

    def _panel(self, parent: ttk.Frame, title: str) -> ttk.Frame:
        outer = ttk.Frame(parent, style="Panel.TFrame", padding=12)
        outer.pack(fill="x", pady=(0, 10))
        ttk.Label(outer, text=title, style="Panel.TLabel", font=("Segoe UI Semibold", 12)).pack(anchor="w", pady=(0, 8))
        return outer

    def _build_connection(self, parent: ttk.Frame) -> None:
        panel = self._panel(parent, "Bluetooth serial connection")
        row = ttk.Frame(panel, style="Panel.TFrame")
        row.pack(fill="x")
        self.port = tk.StringVar()
        self.port_box = ttk.Combobox(
            row,
            textvariable=self.port,
            width=14,
            state="readonly",
            style="Port.TCombobox",
        )
        self.port_box.pack(side="left", padx=(0, 6))
        ttk.Button(row, text="Refresh", command=self.refresh_ports).pack(side="left", padx=(0, 6))
        self.connect_button = ttk.Button(row, text="Connect", command=self.toggle_connection)
        self.connect_button.pack(side="left")
        self.packet_age = tk.StringVar(value="No telemetry")
        ttk.Label(panel, textvariable=self.packet_age, style="Muted.Panel.TLabel").pack(anchor="w", pady=(8, 0))

    def _build_motor(self, parent: ttk.Frame) -> None:
        panel = self._panel(parent, "Motor test (firmware and GUI both cap PWM at 71%)")
        ttk.Label(
            panel,
            text="Positive PWM drives that wheel forward; negative PWM drives it backward.",
            style="Muted.Panel.TLabel",
        ).pack(anchor="w", pady=(0, 6))
        self.left_pwm = tk.DoubleVar(value=0.0)
        self.right_pwm = tk.DoubleVar(value=0.0)
        self.duration = tk.IntVar(value=1000)
        for label, variable in (("Left wheel", self.left_pwm), ("Right wheel", self.right_pwm)):
            row = ttk.Frame(panel, style="Panel.TFrame")
            row.pack(fill="x", pady=2)
            ttk.Label(row, text=label, width=12, style="Panel.TLabel").pack(side="left")
            ttk.Scale(
                row,
                from_=-MAX_PWM,
                to=MAX_PWM,
                variable=variable,
                orient="horizontal",
                command=lambda value, v=variable: self._set_one_decimal(v, value),
            ).pack(side="left", fill="x", expand=True)
            spinbox = ttk.Spinbox(
                row, from_=-MAX_PWM, to=MAX_PWM, increment=0.1,
                format="%.1f", textvariable=variable, width=6,
            )
            spinbox.pack(side="left", padx=(8, 0))
            self._bind_bounded_float(spinbox, variable, -MAX_PWM, MAX_PWM, 0.0)
        row = ttk.Frame(panel, style="Panel.TFrame")
        row.pack(fill="x", pady=(7, 5))
        ttk.Label(row, text="Duration (ms)", width=12, style="Panel.TLabel").pack(side="left")
        duration_box = ttk.Spinbox(row, from_=50, to=10000, increment=50, textvariable=self.duration, width=8)
        duration_box.pack(side="left")
        self._bind_bounded_int(duration_box, self.duration, 50, 10000, 1000)
        ttk.Button(row, text="Run timed test", style="Run.TButton", command=self.run_test).pack(side="left", padx=8)
        ttk.Button(row, text="EMERGENCY STOP", style="Stop.TButton", command=self.stop).pack(side="right")

        preset_config = ttk.Frame(panel, style="Panel.TFrame")
        preset_config.pack(fill="x", pady=(6, 4))
        self.preset_pwm = tk.DoubleVar(value=50.0)
        ttk.Label(preset_config, text="Hold-control power (%)", style="Muted.Panel.TLabel").pack(side="left")
        ttk.Scale(
            preset_config,
            from_=0.1,
            to=MAX_PWM,
            variable=self.preset_pwm,
            orient="horizontal",
            command=lambda value: self._set_one_decimal(self.preset_pwm, value),
        ).pack(side="left", fill="x", expand=True, padx=8)
        preset_box = ttk.Spinbox(
            preset_config, from_=0.1, to=MAX_PWM, increment=0.1,
            format="%.1f", textvariable=self.preset_pwm, width=6,
        )
        preset_box.pack(side="right")
        self._bind_bounded_float(preset_box, self.preset_pwm, 0.1, MAX_PWM, 50.0)

        presets = ttk.Frame(panel, style="Panel.TFrame")
        presets.pack(fill="x")
        for text, action in (("Forward", "forward"), ("Backward", "backward"), ("Spin left", "left"), ("Spin right", "right")):
            button = ttk.Button(presets, text=text)
            button.pack(side="left", padx=(0, 5), expand=True, fill="x")
            button.bind("<ButtonPress-1>", lambda _event, a=action: self.start_preset(a))
            button.bind("<ButtonRelease-1>", lambda _event: self.end_hold())
        ttk.Label(
            panel,
            text="Forward (+,+)  Backward (-,-)  Spin left (-,+)  Spin right (+,-)",
            style="Muted.Panel.TLabel",
        ).pack(anchor="w", pady=(8, 0))
        ttk.Label(
            panel,
            text="Hold a button or use W/S/A/D. Release stops. Space = emergency stop.",
            style="Muted.Panel.TLabel",
        ).pack(anchor="w", pady=(2, 0))
        for key, action in (("w", "forward"), ("s", "backward"), ("a", "left"), ("d", "right")):
            self.bind_all(f"<KeyPress-{key}>", lambda event, a=action: self._keyboard_drive(event, a))
            self.bind_all(f"<KeyPress-{key.upper()}>", lambda event, a=action: self._keyboard_drive(event, a))
            self.bind_all(f"<KeyRelease-{key}>", self._keyboard_release)
            self.bind_all(f"<KeyRelease-{key.upper()}>", self._keyboard_release)
        self.bind_all("<space>", lambda _e: self.stop())

    def _build_encoder(self, parent: ttk.Frame) -> None:
        panel = self._panel(parent, "Wheel encoders")
        calibration = ttk.Frame(panel, style="Panel.TFrame")
        calibration.pack(fill="x", pady=(0, 6))
        self.ticks_per_rev = tk.IntVar(value=360)
        ttk.Label(calibration, text="Ticks / wheel revolution", style="Muted.Panel.TLabel").pack(side="left")
        ticks_box = ttk.Spinbox(calibration, from_=1, to=10000, textvariable=self.ticks_per_rev, width=7)
        ticks_box.pack(side="right")
        self._bind_bounded_int(ticks_box, self.ticks_per_rev, 1, 10000, 360)
        self.encoder_values = {}
        for key, label in (("left_ticks", "Left ticks"), ("left_rot", "Left rotations"), ("right_ticks", "Right ticks"), ("right_rot", "Right rotations")):
            value = tk.StringVar(value="—")
            self.encoder_values[key] = value
            row = ttk.Frame(panel, style="Panel.TFrame")
            row.pack(fill="x")
            ttk.Label(row, text=label, width=18, style="Muted.Panel.TLabel").pack(side="left")
            ttk.Label(row, textvariable=value, style="Value.Panel.TLabel").pack(side="right")
        ttk.Button(panel, text="Zero encoder totals", command=lambda: self.send("ZERO_ENC")).pack(anchor="e", pady=(8, 0))

    def _build_sensors(self, parent: ttk.Frame) -> None:
        panel = self._panel(parent, "Live sensors")
        self.sensor_values: dict[str, tk.StringVar] = {}
        sections = [
            ("IMU", [("ax", "Accel X (mg)"), ("ay", "Accel Y (mg)"), ("az", "Accel Z (mg)"), ("gx", "Gyro X (mdps)"), ("gy", "Gyro Y (mdps)"), ("gz", "Gyro Z (mdps)")]),
            ("Magnetometer", [("mx", "X (mGauss)"), ("my", "Y (mGauss)"), ("mz", "Z (mGauss)")]),
            ("IR receivers", [("ir1", "IR 1"), ("ir2", "IR 2"), ("ir3", "IR 3"), ("ir4", "IR 4")]),
            ("Time of Flight", [("tof1", "ToF 1 (mm)"), ("tof2", "ToF 2 (mm)"), ("tof3", "ToF 3 (mm)")]),
        ]
        grid = ttk.Frame(panel, style="Panel.TFrame")
        grid.pack(fill="x")
        for column, (title, fields) in enumerate(sections):
            box = ttk.Frame(grid, style="Panel.TFrame", padding=(6, 0))
            box.grid(row=0, column=column, sticky="nsew")
            grid.columnconfigure(column, weight=1)
            ttk.Label(box, text=title, style="Panel.TLabel", font=("Segoe UI Semibold", 10)).pack(anchor="w", pady=(0, 4))
            for key, label in fields:
                variable = tk.StringVar(value="not detected")
                self.sensor_values[key] = variable
                ttk.Label(box, text=label, style="Muted.Panel.TLabel").pack(anchor="w")
                ttk.Label(box, textvariable=variable, style="Value.Panel.TLabel").pack(anchor="w", pady=(0, 4))
        self.status_values = tk.StringVar(value="Waiting for telemetry")
        ttk.Label(panel, textvariable=self.status_values, style="Muted.Panel.TLabel").pack(anchor="w", pady=(10, 0))

    def _build_log(self, parent: ttk.Frame) -> None:
        panel = self._panel(parent, "Protocol log and recording")
        buttons = ttk.Frame(panel, style="Panel.TFrame")
        buttons.pack(fill="x", pady=(0, 6))
        self.record_button = ttk.Button(buttons, text="Start CSV recording", command=self.toggle_recording)
        self.record_button.pack(side="left")
        ttk.Button(buttons, text="Clear log", command=lambda: self.log.delete("1.0", "end")).pack(side="right")
        raw = ttk.Frame(panel, style="Panel.TFrame")
        raw.pack(fill="x", pady=(0, 6))
        self.raw_command = tk.StringVar()
        raw_entry = ttk.Entry(raw, textvariable=self.raw_command)
        raw_entry.pack(side="left", fill="x", expand=True, padx=(0, 6))
        raw_entry.bind("<Return>", lambda _event: self.send_raw())
        ttk.Button(raw, text="Send command", command=self.send_raw).pack(side="right")
        self.log = tk.Text(panel, height=10, bg="#0b0f14", fg="#b9c7d5", insertbackground="white", relief="flat", font=("Consolas", 9))
        self.log.pack(fill="both", expand=True)

    def refresh_ports(self) -> None:
        ports = available_ports()
        self.port_box["values"] = ports
        if ports and self.port.get() not in ports:
            self.port.set(ports[0])

    def toggle_connection(self) -> None:
        if self.link.connected:
            self._cancel_hold()
            self.link.disconnect()
            return
        if not self.port.get():
            messagebox.showerror("No serial port", "Pair the HC-05, refresh, and select its outgoing COM port.")
            return
        try:
            self.link.connect(self.port.get(), 9600)
            self.send("HELLO")
            self.send("TELEM 250")
        except Exception as exc:
            messagebox.showerror("Connection failed", str(exc))

    def send(self, command: str, *, log: bool = True) -> bool:
        try:
            self.link.send(command)
            if log:
                self._append_log(f"> {command}")
            return True
        except Exception as exc:
            self._append_log(f"! {exc}")
            return False

    def run_test(self) -> None:
        self._cancel_hold()
        left = self._bounded_float(self.left_pwm, -MAX_PWM, MAX_PWM, 0.0)
        right = self._bounded_float(self.right_pwm, -MAX_PWM, MAX_PWM, 0.0)
        duration = self._bounded_int(self.duration, 50, 10000, 1000)
        self.send(motor_command(left, right, duration))

    def send_raw(self) -> None:
        command = self.raw_command.get().strip()
        if command and self.send(command):
            self.raw_command.set("")

    def start_hold(self, left: float, right: float) -> None:
        if self._drive_job is not None and self._held_drive == (left, right):
            return
        self._cancel_hold(clear_keys=False)
        self._held_drive = (left, right)
        self._refresh_hold(log=True)

    def start_preset(self, action: str) -> None:
        power = self._bounded_float(self.preset_pwm, 0.1, MAX_PWM, 50.0)
        commands = {
            "forward": (power, power),
            "backward": (-power, -power),
            "left": (-power, power),
            "right": (power, -power),
        }
        self.start_hold(*commands[action])

    def _refresh_hold(self, *, log: bool = False) -> None:
        left, right = self._held_drive
        self.send(motor_command(left, right, 400), log=log)
        self._drive_job = self.after(200, self._refresh_hold)

    def end_hold(self, send_stop: bool = True) -> None:
        self._cancel_hold()
        if send_stop:
            self.send("STOP")

    def _cancel_hold(self, *, clear_keys: bool = True) -> None:
        if self._drive_job is not None:
            self.after_cancel(self._drive_job)
            self._drive_job = None
        self._held_drive = (0, 0)
        if clear_keys:
            self._pressed_drive_keys.clear()

    def stop(self) -> None:
        self._cancel_hold()
        self.send("STOP")

    def _keyboard_drive(self, event: tk.Event, action: str) -> None:
        if event.widget.winfo_class() in {"Entry", "TEntry", "Spinbox", "TSpinbox", "Text", "TCombobox"}:
            return
        key = event.keysym.lower()
        if key in self._pressed_drive_keys:
            return
        self._pressed_drive_keys.append(key)
        self.start_preset(action)

    def _keyboard_release(self, event: tk.Event) -> None:
        key = event.keysym.lower()
        if key in self._pressed_drive_keys:
            self._pressed_drive_keys.remove(key)
        if self._pressed_drive_keys:
            actions = {"w": "forward", "s": "backward", "a": "left", "d": "right"}
            self.start_preset(actions[self._pressed_drive_keys[-1]])
        elif self._drive_job is not None:
            self.end_hold()

    def _on_focus_out(self, _event: tk.Event) -> None:
        self.after_idle(self._stop_if_app_unfocused)

    def _stop_if_app_unfocused(self) -> None:
        if self._drive_job is not None and self.focus_get() is None:
            self.end_hold()

    def _stop_active_hold(self) -> None:
        if self._drive_job is not None:
            self.end_hold()

    def _bind_bounded_int(self, widget: ttk.Spinbox, variable: tk.IntVar,
                          minimum: int, maximum: int, fallback: int) -> None:
        clamp = lambda _event=None: self._bounded_int(variable, minimum, maximum, fallback)
        widget.bind(
            "<KeyRelease>",
            lambda _event=None: self._clamp_if_parseable(variable, minimum, maximum),
            add="+",
        )
        widget.bind("<FocusOut>", clamp, add="+")
        widget.bind("<Return>", clamp, add="+")
        widget.configure(command=clamp)

    def _bind_bounded_float(self, widget: ttk.Spinbox, variable: tk.DoubleVar,
                            minimum: float, maximum: float, fallback: float) -> None:
        clamp = lambda _event=None: self._bounded_float(variable, minimum, maximum, fallback)
        widget.bind(
            "<KeyRelease>",
            lambda _event=None: self._clamp_float_if_parseable(variable, minimum, maximum),
            add="+",
        )
        widget.bind("<FocusOut>", clamp, add="+")
        widget.bind("<Return>", clamp, add="+")
        widget.configure(command=clamp)

    @staticmethod
    def _bounded_int(variable: tk.IntVar, minimum: int, maximum: int, fallback: int) -> int:
        try:
            value = int(variable.get())
        except (tk.TclError, ValueError):
            value = fallback
        value = max(minimum, min(maximum, value))
        variable.set(value)
        return value

    @staticmethod
    def _bounded_float(variable: tk.DoubleVar, minimum: float, maximum: float,
                       fallback: float) -> float:
        try:
            value = float(variable.get())
        except (tk.TclError, ValueError):
            value = fallback
        value = round(max(minimum, min(maximum, value)), 1)
        variable.set(value)
        return value

    @staticmethod
    def _set_one_decimal(variable: tk.DoubleVar, value: str) -> None:
        variable.set(round(float(value), 1))

    @staticmethod
    def _clamp_if_parseable(variable: tk.IntVar, minimum: int, maximum: int) -> None:
        try:
            value = int(variable.get())
        except (tk.TclError, ValueError):
            return  # Allow temporary input such as an empty field or just "-".
        variable.set(max(minimum, min(maximum, value)))

    @staticmethod
    def _clamp_float_if_parseable(variable: tk.DoubleVar, minimum: float,
                                  maximum: float) -> None:
        try:
            value = float(variable.get())
        except (tk.TclError, ValueError):
            return
        variable.set(round(max(minimum, min(maximum, value)), 1))

    def _poll_events(self) -> None:
        while True:
            try:
                event = self.events.get_nowait()
            except queue.Empty:
                break
            self._handle_event(event)
        self.after(40, self._poll_events)

    def _handle_event(self, event: LinkEvent) -> None:
        if event.kind == "connected":
            self.connection_text.set(f"Connected: {event.value} @ 9600")
            self.connect_button.configure(text="Disconnect")
        elif event.kind == "disconnected":
            self._cancel_hold()
            self.connection_text.set("Disconnected")
            self.connect_button.configure(text="Connect")
            self.packet_age.set("No telemetry")
        elif event.kind == "error":
            self._append_log(f"! Serial error: {event.value}")
        elif event.kind == "line":
            try:
                parsed = parse_line(event.value)
                if isinstance(parsed, Telemetry):
                    self._display_telemetry(parsed)
                else:
                    self._append_log(f"< {event.value}")
            except ValueError as exc:
                self._append_log(f"! Bad packet ({exc}): {event.value}")

    def _display_telemetry(self, data: Telemetry) -> None:
        self.last_telemetry = data
        self.last_packet_time = time.monotonic()
        self.encoder_values["left_ticks"].set(str(data.encoder_left))
        self.encoder_values["right_ticks"].set(str(data.encoder_right))
        ticks_per_rev = self._bounded_int(self.ticks_per_rev, 1, 10000, 360)
        self.encoder_values["left_rot"].set(f"{data.encoder_left / ticks_per_rev:.3f}")
        self.encoder_values["right_rot"].set(f"{data.encoder_right / ticks_per_rev:.3f}")
        imu = bool(data.flags & FLAG_IMU)
        mag = bool(data.flags & FLAG_MAG)
        ir = bool(data.flags & FLAG_IR)
        mapping = {
            "ax": (data.accel_x_mg, imu), "ay": (data.accel_y_mg, imu), "az": (data.accel_z_mg, imu),
            "gx": (data.gyro_x_mdps, imu), "gy": (data.gyro_y_mdps, imu), "gz": (data.gyro_z_mdps, imu),
            "mx": (data.mag_x_mgauss, mag), "my": (data.mag_y_mgauss, mag), "mz": (data.mag_z_mgauss, mag),
            "ir1": (data.ir_1, ir), "ir2": (data.ir_2, ir), "ir3": (data.ir_3, ir), "ir4": (data.ir_4, ir),
            "tof1": (data.tof_1_mm, bool(data.flags & FLAG_TOF_1)),
            "tof2": (data.tof_2_mm, bool(data.flags & FLAG_TOF_2)),
            "tof3": (data.tof_3_mm, bool(data.flags & FLAG_TOF_3)),
        }
        for key, (value, valid) in mapping.items():
            self.sensor_values[key].set(str(value) if valid and value != MISSING else "not detected")
        states = [
            "BT pin high" if data.flags & FLAG_BT_CONNECTED else "BT pin low",
            "motors active" if data.flags & FLAG_MOTORS_ACTIVE else "motors stopped",
            f"commanded L/R {data.left_pwm:.1f}/{data.right_pwm:.1f}%",
        ]
        self.status_values.set(" • ".join(states))
        if self.link.connected:
            self.connection_text.set(f"Connected: {self.port.get()} @ 9600")
        self._record(data)

    def _update_age(self) -> None:
        if self.last_packet_time:
            age = time.monotonic() - self.last_packet_time
            self.packet_age.set(f"Last telemetry: {age:.1f} s ago")
            if age > 1.0 and self.link.connected:
                self.connection_text.set(f"Connected: {self.port.get()} (telemetry stale)")
        self.after(250, self._update_age)

    def _append_log(self, line: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        self.log.insert("end", f"{stamp} {line}\n")
        self.log.see("end")

    def toggle_recording(self) -> None:
        if self.csv_file is not None:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None
            self.record_button.configure(text="Start CSV recording")
            return
        filename = filedialog.asksaveasfilename(defaultextension=".csv", filetypes=[("CSV", "*.csv")], initialfile="micromouse_telemetry.csv")
        if not filename:
            return
        self.csv_file = Path(filename).open("w", newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(Telemetry.__dataclass_fields__.keys())
        self.record_button.configure(text="Stop CSV recording")

    def _record(self, data: Telemetry) -> None:
        if self.csv_writer is None:
            return
        self.csv_writer.writerow([getattr(data, name) for name in Telemetry.__dataclass_fields__])
        self.csv_file.flush()

    def close(self) -> None:
        self._cancel_hold()
        self.link.disconnect(send_stop=True)
        if self.csv_file is not None:
            self.csv_file.close()
        self.destroy()


def main() -> None:
    HubApp().mainloop()
