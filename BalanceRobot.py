"""D435if IMU-only balancing dashboard. See BALANCING.md for commissioning.

python BalanceRobot.py --demo       # no camera, serial port, or motor access
python BalanceRobot.py --imu-only   # live IMU, no serial port or motor access
python BalanceRobot.py --port COM5  # Mega with BalanceMega sketch; starts disarmed
python BalanceRobot.py --check      # dependency and device inventory only
"""

import argparse
from collections import deque
import csv
from datetime import datetime
import importlib
import json
import math
from pathlib import Path
from queue import Empty, Full, Queue
import secrets
import sys
import threading
import time

from BalanceCore import BalancePID, Settings, TiltEstimator, stationary_reference

ROOT = Path(__file__).resolve().parent
CONTROL_HZ = 100
GYRO_MAX_AGE = 0.05
ACCEL_MAX_AGE = 0.12
SERIAL_MAX_AGE = 0.15


class RealSenseIMU:
    def __init__(self, camera_serial=None):
        import pyrealsense2 as rs
        self.rs = rs
        self.lock = threading.Lock()
        self.events = deque(maxlen=2048)
        self.overflow = False
        self.error = None
        self.pipeline = rs.pipeline()
        cfg = rs.config()
        if camera_serial:
            cfg.enable_device(camera_serial)
        # Explicit motion streams only: no color, infrared or depth stream.
        cfg.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)
        cfg.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 250)
        profile = self.pipeline.start(cfg, self.callback)
        self.name = profile.get_device().get_info(rs.camera_info.name)

    def callback(self, frame):
        try:
            # A callback may deliver one motion frame or a composite frame.
            frames = list(frame.as_frameset()) if frame.is_frameset() else [frame]
            for item in frames:
                if not item.is_motion_frame():
                    continue
                kind = item.get_profile().stream_type()
                if kind not in (self.rs.stream.gyro, self.rs.stream.accel):
                    continue
                v = item.as_motion_frame().get_motion_data()
                vector = (v.x, v.y, v.z)
                if not all(math.isfinite(x) for x in vector):
                    raise ValueError("Non-finite IMU data")
                event = ("gyro" if kind == self.rs.stream.gyro else "accel",
                         item.get_timestamp() / 1000, vector, time.perf_counter())
                with self.lock:
                    if len(self.events) == self.events.maxlen:
                        self.overflow = True
                    self.events.append(event)
        except Exception as error:
            self.error = str(error)

    def drain(self):
        with self.lock:
            if self.overflow or self.error:
                raise RuntimeError(self.error or "IMU queue overflow")
            events = list(self.events)
            self.events.clear()
        return events

    def close(self):
        self.pipeline.stop()


class DemoIMU:
    """Illustrative pendulum, not a hardware validation or parameter identification."""
    name = "DEMO (no hardware access)"

    def __init__(self):
        self.angle = self.rate = self.manual_deg = 0.0
        self.previous = time.perf_counter()
        self.acceleration = 0.0
        self.running = False

    def drain(self):
        now = time.perf_counter()
        dt = min(now - self.previous, 0.01)
        self.previous = now
        if self.running:
            angular_accel = (9.80665 * math.sin(self.angle) -
                             self.acceleration * math.cos(self.angle)) / 0.28 - 0.1 * self.rate
            self.rate += angular_accel * dt
            self.angle += self.rate * dt
        else:
            self.angle = math.radians(self.manual_deg)
            self.rate = 0.0
        a = (0.0, 9.80665 * math.cos(self.angle), -9.80665 * math.sin(self.angle))
        return [("accel", now, a, now), ("gyro", now, (self.rate, 0.0, 0.0), now)]

    def close(self):
        pass


class MegaLink:
    def __init__(self, port=None):
        import serial
        from serial.tools import list_ports
        if port is None:
            candidates = [p.device for p in list_ports.comports()
                          if (p.vid in (0x2341, 0x2A03) or
                              "mega" in (p.description or "").lower())]
            if len(candidates) != 1:
                available = ", ".join(p.device for p in list_ports.comports()) or "none"
                raise RuntimeError(f"Select Mega with --port COMx. Available ports: {available}")
            port = candidates[0]
        self.port = port
        self.serial = serial.Serial(port, 115200, timeout=0, write_timeout=0.02)
        self.buffer = bytearray()
        self.session = self.sequence = 0
        self.last_rx = 0.0
        self.last_status = None
        self.sent = {}
        self.ack_ms = 0.0
        self.max_steps = 0
        try:
            time.sleep(2.0)  # Mega USB connection resets the board.
            self.serial.reset_input_buffer()
            end = time.perf_counter() + 2.0
            next_hello = 0.0
            while time.perf_counter() < end:
                now = time.perf_counter()
                if now >= next_hello:
                    self.write("H")
                    next_hello = now + 0.15
                for line in self.lines():
                    words = line.split()
                    if len(words) == 3 and words[0] == "BAL1":
                        self.max_steps, timeout_ms = map(int, words[1:])
                        if self.max_steps != 2000 or timeout_ms != 120:
                            raise RuntimeError("Unexpected BalanceMega firmware limits")
                        self.stop()
                        return
                time.sleep(0.005)
            raise RuntimeError("BalanceMega firmware not found. Upload BalanceMega.ino first")
        except Exception:
            self.serial.close()
            raise

    def write(self, text):
        data = (text + "\n").encode("ascii")
        if self.serial.write(data) != len(data):
            raise RuntimeError("Incomplete USB serial write")

    def lines(self):
        available = self.serial.in_waiting
        if available > 4096:
            raise RuntimeError("USB serial input backlog")
        self.buffer.extend(self.serial.read(available))
        if len(self.buffer) > 4096:
            raise RuntimeError("Malformed USB serial response")
        result = []
        while b"\n" in self.buffer:
            line, _, remainder = self.buffer.partition(b"\n")
            self.buffer = bytearray(remainder)
            result.append(line.decode("ascii", errors="replace").strip())
        return result

    def poll(self, now):
        for line in self.lines():
            fields = line.split()
            if len(fields) != 7 or fields[0] != "T":
                continue
            session, seq, armed, fault, left, right = map(int, fields[1:])
            self.last_rx = now
            self.last_status = (session, seq, armed, fault, left, right)
            if session == self.session and seq in self.sent:
                self.ack_ms = (now - self.sent[seq]) * 1000
                self.sent = {key: value for key, value in self.sent.items() if key > seq}

    def arm(self):
        previous = self.session
        while self.session == previous:
            self.session = secrets.randbelow(2**31 - 1) + 1
        self.sequence = 0
        self.sent.clear()
        self.last_status = None
        self.write(f"A {self.session}")

    def speed(self, left, right):
        self.sequence += 1
        if self.sequence >= 2**32:
            raise RuntimeError("Sequence exhausted; rearm")
        self.write(f"V {self.session} {self.sequence} {left} {right}")
        self.sent[self.sequence] = time.perf_counter()

    def stop(self):
        self.write("S")
        self.sent.clear()

    def close(self):
        try:
            self.stop()
        finally:
            self.serial.close()


class BalanceEngine(threading.Thread):
    def __init__(self, args, settings):
        super().__init__(name="balance-control", daemon=True)
        self.args, self.settings = args, settings
        self.actions = Queue(maxsize=64)
        self.quit = threading.Event()
        self.lock = threading.Lock()
        self.snapshot = {}
        self.estimator, self.pid = TiltEstimator(), BalancePID()
        self.source = self.link = None
        self.armed = False
        self.directions_checked = False
        self.state = "Connecting..."
        self.note = "Hold upright, then capture the reference"
        self.last_gyro = self.last_accel = 0.0
        self.device_ts = {"gyro": None, "accel": None}
        self.ui_ping = time.perf_counter()
        self.collecting = None
        self.jog_until = 0.0
        self.jog_side = None
        self.arm_time = 0.0
        self.saturation_since = None
        self.timing_misses = 0
        self.saturation_count = 0
        self.peak_tilt = self.peak_loop_ms = 0.0
        self.log_file = self.log_writer = None
        self.log_path = None
        self.last_log = 0.0
        self.last_left = self.last_right = 0
        self.last_fault = "none"
        self.fault_count = 0

    def submit(self, kind, data=None):
        try:
            self.actions.put_nowait((kind, data))
        except Full:
            self.quit.set()  # Action backlog is a fault, never ignore STOP.

    def publish(self, dt=0):
        now = time.perf_counter()
        with self.lock:
            self.snapshot = dict(
                time=now, state=self.state, note=self.note, armed=self.armed,
                pitch=math.degrees(self.estimator.angle), rate=math.degrees(self.estimator.rate),
                trim=self.settings.trim_deg, p=self.pid.p, i=self.pid.i, d=self.pid.d,
                accel=self.pid.acceleration, speed=self.pid.speed,
                left=self.last_left, right=self.last_right, saturated=self.pid.saturated,
                gyro_age_ms=(now - self.last_gyro) * 1000 if self.last_gyro else -1,
                accel_age_ms=(now - self.last_accel) * 1000 if self.last_accel else -1,
                ack_ms=self.link.ack_ms if self.link else 0,
                loop_ms=dt * 1000, calibrated=self.estimator.axis is not None,
                source=self.source.name if self.source else "unavailable",
                port=self.link.port if self.link else "no motor connection",
                directions_checked=self.directions_checked,
                effective_speed=self.settings.effective_speed(self.link.max_steps if self.link else 2000),
                log=str(self.log_path) if self.log_file else "off",
            )

    def disarm(self, reason, fault=False):
        self.armed = False
        self.jog_until = 0.0
        self.pid.reset()
        self.last_left = self.last_right = 0
        self.saturation_since = None
        if self.link:
            self.link.stop()
        self.state = "FAULT / stopped" if fault else "Disarmed"
        self.note = reason
        if fault:
            self.last_fault = reason
            self.fault_count += 1

    def start_log(self):
        if self.log_file:
            return
        if not self.snapshot:
            self.publish()
        directory = ROOT / "balance_logs"
        directory.mkdir(exist_ok=True)
        self.log_path = directory / f"balance_{datetime.now():%Y%m%d_%H%M%S_%f}.csv"
        self.log_file = self.log_path.open("w", newline="", encoding="utf-8")
        self.log_writer = csv.DictWriter(self.log_file, fieldnames=list(self.snapshot))
        self.log_writer.writeheader()
        self.note = f"Logging to {self.log_path.name}"

    def close_log(self):
        if self.log_file:
            self.log_file.close()
            self.log_file = self.log_writer = None

    def handle_action(self, kind, data, now):
        if kind == "ping":
            self.ui_ping = now
        elif kind == "stop":
            self.disarm("Stopped by user")
            self.collecting = None
        elif kind in ("reference", "forward"):
            self.disarm("Hold this pose still while sampling")
            if kind == "forward" and self.estimator.reference is None:
                raise ValueError("Capture upright first")
            if kind == "reference":
                self.estimator = TiltEstimator()
                self.directions_checked = False
            self.collecting = dict(kind=kind, end=now + 2.0, accel=[], gyro=[])
            self.state = "Sampling upright" if kind == "reference" else "Sampling forward tilt"
        elif kind == "settings":
            updated = Settings.from_dict(data)
            hardware_fields = ("left_sign", "right_sign", "microsteps", "full_steps", "wheel_diameter_mm")
            if self.armed or self.jog_until > now:
                raise ValueError("Disarm before applying settings")
            if any(getattr(updated, x) != getattr(self.settings, x) for x in hardware_fields):
                self.directions_checked = False
            self.settings = updated
            self.pid.reset()
            self.state = "Disarmed"
            self.note = "Settings applied; arm explicitly when ready"
        elif kind == "directions":
            self.directions_checked = bool(data)
        elif kind == "arm":
            if self.collecting or self.jog_until > now:
                raise ValueError("Finish calibration or wheel test first")
            if self.estimator.axis is None:
                raise ValueError("Capture upright, then teach forward tilt first")
            if not self.directions_checked:
                raise ValueError("Test both wheels and confirm their forward directions first")
            if abs(math.degrees(self.estimator.angle) - self.settings.trim_deg) > 4:
                raise ValueError("Return within 4 degrees of the upright reference before arming")
            if abs(self.estimator.rate) > math.radians(12):
                raise ValueError("Hold still before arming")
            self.check_freshness(now)
            self.pid.reset()
            if self.link:
                self.link.arm()
            self.armed, self.arm_time = True, now
            self.state = "BALANCING" if self.link else "PREVIEW (no motor output)"
            self.note = "Keep a catch support ready; gains need physical tuning"
        elif kind == "jog":
            if self.armed or self.collecting:
                raise ValueError("Disarm and finish calibration before testing a wheel")
            if not self.link:
                raise ValueError("Wheel tests require the live Mega connection")
            self.check_freshness(now)
            self.link.arm()
            self.jog_until, self.jog_side, self.arm_time = now + 0.5, data, now
            self.state = f"Testing {data} wheel (0.5 s)"
        elif kind == "demo_tilt" and isinstance(self.source, DemoIMU):
            self.source.manual_deg = float(data)
        elif kind == "record":
            self.start_log() if data else self.close_log()

    def check_freshness(self, now):
        if now - self.last_gyro > GYRO_MAX_AGE or now - self.last_accel > ACCEL_MAX_AGE:
            raise ValueError("IMU data is stale or missing")
        if self.link and now - self.link.last_rx > SERIAL_MAX_AGE:
            raise ValueError("Mega status is stale or missing")

    def process_events(self, now):
        for kind, ts, vector, arrival in self.source.drain():
            if now - arrival > 0.05:
                raise ValueError("IMU processing backlog; stop and recalibrate")
            previous = self.device_ts[kind]
            if ts == previous:
                continue  # Duplicate frames cannot refresh the freshness watchdog.
            if previous is not None and ts < previous:
                self.device_ts[kind] = ts
                raise ValueError("IMU timestamp reset; recalibrate")
            self.device_ts[kind] = ts
            if self.collecting:
                self.collecting[kind].append(vector)
            if kind == "gyro":
                self.last_gyro = arrival
                self.estimator.gyro(vector, ts)
            else:
                self.last_accel = arrival
                self.estimator.accel(vector, ts, self.settings.filter_tau_s)
        if self.collecting and now >= self.collecting["end"]:
            collected, self.collecting = self.collecting, None
            gravity, bias = stationary_reference(collected["accel"], collected["gyro"])
            if collected["kind"] == "reference":
                self.estimator.set_reference(gravity, bias)
                self.note = "Upright captured. Tilt FORWARD 5-25 degrees around axle; teach forward"
            else:
                angle = self.estimator.teach_forward(gravity)
                self.note = f"Forward axis learned ({angle:.1f} degrees). Return upright, then test wheels"
            self.state = "Disarmed"

    def run(self):
        try:
            self.settings.validate()
            self.source = DemoIMU() if self.args.demo else RealSenseIMU(self.args.camera_serial)
            if not (self.args.demo or self.args.imu_only):
                self.link = MegaLink(self.args.port)
            self.state = "Disarmed"
            previous = next_tick = time.perf_counter()
            while not self.quit.is_set():
                now = time.perf_counter()
                try:
                    if self.link:
                        self.link.poll(now)
                    self.process_events(now)
                    for _ in range(12):
                        try:
                            kind, data = self.actions.get_nowait()
                        except Empty:
                            break
                        self.handle_action(kind, data, now)
                    if now < next_tick:
                        time.sleep(min(0.002, next_tick - now))
                        continue
                    dt, previous = now - previous, now
                    next_tick = now + 1 / CONTROL_HZ
                    self.peak_loop_ms = max(self.peak_loop_ms, dt * 1000)
                    if dt > 0.02:
                        self.timing_misses += 1
                    self.peak_tilt = max(self.peak_tilt, abs(math.degrees(self.estimator.angle)))
                    active = self.armed or self.jog_until > now
                    if active:
                        self.check_freshness(now)
                        if now - self.ui_ping > 0.35:
                            raise ValueError("Dashboard heartbeat lost")
                        if dt > 0.05:
                            raise ValueError("Control loop exceeded 50ms")
                        if self.link:
                            status = self.link.last_status
                            if status and status[0] == self.link.session and (not status[2] or status[3]):
                                raise ValueError(f"Mega disarmed / fault code {status[3]}")
                            if now - self.arm_time > 0.15 and (not status or status[0] != self.link.session):
                                raise ValueError("Mega did not acknowledge arming")
                            if self.link.sent and now - min(self.link.sent.values()) > 0.12:
                                raise ValueError("Mega command acknowledgement is delayed")
                    if self.armed:
                        if abs(math.degrees(self.estimator.angle)) > self.settings.fall_deg:
                            raise ValueError("Fall angle exceeded; stopped")
                        left, right = self.pid.update(self.estimator.angle, self.estimator.rate, dt,
                                                      self.settings, self.link.max_steps if self.link else 2000)
                        if self.pid.saturated:
                            self.saturation_count += 1
                            if self.saturation_since is None:
                                self.saturation_since = now
                            if now - self.saturation_since > 0.4:
                                raise ValueError("Output saturated for 0.4s; speed/torque/gains may be insufficient")
                        else:
                            self.saturation_since = None
                        self.last_left, self.last_right = left, right
                        if self.link:
                            self.link.speed(left, right)
                    elif self.jog_until:
                        if now >= self.jog_until:
                            self.disarm("Wheel test complete. Verify that this wheel drives the robot forward")
                        else:
                            steps = min(120, round(0.025 * self.settings.steps_per_m))
                            self.last_left = steps * self.settings.left_sign if self.jog_side == "left" else 0
                            self.last_right = steps * self.settings.right_sign if self.jog_side == "right" else 0
                            self.link.speed(self.last_left, self.last_right)
                    if isinstance(self.source, DemoIMU):
                        self.source.running = self.armed
                        self.source.acceleration = self.pid.acceleration
                    self.publish(dt)
                    if self.log_writer and now - self.last_log >= 0.02:
                        self.log_writer.writerow(self.snapshot)
                        self.last_log = now
                except ValueError as error:
                    self.collecting = None
                    self.disarm(str(error), fault=True)
                    self.publish()
                    # Sensor timing faults invalidate calibration. User-input
                    # errors retain it and still force a disarm.
                    if any(x in str(error).lower() for x in ("timestamp", "backlog")):
                        self.estimator = TiltEstimator()
                    time.sleep(0.002)
        except Exception as error:
            self.armed = False
            self.pid.reset()
            self.last_left = self.last_right = 0
            self.state = "Connection / runtime error"
            self.note = str(error)
            self.last_fault = str(error)
            self.fault_count += 1
            self.publish()
        finally:
            for resource in (self.link, self.source):
                if resource:
                    try:
                        resource.close()
                    except Exception:
                        pass  # Firmware independently stops on lost commands.
            self.close_log()
            print(f"Run diagnostics: peak tilt {self.peak_tilt:.1f} deg; "
                  f"peak loop {self.peak_loop_ms:.1f} ms; loops >20ms {self.timing_misses}; "
                  f"saturated updates {self.saturation_count}.")
            print(f"Fault events: {self.fault_count}; last fault: {self.last_fault}")
            print("No encoders: actual wheel speed, missed steps, and position drift were not measured.")
            if self.log_path:
                print(f"CSV: {self.log_path}")


def dashboard(args, settings):
    import pyqtgraph as pg
    from PySide6 import QtCore, QtGui, QtWidgets

    pg.setConfigOptions(antialias=False, background="#171a21", foreground="#d7dce5")
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
    engine = BalanceEngine(args, settings)

    class Window(QtWidgets.QMainWindow):
        def closeEvent(self, event):
            engine.quit.set()
            engine.join(timeout=1.0)
            event.accept()

    window = Window()
    window.setWindowTitle("Two-Wheel IMU Balance — " + ("DEMO" if args.demo else "D435if / Mega"))
    window.resize(1380, 920)
    central = QtWidgets.QWidget()
    window.setCentralWidget(central)
    layout = QtWidgets.QVBoxLayout(central)
    status = QtWidgets.QLabel("Connecting...")
    status.setWordWrap(True)
    status.setStyleSheet("font-size: 16px; padding: 6px;")
    layout.addWidget(status)
    instructions = QtWidgets.QLabel(
        "1. Hold upright, capture reference (2s).  2. Hold FORWARD tilt 5–25°, teach forward (2s).  "
        "3. Return upright; lift wheels and test each.  4. Confirm directions; support robot and arm."
    )
    instructions.setWordWrap(True)
    layout.addWidget(instructions)
    buttons = QtWidgets.QHBoxLayout()
    layout.addLayout(buttons)

    def button(label, kind, data=None):
        b = QtWidgets.QPushButton(label)
        b.clicked.connect(lambda checked=False: engine.submit(kind, data))
        buttons.addWidget(b)
        return b

    button("Capture upright", "reference")
    button("Teach forward tilt", "forward")
    button("Test left", "jog", "left")
    button("Test right", "jog", "right")
    arm_button = button("ARM", "arm")
    stop = button("STOP [Space]", "stop")
    stop.setStyleSheet("background:#a82535; color:white; font-weight:bold; padding:8px")
    shortcut = QtGui.QShortcut(QtGui.QKeySequence("Space"), window)
    shortcut.activated.connect(lambda: engine.submit("stop"))
    direction_box = QtWidgets.QCheckBox("Both wheel tests move the robot forward")
    direction_box.toggled.connect(lambda value: engine.submit("directions", value))
    layout.addWidget(direction_box)

    body = QtWidgets.QHBoxLayout()
    layout.addLayout(body, 1)
    panel = QtWidgets.QWidget()
    panel.setMaximumWidth(370)
    form = QtWidgets.QFormLayout(panel)
    body.addWidget(panel)
    inputs = {}
    definitions = (
        ("kp", "P (m/s² per rad)", 0, 200, 3),
        ("ki", "I (m/s² per rad·s)", 0, 100, 3),
        ("kd", "D (m/s² per rad/s)", 0, 40, 3),
        ("trim_deg", "Balance trim (degrees)", -5, 5, 2),
        ("max_speed_m_s", "Max speed (m/s)", 0.02, 0.8, 3),
        ("max_accel_m_s2", "Max acceleration (m/s²)", 0.1, 10, 2),
        ("filter_tau_s", "IMU filter time (s)", 0.1, 10, 2),
        ("integral_limit", "Integral limit (rad·s)", 0, 1, 3),
        ("fall_deg", "Fall cutoff (degrees)", 8, 35, 1),
        ("wheel_diameter_mm", "Wheel diameter (mm)", 20, 400, 1),
    )
    for key, label, low, high, decimals in definitions:
        widget = QtWidgets.QDoubleSpinBox()
        widget.setRange(low, high)
        widget.setDecimals(decimals)
        widget.setSingleStep(10 ** (-min(decimals, 2)))
        widget.setValue(getattr(settings, key))
        inputs[key] = widget
        form.addRow(label, widget)
    steps_box = QtWidgets.QSpinBox()
    steps_box.setRange(1, 1000)
    steps_box.setValue(settings.full_steps)
    inputs["full_steps"] = steps_box
    form.addRow("Full steps / revolution", steps_box)
    for key, label, values in (("microsteps", "Microsteps (match wiring)", (1, 2, 4, 8, 16)),
                               ("left_sign", "Left forward direction", (1, -1)),
                               ("right_sign", "Right forward direction", (1, -1))):
        widget = QtWidgets.QComboBox()
        for value in values:
            widget.addItem(str(value), value)
        widget.setCurrentIndex(values.index(getattr(settings, key)))
        inputs[key] = widget
        form.addRow(label, widget)

    def read_settings():
        values = {key: (widget.currentData() if isinstance(widget, QtWidgets.QComboBox)
                        else widget.value()) for key, widget in inputs.items()}
        return Settings.from_dict(values)

    def apply():
        try:
            engine.submit("settings", read_settings().to_dict())
        except ValueError as error:
            status.setText(str(error))

    apply_button = QtWidgets.QPushButton("Apply tuning (while disarmed)")
    apply_button.clicked.connect(apply)
    form.addRow(apply_button)
    save_button, load_button = QtWidgets.QPushButton("Save settings"), QtWidgets.QPushButton("Load settings")
    form.addRow(save_button, load_button)

    def save():
        try:
            data = read_settings().to_dict()
            args.settings.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
            engine.submit("settings", data)
        except (ValueError, OSError) as error:
            QtWidgets.QMessageBox.warning(window, "Settings", str(error))

    def load():
        try:
            loaded = Settings.from_dict(json.loads(args.settings.read_text(encoding="utf-8")))
            for key, widget in inputs.items():
                if isinstance(widget, QtWidgets.QComboBox):
                    widget.setCurrentIndex(widget.findData(getattr(loaded, key)))
                else:
                    widget.setValue(getattr(loaded, key))
            engine.submit("settings", loaded.to_dict())
        except (ValueError, OSError) as error:
            QtWidgets.QMessageBox.warning(window, "Settings", str(error))

    save_button.clicked.connect(save)
    load_button.clicked.connect(load)
    record = QtWidgets.QCheckBox("Record CSV (50 Hz)")
    record.toggled.connect(lambda enabled: engine.submit("record", enabled))
    form.addRow(record)
    if args.demo:
        demo_tilt = QtWidgets.QDoubleSpinBox()
        demo_tilt.setRange(-30, 30)
        demo_tilt.setSuffix("°")
        demo_tilt.valueChanged.connect(lambda value: engine.submit("demo_tilt", value))
        form.addRow("Demo tilt (disarmed)", demo_tilt)
        direction_box.setText("Demo: allow simulated controller")
    limits = QtWidgets.QLabel()
    limits.setWordWrap(True)
    form.addRow(limits)
    caveat = QtWidgets.QLabel("Speed and step rate are COMMANDS. No encoder measurement.\n"
                              "Initial gains are a starting point; support the robot during tuning.")
    caveat.setWordWrap(True)
    form.addRow(caveat)
    plots = pg.GraphicsLayoutWidget()
    body.addWidget(plots, 1)
    curves, histories = {}, {key: deque(maxlen=1000) for key in
                             ("time", "pitch", "trim", "rate", "speed", "accel", "p", "i", "d",
                              "left", "right", "loop_ms", "gyro_age_ms", "ack_ms")}
    groups = (
        ("Pitch / upright reference", "deg", ("pitch", "trim")),
        ("Angular velocity", "deg/s", ("rate",)),
        ("Commanded wheel speed", "m/s", ("speed",)),
        ("PID acceleration contributions", "m/s²", ("p", "i", "d", "accel")),
        ("Commanded motor pulse rates", "steps/s", ("left", "right")),
        ("Timing / freshness", "ms", ("loop_ms", "gyro_age_ms", "ack_ms")),
    )
    colors = ("#53a8ff", "#ff9f43", "#59d98e", "#bf75ff")
    for index, (title, units, keys) in enumerate(groups):
        plot = plots.addPlot(row=index // 2, col=index % 2, title=title)
        plot.setLabel("left", units)
        plot.setLabel("bottom", "Recent time", units="s")
        plot.showGrid(x=True, y=True, alpha=0.2)
        plot.addLegend()
        for j, key in enumerate(keys):
            curves[key] = plot.plot(pen=pg.mkPen(colors[j], width=2), name=key)

    last_sample = 0.0

    def refresh():
        nonlocal last_sample
        engine.submit("ping")
        with engine.lock:
            sample = dict(engine.snapshot)
        if not sample:
            return
        if sample["time"] != last_sample:
            last_sample = sample["time"]
            for key in histories:
                histories[key].append(sample[key])
            times = [value - sample["time"] for value in histories["time"]]
            for key, curve in curves.items():
                curve.setData(times, list(histories[key]))
        status.setText(f"{sample['state']} | {sample['source']} | {sample['port']}\n"
                       f"{sample['note']}\n"
                       f"Pitch {sample['pitch']:+.2f}° | gyro age {sample['gyro_age_ms']:.0f}ms | "
                       f"ACK {sample['ack_ms']:.0f}ms | {'SATURATED' if sample['saturated'] else 'output within limits'}")
        active = sample["armed"] or sample["state"].startswith("Testing")
        apply_button.setEnabled(not active)
        load_button.setEnabled(not active)
        save_button.setEnabled(not active)
        for widget in inputs.values():
            widget.setEnabled(not active)
        arm_button.setEnabled(sample["calibrated"] and not active and engine.is_alive())
        if direction_box.isChecked() != sample["directions_checked"]:
            direction_box.blockSignals(True)
            direction_box.setChecked(sample["directions_checked"])
            direction_box.blockSignals(False)
        limits.setText(f"Effective speed cap: {sample['effective_speed']:.3f} m/s\n"
                       "Firmware cap: 2000 steps/s per wheel\n"
                       f"Log: {sample['log']}")

    timer = QtCore.QTimer(window)
    timer.timeout.connect(refresh)
    timer.start(40)
    engine.start()
    window.show()
    app.exec()
    engine.quit.set()
    engine.join(timeout=3)


def inventory():
    missing = []
    for name in ("serial", "pyrealsense2", "PySide6", "pyqtgraph"):
        try:
            importlib.import_module(name)
            print(f"OK {name}")
        except ImportError:
            print(f"MISSING {name}")
            missing.append(name)
    if "serial" not in missing:
        from serial.tools import list_ports
        ports = list(list_ports.comports())
        print("Serial ports: " + (", ".join(f"{p.device} ({p.description})" for p in ports) or "none"))
    if "pyrealsense2" not in missing:
        import pyrealsense2 as rs
        devices = list(rs.context().query_devices())
        print("RealSense devices: " + (", ".join(d.get_info(rs.camera_info.name) for d in devices) or "none"))
    return 1 if missing else 0


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--demo", action="store_true")
    modes.add_argument("--imu-only", action="store_true")
    modes.add_argument("--check", action="store_true")
    parser.add_argument("--port", help="Mega USB COM port; auto-select only if exactly one Arduino is found")
    parser.add_argument("--camera-serial", help="Optional RealSense serial number")
    parser.add_argument("--settings", type=Path, default=ROOT / "balance_settings.json")
    args = parser.parse_args()
    if args.check:
        return inventory()
    settings = Settings()
    if args.settings.exists():
        try:
            settings = Settings.from_dict(json.loads(args.settings.read_text(encoding="utf-8")))
        except (OSError, ValueError) as error:
            parser.error(f"Invalid settings: {error}")
    try:
        dashboard(args, settings)
    except ImportError as error:
        print(f"Missing dependency: {error}. Install: python -m pip install -r balance-requirements.txt")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
