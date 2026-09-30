"""Follow a detected hand's center with a two-axis camera mount.

Press ``I`` at the gravity-resting pose to initialize Y and arm following.
Type ``d`` or ``s`` to disarm and stop, or ``a`` to re-arm. Close Arduino
IDE's Serial Monitor first. A PyQtGraph dashboard opens in a separate process.
"""

from pathlib import Path
from multiprocessing import Event as ProcessEvent, Process, Queue as ProcessQueue, freeze_support
from queue import Empty, Full, Queue
import csv
from datetime import datetime
import math
import re
import threading
import time

from PIDLiveDashboard import run_dashboard

try:
    import cv2
    import mediapipe as mp
    import serial
    from serial.tools import list_ports
except ModuleNotFoundError:
    print("Install dependencies with: python -m pip install -r requirements.txt")
    raise SystemExit(1)


# Camera settings
USB_CAMERA_NUMBER = 1
INTEGRATED_CAMERA_NUMBER = 0
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
PROCESSING_WIDTH = 640
MIRROR_IMAGE = True

# Arduino settings
SERIAL_PORT: str | None = None  # Set to something like "COM5" if needed.
SERIAL_BAUD_RATE = 9600
ARDUINO_RESET_WAIT_SECONDS = 2.0
AUTO_FOLLOW_ON_START = False
ASSUMED_READY_Z_DEGREES = 0.0
ASSUMED_READY_Y_DEGREES = -50.0

# Hand-center follower tuning
# Use your camera's real field-of-view specifications when known.
HORIZONTAL_FIELD_OF_VIEW_DEGREES = 70.0
VERTICAL_FIELD_OF_VIEW_DEGREES = 43.0
CENTER_DEADBAND_X_NORMALIZED = 0.05
CENTER_DEADBAND_Y_NORMALIZED = 0.05
HAND_CENTER_SMOOTHING = 0.12
HAND_CONFIRMATION_SECONDS = 0.50
NO_HAND_STOP_SECONDS = 0.50
CONTROL_FREQUENCY_HZ = 20.0
CONTROL_INTERVAL_SECONDS = 1.0 / CONTROL_FREQUENCY_HZ

# PID TUNING ---------------------------------------------------------------
# Z is horizontal/X and Y is vertical/Y. Each axis has an independent PID so
# it can be tuned for its different mass, gearing, friction, and balance.
Z_PROPORTIONAL_GAIN = 0.65
Z_INTEGRAL_GAIN = 0.05
Z_DERIVATIVE_GAIN = 0.08
Y_PROPORTIONAL_GAIN = 0.65
Y_INTEGRAL_GAIN = 0.05
Y_DERIVATIVE_GAIN = 0.08
DERIVATIVE_FILTER_ALPHA = 0.08  # Lower = smoother/less noise; range 0..1.
INTEGRAL_LIMIT_DEGREE_SECONDS = 15.0
MAX_TRACKING_SPEED_DEGREES_PER_SECOND = 22.0

# There is no cumulative angle/travel limit. This only caps one update so a
# bad detection cannot request one huge motion.
MAX_CORRECTION_DEGREES_PER_UPDATE = 1.25
MIN_CORRECTION_DEGREES = 0.05

# CSV recording
CSV_OUTPUT_DIRECTORY = Path(__file__).with_name("pid_tracking_data")
CSV_RECORD_FREQUENCY_HZ = CONTROL_FREQUENCY_HZ
CSV_RECORD_INTERVAL_SECONDS = 1.0 / CSV_RECORD_FREQUENCY_HZ
START_RECORDING_KEYS = {ord("r"), ord("R")}
STOP_RECORDING_KEYS = {ord("t"), ord("T")}
INITIALIZE_Y_KEYS = {ord("i"), ord("I")}

# Fixed Z direction for immediate full horizontal PID control. Earlier tests
# showed that the original +1 direction increased error on this mechanism.
# Change this to +1.0 only if Z moves horizontally away from the hand.
Z_DIRECTION_SIGN = -1.0

# MediaPipe settings
HAND_MODEL_PATH = Path(__file__).with_name("hand_landmarker.task")
MINIMUM_HAND_DETECTION_CONFIDENCE = 0.30
MINIMUM_HAND_PRESENCE_CONFIDENCE = 0.30
MINIMUM_TRACKING_CONFIDENCE = 0.40

# Display settings and landmarks
HAND_COLOR = (0, 255, 0)
JOINT_COLOR = (0, 255, 0)
FINGERTIP_COLOR = (0, 165, 255)
WRIST_COLOR = (255, 255, 0)
PALM_CENTER_COLOR = (255, 0, 255)
TEXT_COLOR = (0, 255, 0)
WARNING_COLOR = (0, 165, 255)
INFO_COLOR = (255, 255, 0)
LANDMARK_KEY = ord("l")
PAUSE_KEY = ord("p")
EXIT_KEY = ord("q")
WRIST_LANDMARK = 0
FINGERTIP_LANDMARKS = {4, 8, 12, 16, 20}
PALM_CENTER_LANDMARKS = (0, 5, 9, 13, 17)
HAND_CONNECTIONS = (
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12),
    (9, 13), (13, 14), (14, 15), (15, 16),
    (13, 17), (17, 18), (18, 19), (19, 20), (0, 17),
)

AXIS_COMMAND_PATTERN = re.compile(
    r"^\s*([zy])\s*([+-]?(?:\d+(?:\.\d*)?|\.\d+))\s*$", re.IGNORECASE
)
POSITION_RESPONSE_PATTERN = re.compile(
    r"^POS\s+Z=([-+]?\d+(?:\.\d+)?)\s+Y=([-+]?\d+(?:\.\d+)?)$"
)
LEGACY_Z_POSITION_PATTERN = re.compile(
    r"^Z\s*/\s*Motor\s*1:\s*([-+]?\d+(?:\.\d+)?)\s*deg", re.IGNORECASE
)
LEGACY_Y_POSITION_PATTERN = re.compile(
    r"^Y\s*/\s*Motor\s*2:\s*([-+]?\d+(?:\.\d+)?)\s*deg", re.IGNORECASE
)


def print_controls(port_name: str) -> None:
    print()
    print("TWO-AXIS HAND-CENTER FOLLOWER")
    print("=" * 62)
    print(f"Arduino Mega: {port_name} at {SERIAL_BAUD_RATE} baud")
    print("STARTUP: press I once at the gravity-resting pose.")
    print("I labels current Y=+60, moves to Y=-50, then arms tracking.")
    print("  a      Arm automatic hand following")
    print("  d/s    Disarm following and smoothly stop")
    print("  z10    Manually set absolute Z to +10 degrees")
    print("  y-50   Manually set absolute Y to -50 degrees")
    print("  p      Print Arduino software positions")
    print("  zero   Declare the stationary pose Z=0, Y=0")
    print("  i      Initialize Y from gravity rest, then arm tracking")
    print(f"  r      Start a new {CSV_RECORD_FREQUENCY_HZ:.0f} Hz PID CSV recording")
    print("  t      Stop and save the CSV recording")
    print("  h      Arduino help     q  Quit")
    print("Camera window: I initialize, L landmarks, P pause, R record, T save, Q quit")
    print("-" * 62)
    print("NO software travel limits or encoder feedback.")
    print("Arduino assumes gravity rest is Y=+60, then moves to Y=-50.")
    print("Before arming, ensure both axes can move without a collision.")
    print("For the initial Z test, hold your off-center hand still.")
    print("Known direction: negative Y moves the camera upward.")
    print("=" * 62)


def open_camera() -> cv2.VideoCapture | None:
    for number, label in ((USB_CAMERA_NUMBER, "USB"), (INTEGRATED_CAMERA_NUMBER, "integrated")):
        camera = cv2.VideoCapture(number)
        if camera.isOpened():
            print(f"Using {label} camera.")
            return camera
        camera.release()
    return None


def find_arduino_port(manual_port: str | None = SERIAL_PORT) -> str:
    if manual_port:
        return manual_port
    ports = list(list_ports.comports())
    if not ports:
        raise RuntimeError("No serial ports were detected.")
    keywords = ("arduino", "usb serial", "usb-serial", "ch340", "wch", "cp210", "ftdi")

    def score(port) -> int:
        details = " ".join(str(v or "") for v in (port.device, port.description, port.manufacturer, port.hwid)).lower()
        value = sum(10 for keyword in keywords if keyword in details)
        return value - 100 if "bluetooth" in details or "bth" in details else value

    best = max(ports, key=score)
    if score(best) <= 0:
        available = ", ".join(port.device for port in ports)
        raise RuntimeError(f"Could not identify Arduino. Ports: {available}. Set SERIAL_PORT.")
    return best.device


class ArduinoConnection:
    def __init__(self, port_name: str) -> None:
        try:
            self.connection = serial.Serial(port_name, SERIAL_BAUD_RATE, timeout=0, write_timeout=1)
        except serial.SerialException as error:
            raise RuntimeError(
                f"Could not open {port_name}. Close Arduino Serial Monitor and check the COM port."
            ) from error
        self.port_name = port_name
        self.last_command = "NONE"
        self.last_response = "NONE"
        self.last_response_time: float | None = None
        self.last_acknowledgement_time: float | None = None
        self.first_unacknowledged_motion_time: float | None = None
        self.position_received = False
        self.z_position_degrees: float | None = None
        self.y_position_degrees: float | None = None
        self.z_velocity_estimate = 0.0
        self.y_velocity_estimate = 0.0
        self.z_acceleration_estimate = 0.0
        self.y_acceleration_estimate = 0.0
        self.last_position_sample_time: float | None = None
        self.previous_z_position: float | None = None
        self.previous_y_position: float | None = None
        self.z_moving = False
        self.y_moving = False
        self.y_completion_count = 0
        self.receive_buffer = bytearray()
        time.sleep(ARDUINO_RESET_WAIT_SECONDS)
        self.connection.reset_input_buffer()
        self.send_command("p")

    def update_position_estimates(self, z_position: float, y_position: float) -> None:
        """Estimate velocity and acceleration from Arduino position reports."""

        now = time.monotonic()
        if (
            self.last_position_sample_time is not None
            and self.previous_z_position is not None
            and self.previous_y_position is not None
        ):
            dt = now - self.last_position_sample_time
            if dt > 0.001:
                new_z_velocity = (z_position - self.previous_z_position) / dt
                new_y_velocity = (y_position - self.previous_y_position) / dt
                self.z_acceleration_estimate = (
                    new_z_velocity - self.z_velocity_estimate
                ) / dt
                self.y_acceleration_estimate = (
                    new_y_velocity - self.y_velocity_estimate
                ) / dt
                self.z_velocity_estimate = new_z_velocity
                self.y_velocity_estimate = new_y_velocity
        self.z_position_degrees = z_position
        self.y_position_degrees = y_position
        self.previous_z_position = z_position
        self.previous_y_position = y_position
        self.last_position_sample_time = now
        self.position_received = True

    def send_command(self, command: str) -> None:
        if not self.connection.is_open:
            return
        self.connection.write(f"{command}\n".encode("ascii"))
        self.connection.flush()
        self.last_command = command
        if command.startswith("z"):
            self.z_moving = True
        elif command.startswith("y"):
            self.y_moving = True
        if command.startswith(("z", "y")) and self.first_unacknowledged_motion_time is None:
            self.first_unacknowledged_motion_time = time.monotonic()
        print(f"Sent to Arduino: {command}")

    def read_responses(self) -> None:
        if self.connection.in_waiting:
            self.receive_buffer.extend(self.connection.read(self.connection.in_waiting))
        while b"\n" in self.receive_buffer:
            raw, _, remainder = self.receive_buffer.partition(b"\n")
            self.receive_buffer = bytearray(remainder)
            line = raw.decode("utf-8", errors="replace").strip()
            if not line:
                continue
            self.last_response = line
            self.last_response_time = time.monotonic()
            match = POSITION_RESPONSE_PATTERN.fullmatch(line)
            if match:
                self.update_position_estimates(
                    float(match.group(1)), float(match.group(2))
                )
            elif match := LEGACY_Z_POSITION_PATTERN.match(line):
                self.z_position_degrees = float(match.group(1))
                self.position_received = self.y_position_degrees is not None
            elif match := LEGACY_Y_POSITION_PATTERN.match(line):
                self.y_position_degrees = float(match.group(1))
                self.position_received = self.z_position_degrees is not None
                if self.position_received:
                    self.update_position_estimates(
                        self.z_position_degrees, self.y_position_degrees
                    )
            elif line.startswith("ACK "):
                self.last_acknowledgement_time = time.monotonic()
                self.first_unacknowledged_motion_time = None
            elif line == "DONE Z":
                self.z_moving = False
            elif line == "DONE Y":
                self.y_moving = False
                self.y_completion_count += 1
            # Older 9600-baud firmware does not use the new ACK prefix. Any
            # readable line still confirms that serial communication works.
            if self.first_unacknowledged_motion_time is not None:
                self.first_unacknowledged_motion_time = None
            print(f"Arduino: {line}")

    def close(self) -> None:
        if self.connection.is_open:
            try:
                self.send_command("s")
                time.sleep(0.20)
            finally:
                self.connection.close()


def normalize_terminal_command(raw: str) -> tuple[str | None, str]:
    command = raw.strip().lower()
    if not command:
        return None, ""
    if command == "a":
        return "__arm__", ""
    if command == "d":
        return "__disarm__", ""
    if command == "r":
        return "__start_recording__", ""
    if command == "t":
        return "__stop_recording__", ""
    if command == "i":
        return "__initialize_y__", ""
    if command in {"s", "p", "h", "zero"}:
        return command, ""
    match = AXIS_COMMAND_PATTERN.fullmatch(command)
    if not match:
        return None, "Use z10, z-10, y5, y-50, a, d, or s."
    axis, number = match.groups()
    degrees = float(number)
    if not math.isfinite(degrees):
        return None, "Target must be a finite number."
    return f"{axis.lower()}{degrees:g}", ""


def terminal_input_loop(command_queue: Queue[str], stop_event: threading.Event) -> None:
    while not stop_event.is_set():
        try:
            raw = input("Optional command (tracking is automatic)> ")
        except (EOFError, KeyboardInterrupt):
            command_queue.put("__quit__")
            return
        if raw.strip().lower() == "q":
            command_queue.put("__quit__")
            return
        command, error = normalize_terminal_command(raw)
        if error:
            print(f"Not sent: {error}")
        elif command:
            command_queue.put(command)


def create_hand_landmarker():
    if not HAND_MODEL_PATH.exists():
        raise FileNotFoundError(f"Hand model not found: {HAND_MODEL_PATH}")
    options = mp.tasks.vision.HandLandmarkerOptions(
        base_options=mp.tasks.BaseOptions(model_asset_path=str(HAND_MODEL_PATH)),
        running_mode=mp.tasks.vision.RunningMode.VIDEO,
        num_hands=1,
        min_hand_detection_confidence=MINIMUM_HAND_DETECTION_CONFIDENCE,
        min_hand_presence_confidence=MINIMUM_HAND_PRESENCE_CONFIDENCE,
        min_tracking_confidence=MINIMUM_TRACKING_CONFIDENCE,
    )
    return mp.tasks.vision.HandLandmarker.create_from_options(options)


def prepare_for_mediapipe(frame):
    height, width = frame.shape[:2]
    smaller = cv2.resize(frame, (PROCESSING_WIDTH, round(height * PROCESSING_WIDTH / width)), interpolation=cv2.INTER_AREA)
    rgb = cv2.cvtColor(smaller, cv2.COLOR_BGR2RGB)
    return mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)


def draw_hand_landmarks(frame, landmarks) -> None:
    height, width = frame.shape[:2]
    points = [(int(min(1, max(0, p.x)) * width), int(min(1, max(0, p.y)) * height)) for p in landmarks]
    for start, end in HAND_CONNECTIONS:
        cv2.line(frame, points[start], points[end], HAND_COLOR, 2, cv2.LINE_AA)
    for index, point in enumerate(points):
        color, radius = (WRIST_COLOR, 9) if index == WRIST_LANDMARK else ((FINGERTIP_COLOR, 7) if index in FINGERTIP_LANDMARKS else (JOINT_COLOR, 4))
        cv2.circle(frame, point, radius, color, -1, cv2.LINE_AA)


def palm_center(landmarks) -> tuple[float, float]:
    count = len(PALM_CENTER_LANDMARKS)
    x = sum(landmarks[i].x for i in PALM_CENTER_LANDMARKS) / count
    y = sum(landmarks[i].y for i in PALM_CENTER_LANDMARKS) / count
    return min(1, max(0, x)), min(1, max(0, y))


def image_offset_to_angle(offset: float, frame_pixels: int, fov_degrees: float) -> float:
    """Pinhole-camera conversion: angle = atan(pixel offset / focal length)."""
    focal_pixels = frame_pixels / (2 * math.tan(math.radians(fov_degrees) / 2))
    return math.degrees(math.atan(offset * frame_pixels / focal_pixels))


def clamp(value: float, maximum: float) -> float:
    return max(-maximum, min(maximum, value))


class HandCenterFollower:
    """Run a filtered PID position-error controller for both axes."""

    def __init__(self) -> None:
        self.z_direction_sign = Z_DIRECTION_SIGN
        self.reset(time.monotonic(), True)

    def reset(self, now: float, reset_direction: bool = False) -> None:
        self.filtered_center: tuple[float, float] | None = None
        self.hand_seen_since: float | None = None
        self.no_hand_since: float | None = now
        self.last_control_time = 0.0
        self.stop_sent = False
        self.status = "DISARMED"
        self.error_x = self.error_y = 0.0
        self.error_x_pixels = self.error_y_pixels = 0.0
        self.horizontal_angle_error = self.vertical_angle_error = 0.0
        self.z_integral = self.y_integral = 0.0
        self.previous_horizontal_error: float | None = None
        self.previous_vertical_error: float | None = None
        self.z_derivative = self.y_derivative = 0.0
        self.z_p_term = self.z_i_term = self.z_d_term = 0.0
        self.y_p_term = self.y_i_term = self.y_d_term = 0.0
        self.z_velocity = self.y_velocity = 0.0
        self.z_target: float | None = None
        self.y_target: float | None = None
        if reset_direction:
            self.z_direction_sign = Z_DIRECTION_SIGN
            self.z_direction_verified = True

    @property
    def z_direction_text(self) -> str:
        return "FIXED +Z" if self.z_direction_sign > 0 else "FIXED -Z"

    def update(self, center, width: int, height: int, now: float, armed: bool, arduino: ArduinoConnection) -> None:
        if not armed:
            self.status = "DISARMED"
            return
        if center is None:
            if self.no_hand_since is None:
                self.no_hand_since = now
            self.hand_seen_since = None
            self.filtered_center = None
            self.z_integral = self.y_integral = 0.0
            self.previous_horizontal_error = None
            self.previous_vertical_error = None
            self.z_derivative = self.y_derivative = 0.0
            self.z_velocity = self.y_velocity = 0.0
            self.status = "HAND LOST"
            if now - self.no_hand_since >= NO_HAND_STOP_SECONDS and not self.stop_sent:
                arduino.send_command("s")
                self.stop_sent = True
            return

        self.no_hand_since = None
        self.stop_sent = False
        if self.hand_seen_since is None:
            self.hand_seen_since = now
        if self.filtered_center is None:
            self.filtered_center = center
        else:
            old_x, old_y = self.filtered_center
            self.filtered_center = (
                old_x + HAND_CENTER_SMOOTHING * (center[0] - old_x),
                old_y + HAND_CENTER_SMOOTHING * (center[1] - old_y),
            )
        self.error_x = self.filtered_center[0] - 0.5
        self.error_y = self.filtered_center[1] - 0.5
        self.error_x_pixels = self.error_x * width
        self.error_y_pixels = self.error_y * height
        self.horizontal_angle_error = image_offset_to_angle(
            self.error_x, width, HORIZONTAL_FIELD_OF_VIEW_DEGREES
        )
        self.vertical_angle_error = image_offset_to_angle(
            self.error_y, height, VERTICAL_FIELD_OF_VIEW_DEGREES
        )

        if now - self.hand_seen_since < HAND_CONFIRMATION_SECONDS:
            self.status = "CONFIRMING HAND"
            return
        if now - self.last_control_time < CONTROL_INTERVAL_SECONDS:
            return
        dt = (
            CONTROL_INTERVAL_SECONDS
            if self.last_control_time == 0.0
            else min(0.25, now - self.last_control_time)
        )
        self.last_control_time = now

        # The Arduino starts both software positions at zero. If position
        # telemetry has not arrived, command from that same documented zero
        # instead of blocking forever in a WAITING state.
        if self.z_target is None:
            self.z_target = (
                ASSUMED_READY_Z_DEGREES
                if arduino.z_position_degrees is None
                else arduino.z_position_degrees
            )
        if self.y_target is None:
            self.y_target = (
                ASSUMED_READY_Y_DEGREES
                if arduino.y_position_degrees is None
                else arduino.y_position_degrees
            )

        x_active = abs(self.error_x) > CENTER_DEADBAND_X_NORMALIZED
        y_active = abs(self.error_y) > CENTER_DEADBAND_Y_NORMALIZED
        if x_active:
            self.z_integral = clamp(
                self.z_integral + self.horizontal_angle_error * dt,
                INTEGRAL_LIMIT_DEGREE_SECONDS,
            )
        else:
            self.z_integral = 0.0
        if y_active:
            self.y_integral = clamp(
                self.y_integral + self.vertical_angle_error * dt,
                INTEGRAL_LIMIT_DEGREE_SECONDS,
            )
        else:
            self.y_integral = 0.0

        raw_z_derivative = (
            0.0
            if self.previous_horizontal_error is None
            else (self.horizontal_angle_error - self.previous_horizontal_error) / dt
        )
        raw_y_derivative = (
            0.0
            if self.previous_vertical_error is None
            else (self.vertical_angle_error - self.previous_vertical_error) / dt
        )
        alpha = DERIVATIVE_FILTER_ALPHA
        self.z_derivative += alpha * (raw_z_derivative - self.z_derivative)
        self.y_derivative += alpha * (raw_y_derivative - self.y_derivative)
        self.previous_horizontal_error = self.horizontal_angle_error
        self.previous_vertical_error = self.vertical_angle_error

        # Z consumes horizontal/X error; Y consumes vertical/Y error. Both run
        # complete and independent P + I + filtered-D calculations every tick.
        self.z_p_term = Z_PROPORTIONAL_GAIN * self.horizontal_angle_error
        self.z_i_term = Z_INTEGRAL_GAIN * self.z_integral
        self.z_d_term = Z_DERIVATIVE_GAIN * self.z_derivative
        self.y_p_term = Y_PROPORTIONAL_GAIN * self.vertical_angle_error
        self.y_i_term = Y_INTEGRAL_GAIN * self.y_integral
        self.y_d_term = Y_DERIVATIVE_GAIN * self.y_derivative

        self.z_velocity = (
            self.z_direction_sign
            * clamp(
                self.z_p_term + self.z_i_term + self.z_d_term,
                MAX_TRACKING_SPEED_DEGREES_PER_SECOND,
            )
            if x_active else 0.0
        )
        # Image Y grows downward. A hand above center creates a negative
        # velocity, matching the known mechanism direction: -Y is upward.
        self.y_velocity = (
            clamp(
                self.y_p_term + self.y_i_term + self.y_d_term,
                MAX_TRACKING_SPEED_DEGREES_PER_SECOND,
            )
            if y_active else 0.0
        )

        sent = False
        if x_active:
            delta = clamp(
                self.z_velocity * dt, MAX_CORRECTION_DEGREES_PER_UPDATE
            )
            if abs(delta) >= MIN_CORRECTION_DEGREES:
                self.z_target += delta
                arduino.send_command(f"z{self.z_target:.3f}")
                sent = True
        if y_active:
            delta = clamp(
                self.y_velocity * dt, MAX_CORRECTION_DEGREES_PER_UPDATE
            )
            if abs(delta) >= MIN_CORRECTION_DEGREES:
                self.y_target += delta
                arduino.send_command(f"y{self.y_target:.3f}")
                sent = True

        self.status = (
            f"FULL Z(X) + Y(Y) PID AT {CONTROL_FREQUENCY_HZ:.0f} HZ"
            if sent else "CENTERED"
        )


class PIDDataRecorder:
    """Write controller inputs, PID terms, and motor telemetry at control rate."""

    FIELDNAMES = (
        "timestamp_iso", "elapsed_seconds", "hand_detected", "following_armed",
        "hand_x_pixels", "hand_y_pixels_top", "hand_y_pixels_bottom",
        "x_error_pixels", "y_error_pixels", "pixel_error_distance",
        "horizontal_angle_error_degrees", "vertical_angle_error_degrees",
        "z_p_term", "z_i_term", "z_d_term", "z_pid_velocity_command_deg_s",
        "y_p_term", "y_i_term", "y_d_term", "y_pid_velocity_command_deg_s",
        "z_target_degrees", "y_target_degrees",
        "z_reported_position_degrees", "y_reported_position_degrees",
        "z_estimated_velocity_deg_s", "y_estimated_velocity_deg_s",
        "z_estimated_acceleration_deg_s2", "y_estimated_acceleration_deg_s2",
        "z_direction_sign", "arduino_position_received", "last_arduino_command",
        "camera_fps",
    )

    def __init__(self) -> None:
        self.file = None
        self.writer = None
        self.path: Path | None = None
        self.started_at = 0.0
        self.last_record_time = 0.0
        self.row_count = 0

    @property
    def recording(self) -> bool:
        return self.file is not None

    def start(self) -> None:
        if self.recording:
            print(f"CSV recording is already active: {self.path}")
            return
        CSV_OUTPUT_DIRECTORY.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        self.path = CSV_OUTPUT_DIRECTORY / f"pid_tracking_{timestamp}.csv"
        self.file = self.path.open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.file, fieldnames=self.FIELDNAMES)
        self.writer.writeheader()
        self.started_at = time.monotonic()
        self.last_record_time = 0.0
        self.row_count = 0
        print(f"CSV recording STARTED: {self.path}")

    def stop(self) -> Path | None:
        if not self.recording:
            print("CSV recording is not active.")
            return self.path
        self.file.flush()
        self.file.close()
        saved_path = self.path
        self.file = None
        self.writer = None
        print(f"CSV recording SAVED ({self.row_count} rows): {saved_path}")
        return saved_path

    def maybe_record(
        self,
        now: float,
        hand_detected: bool,
        armed: bool,
        follower: HandCenterFollower,
        arduino: ArduinoConnection,
        frame_width: int,
        frame_height: int,
        fps: float,
    ) -> None:
        if not self.recording or now - self.last_record_time < CSV_RECORD_INTERVAL_SECONDS:
            return
        self.last_record_time = now
        if follower.filtered_center is None:
            hand_x = hand_y_top = hand_y_bottom = ""
        else:
            hand_x = follower.filtered_center[0] * frame_width
            hand_y_top = follower.filtered_center[1] * frame_height
            hand_y_bottom = frame_height - hand_y_top
        self.writer.writerow({
            "timestamp_iso": datetime.now().astimezone().isoformat(timespec="milliseconds"),
            "elapsed_seconds": now - self.started_at,
            "hand_detected": hand_detected,
            "following_armed": armed,
            "hand_x_pixels": hand_x,
            "hand_y_pixels_top": hand_y_top,
            "hand_y_pixels_bottom": hand_y_bottom,
            "x_error_pixels": follower.error_x_pixels,
            "y_error_pixels": follower.error_y_pixels,
            "pixel_error_distance": math.hypot(follower.error_x_pixels, follower.error_y_pixels),
            "horizontal_angle_error_degrees": follower.horizontal_angle_error,
            "vertical_angle_error_degrees": follower.vertical_angle_error,
            "z_p_term": follower.z_p_term,
            "z_i_term": follower.z_i_term,
            "z_d_term": follower.z_d_term,
            "z_pid_velocity_command_deg_s": follower.z_velocity,
            "y_p_term": follower.y_p_term,
            "y_i_term": follower.y_i_term,
            "y_d_term": follower.y_d_term,
            "y_pid_velocity_command_deg_s": follower.y_velocity,
            "z_target_degrees": "" if follower.z_target is None else follower.z_target,
            "y_target_degrees": "" if follower.y_target is None else follower.y_target,
            "z_reported_position_degrees": "" if arduino.z_position_degrees is None else arduino.z_position_degrees,
            "y_reported_position_degrees": "" if arduino.y_position_degrees is None else arduino.y_position_degrees,
            "z_estimated_velocity_deg_s": arduino.z_velocity_estimate,
            "y_estimated_velocity_deg_s": arduino.y_velocity_estimate,
            "z_estimated_acceleration_deg_s2": arduino.z_acceleration_estimate,
            "y_estimated_acceleration_deg_s2": arduino.y_acceleration_estimate,
            "z_direction_sign": follower.z_direction_sign,
            "arduino_position_received": arduino.position_received,
            "last_arduino_command": arduino.last_command,
            "camera_fps": fps,
        })
        self.row_count += 1
        if self.row_count % 12 == 0:
            self.file.flush()


def draw_target(frame, follower: HandCenterFollower) -> None:
    height, width = frame.shape[:2]
    cx, cy = width // 2, height // 2
    dx, dy = round(width * CENTER_DEADBAND_X_NORMALIZED), round(height * CENTER_DEADBAND_Y_NORMALIZED)
    centered = abs(follower.error_x) <= CENTER_DEADBAND_X_NORMALIZED and abs(follower.error_y) <= CENTER_DEADBAND_Y_NORMALIZED
    color = TEXT_COLOR if centered else WARNING_COLOR
    cv2.rectangle(frame, (cx - dx, cy - dy), (cx + dx, cy + dy), color, 2)
    cv2.drawMarker(frame, (cx, cy), color, cv2.MARKER_CROSS, 28, 2)

    # These arrows show the screen direction the mount is being asked to turn.
    # Their lengths represent pixel error and their thickness represents the
    # requested PID velocity. They remain useful without Arduino feedback.
    if abs(follower.error_x_pixels) > dx:
        arrow_x = int(math.copysign(min(190, max(45, abs(follower.error_x_pixels))), follower.error_x_pixels))
        thickness = 2 + round(4 * abs(follower.z_velocity) / MAX_TRACKING_SPEED_DEGREES_PER_SECOND)
        cv2.arrowedLine(frame, (cx, cy), (cx + arrow_x, cy), (255, 0, 255), thickness, cv2.LINE_AA, tipLength=0.18)
    if abs(follower.error_y_pixels) > dy:
        arrow_y = int(math.copysign(min(150, max(45, abs(follower.error_y_pixels))), follower.error_y_pixels))
        thickness = 2 + round(4 * abs(follower.y_velocity) / MAX_TRACKING_SPEED_DEGREES_PER_SECOND)
        cv2.arrowedLine(frame, (cx, cy), (cx, cy + arrow_y), (255, 0, 255), thickness, cv2.LINE_AA, tipLength=0.18)


def draw_status(frame, hand_detected: bool, follower: HandCenterFollower, armed: bool, initializing: bool, arduino: ArduinoConnection, recorder: PIDDataRecorder, fps: float) -> None:
    position = (
        "NO REPORT (PID USING STARTUP REFERENCE)"
        if not arduino.position_received
        else f"Z={arduino.z_position_degrees:+.2f}  Y={arduino.y_position_degrees:+.2f} deg"
    )
    no_ack = (
        arduino.first_unacknowledged_motion_time is not None
        and time.monotonic() - arduino.first_unacknowledged_motion_time > 0.75
    )
    if no_ack:
        serial_text = "SERIAL: COMMAND SENT BUT NO RESPONSE - CHECK COM PORT"
        serial_color = (0, 0, 255)
    elif arduino.position_received:
        serial_text = "SERIAL: ARDUINO POSITION FEEDBACK RECEIVED"
        serial_color = TEXT_COLOR
    else:
        serial_text = "SERIAL: NO POSITION REPORT YET"
        serial_color = WARNING_COLOR
    lines = (
        ("HAND: DETECTED" if hand_detected else "HAND: NOT DETECTED", TEXT_COLOR if hand_detected else WARNING_COLOR),
        (("INITIALIZING Y: +60 -> -50" if initializing else f"FOLLOW: {'ARMED' if armed else 'DISARMED'}  {follower.status}"), INFO_COLOR if initializing else (TEXT_COLOR if armed else WARNING_COLOR)),
        (f"PIXEL ERROR: X={follower.error_x_pixels:+.0f}px  Y={follower.error_y_pixels:+.0f}px", INFO_COLOR),
        (f"ANGLE ERROR: X={follower.horizontal_angle_error:+.1f}  Y={follower.vertical_angle_error:+.1f} deg", INFO_COLOR),
        (f"Z HORIZONTAL PID: P={follower.z_p_term:+.1f} I={follower.z_i_term:+.1f} D={follower.z_d_term:+.1f} OUT={follower.z_velocity:+.1f}", INFO_COLOR),
        (f"Y VERTICAL PID: P={follower.y_p_term:+.1f} I={follower.y_i_term:+.1f} D={follower.y_d_term:+.1f} OUT={follower.y_velocity:+.1f}", INFO_COLOR),
        (f"Z DIRECTION: {follower.z_direction_text}", INFO_COLOR),
        (f"ARDUINO: {position}", INFO_COLOR),
        (serial_text, serial_color),
        ((f"CSV: RECORDING {recorder.row_count} rows" if recorder.recording else "CSV: OFF (R=start, T=save)"), (0, 0, 255) if recorder.recording else INFO_COLOR),
        (f"LAST: {arduino.last_command}  PORT: {arduino.port_name}  FPS: {fps:.1f}", INFO_COLOR),
    )
    for index, (text, color) in enumerate(lines):
        cv2.putText(frame, text, (20, 35 + index * 32), cv2.FONT_HERSHEY_SIMPLEX, 0.60, color, 2, cv2.LINE_AA)


def main() -> None:
    port_name = find_arduino_port()
    arduino = ArduinoConnection(port_name)
    camera = open_camera()
    if camera is None:
        arduino.close()
        print("No camera detected.")
        return
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    print_controls(port_name)

    queue: Queue[str] = Queue()
    stop_event = threading.Event()
    threading.Thread(target=terminal_input_loop, args=(queue, stop_event), daemon=True).start()
    paused = False
    show_landmarks = True
    armed = AUTO_FOLLOW_ON_START
    frame = None
    result = None
    center = None
    last_timestamp_ms = 0
    previous_frame_time = time.perf_counter()
    fps = 0.0
    follower = HandCenterFollower()
    initialization_in_progress = False
    expected_y_completion_count = 0
    recorder = PIDDataRecorder()
    dashboard_queue = ProcessQueue(maxsize=100)
    dashboard_stop_event = ProcessEvent()
    dashboard_process = Process(
        target=run_dashboard,
        args=(dashboard_queue, dashboard_stop_event),
        daemon=True,
        name="pid-live-dashboard",
    )
    dashboard_process.start()
    dashboard_started_at = time.monotonic()
    dashboard_last_publish = 0.0
    if armed:
        print("Following ARMED automatically. Show your hand to the camera.")
        print("Hold it still and off-center briefly while Z learns its direction.")

    try:
        with create_hand_landmarker() as landmarker:
            while not stop_event.is_set():
                while True:
                    try:
                        command = queue.get_nowait()
                    except Empty:
                        break
                    if command == "__quit__":
                        stop_event.set()
                        break
                    if command == "__arm__":
                        armed = True
                        follower.reset(time.monotonic(), True)
                        arduino.send_command("p")
                        print("Following ARMED. Hold an off-center hand still for the Z test.")
                        continue
                    if command == "__disarm__":
                        armed = False
                        follower.reset(time.monotonic())
                        arduino.send_command("s")
                        print("Following DISARMED.")
                        continue
                    if command == "__start_recording__":
                        recorder.start()
                        continue
                    if command == "__stop_recording__":
                        recorder.stop()
                        continue
                    if command == "__initialize_y__":
                        armed = False
                        follower.reset(time.monotonic())
                        initialization_in_progress = True
                        expected_y_completion_count = arduino.y_completion_count + 1
                        arduino.send_command("i")
                        print("Y initialization requested: current pose +60 -> target -50.")
                        continue
                    if command == "s" or command.startswith(("z", "y")):
                        armed = False
                        follower.reset(time.monotonic())
                        if command.startswith(("z", "y")):
                            print("Manual command: following disarmed.")
                    arduino.send_command(command)
                if stop_event.is_set():
                    break

                if not paused:
                    success, frame = camera.read()
                    if not success:
                        print("Could not read camera frame.")
                        break
                    if MIRROR_IMAGE:
                        frame = cv2.flip(frame, 1)
                    last_timestamp_ms = max(last_timestamp_ms + 1, int(time.monotonic() * 1000))
                    result = landmarker.detect_for_video(prepare_for_mediapipe(frame), last_timestamp_ms)
                    center = palm_center(result.hand_landmarks[0]) if result.hand_landmarks else None
                    arduino.read_responses()
                    height, width = frame.shape[:2]
                    follower.update(center, width, height, time.monotonic(), armed, arduino)
                    current_time = time.perf_counter()
                    elapsed = current_time - previous_frame_time
                    previous_frame_time = current_time
                    measured = 1 / elapsed if elapsed > 0 else 0
                    fps = measured if fps == 0 else 0.9 * fps + 0.1 * measured
                    recorder.maybe_record(
                        time.monotonic(),
                        center is not None,
                        armed,
                        follower,
                        arduino,
                        width,
                        height,
                        fps,
                    )
                    dashboard_time = time.monotonic()
                    if dashboard_time - dashboard_last_publish >= CONTROL_INTERVAL_SECONDS:
                        dashboard_last_publish = dashboard_time
                        dashboard_sample = {
                            "time": dashboard_time - dashboard_started_at,
                            "x_error": follower.error_x_pixels,
                            "y_error": follower.error_y_pixels,
                            "z_output": follower.z_velocity,
                            "y_output": follower.y_velocity,
                            "z_p": follower.z_p_term,
                            "z_i": follower.z_i_term,
                            "z_d": follower.z_d_term,
                            "y_p": follower.y_p_term,
                            "y_i": follower.y_i_term,
                            "y_d": follower.y_d_term,
                            "z_target": 0.0 if follower.z_target is None else follower.z_target,
                            "y_target": ASSUMED_READY_Y_DEGREES if follower.y_target is None else follower.y_target,
                            "hand_detected": center is not None,
                            "serial_ok": arduino.position_received,
                            "fps": fps,
                        }
                        try:
                            dashboard_queue.put_nowait(dashboard_sample)
                        except Full:
                            # Never allow a slow/closed graph window to delay
                            # camera processing or motor control.
                            pass
                else:
                    arduino.read_responses()

                if (
                    initialization_in_progress
                    and arduino.y_completion_count >= expected_y_completion_count
                ):
                    initialization_in_progress = False
                    armed = True
                    follower.reset(time.monotonic(), True)
                    print("Y initialization complete. Hand following ARMED.")
                elif (
                    initialization_in_progress
                    and arduino.last_response.startswith(("REJECTED:", "INVALID:"))
                ):
                    initialization_in_progress = False
                    print(f"Y initialization failed: {arduino.last_response}")
                    if arduino.last_response.startswith("INVALID:"):
                        print(
                            "The Mega is running outdated firmware without the i command. "
                            "Quit Python, upload two_axis_pid_controller.ino, then restart."
                        )

                if frame is not None:
                    display = frame.copy()
                    hand_detected = bool(result and result.hand_landmarks)
                    draw_target(display, follower)
                    if show_landmarks and hand_detected:
                        draw_hand_landmarks(display, result.hand_landmarks[0])
                    if follower.filtered_center:
                        height, width = display.shape[:2]
                        point = (int(follower.filtered_center[0] * width), int(follower.filtered_center[1] * height))
                        cv2.circle(display, point, 12, PALM_CENTER_COLOR, -1, cv2.LINE_AA)
                        cv2.line(display, (width // 2, height // 2), point, PALM_CENTER_COLOR, 2)
                    draw_status(display, hand_detected, follower, armed, initialization_in_progress, arduino, recorder, fps)
                    cv2.imshow("Two-Axis Hand-Center Follower", display)

                key = cv2.waitKey(1) & 0xFF
                if key == LANDMARK_KEY:
                    show_landmarks = not show_landmarks
                elif key == PAUSE_KEY:
                    paused = not paused
                    if paused:
                        armed = False
                        follower.reset(time.monotonic())
                        arduino.send_command("s")
                    print("Paused; following disarmed." if paused else "Camera resumed.")
                elif key in START_RECORDING_KEYS:
                    recorder.start()
                elif key in STOP_RECORDING_KEYS:
                    recorder.stop()
                elif key in INITIALIZE_Y_KEYS:
                    armed = False
                    follower.reset(time.monotonic())
                    initialization_in_progress = True
                    expected_y_completion_count = arduino.y_completion_count + 1
                    arduino.send_command("i")
                    print("Y initialization requested: current pose +60 -> target -50.")
                elif key == EXIT_KEY:
                    stop_event.set()
    finally:
        stop_event.set()
        dashboard_stop_event.set()
        try:
            dashboard_queue.put_nowait(None)
        except Full:
            pass
        dashboard_process.join(timeout=2.0)
        if dashboard_process.is_alive():
            dashboard_process.terminate()
            dashboard_process.join(timeout=1.0)
        if recorder.recording:
            recorder.stop()
        camera.release()
        cv2.destroyAllWindows()
        arduino.close()


if __name__ == "__main__":
    freeze_support()
    main()
