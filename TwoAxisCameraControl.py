"""Follow a detected hand's center with a two-axis camera mount.

Following arms automatically when the program starts. Type ``d`` or ``s`` to
disarm and stop, or ``a`` to re-arm. Close Arduino IDE's Serial Monitor first.
"""

from pathlib import Path
from queue import Empty, Queue
import math
import re
import threading
import time

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
AUTO_FOLLOW_ON_START = True

# Hand-center follower tuning
# Use your camera's real field-of-view specifications when known.
HORIZONTAL_FIELD_OF_VIEW_DEGREES = 70.0
VERTICAL_FIELD_OF_VIEW_DEGREES = 43.0
CENTER_DEADBAND_X_NORMALIZED = 0.04
CENTER_DEADBAND_Y_NORMALIZED = 0.04
HAND_CENTER_SMOOTHING = 0.25
HAND_CONFIRMATION_SECONDS = 0.50
NO_HAND_STOP_SECONDS = 0.50
CONTROL_FREQUENCY_HZ = 12.0
CONTROL_INTERVAL_SECONDS = 1.0 / CONTROL_FREQUENCY_HZ

# PI output is a requested angular velocity. P reacts to current angular error;
# I accumulates persistent error so the mount continues through friction/load.
PROPORTIONAL_GAIN = 0.90
INTEGRAL_GAIN = 0.20
INTEGRAL_LIMIT_DEGREE_SECONDS = 30.0
MAX_TRACKING_SPEED_DEGREES_PER_SECOND = 18.0

# There is no cumulative angle/travel limit. This only caps one update so a
# bad detection cannot request one huge motion.
MAX_CORRECTION_DEGREES_PER_UPDATE = 2.0
MIN_CORRECTION_DEGREES = 0.05

# Z direction is learned after arming. Hold the hand still and clearly left or
# right while the camera makes this small test motion.
Z_DIRECTION_CALIBRATION_MIN_ERROR_PIXELS = 70.0
Z_DIRECTION_EVALUATION_SECONDS = 0.75
Z_DIRECTION_IMPROVEMENT_THRESHOLD_PIXELS = 12.0

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
    print("AUTOMATIC FOLLOWING STARTS ARMED.")
    print("  a      Arm automatic hand following")
    print("  d/s    Disarm following and smoothly stop")
    print("  z10    Manually set absolute Z to +10 degrees")
    print("  y-50   Manually set absolute Y to -50 degrees")
    print("  p      Print Arduino software positions")
    print("  zero   Declare the stationary pose Z=0, Y=0")
    print("  h      Arduino help     q  Quit")
    print("Camera window: L landmarks, P pause, Q quit")
    print("-" * 62)
    print("NO software travel limits. Startup zero is assumed, not measured.")
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
        self.z_moving = False
        self.y_moving = False
        self.receive_buffer = bytearray()
        time.sleep(ARDUINO_RESET_WAIT_SECONDS)
        self.connection.reset_input_buffer()
        self.send_command("p")

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
                self.z_position_degrees = float(match.group(1))
                self.y_position_degrees = float(match.group(2))
                self.position_received = True
            elif match := LEGACY_Z_POSITION_PATTERN.match(line):
                self.z_position_degrees = float(match.group(1))
                self.position_received = self.y_position_degrees is not None
            elif match := LEGACY_Y_POSITION_PATTERN.match(line):
                self.y_position_degrees = float(match.group(1))
                self.position_received = self.z_position_degrees is not None
            elif line.startswith("ACK "):
                self.last_acknowledgement_time = time.monotonic()
                self.first_unacknowledged_motion_time = None
            elif line == "DONE Z":
                self.z_moving = False
            elif line == "DONE Y":
                self.y_moving = False
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
    """Run a 12 Hz PI position-error controller for both camera axes."""

    def __init__(self) -> None:
        self.z_direction_sign = 1.0
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
        self.z_velocity = self.y_velocity = 0.0
        self.z_target: float | None = None
        self.y_target: float | None = None
        self.z_check_started_at: float | None = None
        self.z_check_initial_error_pixels = 0.0
        if reset_direction:
            self.z_direction_sign = 1.0
            self.z_direction_verified = False

    @property
    def z_direction_text(self) -> str:
        if self.z_check_started_at is not None:
            return "CHECKING WHILE MOVING"
        if not self.z_direction_verified:
            return "UNVERIFIED"
        return "NORMAL" if self.z_direction_sign > 0 else "REVERSED"

    def update(self, center, width: int, height: int, now: float, armed: bool, arduino: ArduinoConnection) -> None:
        if not armed:
            self.status = "DISARMED"
            return
        if center is None:
            if self.no_hand_since is None:
                self.no_hand_since = now
            self.hand_seen_since = None
            self.filtered_center = None
            self.z_check_started_at = None
            self.z_integral = self.y_integral = 0.0
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
            self.z_target = arduino.z_position_degrees or 0.0
        if self.y_target is None:
            self.y_target = arduino.y_position_degrees or 0.0

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

        self.z_velocity = (
            self.z_direction_sign
            * clamp(
                PROPORTIONAL_GAIN * self.horizontal_angle_error
                + INTEGRAL_GAIN * self.z_integral,
                MAX_TRACKING_SPEED_DEGREES_PER_SECOND,
            )
            if x_active else 0.0
        )
        # Image Y grows downward. A hand above center creates a negative
        # velocity, matching the known mechanism direction: -Y is upward.
        self.y_velocity = (
            clamp(
                PROPORTIONAL_GAIN * self.vertical_angle_error
                + INTEGRAL_GAIN * self.y_integral,
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

        # Check Z direction during normal PI motion instead of blocking motion
        # for a separate calibration. Hold the hand reasonably still while
        # UNVERIFIED/CHECKING is displayed.
        if not self.z_direction_verified and abs(self.error_x_pixels) >= Z_DIRECTION_CALIBRATION_MIN_ERROR_PIXELS:
            if self.z_check_started_at is None:
                self.z_check_started_at = now
                self.z_check_initial_error_pixels = abs(self.error_x_pixels)
            elif now - self.z_check_started_at >= Z_DIRECTION_EVALUATION_SECONDS:
                change = abs(self.error_x_pixels) - self.z_check_initial_error_pixels
                self.z_check_started_at = None
                if change > Z_DIRECTION_IMPROVEMENT_THRESHOLD_PIXELS:
                    self.z_direction_sign *= -1.0
                    self.z_integral = 0.0
                    self.z_direction_verified = True
                    print("Horizontal error increased: Z direction automatically reversed.")
                elif change < -Z_DIRECTION_IMPROVEMENT_THRESHOLD_PIXELS:
                    self.z_direction_verified = True
                    print("Horizontal error decreased: Z direction confirmed.")

        self.status = "PI FOLLOWING AT 12 HZ" if sent else "CENTERED"


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
    # requested PI velocity. They remain useful even without Arduino feedback.
    if abs(follower.error_x_pixels) > dx:
        arrow_x = int(math.copysign(min(190, max(45, abs(follower.error_x_pixels))), follower.error_x_pixels))
        thickness = 2 + round(4 * abs(follower.z_velocity) / MAX_TRACKING_SPEED_DEGREES_PER_SECOND)
        cv2.arrowedLine(frame, (cx, cy), (cx + arrow_x, cy), (255, 0, 255), thickness, cv2.LINE_AA, tipLength=0.18)
    if abs(follower.error_y_pixels) > dy:
        arrow_y = int(math.copysign(min(150, max(45, abs(follower.error_y_pixels))), follower.error_y_pixels))
        thickness = 2 + round(4 * abs(follower.y_velocity) / MAX_TRACKING_SPEED_DEGREES_PER_SECOND)
        cv2.arrowedLine(frame, (cx, cy), (cx, cy + arrow_y), (255, 0, 255), thickness, cv2.LINE_AA, tipLength=0.18)


def draw_status(frame, hand_detected: bool, follower: HandCenterFollower, armed: bool, arduino: ArduinoConnection, fps: float) -> None:
    position = (
        "NO REPORT (PI USING STARTUP ZERO)"
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
        (f"FOLLOW: {'ARMED' if armed else 'DISARMED'}  {follower.status}", TEXT_COLOR if armed else WARNING_COLOR),
        (f"PIXEL ERROR: X={follower.error_x_pixels:+.0f}px  Y={follower.error_y_pixels:+.0f}px", INFO_COLOR),
        (f"ANGLE ERROR: X={follower.horizontal_angle_error:+.1f}  Y={follower.vertical_angle_error:+.1f} deg", INFO_COLOR),
        (f"PI REQUEST: Z={follower.z_velocity:+.1f}  Y={follower.y_velocity:+.1f} deg/s", INFO_COLOR),
        (f"Z DIRECTION: {follower.z_direction_text}", INFO_COLOR),
        (f"ARDUINO: {position}", INFO_COLOR),
        (serial_text, serial_color),
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
                else:
                    arduino.read_responses()

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
                    draw_status(display, hand_detected, follower, armed, arduino, fps)
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
                elif key == EXIT_KEY:
                    stop_event.set()
    finally:
        stop_event.set()
        camera.release()
        cv2.destroyAllWindows()
        arduino.close()


if __name__ == "__main__":
    main()
