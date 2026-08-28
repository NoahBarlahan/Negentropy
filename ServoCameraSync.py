"""Track a hand horizontally and aim an Arduino-controlled camera servo.

Arduino protocol
----------------
The Arduino sketch listens at 9600 baud for a newline-terminated angle from
0 through 180. Screen-left maps to 0 degrees, the center maps to 90 degrees,
and screen-right maps to 180 degrees. Fast hand jumps are rejected; accepted
positions and servo commands are smoothed so the mechanism moves gradually.

Close Arduino IDE's Serial Monitor before starting this program because only
one application can own a serial port at a time.
"""

from pathlib import Path
import time

try:
    import cv2
    import mediapipe as mp
    import serial
    from serial.tools import list_ports
except ModuleNotFoundError:
    print("Servo-camera dependencies are not installed.")
    print("Install them with:")
    print("python -m pip install -r requirements.txt")
    raise SystemExit(1)


# Camera configuration ------------------------------------------------------
USB_CAMERA_NUMBER = 1
INTEGRATED_CAMERA_NUMBER = 0
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
PROCESSING_WIDTH = 640
MIRROR_IMAGE = True

# Arduino and servo configuration ------------------------------------------
# Leave SERIAL_PORT as None to automatically find an Arduino/USB serial port.
# If automatic detection is wrong, replace None with a value such as "COM5".
SERIAL_PORT: str | None = None
SERIAL_BAUD_RATE = 9600
ARDUINO_RESET_WAIT_SECONDS = 2.0

MINIMUM_SERVO_ANGLE = 0
CENTER_SERVO_ANGLE = 90
MAXIMUM_SERVO_ANGLE = 180
CENTER_SERVO_ON_EXIT = True
REVERSE_SERVO_DIRECTION = False

# Motion filtering ---------------------------------------------------------
# A hand jump faster than this fraction of the frame width per second is
# ignored. Lower this value to accept only slower hand movement.
MAX_VALID_HAND_SPEED_NORMALIZED_PER_SECOND = 0.70

# A position must remain slow for this long after a fast jump before the new
# location can become a servo target.
SLOW_POSITION_CONFIRMATION_SECONDS = 0.25

# Exponential smoothing: lower values are smoother but respond more slowly.
# This must be greater than 0 and no greater than 1.
HAND_POSITION_SMOOTHING = 0.18

# Brief detection dropouts hold the last target. A longer loss recenters.
NO_HAND_CONFIRMATION_SECONDS = 0.50

# Even accepted position changes cannot move the requested angle faster than
# this rate, preventing abrupt servo movement.
MAX_SERVO_SPEED_DEGREES_PER_SECOND = 55.0
SERVO_COMMAND_DEADBAND_DEGREES = 1
MINIMUM_SERVO_COMMAND_INTERVAL_SECONDS = 0.04

# MediaPipe configuration ---------------------------------------------------
HAND_MODEL_PATH = Path(__file__).with_name("hand_landmarker.task")
MINIMUM_HAND_DETECTION_CONFIDENCE = 0.30
MINIMUM_HAND_PRESENCE_CONFIDENCE = 0.30
MINIMUM_TRACKING_CONFIDENCE = 0.40

# Display configuration -----------------------------------------------------
HAND_COLOR = (0, 255, 0)
JOINT_COLOR = (0, 255, 0)
FINGERTIP_COLOR = (0, 165, 255)
WRIST_COLOR = (255, 255, 0)
PALM_CENTER_COLOR = (255, 0, 255)
TEXT_COLOR = (0, 255, 0)
WARNING_COLOR = (0, 165, 255)
CONNECTION_THICKNESS = 2
JOINT_RADIUS = 4
FINGERTIP_RADIUS = 7
WRIST_RADIUS = 9

# Keyboard controls ---------------------------------------------------------
LANDMARK_KEY = ord("l")
PAUSE_KEY = ord("p")
CENTER_KEY = ord("c")
EXIT_KEY = ord("q")

WRIST_LANDMARK = 0
FINGERTIP_LANDMARKS = {4, 8, 12, 16, 20}
PALM_CENTER_LANDMARKS = (0, 5, 9, 13, 17)

HAND_CONNECTIONS = (
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12),
    (9, 13), (13, 14), (14, 15), (15, 16),
    (13, 17), (17, 18), (18, 19), (19, 20),
    (0, 17),
)


def print_controls(port_name: str) -> None:
    """Print the position mapping, filters, and keyboard controls."""

    print()
    print("SERVO CAMERA SYNC")
    print("----------------------------------------")
    print(f"Arduino port: {port_name} at {SERIAL_BAUD_RATE} baud")
    print(f"SCREEN LEFT   -> {MINIMUM_SERVO_ANGLE} degrees")
    print(f"SCREEN CENTER -> {CENTER_SERVO_ANGLE} degrees")
    print(f"SCREEN RIGHT  -> {MAXIMUM_SERVO_ANGLE} degrees")
    print(f"NO HAND       -> {CENTER_SERVO_ANGLE} degrees")
    print(
        "Fast-motion limit: "
        f"{MAX_VALID_HAND_SPEED_NORMALIZED_PER_SECOND:.2f} frame widths/s"
    )
    print("----------------------------------------")
    print("L   Show or hide hand landmarks")
    print("P   Pause or resume camera processing")
    print("C   Manually center servo")
    print("Q   Center servo and quit")
    print()


def open_camera() -> cv2.VideoCapture | None:
    """Open the external camera first, then try the integrated camera."""

    camera = cv2.VideoCapture(USB_CAMERA_NUMBER)
    if camera.isOpened():
        print("Using USB camera.")
        return camera

    camera.release()
    camera = cv2.VideoCapture(INTEGRATED_CAMERA_NUMBER)
    if camera.isOpened():
        print("Using integrated camera.")
        return camera

    camera.release()
    return None


def find_arduino_port(manual_port: str | None = SERIAL_PORT) -> str:
    """Return a manual port or select the most likely USB Arduino port."""

    if manual_port:
        return manual_port

    ports = list(list_ports.comports())
    if not ports:
        raise RuntimeError("No serial ports were detected.")

    # Favor devices whose metadata explicitly mentions common Arduino USB
    # interfaces. Bluetooth ports receive a penalty so they are not selected.
    keywords = (
        "arduino",
        "usb serial",
        "usb-serial",
        "ch340",
        "wch",
        "cp210",
        "ftdi",
    )

    def port_score(port) -> int:
        details = " ".join(
            str(value or "")
            for value in (
                port.device,
                port.description,
                port.manufacturer,
                port.hwid,
            )
        ).lower()
        score = sum(10 for keyword in keywords if keyword in details)
        if "bluetooth" in details or "bth" in details:
            score -= 100
        return score

    ranked_ports = sorted(ports, key=port_score, reverse=True)
    best_port = ranked_ports[0]

    if port_score(best_port) <= 0:
        port_list = ", ".join(port.device for port in ports)
        raise RuntimeError(
            "Could not confidently identify the Arduino. "
            f"Available ports: {port_list}. Set SERIAL_PORT manually."
        )

    return best_port.device


class ServoController:
    """Send angle changes to the Arduino using its newline serial protocol."""

    def __init__(self, port_name: str) -> None:
        try:
            self.connection = serial.Serial(
                port=port_name,
                baudrate=SERIAL_BAUD_RATE,
                timeout=0,
                write_timeout=1,
            )
        except serial.SerialException as error:
            raise RuntimeError(
                f"Could not open {port_name}. Close Arduino Serial Monitor "
                "and confirm the correct COM port."
            ) from error

        self.port_name = port_name
        self.last_angle: int | None = None
        self.last_response = ""

        # Opening a serial connection resets many Arduino boards. Wait until
        # setup() completes before sending the initial center command.
        time.sleep(ARDUINO_RESET_WAIT_SECONDS)
        self.connection.reset_input_buffer()
        self.command_angle(CENTER_SERVO_ANGLE, force=True)

    def command_angle(self, angle: int, force: bool = False) -> bool:
        """Send one valid, changed servo angle. Return True when data was sent."""

        if not 0 <= angle <= 180:
            raise ValueError("Servo angle must be between 0 and 180 degrees.")

        if not force and angle == self.last_angle:
            return False

        self.connection.write(f"{angle}\n".encode("ascii"))
        self.connection.flush()
        self.last_angle = angle
        print(f"Servo command: {angle} degrees")
        return True

    def read_responses(self) -> None:
        """Read and print any complete status lines returned by the Arduino."""

        while self.connection.in_waiting:
            line = self.connection.readline().decode("utf-8", errors="replace")
            line = line.strip()
            if line:
                self.last_response = line
                print(f"Arduino: {line}")

    def close(self) -> None:
        """Optionally center the servo, then release the serial port."""

        if not self.connection.is_open:
            return

        if CENTER_SERVO_ON_EXIT:
            self.command_angle(CENTER_SERVO_ANGLE)
            time.sleep(0.15)
        self.connection.close()


def create_hand_landmarker():
    """Load the pretrained MediaPipe hand model in tracking/video mode."""

    if not HAND_MODEL_PATH.exists():
        raise FileNotFoundError(f"Hand model not found: {HAND_MODEL_PATH}")

    options = mp.tasks.vision.HandLandmarkerOptions(
        base_options=mp.tasks.BaseOptions(
            model_asset_path=str(HAND_MODEL_PATH)
        ),
        running_mode=mp.tasks.vision.RunningMode.VIDEO,
        num_hands=1,
        min_hand_detection_confidence=MINIMUM_HAND_DETECTION_CONFIDENCE,
        min_hand_presence_confidence=MINIMUM_HAND_PRESENCE_CONFIDENCE,
        min_tracking_confidence=MINIMUM_TRACKING_CONFIDENCE,
    )
    return mp.tasks.vision.HandLandmarker.create_from_options(options)


def prepare_for_mediapipe(frame):
    """Create a smaller RGB MediaPipe image from an OpenCV BGR frame."""

    frame_height, frame_width = frame.shape[:2]
    processing_height = round(frame_height * PROCESSING_WIDTH / frame_width)
    smaller_frame = cv2.resize(
        frame,
        (PROCESSING_WIDTH, processing_height),
        interpolation=cv2.INTER_AREA,
    )
    rgb_frame = cv2.cvtColor(smaller_frame, cv2.COLOR_BGR2RGB)
    return mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)


def normalized_to_pixel(landmark, width: int, height: int) -> tuple[int, int]:
    """Convert a normalized MediaPipe coordinate into a display pixel."""

    x = min(1.0, max(0.0, landmark.x))
    y = min(1.0, max(0.0, landmark.y))
    return int(x * width), int(y * height)


def draw_hand_landmarks(frame, hand_landmarks) -> None:
    """Draw the hand skeleton with highlighted wrist and fingertips."""

    height, width = frame.shape[:2]
    points = [
        normalized_to_pixel(landmark, width, height)
        for landmark in hand_landmarks
    ]

    for start_index, end_index in HAND_CONNECTIONS:
        cv2.line(
            frame,
            points[start_index],
            points[end_index],
            HAND_COLOR,
            CONNECTION_THICKNESS,
            cv2.LINE_AA,
        )

    for index, point in enumerate(points):
        if index == WRIST_LANDMARK:
            color, radius = WRIST_COLOR, WRIST_RADIUS
        elif index in FINGERTIP_LANDMARKS:
            color, radius = FINGERTIP_COLOR, FINGERTIP_RADIUS
        else:
            color, radius = JOINT_COLOR, JOINT_RADIUS
        cv2.circle(frame, point, radius, color, -1, cv2.LINE_AA)


def palm_center(hand_landmarks) -> tuple[float, float]:
    """Return a stable normalized center using the wrist and four knuckles."""

    center_x = sum(
        hand_landmarks[index].x for index in PALM_CENTER_LANDMARKS
    ) / len(PALM_CENTER_LANDMARKS)
    center_y = sum(
        hand_landmarks[index].y for index in PALM_CENTER_LANDMARKS
    ) / len(PALM_CENTER_LANDMARKS)
    return (
        min(1.0, max(0.0, center_x)),
        min(1.0, max(0.0, center_y)),
    )


def normalized_x_to_servo_angle(normalized_x: float) -> float:
    """Linearly map a horizontal image position to the servo angle range."""

    clamped_x = min(1.0, max(0.0, normalized_x))
    if REVERSE_SERVO_DIRECTION:
        clamped_x = 1.0 - clamped_x

    servo_range = MAXIMUM_SERVO_ANGLE - MINIMUM_SERVO_ANGLE
    return MINIMUM_SERVO_ANGLE + clamped_x * servo_range


class HandPositionFilter:
    """Reject fast observations and smooth accepted palm positions."""

    def __init__(self) -> None:
        self.previous_observed_x: float | None = None
        self.previous_observed_time: float | None = None
        self.slow_since: float | None = None
        self.filtered_x: float | None = None
        self.no_hand_since: float | None = None
        self.target_angle = float(CENTER_SERVO_ANGLE)
        self.last_center: tuple[float, float] | None = None
        self.last_speed = 0.0
        self.status = "NO HAND - CENTERING"

    def update(
        self,
        observed_center: tuple[float, float] | None,
        current_time: float,
    ) -> float:
        """Process one observation and return the latest valid target angle."""

        if observed_center is None:
            self.last_center = None
            self.previous_observed_x = None
            self.previous_observed_time = None
            self.slow_since = None
            self.last_speed = 0.0

            if self.no_hand_since is None:
                self.no_hand_since = current_time

            missing_time = current_time - self.no_hand_since
            if missing_time >= NO_HAND_CONFIRMATION_SECONDS:
                self.filtered_x = None
                self.target_angle = float(CENTER_SERVO_ANGLE)
                self.status = "NO HAND - CENTERING"
            else:
                self.status = "HAND LOST BRIEFLY - HOLDING"
            return self.target_angle

        self.no_hand_since = None
        self.last_center = observed_center
        observed_x = observed_center[0]

        if (
            self.previous_observed_x is None
            or self.previous_observed_time is None
        ):
            speed = 0.0
        else:
            elapsed = current_time - self.previous_observed_time
            speed = (
                abs(observed_x - self.previous_observed_x) / elapsed
                if elapsed > 0.0
                else float("inf")
            )

        self.previous_observed_x = observed_x
        self.previous_observed_time = current_time
        self.last_speed = speed

        if speed > MAX_VALID_HAND_SPEED_NORMALIZED_PER_SECOND:
            self.slow_since = None
            self.status = "FAST MOVEMENT IGNORED"
            return self.target_angle

        if self.slow_since is None:
            self.slow_since = current_time

        if current_time - self.slow_since < SLOW_POSITION_CONFIRMATION_SECONDS:
            self.status = "CHECKING SLOW POSITION..."
            return self.target_angle

        if self.filtered_x is None:
            self.filtered_x = observed_x
        else:
            alpha = min(1.0, max(0.0001, HAND_POSITION_SMOOTHING))
            self.filtered_x += alpha * (observed_x - self.filtered_x)

        self.target_angle = normalized_x_to_servo_angle(self.filtered_x)
        self.status = "TRACKING SLOW HAND MOVEMENT"
        return self.target_angle


class SmoothServoMotion:
    """Move toward a target at a limited rate and avoid serial chatter."""

    def __init__(self, current_time: float) -> None:
        self.current_angle = float(CENTER_SERVO_ANGLE)
        self.target_angle = float(CENTER_SERVO_ANGLE)
        self.previous_update_time = current_time
        self.last_command_time = current_time
        self.last_commanded_angle = CENTER_SERVO_ANGLE

    def set_target(self, angle: float) -> None:
        self.target_angle = min(
            float(MAXIMUM_SERVO_ANGLE),
            max(float(MINIMUM_SERVO_ANGLE), angle),
        )

    def update(self, current_time: float, servo: ServoController) -> int:
        """Rate-limit motion and send a command only when meaningfully changed."""

        elapsed = max(0.0, min(0.1, current_time - self.previous_update_time))
        self.previous_update_time = current_time
        maximum_change = MAX_SERVO_SPEED_DEGREES_PER_SECOND * elapsed
        remaining_change = self.target_angle - self.current_angle

        if abs(remaining_change) <= maximum_change:
            self.current_angle = self.target_angle
        elif remaining_change > 0.0:
            self.current_angle += maximum_change
        else:
            self.current_angle -= maximum_change

        command = int(round(self.current_angle))
        enough_change = (
            abs(command - self.last_commanded_angle)
            >= SERVO_COMMAND_DEADBAND_DEGREES
        )
        enough_time = (
            current_time - self.last_command_time
            >= MINIMUM_SERVO_COMMAND_INTERVAL_SECONDS
        )
        if enough_change and enough_time:
            if servo.command_angle(command):
                self.last_commanded_angle = command
                self.last_command_time = current_time

        return command


def draw_status(
    frame,
    port_name: str,
    position_filter: HandPositionFilter,
    motion: SmoothServoMotion,
    fps: float,
) -> None:
    """Draw tracking, speed, target-angle, and frame-rate status."""

    color = (
        TEXT_COLOR
        if position_filter.status == "TRACKING SLOW HAND MOVEMENT"
        else WARNING_COLOR
    )

    cv2.putText(
        frame,
        f"STATUS: {position_filter.status}",
        (20, 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.70,
        color,
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        frame,
        f"SERVO: {motion.current_angle:.0f} deg  "
        f"TARGET: {motion.target_angle:.0f} deg  PORT: {port_name}",
        (20, 68),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        color,
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        frame,
        f"HAND SPEED: {position_filter.last_speed:.2f} widths/s  "
        f"FPS: {fps:.1f}",
        (20, 101),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.62,
        color,
        2,
        cv2.LINE_AA,
    )


def main() -> None:
    """Track a palm center and smoothly synchronize it with the servo."""

    port_name = find_arduino_port()
    servo = ServoController(port_name)
    camera = open_camera()

    if camera is None:
        servo.close()
        print("No camera detected.")
        return

    camera.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    print_controls(port_name)

    paused = False
    show_landmarks = True
    frame = None
    latest_result = None
    current_time = time.monotonic()
    position_filter = HandPositionFilter()
    motion = SmoothServoMotion(current_time)
    last_timestamp_ms = 0
    previous_frame_time = time.perf_counter()
    smoothed_fps = 0.0

    try:
        with create_hand_landmarker() as landmarker:
            while True:
                if not paused:
                    success, frame = camera.read()
                    if not success:
                        print("Error: Could not read camera frame.")
                        break

                    if MIRROR_IMAGE:
                        frame = cv2.flip(frame, 1)

                    mp_image = prepare_for_mediapipe(frame)
                    timestamp_ms = max(
                        last_timestamp_ms + 1,
                        int(time.monotonic() * 1000),
                    )
                    last_timestamp_ms = timestamp_ms
                    latest_result = landmarker.detect_for_video(
                        mp_image,
                        timestamp_ms,
                    )

                    current_time = time.monotonic()
                    observed_center = (
                        palm_center(latest_result.hand_landmarks[0])
                        if latest_result.hand_landmarks
                        else None
                    )
                    target_angle = position_filter.update(
                        observed_center,
                        current_time,
                    )
                    motion.set_target(target_angle)
                    motion.update(current_time, servo)

                    current_frame_time = time.perf_counter()
                    elapsed = current_frame_time - previous_frame_time
                    previous_frame_time = current_frame_time
                    if elapsed > 0:
                        current_fps = 1.0 / elapsed
                        smoothed_fps = (
                            current_fps
                            if smoothed_fps == 0.0
                            else 0.90 * smoothed_fps + 0.10 * current_fps
                        )

                servo.read_responses()

                if frame is not None:
                    display = frame.copy()
                    if (
                        show_landmarks
                        and latest_result
                        and latest_result.hand_landmarks
                    ):
                        draw_hand_landmarks(
                            display,
                            latest_result.hand_landmarks[0],
                        )

                    if position_filter.last_center is not None:
                        frame_height, frame_width = display.shape[:2]
                        center_x = int(
                            position_filter.last_center[0] * frame_width
                        )
                        center_y = int(
                            position_filter.last_center[1] * frame_height
                        )
                        cv2.circle(
                            display,
                            (center_x, center_y),
                            12,
                            PALM_CENTER_COLOR,
                            -1,
                            cv2.LINE_AA,
                        )

                    draw_status(
                        display,
                        port_name,
                        position_filter,
                        motion,
                        smoothed_fps,
                    )
                    cv2.imshow("Servo Camera Sync", display)

                key = cv2.waitKey(1) & 0xFF
                if key == LANDMARK_KEY:
                    show_landmarks = not show_landmarks
                elif key == PAUSE_KEY:
                    paused = not paused
                    print("Paused." if paused else "Resumed.")
                elif key == CENTER_KEY:
                    position_filter.target_angle = float(CENTER_SERVO_ANGLE)
                    position_filter.filtered_x = None
                    position_filter.slow_since = None
                    position_filter.status = "MANUALLY CENTERING"
                    motion.set_target(CENTER_SERVO_ANGLE)
                elif key == EXIT_KEY:
                    break

    finally:
        camera.release()
        cv2.destroyAllWindows()
        servo.close()


if __name__ == "__main__":
    main()
