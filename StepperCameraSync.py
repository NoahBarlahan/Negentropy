"""Track a hand horizontally and control an A4988 stepper through Arduino.

This program is the computer-side companion to:
    StepperHandTrackingMega/StepperHandTrackingMega.ino

MediaPipe detects the palm center. The horizontal position is filtered using
the same approach as ServoCameraSync.py, then sent to the Arduino as a value
from 0 through 180:

    screen left   -> hand value 0   -> stepper target 0 degrees
    screen center -> hand value 90  -> stepper target 180 degrees
    screen right  -> hand value 180 -> stepper target 360/0 degrees

The Arduino performs the final 0-to-360-degree mapping and uses AccelStepper
for non-blocking acceleration and deceleration. Close Arduino IDE's Serial
Monitor before running this file because only one application can own a COM
port at a time.
"""

from pathlib import Path
import time

try:
    import cv2
    import mediapipe as mp
    import serial
    from serial.tools import list_ports
except ModuleNotFoundError:
    print("Stepper-camera dependencies are not installed.")
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

# Arduino and stepper communication ----------------------------------------
# Leave this as None to automatically select the most likely Arduino port.
# If detection chooses incorrectly, enter the Mega's port, such as "COM5".
SERIAL_PORT: str | None = None
SERIAL_BAUD_RATE = 9600
ARDUINO_RESET_WAIT_SECONDS = 2.0

# These values match HAND_INPUT_MINIMUM and HAND_INPUT_MAXIMUM in the Arduino
# sketch. The Mega maps this 0-to-180 input range onto one 0-to-360-degree
# stepper revolution.
MINIMUM_HAND_INPUT = 0
CENTER_HAND_INPUT = 90
MAXIMUM_HAND_INPUT = 180
STEPPER_DEGREES_PER_HAND_INPUT = 360.0 / (
    MAXIMUM_HAND_INPUT - MINIMUM_HAND_INPUT
)

# The Python program automatically sends g after opening the port. On exit it
# sends s so the Arduino decelerates the stepper to a stop.
AUTO_ENABLE_TRACKING = True
STOP_STEPPER_ON_EXIT = True
REVERSE_STEPPER_DIRECTION = False

# Hand motion filtering -----------------------------------------------------
# A hand jump faster than this fraction of the frame width per second is
# ignored. Lower this to reject more movement; raise it to be more responsive.
MAX_VALID_HAND_SPEED_NORMALIZED_PER_SECOND = 0.70

# After a fast jump, the position must remain slow for this long before it is
# accepted as a new target.
SLOW_POSITION_CONFIRMATION_SECONDS = 0.25

# Exponential smoothing. Lower values give smoother but slower tracking.
HAND_POSITION_SMOOTHING = 0.18

# Brief detection dropouts hold the last target. A longer loss sends the
# center hand value, which the Arduino maps to a 180-degree stepper target.
NO_HAND_CONFIRMATION_SECONDS = 0.50

# Rate-limit the computer's requested position so a valid target still cannot
# command a sudden jump. The Arduino adds physical acceleration/deceleration.
MAX_INPUT_SPEED_UNITS_PER_SECOND = 55.0
INPUT_COMMAND_DEADBAND = 1
MINIMUM_COMMAND_INTERVAL_SECONDS = 0.04

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


def hand_input_to_stepper_angle(hand_input: float) -> float:
    """Convert the transmitted 0-to-180 hand value to 0-to-360 degrees."""

    clamped_input = min(
        float(MAXIMUM_HAND_INPUT),
        max(float(MINIMUM_HAND_INPUT), hand_input),
    )
    return (
        clamped_input - MINIMUM_HAND_INPUT
    ) * STEPPER_DEGREES_PER_HAND_INPUT


def print_controls(port_name: str) -> None:
    """Print the serial mapping, motion filters, and keyboard controls."""

    print()
    print("STEPPER CAMERA SYNC")
    print("----------------------------------------")
    print(f"Arduino Mega: {port_name} at {SERIAL_BAUD_RATE} baud")
    print("The program automatically sends g to enable the Arduino.")
    print("SCREEN LEFT   -> input 0   -> target 0 degrees")
    print("SCREEN CENTER -> input 90  -> target 180 degrees")
    print("SCREEN RIGHT  -> input 180 -> target 360/0 degrees")
    print("NO HAND       -> input 90  -> target 180 degrees")
    print(
        "Fast-motion limit: "
        f"{MAX_VALID_HAND_SPEED_NORMALIZED_PER_SECOND:.2f} frame widths/s"
    )
    print("----------------------------------------")
    print("L   Show or hide hand landmarks")
    print("P   Pause or resume camera processing")
    print("C   Manually send the center target")
    print("Q   Stop stepper tracking and quit")
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
    """Return the configured port or find the most likely Arduino USB port."""

    if manual_port:
        return manual_port

    ports = list(list_ports.comports())
    if not ports:
        raise RuntimeError("No serial ports were detected.")

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
            "Could not confidently identify the Arduino Mega. "
            f"Available ports: {port_list}. Set SERIAL_PORT manually."
        )

    return best_port.device


class StepperController:
    """Send enable, stop, and hand-position commands to the Arduino Mega."""

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
        self.enabled = False
        self.last_hand_input: int | None = None
        self.last_response = ""

        # Opening a serial connection resets many Arduino boards. Wait for the
        # Mega's setup() to finish before sending any protocol commands.
        time.sleep(ARDUINO_RESET_WAIT_SECONDS)
        self.connection.reset_input_buffer()

        if AUTO_ENABLE_TRACKING:
            self.enable_tracking()
            time.sleep(0.05)
            self.command_hand_input(CENTER_HAND_INPUT, force=True)

    def write_line(self, line: str) -> None:
        """Send one newline-terminated ASCII command to the Arduino."""

        self.connection.write(f"{line}\n".encode("ascii"))
        self.connection.flush()

    def enable_tracking(self) -> None:
        """Send g so the Arduino begins accepting hand-position values."""

        self.write_line("g")
        self.enabled = True
        self.last_hand_input = None
        print("Stepper tracking enabled (sent g).")

    def stop_tracking(self) -> None:
        """Send s so AccelStepper decelerates the motor to a stop."""

        if not self.connection.is_open or not self.enabled:
            return
        self.write_line("s")
        self.enabled = False
        print("Stepper tracking disabled (sent s).")

    def command_hand_input(
        self,
        hand_input: int,
        force: bool = False,
    ) -> bool:
        """Send one changed 0-to-180 hand value; return True if sent."""

        if not MINIMUM_HAND_INPUT <= hand_input <= MAXIMUM_HAND_INPUT:
            raise ValueError("Stepper hand input must be from 0 through 180.")
        if not self.enabled:
            return False
        if not force and hand_input == self.last_hand_input:
            return False

        self.write_line(str(hand_input))
        self.last_hand_input = hand_input
        requested_angle = hand_input_to_stepper_angle(hand_input)
        print(
            f"Hand input: {hand_input} -> "
            f"stepper target: {requested_angle:.0f} degrees"
        )
        return True

    def read_responses(self) -> None:
        """Print complete status lines returned by the Arduino sketch."""

        while self.connection.in_waiting:
            line = self.connection.readline().decode(
                "utf-8",
                errors="replace",
            )
            line = line.strip()
            if line:
                self.last_response = line
                print(f"Arduino: {line}")

    def close(self) -> None:
        """Stop tracking and release the serial port."""

        if not self.connection.is_open:
            return
        if STOP_STEPPER_ON_EXIT:
            self.stop_tracking()
            time.sleep(0.15)
            self.read_responses()
        self.connection.close()


def create_hand_landmarker():
    """Load the pretrained MediaPipe hand model in video mode."""

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
    """Convert one normalized MediaPipe coordinate to a display pixel."""

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
    """Return a stable palm center using the wrist and four knuckles."""

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


def normalized_x_to_hand_input(normalized_x: float) -> float:
    """Map horizontal palm position to the Arduino's 0-to-180 input."""

    clamped_x = min(1.0, max(0.0, normalized_x))
    if REVERSE_STEPPER_DIRECTION:
        clamped_x = 1.0 - clamped_x

    input_range = MAXIMUM_HAND_INPUT - MINIMUM_HAND_INPUT
    return MINIMUM_HAND_INPUT + clamped_x * input_range


class HandPositionFilter:
    """Reject fast observations and smooth accepted palm positions."""

    def __init__(self) -> None:
        self.previous_observed_x: float | None = None
        self.previous_observed_time: float | None = None
        self.slow_since: float | None = None
        self.filtered_x: float | None = None
        self.no_hand_since: float | None = None
        self.target_input = float(CENTER_HAND_INPUT)
        self.last_center: tuple[float, float] | None = None
        self.last_speed = 0.0
        self.status = "NO HAND - CENTERING"

    def update(
        self,
        observed_center: tuple[float, float] | None,
        current_time: float,
    ) -> float:
        """Process one observation and return the latest valid hand input."""

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
                self.target_input = float(CENTER_HAND_INPUT)
                self.status = "NO HAND - CENTERING"
            else:
                self.status = "HAND LOST BRIEFLY - HOLDING"
            return self.target_input

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
            return self.target_input

        if self.slow_since is None:
            self.slow_since = current_time

        if current_time - self.slow_since < SLOW_POSITION_CONFIRMATION_SECONDS:
            self.status = "CHECKING SLOW POSITION..."
            return self.target_input

        if self.filtered_x is None:
            self.filtered_x = observed_x
        else:
            alpha = min(1.0, max(0.0001, HAND_POSITION_SMOOTHING))
            self.filtered_x += alpha * (observed_x - self.filtered_x)

        self.target_input = normalized_x_to_hand_input(self.filtered_x)
        self.status = "TRACKING SLOW HAND MOVEMENT"
        return self.target_input


class SmoothStepperInput:
    """Rate-limit hand inputs before sending them to the Arduino."""

    def __init__(self, current_time: float) -> None:
        self.current_input = float(CENTER_HAND_INPUT)
        self.target_input = float(CENTER_HAND_INPUT)
        self.previous_update_time = current_time
        self.last_command_time = current_time
        self.last_commanded_input = CENTER_HAND_INPUT

    def set_target(self, hand_input: float) -> None:
        self.target_input = min(
            float(MAXIMUM_HAND_INPUT),
            max(float(MINIMUM_HAND_INPUT), hand_input),
        )

    def update(
        self,
        current_time: float,
        stepper: StepperController,
    ) -> int:
        """Move toward the input target and transmit meaningful changes."""

        elapsed = max(0.0, min(0.1, current_time - self.previous_update_time))
        self.previous_update_time = current_time
        maximum_change = MAX_INPUT_SPEED_UNITS_PER_SECOND * elapsed
        remaining_change = self.target_input - self.current_input

        if abs(remaining_change) <= maximum_change:
            self.current_input = self.target_input
        elif remaining_change > 0.0:
            self.current_input += maximum_change
        else:
            self.current_input -= maximum_change

        command = int(round(self.current_input))
        enough_change = (
            abs(command - self.last_commanded_input)
            >= INPUT_COMMAND_DEADBAND
        )
        enough_time = (
            current_time - self.last_command_time
            >= MINIMUM_COMMAND_INTERVAL_SECONDS
        )
        if enough_change and enough_time:
            if stepper.command_hand_input(command):
                self.last_commanded_input = command
                self.last_command_time = current_time

        return command


def draw_status(
    frame,
    port_name: str,
    position_filter: HandPositionFilter,
    motion: SmoothStepperInput,
    fps: float,
) -> None:
    """Draw tracking, target, speed, serial port, and frame-rate status."""

    color = (
        TEXT_COLOR
        if position_filter.status == "TRACKING SLOW HAND MOVEMENT"
        else WARNING_COLOR
    )
    requested_angle = hand_input_to_stepper_angle(motion.current_input)
    target_angle = hand_input_to_stepper_angle(motion.target_input)

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
        f"REQUEST: {requested_angle:.0f} deg  "
        f"TARGET: {target_angle:.0f} deg  PORT: {port_name}",
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
    """Track the palm center and synchronize it with the stepper target."""

    port_name = find_arduino_port()
    stepper = StepperController(port_name)
    camera = open_camera()

    if camera is None:
        stepper.close()
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
    motion = SmoothStepperInput(current_time)
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
                    target_input = position_filter.update(
                        observed_center,
                        current_time,
                    )
                    motion.set_target(target_input)
                    motion.update(current_time, stepper)

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

                stepper.read_responses()

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
                    cv2.imshow("Stepper Camera Sync", display)

                key = cv2.waitKey(1) & 0xFF
                if key == LANDMARK_KEY:
                    show_landmarks = not show_landmarks
                elif key == PAUSE_KEY:
                    paused = not paused
                    print("Paused." if paused else "Resumed.")
                elif key == CENTER_KEY:
                    position_filter.target_input = float(CENTER_HAND_INPUT)
                    position_filter.filtered_x = None
                    position_filter.slow_since = None
                    position_filter.status = "MANUALLY CENTERING"
                    motion.set_target(CENTER_HAND_INPUT)
                elif key == EXIT_KEY:
                    break

    finally:
        camera.release()
        cv2.destroyAllWindows()
        stepper.close()


if __name__ == "__main__":
    main()
