"""Control an Arduino camera servo with MediaPipe hand gestures.

Arduino protocol
----------------
The Arduino sketch listens at 9600 baud for a line containing an integer from
0 through 180. This program sends exactly that protocol:

    confirmed fist       -> ``0\n``
    no hand              -> ``90\n``
    confirmed open hand  -> ``180\n``

Gestures and hand loss must remain stable briefly before a command is sent.
Only changed angles are sent, preventing unnecessary serial traffic and servo
jitter. Close Arduino IDE's Serial Monitor before starting this program because
only one application can own a serial port at a time.
"""

import math
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

FIST_SERVO_ANGLE = 0
NO_HAND_SERVO_ANGLE = 90
OPEN_SERVO_ANGLE = 180
CENTER_SERVO_ON_EXIT = True

# MediaPipe configuration ---------------------------------------------------
HAND_MODEL_PATH = Path(__file__).with_name("hand_landmarker.task")
MINIMUM_HAND_DETECTION_CONFIDENCE = 0.30
MINIMUM_HAND_PRESENCE_CONFIDENCE = 0.30
MINIMUM_TRACKING_CONFIDENCE = 0.40

# A sign must remain unchanged this long before it controls the servo. Hand
# loss is also delayed, preventing a single missed frame from snapping to 90°.
GESTURE_CONFIRMATION_SECONDS = 0.50
NO_HAND_CONFIRMATION_SECONDS = 0.50

# Finger-angle thresholds copied from the tested hand-tracking prototype.
EXTENDED_FINGER_ANGLE = 150.0
CURLED_FINGER_ANGLE = 115.0
OPEN_HAND_MINIMUM_EXTENDED_FINGERS = 3
FIST_MAXIMUM_EXTENDED_FINGERS = 1

# Display configuration -----------------------------------------------------
HAND_COLOR = (0, 255, 0)
JOINT_COLOR = (0, 255, 0)
FINGERTIP_COLOR = (0, 165, 255)
WRIST_COLOR = (255, 255, 0)
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

# (base, middle, tip) for index, middle, ring, and pinky. The thumb is omitted
# because its direction varies more between people and camera angles.
FINGER_ANGLE_LANDMARKS = (
    (5, 6, 8),
    (9, 10, 12),
    (13, 14, 16),
    (17, 18, 20),
)

HAND_CONNECTIONS = (
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12),
    (9, 13), (13, 14), (14, 15), (15, 16),
    (13, 17), (17, 18), (18, 19), (19, 20),
    (0, 17),
)


def print_controls(port_name: str) -> None:
    """Print the gesture mapping and keyboard controls."""

    print()
    print("SERVO CAMERA SYNC")
    print("----------------------------------------")
    print(f"Arduino port: {port_name} at {SERIAL_BAUD_RATE} baud")
    print(f"FIST     -> {FIST_SERVO_ANGLE} degrees")
    print(f"NO HAND  -> {NO_HAND_SERVO_ANGLE} degrees")
    print(f"OPEN     -> {OPEN_SERVO_ANGLE} degrees")
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
        self.command_angle(NO_HAND_SERVO_ANGLE, force=True)

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
            self.command_angle(NO_HAND_SERVO_ANGLE)
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


def joint_angle(first, middle, last) -> float:
    """Return the three-dimensional angle at a finger's middle landmark."""

    first_vector = (
        first.x - middle.x,
        first.y - middle.y,
        first.z - middle.z,
    )
    last_vector = (
        last.x - middle.x,
        last.y - middle.y,
        last.z - middle.z,
    )
    dot_product = sum(
        first_value * last_value
        for first_value, last_value in zip(first_vector, last_vector)
    )
    first_length = math.sqrt(sum(value * value for value in first_vector))
    last_length = math.sqrt(sum(value * value for value in last_vector))

    if first_length == 0.0 or last_length == 0.0:
        return 0.0

    cosine = dot_product / (first_length * last_length)
    return math.degrees(math.acos(min(1.0, max(-1.0, cosine))))


def classify_hand_gesture(hand_landmarks) -> str:
    """Classify the current hand as OPEN, FIST, or UNKNOWN."""

    angles = [
        joint_angle(
            hand_landmarks[base],
            hand_landmarks[middle],
            hand_landmarks[tip],
        )
        for base, middle, tip in FINGER_ANGLE_LANDMARKS
    ]
    extended_count = sum(angle >= EXTENDED_FINGER_ANGLE for angle in angles)
    average_angle = sum(angles) / len(angles)

    if extended_count >= OPEN_HAND_MINIMUM_EXTENDED_FINGERS:
        return "OPEN"

    if (
        extended_count <= FIST_MAXIMUM_EXTENDED_FINGERS
        and average_angle <= CURLED_FINGER_ANGLE
    ):
        return "FIST"

    return "UNKNOWN"


def update_stable_state(
    observed_state: str,
    candidate_state: str,
    candidate_since: float,
    current_time: float,
) -> tuple[str, float, str | None]:
    """Confirm a state only after its configured stability period."""

    if observed_state != candidate_state:
        return observed_state, current_time, None

    confirmation_time = (
        NO_HAND_CONFIRMATION_SECONDS
        if observed_state == "NO_HAND"
        else GESTURE_CONFIRMATION_SECONDS
    )

    if (
        observed_state in {"OPEN", "FIST", "NO_HAND"}
        and current_time - candidate_since >= confirmation_time
    ):
        return candidate_state, candidate_since, observed_state

    return candidate_state, candidate_since, None


def state_to_servo_angle(confirmed_state: str | None) -> int | None:
    """Map a confirmed visual state to its requested servo position."""

    return {
        "FIST": FIST_SERVO_ANGLE,
        "NO_HAND": NO_HAND_SERVO_ANGLE,
        "OPEN": OPEN_SERVO_ANGLE,
    }.get(confirmed_state)


def draw_status(
    frame,
    port_name: str,
    observed_state: str,
    confirmed_state: str | None,
    servo_angle: int | None,
    fps: float,
) -> None:
    """Draw gesture, servo, port, and frame-rate status."""

    if confirmed_state and observed_state == confirmed_state:
        state_text = confirmed_state.replace("_", " ")
        color = TEXT_COLOR
    elif observed_state in {"OPEN", "FIST", "NO_HAND"}:
        state_text = f"CHECKING {observed_state.replace('_', ' ')}..."
        color = WARNING_COLOR
    else:
        state_text = "UNKNOWN - HOLDING LAST ANGLE"
        color = WARNING_COLOR

    cv2.putText(
        frame,
        f"HAND STATE: {state_text}",
        (20, 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.70,
        color,
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        frame,
        f"SERVO: {servo_angle if servo_angle is not None else '--'} deg  "
        f"ARDUINO: {port_name}",
        (20, 68),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        color,
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        frame,
        f"FPS: {fps:.1f}",
        (20, 101),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.62,
        color,
        2,
        cv2.LINE_AA,
    )


def main() -> None:
    """Run hand detection and synchronize confirmed states with the servo."""

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
    observed_state = "NO_HAND"
    candidate_state = "NO_HAND"
    candidate_since = time.monotonic()
    confirmed_state: str | None = "NO_HAND"
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

                    if latest_result.hand_landmarks:
                        observed_state = classify_hand_gesture(
                            latest_result.hand_landmarks[0]
                        )
                    else:
                        observed_state = "NO_HAND"

                    current_time = time.monotonic()
                    (
                        candidate_state,
                        candidate_since,
                        newly_confirmed_state,
                    ) = update_stable_state(
                        observed_state,
                        candidate_state,
                        candidate_since,
                        current_time,
                    )

                    if newly_confirmed_state:
                        confirmed_state = newly_confirmed_state
                        requested_angle = state_to_servo_angle(confirmed_state)
                        if requested_angle is not None:
                            servo.command_angle(requested_angle)

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

                    draw_status(
                        display,
                        port_name,
                        observed_state,
                        confirmed_state,
                        servo.last_angle,
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
                    servo.command_angle(NO_HAND_SERVO_ANGLE)
                    confirmed_state = "NO_HAND"
                elif key == EXIT_KEY:
                    break

    finally:
        camera.release()
        cv2.destroyAllWindows()
        servo.close()


if __name__ == "__main__":
    main()
