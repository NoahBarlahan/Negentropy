# Personal VLA Cinematography and Sports-Coaching Robot

## Project goal

The goal of this project is to build a personal robot that can understand
spoken filming instructions, locate and follow a person, control a camera,
record activities, analyze athletic performance, and provide useful feedback.

An example final command is:

> “Follow me, record my next ollie, keep me in frame, rate it, and tell me what
> to improve.”

The finished robot will combine mechanical design, embedded electronics,
computer vision, feedback control, machine learning, and a
Vision-Language-Action (VLA) system.

## System overview

### Mechanical and electrical system

The physical platform may include:

- A motorized pan-and-tilt camera
- A mobile wheeled base
- Motors, encoders, and motor drivers
- A microcontroller and onboard computer
- Batteries and power-management hardware
- Obstacle, limit, and emergency-stop sensors

The mechanical objective is smooth, stable, and quiet camera movement while
the robot safely tracks a moving subject.

### Software and computer vision

The software will be responsible for:

- Camera input and video pipelines
- Person, hand, body-pose, and equipment tracking
- Activity and event detection
- Buffered video recording and metadata
- Automatic subject framing
- Motor and movement commands
- Data logging and safety monitoring

For skateboarding, the system may measure body and foot position, board angle,
jump height, approach speed, pop timing, landing alignment, and ride-away
stability.

For climbing, it may measure body pose, hand and foot placement,
center-of-mass movement, timing, and route sequences.

### Framing and motion control

The controller will compare the subject’s position in the image with the
desired framing position. That error will control camera pan, camera tilt,
robot turning, following distance, and movement speed.

This part of the project will explore:

- Feedback and PID control
- Encoder feedback and motor limits
- Latency and control-loop timing
- Mechanical backlash
- Acceleration and trajectory smoothing
- Stable shot composition

### Sports analysis and machine learning

The first sports-analysis system will use measurable features and a clear,
explainable scoring rubric. Later models can learn from labeled attempts, pose
data, equipment motion, personal performance history, and user feedback.

These models may eventually:

- Recognize and classify attempts
- Separate activity phases
- Predict performance scores
- Compare current and previous attempts
- Recommend specific improvements

## Vision-Language-Action architecture

The VLA system connects three responsibilities:

- **Vision** determines what is happening in the environment.
- **Language** determines what the user requested.
- **Action** selects and executes safe, tested robot skills.

An instruction could produce the following skill sequence:

```text
find person
    -> track subject
    -> choose wide framing
    -> begin recording
    -> detect ollie
    -> stop recording
    -> analyze performance
    -> provide feedback
```

The language model will select high-level skills and goals. It will not send
unrestricted commands directly to the motors. Low-level movement remains inside
tested controllers with speed limits, safety checks, and recovery behavior.

## Current prototype

The current repository contains a lightweight MediaPipe hand-tracking program.
It opens a camera feed, mirrors the image, tracks 21 hand landmarks, marks the
wrist, fingertips, and palm center, reports handedness, and recognizes an open
hand or fist. A gesture must remain stable for 0.5 seconds before it is
confirmed.

The user can start and stop a synchronized data-and-video session from the
camera window. Every session produces a timestamp-matched pair under
`hand_tracking_data/`: a CSV such as `hand_tracking_...csv` and its mirrored
annotated camera video `hand_tracking_...mp4`. The saved video includes the
hand skeleton, palm-center dot, gesture and detection status, position values,
recording counters, and FPS exactly as shown in the live window.

The CSV writes a sample every 0.25 seconds containing elapsed time, whether a
hand was detected, the palm-center X and Y camera-pixel coordinates, its
confirmed gesture, and explicit open/fist yes-or-no values. The uncalibrated
coordinate origin is the bottom-left of the displayed camera frame: X increases
to the right and Y increases upward. Position cells remain blank when no hand
is detected. The MP4 uses a fixed real-time frame clock so its duration remains
aligned with CSV elapsed time even when processing FPS changes.

### Interactive path and movement visualization

`VisualizeHandTracking.py` converts a coordinate CSV into an interactive HTML
report with two synchronized panels:

- A time-colored X/Y hand path with raw and smoothed traces
- Direction arrows whose length and shaft thickness increase with velocity;
  marker color also represents speed in pixels per second

Hover over a point to inspect its time, position, speed, and gesture. Use the
legend to show or hide the raw path, the mouse to zoom or pan, and the slider or
Play button to move through the recording. Detection gaps remain disconnected,
so the report does not invent movement while the hand is missing.

Create a report from the newest compatible coordinate CSV:

```powershell
& ".\.venv\Scripts\python.exe" ".\VisualizeHandTracking.py" --open
```

Or select a particular recording:

```powershell
& ".\.venv\Scripts\python.exe" ".\VisualizeHandTracking.py" `
    ".\hand_tracking_data\hand_tracking_YYYYMMDD_HHMMSS.csv" --open
```

Reports are saved under `hand_tracking_visuals/`. They work offline and can
export either panel to PNG from Plotly's camera button. The newest-file search
automatically ignores older CSVs that do not contain X/Y position columns.

This prototype develops the real-time perception and video-pipeline foundation
needed for later gesture commands, recording control, full-body tracking, and
robot behavior.

### Run the prototype

Install the Python dependencies:

```powershell
python -m venv .venv
& ".\.venv\Scripts\Activate.ps1"
python -m pip install -r requirements.txt
```

Run the tracker:

```powershell
& ".\.venv\Scripts\python.exe" ".\InitialCameraDebug"
```

Controls:

| Key | Action |
| --- | --- |
| `L` | Show or hide hand landmarks |
| `P` | Pause or resume the camera |
| `R` | Start synchronized CSV and MP4 recording |
| `S` | Stop and save both recording files |
| `Q` | Quit |

## Development roadmap

### Servo camera synchronization prototype

`ServoCameraSync.py` connects MediaPipe hand tracking to the Arduino sketch
over USB serial at 9600 baud. It tracks the center of the palm using the wrist
and four knuckle landmarks, then maps the horizontal camera position directly
to a servo angle:

| Palm position in the mirrored image | Servo target |
| --- | ---: |
| Far left | 0° |
| Center | 90° |
| Far right | 180° |
| No hand for 0.5 seconds | 90° |

Fast hand jumps are rejected. A position must remain slow for 0.25 seconds
before it becomes valid, accepted positions are smoothed, and servo movement
is limited to 55 degrees per second. This prevents quick tracking errors from
causing sudden mechanical motion. The program centers the servo during startup
and exit and automatically selects the likely Arduino USB port. The main tuning
variables are near the beginning of the file: `MAX_VALID_HAND_SPEED_NORMALIZED_PER_SECOND`,
`SLOW_POSITION_CONFIRMATION_SECONDS`, `HAND_POSITION_SMOOTHING`,
`MAX_SERVO_SPEED_DEGREES_PER_SECOND`, and `NO_HAND_CONFIRMATION_SECONDS`.
Set `SERIAL_PORT` if a manual COM port is needed, or set
`REVERSE_SERVO_DIRECTION = True` if the physical servo moves opposite the
displayed hand direction.

Close Arduino IDE's Serial Monitor before running the Python program because
only one application can use a serial port at a time. Keep the Arduino sketch
uploaded, then run:

```powershell
python -m pip install -r requirements.txt
& ".\.venv\Scripts\python.exe" ".\ServoCameraSync.py"
```

Controls:

| Key | Action |
| --- | --- |
| `L` | Show or hide hand landmarks |
| `P` | Pause or resume camera processing |
| `C` | Center the servo at 90° |
| `Q` | Center the servo and quit |

### Mega and A4988 stepper prototype

`StepperHandTrackingMega/StepperHandTrackingMega.ino` is an upload-ready
Arduino Mega sketch for a full-step NEMA 17 controlled through an A4988. The
computer remains responsible for MediaPipe detection; the Mega receives the
same default `0–180` numeric hand-position values, smooths and calibrates them
to `0–360°`, and selects the nearest equivalent target among 200 full steps.

Install **AccelStepper** from Arduino IDE's Library Manager before compiling.
The sketch uses non-blocking `moveTo()` and repeated `run()` calls for
acceleration and deceleration.

Wiring:

- Mega pin 32 to A4988 DIR
- Mega pin 34 to A4988 STEP
- A4988 RESET and SLEEP together to Mega 5V
- Separate A4988 motor power with ground connected to Mega ground
- MS1, MS2, and MS3 low for full-step mode

At 9600 baud with a newline ending, send `g` to enable tracking, `s` to disable
and decelerate, and numeric hand values such as `0`, `90`, or `180`. Before `g`,
numeric values are ignored and the motor remains still. The main tuning values
are `STEPS_PER_REVOLUTION`, `MAX_SPEED_STEPS_PER_SECOND`,
`ACCELERATION_STEPS_PER_SECOND_SQUARED`, `HAND_INPUT_MINIMUM`,
`HAND_INPUT_MAXIMUM`, and `HAND_SMOOTHING_ALPHA`.

The startup position of 0 steps is only a software assumption. The Mega and
A4988 cannot know the physical shaft position after power loss; add a limit
switch or homing sensor before relying on repeatable absolute orientation.

Arduino Serial Monitor and the Python tracker cannot own the USB serial port at
the same time. Use Serial Monitor for manual `g` and numeric-value tests. For
camera integration, close Serial Monitor and have the computer send `g\n` once
after opening the port, followed by the numeric hand values.

| Phase | System | End goal |
| --- | --- | --- |
| **1** | Smart fixed camera | Detect activities and automatically save complete clips with metadata |
| **2** | Robotic cameraperson | Physically follow the subject and maintain smooth framing |
| **3** | Sports-analysis system | Measure, score, and explain athletic performance |
| **4** | VLA mobile filmmaker | Understand spoken goals and coordinate perception, filming, navigation, analysis, and safety |

## What this project is intended to teach

This project is a practical path toward understanding how complete intelligent
robots are designed and integrated. Key learning areas include:

- Python, OpenCV, MediaPipe, and video processing
- Software architecture and state machines
- Mechanical design and embedded electronics
- Motors, encoders, PID control, and camera geometry
- ROS 2, navigation, and mobile robotics
- Pose estimation and object tracking
- Feature engineering and machine learning
- VLA architecture and skill planning
- Testing, safety, fault handling, and recovery

## Final outcome

The final system should receive a spoken instruction, locate and follow the
user, control the camera, record the requested activity, analyze the footage,
and return understandable feedback.

The overall objective is a modular intelligent robot that acts as a personal
cameraperson, activity tracker, and sports-performance coach.
