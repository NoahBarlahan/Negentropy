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
wrist and fingertips, reports handedness, and recognizes an open hand or fist.
A gesture must remain stable for 0.5 seconds before it is confirmed.

The user can start and stop a CSV data session from the camera window. While
recording, the program writes a sample every 0.25 seconds containing elapsed
time, whether a hand was detected, its confirmed position, and explicit
open/fist yes-or-no values. Files are saved under `hand_tracking_data/`.

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
| `R` | Start a new CSV recording session |
| `S` | Stop and save the active CSV session |
| `Q` | Quit |

## Development roadmap

### Servo camera synchronization prototype

`ServoCameraSync.py` connects the current MediaPipe gesture detector to the
Arduino sketch over USB serial at 9600 baud:

| Confirmed camera state | Servo command |
| --- | ---: |
| Fist | 0° |
| No hand | 90° |
| Open hand | 180° |

Open and fist gestures must remain stable for 0.5 seconds before a command is
sent. Hand loss is also confirmed for 0.5 seconds to prevent a single missed
frame from moving the servo. The program sends only changed angles, centers the
servo during startup and exit, and automatically selects the likely Arduino
USB port. Set `SERIAL_PORT` near the beginning of the file if a manual COM port
is needed.

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
