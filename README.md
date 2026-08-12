# Negentropy

Negentropy is a personal cinematography robot project. Its long-term purpose is
to autonomously understand a filming request, follow a subject, frame the shot,
record complete activities, and analyze athletic movement.

This release is the completed **real-time hand-tracking prototype**. It provides
the first lightweight perception component for the larger Smart Fixed Camera
phase of the project.

## Current product

The application opens a live camera feed and uses Google's pretrained MediaPipe
Hand Landmarker to detect and track a human hand against changing backgrounds.
It does not use background subtraction, color matching, or a manually defined
hand shape.

Current capabilities:

- Tries the USB camera first and falls back to the integrated camera.
- Mirrors the video for a natural selfie-style view.
- Processes a smaller image internally to maintain a high frame rate.
- Detects and tracks one hand using a pretrained visual model.
- Tracks 21 landmarks across the wrist, palm, joints, and fingertips.
- Draws green hand connections and joint markers.
- Draws orange fingertip markers and a cyan wrist marker.
- Reports whether MediaPipe classified the hand as left or right.
- Displays estimated hand/no-hand confidence and current FPS.
- Allows landmark visibility and camera playback to be controlled by keyboard.
- Includes extensive comments explaining the implementation.

## How it works

```text
Camera frame
    -> mirror the image
    -> resize a copy to 640 pixels wide
    -> convert OpenCV BGR pixels to RGB
    -> run MediaPipe Hand Landmarker
    -> receive 21 normalized hand landmarks
    -> update the temporal confidence estimate
    -> draw landmarks, status, and FPS on the full-size frame
    -> display the result
```

MediaPipe runs in video mode, allowing it to reuse tracking information between
frames instead of treating every frame as an unrelated photograph.

## Requirements

- Python 3.12 (tested)
- A USB or integrated camera
- Windows, macOS, or Linux with camera access
- The dependencies listed in `requirements.txt`
- The `hand_landmarker.task` pretrained model beside the main script

The current implementation is CPU-friendly and does not require an NVIDIA GPU.

## Project files

```text
Negentropy/
|-- InitialCameraDebug       Main hand-tracking program
|-- hand_landmarker.task     Pretrained MediaPipe hand model
|-- requirements.txt         Python dependencies
`-- README.md                Project documentation
```

## Installation

Open PowerShell in the `Negentropy` directory.

Create an isolated environment:

```powershell
python -m venv .venv
```

Activate it:

```powershell
& ".\.venv\Scripts\Activate.ps1"
```

Install the required packages:

```powershell
python -m pip install -r requirements.txt
```

The repository expects `hand_landmarker.task` to already be in the project
directory. If it is missing, download Google's official float16 Hand Landmarker
model and save it with that exact filename:

<https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task>

## Running the tracker

From the project directory, run:

```powershell
& ".\.venv\Scripts\python.exe" ".\InitialCameraDebug"
```

The terminal reports which camera was opened and lists the available controls.

## Controls

| Key | Action |
| --- | --- |
| `L` | Show or hide hand landmarks |
| `P` | Pause or resume the camera |
| `Q` | Quit and release the camera |

## Landmark colors

| Color | Meaning |
| --- | --- |
| Green | Finger joints and hand connections |
| Orange | Fingertips |
| Cyan | Wrist/base of the hand |

## Configuration

The tuning variables are near the beginning of `InitialCameraDebug`.

Important settings include:

- `PROCESSING_WIDTH`: Lower values improve speed; higher values preserve detail.
- `NUMBER_OF_HANDS`: Maximum number of hands MediaPipe should track.
- `MINIMUM_HAND_DETECTION_CONFIDENCE`: Initial hand-detection threshold.
- `MINIMUM_HAND_PRESENCE_CONFIDENCE`: Required hand-presence threshold.
- `MINIMUM_TRACKING_CONFIDENCE`: Required tracking threshold between frames.
- `DETECTED_CONFIDENCE_GAIN`: How quickly displayed hand confidence rises.
- `MISSING_CONFIDENCE_LOSS`: How quickly displayed hand confidence falls.

Lower model thresholds accept less-certain detections but can increase false
positives. Higher thresholds reduce false positives but can lose partially
hidden, distant, or poorly lit hands.

## Confidence note

The displayed confidence is a **temporal estimate**, not MediaPipe's raw model
probability. MediaPipe's public Hand Landmarker result provides landmarks and
handedness but does not expose the raw hand-presence score.

The application therefore calculates an understandable state estimate:

```text
Repeated hand detections -> hand confidence rises
Repeated missed frames   -> hand confidence falls
No-hand confidence       -> 1 - hand confidence
```

This makes momentary tracking loss less distracting, but the value should not
be interpreted as a scientifically calibrated probability.

## Development roadmap

| Phase | What you build | Skills developed | End goal |
| --- | --- | --- | --- |
| **1. Smart Fixed Camera** | Gesture detection, full-body tracking, event detection, buffered recording, and metadata | Python, OpenCV, MediaPipe, state machines, and video pipelines | Automatically detect and save complete activity clips |
| **2. Robotic Cameraperson** | Framing controller, pan-tilt mechanism, smooth tracking, and shot modes | PID control, motors, embedded systems, and camera geometry | Physically keep the subject framed while they move |
| **3. Sports Analysis System** | Skateboard tracking, ollie phase detection, body/board measurements, and scoring | Object detection, time-series analysis, feature engineering, and machine learning | Record an ollie, rate it, and explain improvements |
| **4. VLA Mobile Filmmaker** | Voice commands, skill planning, follow-me base, safety, and recovery | ROS 2, speech, VLA architecture, navigation, and system integration | Understand commands such as “follow me and record my next ollie” |

## Next milestone

The next logical step in Phase 1 is gesture recognition. The existing 21 hand
landmarks can be converted into gestures such as start recording, stop
recording, or mark an event. That state can then drive a buffered video recorder
that preserves several seconds before and after each detected activity.

## Known limitations

- Only one hand is tracked by default.
- Fast motion, severe occlusion, poor lighting, or a very small hand can cause
  temporary tracking loss.
- The confidence indicator is temporal rather than a raw neural-network score.
- The program tracks hand landmarks but does not yet recognize gestures.
- It does not yet record video clips or produce metadata.
