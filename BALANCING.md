# Two-wheel balancing with the D435if IMU

This controller targets your **Arduino Mega 2560, two A4988 drivers, 200-step
motors, 1/8 microstepping, direct drive, and 5-inch wheels**. It opens only
RealSense accelerometer and gyroscope streams. It does not open video streams.
The original camera-control and ESP32 programs are preserved.

## Preliminary wheel test on a stand

For initial motor tests, upload **WheelTestMega/WheelTestMega.ino** to the Mega
instead of the balancing sketch. It requires **AccelStepper** (already present
on this computer). Open Arduino Serial Monitor at **115200 baud**, with
**Newline** selected. No camera, IMU, Python program or calibration is needed.
The same pins and 1/8 microstep wiring are used. Both wheels must be clear of
the floor on your test stand. The motors remain still after upload/reset.

| Serial command | Result |
| --- | --- |
| `l30`, then `r30` | First small tests: each wheel moves forward 30 degrees |
| `l90`, `l-90` | Left wheel forward/backward a quarter revolution |
| `r90`, `r-90` | Right wheel forward/backward a quarter revolution |
| `b90`, `b-90` | Both wheels forward/backward together |
| `t90`, `t-90` | Wheels in opposite directions for a turning test |
| `l360`, `r360`, `b360` | One complete wheel revolution |
| `speed15` | Set a slower maximum speed of 15 degrees/s; default is 30 |
| `invert l`, `invert r` | Reverse that wheel's forward definition, while stopped |
| `s` or `STOP` | Immediately stop STEP pulses |
| `p`, `zero`, `h` | Print status, reset software counts without moving, or help |

Wait for **DONE** before another movement command. All moves are relative to
the current position, limited to +/-360 degrees, with gentle acceleration and
automatic completion. They do not run continuously. `t90` describes each wheel's
rotation, not a 90-degree robot heading change; wheel spacing would be needed
for a robot-heading calculation.

Mark the tire at the top and check `90` gives a quarter turn and `360` returns
the mark to its starting point. At 200 full steps/revolution and 1/8 stepping,
90 degrees requests 400 pulses and 360 requests 1600. If the angle differs,
check microstep wiring and `FULL_STEPS_PER_REVOLUTION`/`MICROSTEPS` in the sketch.
Software counts cannot detect missed steps or wheel slip.

The diagnostic version prints **READY WHEELTEST v3** on reset. Each complete
command is echoed as `RX #...`, followed by `ACCEPTED MOVE #...` or an explicit
rejection. `OUTPUT L/R` appears only after the step-writing routine executes;
`PROGRESS` and `DONE OUTPUT` report pulses written versus requested, independently
for both wheels. For `l90`, the final report should show `L=400/400 R=0/0`.
`p` can request progress during motion. A partial command without a newline is
discarded after 1.5 seconds with a message to select Newline in Serial Monitor.
Progress logging is buffered so serial output does not pause motor stepping.

If no `RX` appears, check the port, baud, uploaded sketch, and line ending.
If a command is rejected, no new movement was scheduled. If `OUTPUT` and the
expected final counts appear but the tire only buzzes, the code has executed
its STEP writes; that does not confirm signal arrival at the A4988, correctly
paired motor coils, driver power/settings, or actual rotation. Software cannot
measure the electrical signal or diagnose those hardware conditions by itself.

For a microstepping isolation test, remove motor power and USB power before
changing wiring. On both drivers, connect MS1, MS2, MS3 to GND instead of any
existing HIGH connections for full-step mode. Reconnect power, send `micro1`
then `speed15`, and test `l90` / `r90` separately. Each quarter-turn now requests
50 pulses. `micro1/2/4/8/16` only changes software scaling and resets software
counts; it does not drive the microstep pins or change saved balancing settings.
After testing, restore H/H/L wiring and send `micro8` for 1/8 stepping. The
default at reset remains micro8. Full-step motion naturally has visible steps.

If one motor rocks back and forth, check both coil circuits with all power
removed and the motor disconnected from the driver. For a four-wire motor,
identify its two low-resistance wire pairs using a meter. One complete pair
belongs on 1A/1B and the other on 2A/2B; colors alone do not establish pairing.
Never plug/unplug a motor while the A4988 is powered. To isolate the motor/cable
from the driver/control side, swap the two complete motor connections with all
power off, only for matching motors with compatible current limits and pinouts.
If the rocking follows the motor/cable, investigate that assembly; if it stays
on the left driver, investigate that driver's wiring, supply and current setting.
Use the motor's rated current and the actual carrier's sense resistor value to
set VREF; do not guess by turning the trimmer until it runs.

Reference: [Pololu A4988 wiring and microstep table](https://www.pololu.com/product/1182),
[current-limit guidance](https://www.pololu.com/product/1182/faqs).

Both positive wheel commands should drive the robot **forward** if it were on
the floor; viewing the mirrored wheels from opposite sides gives opposite
clockwise/counterclockwise appearances. Use `invert l/r` if necessary, then copy
the **forward signs** printed by `p` into the balancing dashboard. Runtime
inversions reset on upload/power cycling; edit `DEFAULT_REVERSE_LEFT/RIGHT` to
retain them in the test sketch. The wheel program does not modify saved PID
settings automatically.

An accepted move still completes if USB disconnects; software stop requires
the connection. Keep motor power accessible on the stand. Without EN wiring,
the stopped drivers can retain holding torque. When testing is done, upload
**BalanceMega/BalanceMega.ino** again before running `BalanceRobot.py`.

## Install and run

From this project folder in PowerShell:

```powershell
python -m venv .balance-venv
& ".\.balance-venv\Scripts\python.exe" -m pip install -r balance-requirements.txt
& ".\.balance-venv\Scripts\python.exe" .\BalanceRobot.py --check
```

Upload **BalanceMega/BalanceMega.ino** using Arduino IDE with **Arduino Mega
2560** selected. This replaces the camera sketch on the board and does not
require AccelStepper or another external library. Keep the existing wiring:

| Wheel assignment | DIR | STEP |
| --- | --- | --- |
| Left / previous motor 1 | 32 | 34 |
| Right / previous motor 2 | 38 | 36 |

If motor 1 is physically the right wheel, relabel the constants in the sketch.
Both A4988s must have MS1 HIGH, MS2 HIGH, MS3 LOW for the default 1/8 stepping.
Changing the dashboard microsteps does **not** change the driver's wiring.
Keep the same motor power connections, current limits, and shared ground used
by the working prototype. Optional active-low EN pins are left unassigned.

Close Arduino Serial Monitor and the previous control programs. Run:

```powershell
# Dashboard demo, with no access to hardware:
& ".\.balance-venv\Scripts\python.exe" .\BalanceRobot.py --demo

# Real IMU and calculated outputs, without opening a motor serial port:
& ".\.balance-venv\Scripts\python.exe" .\BalanceRobot.py --imu-only

# Live robot; substitute the Mega's COM port listed by --check:
& ".\.balance-venv\Scripts\python.exe" .\BalanceRobot.py --port COM5
```

Without --port, live mode auto-selects only when exactly one recognized Arduino
port exists. Baud is now **115200**, not the older sketch's 9600. The host checks
the new firmware identity before sending any movement commands. If multiple
RealSense devices are connected, use `--camera-serial SERIAL_NUMBER`.

## Initialize and test

1. Secure the camera so it cannot move relative to the robot. Tape is acceptable
   for a temporary test if it is rigid. Its cable also must not tug the body.
2. Support the robot at its desired upright position, with the wheels stopped.
   Click **Capture upright** and hold completely still for two seconds. This
   averages gravity and measures gyro bias. Initialization does not arm motors.
3. Tip the whole robot **forward 5–25 degrees about the wheel axle**, hold it
   still, then click **Teach forward tilt** for another two-second measurement.
   This learns the axis and forward sign for an arbitrarily mounted camera.
   A single stationary reference cannot identify that axle. Avoid sideways
   tilt or twisting. Return upright; the pitch plot should return near zero.
4. Check in IMU-only mode that forward tip produces positive pitch and positive
   angular velocity while tipping, and backward tip produces negative values.
5. In live mode, lift/support the robot with wheels clear of the floor. **Test
   left** and **Test right** each give that wheel a short low-speed forward
   command. Both must roll the robot forward if placed on the floor. Change
   the relevant direction sign and **Apply tuning** if one is reversed, then
   retest. Check **Both wheel tests move the robot forward** only after testing.
6. Place the wheels on the floor and hold the body upright inside a catch
   support. Click **ARM**. Arming requires completed calibration, fresh data,
   confirmed wheel directions, low angular velocity, and pitch within 4 degrees
   of the reference. The wheels should move underneath the direction of fall.
   If correction goes the wrong way, STOP and recheck the calibration/directions.

In demo mode the **Demo tilt** field supplies the calibration poses: zero for
upright, about +10 for forward teaching, then zero before arming. The demo is
an illustrative pendulum, not proof that these gains work on your robot.

## Tune using the dashboard

Graphs show pitch, angular velocity, wheel-speed command, P/I/D acceleration
contributions, left/right step-rate commands, and loop/IMU/acknowledgement
timing. **STOP** or **Space** disarms. Closing the dashboard stops output.

Edit settings while disarmed, then click **Apply tuning** and explicitly arm
again. This avoids sudden gain or direction changes during balance. Save/load
persists tuning to `balance_settings.json`; startup references and permission
to arm are intentionally recalibrated each run. Record CSV saves 50 Hz samples
under `balance_logs/`. The terminal reports timing and saturation counts at exit.

| Control | Effect |
| --- | --- |
| P | Wheel acceleration per radian of lean; more helps catch lean but can cause oscillation |
| D | Acceleration per radian/second of forward tipping; provides damping |
| I | Corrects sustained lean error; start at zero because it can wind up and cause drift |
| Balance trim | Small target-angle correction for imperfect upright reference |
| Max speed | Limits integrated commanded wheel speed |
| Max acceleration | Limits how rapidly commanded wheel speed changes |
| IMU filter time | Larger values trust gyro longer; smaller values correct drift faster but admit acceleration disturbances |
| Fall cutoff | Stops commands after excessive tilt; does not catch the robot |

Start with I=0. Use brief supported runs and small P/D changes. Increase P if
correction is weak; increase D if it oscillates. If response is wrong-way or
timing is bad, fix that before changing gains. If it drifts, first check rigid
mounting, calibration and trim. An angle-only controller cannot reliably hold
position without wheel feedback; integral gain is not a substitute for encoders.
The initial P=18, D=3 and limits are commissioning values, not measured gains.

## How the control and stops work

The estimator projects gyro data onto the taught axle, subtracts startup bias,
and integrates using device timestamps. A gravity-based angle slowly corrects
that estimate with a complementary filter. Large acceleration-magnitude errors
are rejected. Smaller wheel accelerations still contaminate the tilt estimate.

The PID uses lean angle and gyro rate to request **wheel acceleration in m/s²**,
then integrates acceleration into wheel-speed commands. At 127 mm diameter,
200 full steps/rev, and 1/8 stepping, there are about **4010 pulses per meter**.
Both wheels receive the same forward motion, with independent electrical signs.
There is no steering or closed-loop wheel speed/position control in this version.

The host targets 100 Hz output and stops/disarms for missing IMU/Mega status,
timestamp gaps, a 50 ms control-loop gap, a lost dashboard heartbeat, excessive
lean, or sustained output saturation. The Mega generates pulses with a 20 kHz
Timer1 interrupt, caps each wheel at 2000 steps/s, and stops after 120 ms without
a valid command even if its main loop stalls. Packets include a session and
increasing sequence. Old speed packets cannot rearm a timed-out board; explicit
rearming creates a fresh session. Timer1 is reserved by this sketch.

Stopping means **no more STEP pulses**. Without EN wiring the A4988s may retain
holding torque. Software stop is not a physical catch or a motor-power switch.

## Issues to watch in physical runs

- **Torque / missed steps:** unknown load, center-of-mass height, supply voltage,
  A4988 current limit, and stepper speed determine whether it can recover.
  A five-inch direct-drive wheel may need more torque than the motors can deliver.
  Increasing PID gains cannot recover torque the hardware does not have.
- **Drift:** no encoders means no measured wheel velocity or ground position.
  The plotted speed is the integrated command. Missed steps and wheel slip are
  invisible. Even a balancing robot may steadily roll away.
- **Camera movement:** loose tape shifts the reference or teaches a false axis.
  Moving the camera after calibration requires reinitialization.
- **Windows/USB jitter:** Python on this computer is not a real-time controller.
  Watch gyro age, loop intervals and command acknowledgement delay. Avoid other
  heavy workloads. Persistent timing faults may require an onboard IMU and a
  control loop entirely on the microcontroller.
- **Saturation:** sustained speed/acceleration limits trigger a stop. Logs help
  distinguish weak response from timing trouble, but do not measure available
  torque. The firmware's 2000 steps/s cap corresponds to about 0.499 m/s at the
  default geometry and may be below what a fast fall needs.
- **Sideways falls and cables:** two wheels correct forward/backward lean only.
  Support side stability, and leave sufficient slack in both USB cables.

Physical balance and motor direction must be verified on the assembled robot.
Software checks and the demo cannot establish those results remotely.

The implementation check on this computer found no connected RealSense device
or serial port. The firmware was compiled and linked for ATmega2560; it was not
uploaded to a board. Control/serial tests and the simulated dashboard were run.
In the dashboard check, loop intervals reached about 35 ms despite a 10 ms
target. Watch timing during supported physical trials; simulation does not
establish that the Windows/USB loop is fast enough for this robot.

## Software verification

Run the hardware-independent tests with:

```powershell
python -m unittest discover -s tests -p "test_balance*.py" -v
```

IMU API reference: [RealSense motion example](https://github.com/realsenseai/librealsense/blob/master/examples/motion/rs-motion.cpp).
