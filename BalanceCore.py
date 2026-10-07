"""Hardware-independent tilt estimation and acceleration PID for BalanceRobot."""

from dataclasses import asdict, dataclass, fields
import math


def clamp(value, low, high):
    return max(low, min(high, value))


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def norm(a):
    return math.sqrt(dot(a, a))


def unit(a):
    length = norm(a)
    if length < 1e-6 or not math.isfinite(length):
        raise ValueError("Invalid gravity vector / balancing axis")
    return tuple(x / length for x in a)


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def mean(vectors):
    return tuple(sum(v[i] for v in vectors) / len(vectors) for i in range(3))


@dataclass
class Settings:
    # PID produces wheel acceleration, then integrates to commanded wheel speed.
    kp: float = 18.0
    ki: float = 0.0
    kd: float = 3.0
    trim_deg: float = 0.0
    max_speed_m_s: float = 0.30
    max_accel_m_s2: float = 3.0
    filter_tau_s: float = 1.5
    integral_limit: float = 0.15  # rad*s
    fall_deg: float = 20.0
    wheel_diameter_mm: float = 127.0
    full_steps: int = 200
    microsteps: int = 8
    left_sign: int = 1
    right_sign: int = -1

    def validate(self):
        bounds = {
            "kp": (0, 200), "ki": (0, 100), "kd": (0, 40),
            "trim_deg": (-5, 5), "max_speed_m_s": (0.02, 0.8),
            "max_accel_m_s2": (0.1, 10), "filter_tau_s": (0.1, 10),
            "integral_limit": (0, 1), "fall_deg": (8, 35),
            "wheel_diameter_mm": (20, 400), "full_steps": (1, 1000),
        }
        for name, (low, high) in bounds.items():
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (float, int)):
                raise ValueError(f"{name} must be numeric")
            if not math.isfinite(value) or not low <= value <= high:
                raise ValueError(f"{name} must be between {low} and {high}")
        if int(self.full_steps) != self.full_steps:
            raise ValueError("full_steps must be an integer")
        if any(isinstance(getattr(self, key), bool) for key in
               ("microsteps", "left_sign", "right_sign")):
            raise ValueError("Microsteps and motor signs must be numeric, not boolean")
        if self.microsteps not in (1, 2, 4, 8, 16):
            raise ValueError("microsteps must match the A4988 wiring: 1/2/4/8/16")
        if self.left_sign not in (-1, 1) or self.right_sign not in (-1, 1):
            raise ValueError("Motor direction signs must be +1 or -1")
        self.full_steps = int(self.full_steps)
        self.microsteps = int(self.microsteps)
        self.left_sign, self.right_sign = int(self.left_sign), int(self.right_sign)
        return self

    @property
    def steps_per_m(self):
        return self.full_steps * self.microsteps / (math.pi * self.wheel_diameter_mm / 1000)

    def effective_speed(self, firmware_limit):
        return min(self.max_speed_m_s, firmware_limit / self.steps_per_m)

    @classmethod
    def from_dict(cls, data):
        if not isinstance(data, dict) or set(data) - {f.name for f in fields(cls)}:
            raise ValueError("Unknown settings fields")
        return cls(**data).validate()

    def to_dict(self):
        return asdict(self)


def stationary_reference(accels, gyros):
    if len(accels) < 30 or len(gyros) < 100:
        raise ValueError("Too few IMU samples; check the motion streams")
    gravity, bias = mean(accels), mean(gyros)
    accel_scatter = math.sqrt(sum(norm(tuple(a[i] - gravity[i] for i in range(3))) ** 2
                                  for a in accels) / len(accels))
    gyro_scatter = math.sqrt(sum(norm(tuple(g[i] - bias[i] for i in range(3))) ** 2
                                 for g in gyros) / len(gyros))
    if not 8.8 <= norm(gravity) <= 10.8:
        raise ValueError("Gravity magnitude is unexpected; hold the robot still")
    if accel_scatter > 0.20 or gyro_scatter > 0.025 or norm(bias) > 0.10:
        raise ValueError("Robot moved during calibration; hold it still and retry")
    return gravity, bias


class TiltEstimator:
    """Mount-independent pitch about an axis taught by a forward axle rotation.

    Accelerometer vectors rotate opposite to the body; cross(forward, reference)
    therefore gives the positive forward gyroscope axis. No camera extrinsics or
    assumed RealSense X/Y/Z mapping is required.
    """

    def __init__(self):
        self.reference = None
        self.axis = None
        self.bias = (0.0, 0.0, 0.0)
        self.angle = self.rate = 0.0
        self.last_gyro_ts = self.last_accel_ts = None
        self.accel_angle = 0.0

    def set_reference(self, gravity, bias):
        self.reference = unit(gravity)
        self.bias = tuple(bias)
        self.axis = None
        self.angle = self.rate = 0.0
        self.last_gyro_ts = self.last_accel_ts = None

    def teach_forward(self, gravity):
        if self.reference is None:
            raise ValueError("Capture upright first")
        forward = unit(gravity)
        angle = math.acos(clamp(dot(forward, self.reference), -1, 1))
        if not math.radians(5) <= angle <= math.radians(25):
            raise ValueError("Hold a forward tilt of 5-25 degrees around the wheel axle")
        self.axis = unit(cross(forward, self.reference))
        self.angle = self.accel_angle = angle
        self.last_gyro_ts = self.last_accel_ts = None
        return math.degrees(angle)

    def gravity_angle(self, accel):
        a = unit(accel)
        a = unit(tuple(a[i] - dot(a, self.axis) * self.axis[i] for i in range(3)))
        return math.atan2(dot(self.axis, cross(a, self.reference)), dot(a, self.reference))

    def gyro(self, vector, ts):
        if self.axis is None:
            return
        self.rate = dot(tuple(vector[i] - self.bias[i] for i in range(3)), self.axis)
        if self.last_gyro_ts is not None:
            dt = ts - self.last_gyro_ts
            if dt == 0:
                return  # Duplicate frame/timestamp: never integrate twice.
            if dt < 0 or dt > 0.05:
                raise ValueError("Gyroscope timestamp gap/reset; recalibrate")
            self.angle = wrap(self.angle + self.rate * dt)
        self.last_gyro_ts = ts

    def accel(self, vector, ts, tau):
        if self.axis is None:
            return
        if self.last_accel_ts == ts:
            return
        if self.last_accel_ts is not None and ts < self.last_accel_ts:
            raise ValueError("Accelerometer timestamp reset; recalibrate")
        dt = 0.004 if self.last_accel_ts is None else ts - self.last_accel_ts
        self.last_accel_ts = ts
        # Reject large translational accelerations; smaller ones still cause bias.
        if abs(norm(vector) - 9.80665) > 1.3:
            return
        self.accel_angle = self.gravity_angle(vector)
        alpha = 1 - math.exp(-min(dt, 0.1) / tau)
        self.angle = wrap(self.angle + alpha * wrap(self.accel_angle - self.angle))


class BalancePID:
    def __init__(self):
        self.reset()

    def reset(self):
        self.integral = self.speed = 0.0
        self.p = self.i = self.d = self.acceleration = 0.0
        self.saturated = False

    def update(self, angle, rate, dt, settings, firmware_limit=2000):
        if not all(math.isfinite(x) for x in (angle, rate, dt)) or not 0 < dt <= 0.05:
            raise ValueError("Invalid PID input or control-loop timing gap")
        error = wrap(angle - math.radians(settings.trim_deg))
        self.p, self.d = settings.kp * error, settings.kd * rate
        candidate = clamp(self.integral + error * dt,
                          -settings.integral_limit, settings.integral_limit)
        raw = self.p + settings.ki * candidate + self.d
        limit = settings.effective_speed(firmware_limit)
        # Freeze integral when it would drive either saturated limit farther out.
        worsening_accel = abs(raw) > settings.max_accel_m_s2 and raw * error > 0
        worsening_speed = abs(self.speed) >= limit - 1e-9 and self.speed * raw > 0
        if not (worsening_accel or worsening_speed):
            self.integral = candidate
        self.i = settings.ki * self.integral
        raw = self.p + self.i + self.d
        self.acceleration = clamp(raw, -settings.max_accel_m_s2, settings.max_accel_m_s2)
        requested = self.speed + self.acceleration * dt
        self.speed = clamp(requested, -limit, limit)
        self.saturated = abs(raw) > settings.max_accel_m_s2 or abs(requested) > limit
        steps = round(self.speed * settings.steps_per_m)
        return steps * settings.left_sign, steps * settings.right_sign
