"""Control geometry, limits, and stop behavior; no motor hardware access."""
import argparse
import math
import unittest
from unittest.mock import patch

from BalanceCore import BalancePID, Settings, TiltEstimator, cross, dot, stationary_reference, unit
from BalanceRobot import BalanceEngine, MegaLink


def rotate(v, axis, angle):
    c, s = math.cos(angle), math.sin(angle)
    axv = cross(axis, v)
    return tuple(v[i] * c + axv[i] * s + axis[i] * dot(axis, v) * (1 - c) for i in range(3))


class TiltTests(unittest.TestCase):
    def test_arbitrary_mounting_and_forward_sign(self):
        for gravity, axis in (((0, 9.80665, 0), (1, 0, 0)),
                              ((0, 0, -9.80665), (0, 1, 0)),
                              ((6, 7, 3), unit(cross((6, 7, 3), (1, -2, 4))))):
            with self.subTest(gravity=gravity):
                e = TiltEstimator()
                e.set_reference(gravity, (0, 0, 0))
                forward = rotate(gravity, axis, -math.radians(12))
                self.assertAlmostEqual(e.teach_forward(forward), 12)
                self.assertAlmostEqual(dot(e.axis, axis), 1)
                self.assertAlmostEqual(e.gravity_angle(gravity), 0)
                backward = rotate(gravity, axis, math.radians(8))
                self.assertAlmostEqual(math.degrees(e.gravity_angle(backward)), -8)

    def test_gyro_bias_and_timestamp_gap(self):
        e = TiltEstimator()
        e.set_reference((0, 9.80665, 0), (0.01, 0.02, -0.01))
        e.teach_forward(rotate((0, 9.80665, 0), (1, 0, 0), -0.2))
        e.gyro((0.11, 0.02, -0.01), 1.0)
        e.gyro((0.11, 0.02, -0.01), 1.01)
        self.assertAlmostEqual(e.angle, 0.201)
        with self.assertRaisesRegex(ValueError, "timestamp"):
            e.gyro((0.11, 0.02, -0.01), 1.5)

    def test_bad_or_moving_reference_is_rejected(self):
        with self.assertRaises(ValueError):
            stationary_reference([(0, 9.8, 0)] * 5, [(0, 0, 0)] * 100)
        with self.assertRaises(ValueError):
            stationary_reference([(0, 0, 0)] * 40, [(0, 0, 0)] * 150)
        moving = [(0, 9.8, 0)] * 20 + [(2, 9.8, 0)] * 20
        with self.assertRaises(ValueError):
            stationary_reference(moving, [(0, 0, 0)] * 150)

    def test_axis_requires_enough_forward_tilt(self):
        e = TiltEstimator()
        e.set_reference((0, 9.8, 0), (0, 0, 0))
        with self.assertRaises(ValueError):
            e.teach_forward((0, 9.8, 0))

    def test_duplicate_frames_do_not_integrate_twice(self):
        e = TiltEstimator()
        e.set_reference((0, 9.8, 0), (0, 0, 0))
        e.teach_forward(rotate((0, 9.8, 0), (1, 0, 0), -0.2))
        e.gyro((0.1, 0, 0), 1.0)
        e.gyro((0.1, 0, 0), 1.01)
        angle = e.angle
        e.gyro((0.1, 0, 0), 1.01)
        self.assertEqual(e.angle, angle)
        e.accel((0, 9.8, 0), 1.01, 1.5)
        angle = e.angle
        e.accel((0, 9.8, 0), 1.01, 1.5)
        self.assertEqual(e.angle, angle)


class PIDTests(unittest.TestCase):
    def test_forward_fall_requests_forward_acceleration(self):
        pid, settings = BalancePID(), Settings()
        left, right = pid.update(0.1, 0.1, 0.01, settings)
        self.assertGreater(pid.acceleration, 0)
        self.assertGreater(left, 0)
        self.assertLess(right, 0)
        speed = pid.speed
        pid.update(-0.1, -0.1, 0.01, settings)
        self.assertLess(pid.speed, speed)

    def test_speed_acceleration_limits_and_antiwindup(self):
        pid = BalancePID()
        settings = Settings(ki=20, max_speed_m_s=0.10, max_accel_m_s2=1)
        for _ in range(1000):
            left, right = pid.update(0.5, 1, 0.01, settings)
            self.assertLessEqual(abs(pid.speed), 0.10)
            self.assertLessEqual(abs(pid.acceleration), 1)
        self.assertAlmostEqual(pid.integral, 0)
        self.assertTrue(pid.saturated)
        self.assertLessEqual(abs(left), 402)

    def test_firmware_limit_changes_effective_speed(self):
        settings = Settings(microsteps=16, max_speed_m_s=0.8)
        pid = BalancePID()
        for _ in range(100):
            left, right = pid.update(0.2, 0, 0.01, settings)
        self.assertLessEqual(abs(left), 2000)
        self.assertAlmostEqual(settings.steps_per_m, 3200 / (math.pi * 0.127))

    def test_invalid_pid_input_fails_closed(self):
        pid = BalancePID()
        for angle, rate, dt in ((float("nan"), 0, 0.01), (0, 0, 0.051), (0, 0, -1)):
            with self.assertRaises(ValueError):
                pid.update(angle, rate, dt, Settings())

    def test_reset_and_settings_validation(self):
        pid = BalancePID()
        pid.update(0.1, 0, 0.01, Settings())
        pid.reset()
        self.assertEqual((pid.speed, pid.integral, pid.acceleration), (0, 0, 0))
        for data in ({"kp": float("nan")}, {"left_sign": 0}, {"microsteps": 3}, {"surprise": 1}):
            with self.assertRaises(ValueError):
                Settings.from_dict(data)

    def test_nominal_demo_model_recovers_small_lean(self):
        settings, pid = Settings(), BalancePID()
        theta, rate = math.radians(2), 0.0
        for _ in range(500):
            pid.update(theta, rate, 0.01, settings)
            acceleration = (9.80665 * math.sin(theta) - pid.acceleration * math.cos(theta)) / 0.28 - 0.1 * rate
            rate += acceleration * 0.01
            theta += rate * 0.01
        self.assertLess(abs(math.degrees(theta)), 0.1)


class StopTests(unittest.TestCase):
    def engine(self):
        args = argparse.Namespace(demo=True, imu_only=False, port=None, camera_serial=None)
        return BalanceEngine(args, Settings())

    def test_stale_data_and_uncalibrated_arm_are_rejected(self):
        e = self.engine()
        with self.assertRaisesRegex(ValueError, "Capture upright"):
            e.handle_action("arm", None, 10)
        e.last_accel = e.last_gyro = 1
        with self.assertRaisesRegex(ValueError, "stale"):
            e.check_freshness(2)

    def test_reference_never_arms_and_stop_clears_output(self):
        e = self.engine()
        e.armed = True
        e.pid.update(0.1, 0.1, 0.01, e.settings)
        e.last_left = e.last_right = 100
        e.handle_action("reference", None, 10)
        self.assertFalse(e.armed)
        self.assertIsNotNone(e.collecting)
        self.assertEqual(e.pid.speed, 0)
        self.assertEqual((e.last_left, e.last_right), (0, 0))
        e.handle_action("stop", None, 10.1)
        self.assertIsNone(e.collecting)

    def test_no_live_tuning_or_direction_change_while_armed(self):
        e = self.engine()
        e.armed = True
        with self.assertRaisesRegex(ValueError, "Disarm"):
            e.handle_action("settings", Settings(kp=25).to_dict(), 10)

    def test_duplicate_sensor_frames_cannot_refresh_watchdog(self):
        e = self.engine()
        class RepeatingIMU:
            def drain(self):
                return [("gyro", 1.0, (0, 0, 0), 10),
                        ("accel", 1.0, (0, 9.8, 0), 10)]
        e.source = RepeatingIMU()
        e.process_events(10)
        self.assertEqual(e.last_gyro, 10)
        # Same device timestamps, but new host arrivals.
        e.source.drain = lambda: [("gyro", 1.0, (0, 0, 0), 11),
                                 ("accel", 1.0, (0, 9.8, 0), 11)]
        e.process_events(11)
        self.assertEqual(e.last_gyro, 10)
        with self.assertRaisesRegex(ValueError, "stale"):
            e.check_freshness(11)


class SerialTests(unittest.TestCase):
    def link(self):
        class MemorySerial:
            def __init__(self):
                self.input = bytearray()
                self.output = bytearray()
            @property
            def in_waiting(self):
                return len(self.input)
            def read(self, n):
                value = bytes(self.input[:n])
                del self.input[:n]
                return value
            def write(self, data):
                self.output.extend(data)
                return len(data)
        link = MegaLink.__new__(MegaLink)
        link.serial = MemorySerial()
        link.buffer = bytearray()
        link.session = 123
        link.sequence = 0
        link.sent = {}
        link.ack_ms = 0
        link.last_rx = 0
        link.last_status = None
        return link

    def test_partial_status_and_acknowledgement(self):
        link = self.link()
        link.serial.input.extend(b"T 123 8 1")
        link.poll(10)
        self.assertIsNone(link.last_status)
        link.sent = {8: 9.96, 9: 9.97}
        link.serial.input.extend(b" 0 100 -100\n")
        link.poll(10)
        self.assertEqual(link.last_status, (123, 8, 1, 0, 100, -100))
        self.assertAlmostEqual(link.ack_ms, 40)
        self.assertEqual(link.sent, {9: 9.97})

    def test_new_session_explicit_stop_and_integer_commands(self):
        link = self.link()
        with patch("BalanceRobot.secrets.randbelow", side_effect=[122, 456]):
            link.arm()
        self.assertEqual(link.session, 457)
        link.speed(100, -100)
        link.stop()
        self.assertEqual(bytes(link.serial.output), b"A 457\nV 457 1 100 -100\nS\n")
        self.assertEqual(link.sent, {})

    def test_incomplete_write_raises(self):
        link = self.link()
        link.serial.write = lambda data: len(data) - 1
        with self.assertRaisesRegex(RuntimeError, "Incomplete"):
            link.speed(100, 100)


if __name__ == "__main__":
    unittest.main()
