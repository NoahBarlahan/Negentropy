"""Exercise the real Qt dashboard/worker in DEMO mode, with zero hardware access."""
import argparse
from pathlib import Path
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from BalanceCore import Settings
from BalanceRobot import dashboard
from PySide6 import QtCore, QtGui, QtWidgets


app = QtWidgets.QApplication([])
QtGui.QFontDatabase.addApplicationFont("C:/Windows/Fonts/segoeui.ttf")
app.setFont(QtGui.QFont("Segoe UI", 10))
test_args = argparse.Namespace(demo=True, imu_only=False, port=None, camera_serial=None,
                               settings=Path(".balance-build/smoke-settings.json"))
failures = []


def main_window():
    return next(w for w in app.topLevelWidgets() if isinstance(w, QtWidgets.QMainWindow))


def press(text):
    next(b for b in main_window().findChildren(QtWidgets.QPushButton) if b.text() == text).click()


def assert_state(text):
    labels = "\n".join(label.text() for label in main_window().findChildren(QtWidgets.QLabel))
    if text not in labels:
        failures.append(f"Missing state {text!r}: {labels[:350]}")


def tilt(value):
    next(w for w in main_window().findChildren(QtWidgets.QDoubleSpinBox) if w.suffix() == "°").setValue(value)


def allow():
    next(w for w in main_window().findChildren(QtWidgets.QCheckBox)
         if w.text().startswith("Demo:")).setChecked(True)


def capture():
    press("Save settings")
    press("Load settings")


def recording(enabled):
    next(w for w in main_window().findChildren(QtWidgets.QCheckBox)
         if w.text() == "Record CSV (50 Hz)").setChecked(enabled)


def report():
    assert_state("Disarmed")
    workers = [t for t in threading.enumerate() if t.name == "balance-control"]
    if len(workers) != 1 or not workers[0].is_alive():
        failures.append("Control worker stopped unexpectedly")
    if workers and workers[0].fault_count:
        failures.append(f"Unexpected runtime fault: {workers[0].last_fault}")
    if workers and workers[0].log_path:
        lines = workers[0].log_path.read_text(encoding="utf-8").splitlines()
        if len(lines) < 10 or "gyro_age_ms" not in lines[0]:
            failures.append("CSV recording is missing samples or telemetry columns")
    else:
        failures.append("CSV recording did not create a file")
    print("GUI SMOKE: " + ("PASS" if not failures else "FAIL " + repr(failures)))
    # Screenshot rendering can pause Python while C++ paints. Capture after
    # assertions and only while stopped, then immediately close the worker.
    # Stop the worker before an offscreen screenshot pauses the interpreter.
    for worker in workers:
        worker.quit.set()
        worker.join(timeout=1)
    main_window().grab().save(".balance-build/dashboard-smoke.png")
    main_window().close()


def timed(ms, action):
    def wrapped():
        try:
            action()
        except Exception as error:
            failures.append(str(error))
    QtCore.QTimer.singleShot(ms, wrapped)


def schedule():
    # Timers start after dashboard construction / entry to the Qt event loop.
    timed(300, lambda: press("Capture upright"))
    timed(2600, lambda: assert_state("Upright captured"))
    timed(2800, lambda: tilt(10))
    timed(3100, lambda: press("Teach forward tilt"))
    timed(5500, lambda: assert_state("Forward axis learned"))
    timed(5700, lambda: tilt(0))
    # Complementary filter converges from the held teaching pose in ~3 seconds.
    timed(9000, allow)
    timed(9050, lambda: recording(True))
    timed(9200, lambda: press("ARM"))
    timed(9500, lambda: assert_state("PREVIEW"))
    timed(9800, lambda: press("STOP [Space]"))
    timed(9850, lambda: recording(False))
    timed(10100, capture)
    timed(10400, report)


QtCore.QTimer.singleShot(0, schedule)
dashboard(test_args, Settings())
if not test_args.settings.exists():
    failures.append("Settings save did not create the file")
raise SystemExit(1 if failures else 0)
