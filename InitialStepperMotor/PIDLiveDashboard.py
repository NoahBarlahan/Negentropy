"""PyQtGraph dashboard process for TwoAxisCameraControl.py."""

from collections import deque
from queue import Empty
import math
import sys


ROLLING_WINDOW_SECONDS = 20.0
EXPECTED_SAMPLE_RATE_HZ = 20
MAX_POINTS = round(ROLLING_WINDOW_SECONDS * EXPECTED_SAMPLE_RATE_HZ)


def run_dashboard(data_queue, stop_event) -> None:
    """Run the Qt event loop in a separate process from camera control."""

    try:
        import pyqtgraph as pg
        from PySide6 import QtCore, QtWidgets
    except ModuleNotFoundError as error:
        print(f"Live dashboard unavailable: {error}")
        print("Install it with: python -m pip install -r requirements.txt")
        return

    pg.setConfigOptions(antialias=True, background="#171a21", foreground="#d7dce5")
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)

    window = QtWidgets.QMainWindow()
    window.setWindowTitle("Two-Axis Camera PID Dashboard")
    window.resize(1450, 900)
    central = QtWidgets.QWidget()
    layout = QtWidgets.QVBoxLayout(central)
    status_label = QtWidgets.QLabel("Waiting for camera data...")
    status_label.setStyleSheet("font-size: 15px; padding: 8px;")
    layout.addWidget(status_label)
    graphics = pg.GraphicsLayoutWidget()
    layout.addWidget(graphics, 1)
    window.setCentralWidget(central)

    error_plot = graphics.addPlot(row=0, col=0, title="Image-center error")
    error_plot.setLabel("left", "Error", units="px")
    error_plot.setLabel("bottom", "Recent time", units="s")
    error_plot.showGrid(x=True, y=True, alpha=0.25)
    error_plot.addLine(y=0, pen=pg.mkPen("#87909f", width=1))
    error_plot.addLegend(offset=(10, 10))
    x_error_curve = error_plot.plot(pen=pg.mkPen("#53a8ff", width=2), name="X -> Z horizontal")
    y_error_curve = error_plot.plot(pen=pg.mkPen("#ff9f43", width=2), name="Y -> Y vertical")

    output_plot = graphics.addPlot(row=0, col=1, title="PID angular-velocity request")
    output_plot.setLabel("left", "Command", units="deg/s")
    output_plot.setLabel("bottom", "Recent time", units="s")
    output_plot.showGrid(x=True, y=True, alpha=0.25)
    output_plot.addLine(y=0, pen=pg.mkPen("#87909f", width=1))
    output_plot.addLegend(offset=(10, 10))
    z_output_curve = output_plot.plot(pen=pg.mkPen("#bf75ff", width=2), name="Z horizontal")
    y_output_curve = output_plot.plot(pen=pg.mkPen("#59d98e", width=2), name="Y vertical")

    z_plot = graphics.addPlot(row=1, col=0, title="Z horizontal PID terms")
    z_plot.setLabel("left", "Contribution", units="deg/s")
    z_plot.setLabel("bottom", "Recent time", units="s")
    z_plot.showGrid(x=True, y=True, alpha=0.25)
    z_plot.addLine(y=0, pen=pg.mkPen("#87909f", width=1))
    z_plot.addLegend(offset=(10, 10))
    z_p_curve = z_plot.plot(pen=pg.mkPen("#53a8ff", width=2), name="P")
    z_i_curve = z_plot.plot(pen=pg.mkPen("#59d98e", width=2), name="I")
    z_d_curve = z_plot.plot(pen=pg.mkPen("#ff6b6b", width=2), name="D")

    y_plot = graphics.addPlot(row=1, col=1, title="Y vertical PID terms")
    y_plot.setLabel("left", "Contribution", units="deg/s")
    y_plot.setLabel("bottom", "Recent time", units="s")
    y_plot.showGrid(x=True, y=True, alpha=0.25)
    y_plot.addLine(y=0, pen=pg.mkPen("#87909f", width=1))
    y_plot.addLegend(offset=(10, 10))
    y_p_curve = y_plot.plot(pen=pg.mkPen("#53a8ff", width=2), name="P")
    y_i_curve = y_plot.plot(pen=pg.mkPen("#59d98e", width=2), name="I")
    y_d_curve = y_plot.plot(pen=pg.mkPen("#ff6b6b", width=2), name="D")

    names = (
        "time", "x_error", "y_error", "z_output", "y_output",
        "z_p", "z_i", "z_d", "y_p", "y_i", "y_d",
    )
    history = {name: deque(maxlen=MAX_POINTS) for name in names}
    latest = None

    def update_dashboard() -> None:
        nonlocal latest
        if stop_event.is_set():
            window.close()
            return

        while True:
            try:
                sample = data_queue.get_nowait()
            except Empty:
                break
            if sample is None:
                window.close()
                return
            latest = sample
            for name in names:
                history[name].append(float(sample[name]))

        if latest is None or not history["time"]:
            return

        newest_time = history["time"][-1]
        relative_time = [value - newest_time for value in history["time"]]
        x_error_curve.setData(relative_time, history["x_error"])
        y_error_curve.setData(relative_time, history["y_error"])
        z_output_curve.setData(relative_time, history["z_output"])
        y_output_curve.setData(relative_time, history["y_output"])
        z_p_curve.setData(relative_time, history["z_p"])
        z_i_curve.setData(relative_time, history["z_i"])
        z_d_curve.setData(relative_time, history["z_d"])
        y_p_curve.setData(relative_time, history["y_p"])
        y_i_curve.setData(relative_time, history["y_i"])
        y_d_curve.setData(relative_time, history["y_d"])

        x_rms = math.sqrt(sum(value * value for value in history["x_error"]) / len(history["x_error"]))
        y_rms = math.sqrt(sum(value * value for value in history["y_error"]) / len(history["y_error"]))
        hand_text = "HAND" if latest["hand_detected"] else "NO HAND"
        serial_text = "SERIAL OK" if latest["serial_ok"] else "NO MOTOR POSITION"
        status_label.setText(
            f"{hand_text}  |  {serial_text}  |  Camera {latest['fps']:.1f} FPS  |  "
            f"RMS error: X {x_rms:.1f}px, Y {y_rms:.1f}px  |  "
            f"Z target {latest['z_target']:+.1f}°, Y target {latest['y_target']:+.1f}°"
        )

    timer = QtCore.QTimer()
    timer.timeout.connect(update_dashboard)
    timer.start(50)
    window.show()
    app.exec()


if __name__ == "__main__":
    print("Run TwoAxisCameraControl.py; it launches this dashboard automatically.")
