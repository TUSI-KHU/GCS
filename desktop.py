import sys
import time
import math
import csv
import argparse
from pathlib import Path
from collections import deque
from datetime import datetime

from PyQt6.QtWidgets import (
    QApplication,
    QWidget,
    QPushButton,
    QLabel,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QMessageBox,
    QInputDialog,
)

from PyQt6.QtCore import QThread, QTimer, pyqtSignal

import pyqtgraph as pg


ROOT_DIR = Path(__file__).resolve().parent
LOG_DIR = ROOT_DIR / "logs"

try:
    from uplink import PyroUplink
except ImportError:
    PyroUplink = None


class CSVLogger:
    def __init__(self):
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        LOG_DIR.mkdir(exist_ok=True)

        self.file = open(
            LOG_DIR / f"flight_log_{ts}.csv",
            "w",
            newline=""
        )

        self.writer = csv.writer(self.file)

        self.writer.writerow([
            "timestamp",
            "altitude",
            "velocity",
            "accel_z",
            "gyro_z",
            "battery"
        ])

    def log(self, row):
        self.writer.writerow(row)
        self.file.flush()

    def close(self):
        self.file.close()


class PlotWidget(pg.PlotWidget):
    def __init__(self, title, ylabel):
        super().__init__()

        self.setTitle(title)

        self.setLabel("left", ylabel)
        self.setLabel("bottom", "Time")

        self.showGrid(x=True, y=True)

        self.curve = self.plot()

    def update_plot(self, x, y):
        self.curve.setData(x, y)


class DeployWorker(QThread):
    finished = pyqtSignal(dict)

    def __init__(self, uplink):
        super().__init__()
        self.uplink = uplink

    def run(self):
        result = self.uplink.force_deploy()
        self.finished.emit(result.to_dict())


class GCS(QWidget):
    def __init__(self, uplink=None, uplink_error=None):
        super().__init__()

        self.setWindowTitle("Rocket GCS")
        self.resize(1400, 800)

        self.logger = CSVLogger()
        self.uplink = uplink
        self.deploy_worker = None

        self.time_data = deque(maxlen=1000)

        self.alt_data = deque(maxlen=1000)
        self.vel_data = deque(maxlen=1000)

        self.acc_data = deque(maxlen=1000)
        self.gyro_data = deque(maxlen=1000)

        self.alt_label = QLabel("Altitude: 0")
        self.vel_label = QLabel("Velocity: 0")
        self.bat_label = QLabel("Battery: 0")
        self.pyro_label = QLabel(self._format_uplink_status(uplink_error))

        self.arm_btn = QPushButton("ARM")
        self.disarm_btn = QPushButton("DISARM")

        self.parachute_btn = QPushButton("FORCE PARACHUTE")
        self.reset_btn = QPushButton("FORCE RESET")

        self.alt_plot = PlotWidget("Altitude", "m")
        self.vel_plot = PlotWidget("Velocity", "m/s")

        self.acc_plot = PlotWidget("Accel Z", "m/s²")
        self.gyro_plot = PlotWidget("Gyro Z", "deg/s")

        top = QHBoxLayout()

        top.addWidget(self.alt_label)
        top.addWidget(self.vel_label)
        top.addWidget(self.bat_label)
        top.addWidget(self.pyro_label)

        buttons = QHBoxLayout()

        buttons.addWidget(self.arm_btn)
        buttons.addWidget(self.disarm_btn)

        buttons.addWidget(self.parachute_btn)
        buttons.addWidget(self.reset_btn)

        grid = QGridLayout()

        grid.addWidget(self.alt_plot, 0, 0)
        grid.addWidget(self.vel_plot, 0, 1)

        grid.addWidget(self.acc_plot, 1, 0)
        grid.addWidget(self.gyro_plot, 1, 1)

        root = QVBoxLayout()

        root.addLayout(top)
        root.addLayout(buttons)
        root.addLayout(grid)

        self.setLayout(root)

        self.timer = QTimer()

        self.timer.timeout.connect(self.update_loop)

        self.timer.start(20)

        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.update_pyro_status)
        self.status_timer.start(1000)

        self.parachute_btn.clicked.connect(self.force_parachute)
        self.reset_btn.clicked.connect(self.reset_view)
        self.arm_btn.setEnabled(False)
        self.disarm_btn.setEnabled(False)
        self.parachute_btn.setEnabled(self.uplink is not None and self.uplink.is_open())

    def _format_uplink_status(self, error=None):
        if error:
            return f"PYRO: disconnected ({error})"
        if self.uplink is None:
            return "PYRO: unavailable"
        mode = "simulate" if self.uplink.simulate else (self.uplink.port or "auto")
        state = "connected" if self.uplink.is_open() else "disconnected"
        return f"PYRO: {state} [{mode}]"

    def update_pyro_status(self):
        self.pyro_label.setText(self._format_uplink_status())
        self.parachute_btn.setEnabled(
            self.uplink is not None
            and self.uplink.is_open()
            and self.deploy_worker is None
        )

    def force_parachute(self):
        if self.uplink is None or not self.uplink.is_open():
            QMessageBox.warning(self, "PYRO", "PYRO uplink is not connected.")
            return

        token, ok = QInputDialog.getText(
            self,
            "Confirm PYRO",
            'Type "DEPLOY" to force parachute deployment:',
        )
        if not ok or token != "DEPLOY":
            return

        self.parachute_btn.setEnabled(False)
        self.pyro_label.setText("PYRO: deploying...")
        self.deploy_worker = DeployWorker(self.uplink)
        self.deploy_worker.finished.connect(self.on_deploy_finished)
        self.deploy_worker.start()

    def on_deploy_finished(self, result):
        self.deploy_worker = None
        self.update_pyro_status()
        message = result.get("message") or str(result)
        if result.get("success"):
            QMessageBox.information(self, "PYRO", message)
        else:
            QMessageBox.warning(self, "PYRO", message)

    def reset_view(self):
        self.time_data.clear()
        self.alt_data.clear()
        self.vel_data.clear()
        self.acc_data.clear()
        self.gyro_data.clear()

    def update_loop(self):

        now = time.time()

        timestamp = int(now * 1000)

        altitude = 100 + 20 * math.sin(now)
        velocity = 15 * math.cos(now)

        accel_z = 9.81 + 0.5 * math.sin(now * 3)
        gyro_z = 30 * math.sin(now * 2)

        battery = 8.2

        self.logger.log([
            timestamp,
            altitude,
            velocity,
            accel_z,
            gyro_z,
            battery
        ])

        t = timestamp / 1000.0

        self.time_data.append(t)

        self.alt_data.append(altitude)
        self.vel_data.append(velocity)

        self.acc_data.append(accel_z)
        self.gyro_data.append(gyro_z)

        self.alt_label.setText(
            f"Altitude: {altitude:.2f} m"
        )

        self.vel_label.setText(
            f"Velocity: {velocity:.2f} m/s"
        )

        self.bat_label.setText(
            f"Battery: {battery:.2f} V"
        )

        x = list(self.time_data)

        self.alt_plot.update_plot(
            x,
            list(self.alt_data)
        )

        self.vel_plot.update_plot(
            x,
            list(self.vel_data)
        )

        self.acc_plot.update_plot(
            x,
            list(self.acc_data)
        )

        self.gyro_plot.update_plot(
            x,
            list(self.gyro_data)
        )

    def closeEvent(self, event):
        self.logger.close()
        if self.uplink is not None:
            self.uplink.close()
        event.accept()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Integrated NURA GCS")
    parser.add_argument("--serial-port", default=None, help="Teensy serial port")
    parser.add_argument("--simulate", action="store_true", help="Run without hardware")
    args, qt_args = parser.parse_known_args()
    if not args.simulate and not args.serial_port:
        parser.error("--serial-port is required unless --simulate is used")

    uplink = None
    uplink_error = None
    if PyroUplink is None:
        uplink_error = "pyro module missing"
    else:
        uplink = PyroUplink(port=args.serial_port, simulate=args.simulate)
        try:
            uplink.open()
        except RuntimeError as exc:
            uplink_error = str(exc)

    app = QApplication([sys.argv[0], *qt_args])

    win = GCS(uplink=uplink, uplink_error=uplink_error)

    win.show()

    sys.exit(app.exec())
