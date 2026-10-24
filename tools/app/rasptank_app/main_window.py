"""Main window of the RaspTank app: status, teleoperation, calibration, simulation."""
import math
import os
import signal
import subprocess
from pathlib import Path

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont, QImage, QPixmap
from PyQt5.QtWidgets import (QCheckBox, QComboBox, QFormLayout, QGridLayout, QGroupBox,
                             QHBoxLayout, QLabel, QMainWindow, QMessageBox, QPlainTextEdit,
                             QPushButton, QSlider, QSpinBox, QTabWidget, QVBoxLayout, QWidget)

from . import status
from .calib_page import CalibrationPage

ROOT = Path(__file__).resolve().parents[3]
POSES = ['plegado', 'buscar', 'pre_coger', 'cerrar', 'abrir', 'transportar', 'soltar']
WORLDS = ['mision', 'linea', 'vacio', 'calibracion']


def big_button(text, colour, height=48):
    b = QPushButton(text)
    b.setMinimumHeight(height)
    b.setStyleSheet(f'QPushButton {{ background: {colour}; color: white; font-weight: bold; '
                    f'font-size: 15px; border-radius: 6px; padding: 4px 14px; }}'
                    f'QPushButton:disabled {{ background: #bdc3c7; }}')
    return b


class MainWindow(QMainWindow):
    def __init__(self, connect, domain=status.SIM_DOMAIN):
        """`connect(domain)` returns a RosLink (or raises)."""
        super().__init__()
        self.connect_fn = connect
        self.link = None
        self.sim_process = None
        self.drive_cmd = None
        self.setWindowTitle('RaspTank')
        self.resize(1100, 760)

        central = QWidget()
        layout = QVBoxLayout(central)
        layout.addLayout(self._header(domain))
        self.tabs = QTabWidget()
        self.tabs.addTab(self._status_page(), 'Estado')
        self.tabs.addTab(self._drive_page(), 'Mover')
        self.calibration = CalibrationPage(self)
        self.tabs.addTab(self.calibration, 'Calibración')
        self.tabs.addTab(self._sim_page(), 'Simulación')
        layout.addWidget(self.tabs)
        self.setCentralWidget(central)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(200)
        self.drive_timer = QTimer(self)
        self.drive_timer.timeout.connect(self._drive_tick)
        self.drive_timer.start(100)
        self.reconnect()

    # ------------------------------------------------------------ header

    def _header(self, domain):
        row = QHBoxLayout()
        self.mode_label = QLabel('SIN CONEXIÓN')
        self.mode_label.setAlignment(Qt.AlignCenter)
        self.mode_label.setMinimumWidth(260)
        font = QFont()
        font.setPointSize(15)
        font.setBold(True)
        self.mode_label.setFont(font)
        row.addWidget(self.mode_label)
        row.addWidget(QLabel('Dominio ROS:'))
        self.domain = QSpinBox()
        self.domain.setRange(0, 232)
        self.domain.setValue(domain)
        self.domain.setToolTip('41 = simulación; cada robot real tiene el suyo (p. ej. 10)')
        row.addWidget(self.domain)
        b = QPushButton('Conectar')
        b.clicked.connect(self.reconnect)
        row.addWidget(b)
        row.addStretch(1)
        self.rearm_button = big_button('Rearmar', '#27ae60')
        self.rearm_button.clicked.connect(self.rearm)
        row.addWidget(self.rearm_button)
        self.stop_button = big_button('PARADA DE EMERGENCIA', '#c0392b', 56)
        self.stop_button.clicked.connect(self.emergency_stop)
        row.addWidget(self.stop_button)
        return row

    def reconnect(self):
        if self.link is not None:
            self.calibration.cancel()
            self.link.close()
            self.link = None
        try:
            self.link = self.connect_fn(self.domain.value())
            self.statusBar().showMessage(f'Conectado al dominio {self.domain.value()}', 4000)
        except Exception as e:  # noqa: BLE001  (no ROS: the app still opens)
            self.statusBar().showMessage(f'Sin ROS 2: {e}')

    def emergency_stop(self):
        self.drive_cmd = None
        self.calibration.cancel()
        if self.link is not None:
            self.link.drive(0.0, 0.0)
            self.link.emergency_stop(True)

    def rearm(self):
        if self.link is None:
            return
        if self.real_robot() and QMessageBox.question(
                self, 'Rearmar', 'El robot real podrá volver a moverse. ¿Está todo despejado?') \
                != QMessageBox.Yes:
            return
        self.link.emergency_stop(False)

    def real_robot(self):
        return self.link is not None and self.link.snapshot().driver == 'pca9685'

    # ------------------------------------------------------------ status page

    def _status_page(self):
        page = QWidget()
        layout = QHBoxLayout(page)
        box = QGroupBox('Robot')
        form = QFormLayout(box)
        self.fields = {}
        for key, label in [('base', 'Base (motores)'), ('battery', 'Batería'),
                           ('distance', 'Ultrasonido'), ('line', 'Sensores de línea'),
                           ('pose', 'Odometría (lo que cree)'), ('truth', 'Posición real (sim.)'),
                           ('arm', 'Brazo'), ('governor', 'Gobernador'), ('camera', 'Cámara'),
                           ('mission', 'Misión')]:
            self.fields[key] = QLabel('—')
            form.addRow(label + ':', self.fields[key])
        self.diagnosis = QLabel('')
        self.diagnosis.setWordWrap(True)
        form.addRow(self.diagnosis)
        layout.addWidget(box, 1)
        self.camera = QLabel('sin imagen')
        self.camera.setAlignment(Qt.AlignCenter)
        self.camera.setMinimumSize(480, 360)
        self.camera.setStyleSheet('background: #222; color: #aaa;')
        layout.addWidget(self.camera, 1)
        return page

    def refresh(self):
        s = self.link.snapshot() if self.link is not None else status.Snapshot()
        text, colour = status.mode(s)
        self.mode_label.setText(text)
        self.mode_label.setStyleSheet(f'background: {colour}; color: white; border-radius: 6px; '
                                      f'padding: 6px;')
        alive = status.base_alive(s)
        self.rearm_button.setVisible(bool(s.stopped))
        f = self.fields
        f['base'].setText(('responde · ' + {'fake': 'simulada', 'pca9685': 'motores reales'}
                           .get(s.driver, '?')) if alive else 'no responde')
        f['battery'].setText(status.fmt(s.voltage, '{:.2f} V') +
                             (f'  ({s.battery_pct:.0%})' if s.battery_pct is not None else ''))
        f['distance'].setText(status.fmt(s.distance, '{:.2f} m'))
        f['line'].setText('—' if s.line is None else
                          '  '.join(('■' if v else '□') for v in s.line) + '   (izq. centro der.)')
        f['pose'].setText(status.fmt(s.pose, 'x {0[0]:.2f}  y {0[1]:.2f}  {0[2]:.0f}°'))
        f['truth'].setText(status.fmt(s.truth, 'x {0[0]:.2f}  y {0[1]:.2f}  {0[2]:.0f}°'))
        f['arm'].setText(s.arm or '—')
        f['governor'].setText('—' if s.governor is None else
                              ('FRENANDO (obstáculo delante)' if s.governor else 'libre'))
        f['camera'].setText(f'{s.camera_hz:.1f} imágenes/s')
        f['mission'].setText(s.mission or '—')
        self.diagnosis.setText(status.diagnosis(s))
        if self.tabs.currentIndex() == 0 and self.link is not None:
            self._show_camera()
        self.calibration.refresh(s)
        self._refresh_sim()

    def _show_camera(self):
        jpeg = self.link.last_jpeg()
        if not jpeg:
            return
        image = QImage.fromData(jpeg, 'JPG')
        if not image.isNull():
            self.camera.setPixmap(QPixmap.fromImage(image).scaled(
                self.camera.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    # ------------------------------------------------------------ drive page

    def _drive_page(self):
        page = QWidget()
        layout = QHBoxLayout(page)
        box = QGroupBox('Orugas (mantén pulsado)')
        grid = QGridLayout(box)
        for text, (r, c), (v, w) in [('▲', (0, 1), (1, 0)), ('◀', (1, 0), (0, 1)),
                                     ('▶', (1, 2), (0, -1)), ('▼', (2, 1), (-1, 0))]:
            b = QPushButton(text)
            b.setMinimumSize(80, 64)
            b.setStyleSheet('font-size: 24px;')
            b.pressed.connect(lambda v=v, w=w: self._drive_start(v, w))
            b.released.connect(self._drive_stop)
            grid.addWidget(b, r, c)
        self.speed = QSlider(Qt.Horizontal)
        self.speed.setRange(2, 20)
        self.speed.setValue(8)
        self.speed_label = QLabel()
        self.speed.valueChanged.connect(
            lambda v: self.speed_label.setText(f'{v / 100:.2f} m/s · {v * 6:.0f} °/s'))
        self.speed.valueChanged.emit(self.speed.value())
        grid.addWidget(QLabel('Velocidad:'), 3, 0)
        grid.addWidget(self.speed, 3, 1)
        grid.addWidget(self.speed_label, 3, 2)
        layout.addWidget(box)

        arm = QGroupBox('Brazo y cámara')
        v = QVBoxLayout(arm)
        for name in POSES:
            b = QPushButton(name)
            b.clicked.connect(lambda _, n=name: self.link and self.link.arm_pose(n))
            v.addWidget(b)
        v.addWidget(QLabel('Inclinación de la cámara (° hacia abajo):'))
        tilt = QSlider(Qt.Horizontal)
        tilt.setRange(-20, 60)
        tilt.setValue(20)
        tilt.sliderReleased.connect(
            lambda: self.link and self.link.camera_tilt(math.radians(tilt.value())))
        v.addWidget(tilt)
        v.addStretch(1)
        layout.addWidget(arm)
        return page

    def _drive_start(self, v, w):
        if self.link is None:
            return
        k = self.speed.value() / 100.0
        self.drive_cmd = (v * k, w * math.radians(self.speed.value() * 6))

    def _drive_stop(self):
        self.drive_cmd = None
        if self.link is not None:
            self.link.drive(0.0, 0.0)

    def _drive_tick(self):
        # repeated while the button is held: if the app hangs, base_node's
        # watchdog stops the robot
        if self.drive_cmd is not None and self.link is not None:
            if not self.isActiveWindow():
                self._drive_stop()
                return
            self.link.drive(*self.drive_cmd)

    # ------------------------------------------------------------ simulation page

    def _sim_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        row = QHBoxLayout()
        row.addWidget(QLabel('Mundo:'))
        self.world = QComboBox()
        self.world.addItems(WORLDS)
        row.addWidget(self.world)
        self.sim_options = {}
        for key, text, default in [('rviz', 'RViz', True), ('mission', 'Misión', False),
                                   ('route', 'Rutas', False), ('governor', 'Gobernador', True)]:
            c = QCheckBox(text)
            c.setChecked(default)
            self.sim_options[key] = c
            row.addWidget(c)
        row.addStretch(1)
        self.sim_start = QPushButton('Abrir simulación')
        self.sim_start.clicked.connect(self.start_sim)
        self.sim_stop = QPushButton('Cerrar simulación')
        self.sim_stop.clicked.connect(self.stop_sim)
        row.addWidget(self.sim_start)
        row.addWidget(self.sim_stop)
        layout.addLayout(row)
        self.sim_log = QPlainTextEdit()
        self.sim_log.setReadOnly(True)
        self.sim_log.setMaximumBlockCount(2000)
        layout.addWidget(self.sim_log)
        return page

    def start_sim(self):
        if self.sim_process is not None and self.sim_process.poll() is None:
            return
        args = [str(ROOT / 'scripts' / 'sim.sh'), self.world.currentText()]
        args += [f'{k}:={"true" if c.isChecked() else "false"}' for k, c in self.sim_options.items()]
        env = dict(os.environ, ROS_DOMAIN_ID=str(self.domain.value()))
        self.sim_log.appendPlainText('$ ' + ' '.join(args))
        self.sim_process = subprocess.Popen(args, env=env, stdout=subprocess.PIPE,
                                            stderr=subprocess.STDOUT, text=True, bufsize=1,
                                            start_new_session=True)
        os.set_blocking(self.sim_process.stdout.fileno(), False)

    def stop_sim(self):
        p = self.sim_process
        if p is not None and p.poll() is None:
            os.killpg(p.pid, signal.SIGINT)

    def _refresh_sim(self):
        p = self.sim_process
        running = p is not None and p.poll() is None
        self.sim_start.setEnabled(not running and not self.real_robot())
        self.sim_stop.setEnabled(running)
        if p is not None and p.stdout is not None:
            try:
                chunk = p.stdout.read()
            except (OSError, ValueError):
                chunk = None
            if chunk:
                self.sim_log.appendPlainText(chunk.rstrip())

    def closeEvent(self, event):
        self.calibration.cancel()
        self.stop_sim()
        if self.link is not None:
            self.link.close()
        super().closeEvent(event)
