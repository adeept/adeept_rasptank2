"""Calibration tab: a step by step wizard over rasptank_calibration.flow.Flow.

The movements run in a worker thread (they take seconds); the results come
back to the GUI through Qt signals. «Cancelar» and the emergency stop abort
the movement at once.
"""
import threading

from PyQt5.QtCore import QObject, pyqtSignal
from PyQt5.QtWidgets import (QButtonGroup, QCheckBox, QDoubleSpinBox, QFormLayout, QGroupBox,
                             QHBoxLayout, QLabel, QListWidget, QMessageBox, QPlainTextEdit,
                             QPushButton, QRadioButton, QSpinBox, QTableWidget, QTableWidgetItem,
                             QVBoxLayout, QWidget)

from rasptank_core.calibration import PARAMS

LABELS = {
    'max_track_speed': ('Velocidad de oruga a tope', 'm/s', 3),
    'track_separation': ('Separación efectiva de orugas', 'm', 3),
    'min_throttle': ('Zona muerta (potencia mínima)', '', 3),
    'left_gain': ('Ganancia oruga izquierda', '', 3),
    'right_gain': ('Ganancia oruga derecha', '', 3),
    'battery_reference': ('Voltaje de referencia', 'V', 2),
}
ICONS = {'pendiente': '○', 'midiendo': '◐', 'hecho': '●', 'error': '✕', 'saltado': '–'}


class Worker(QObject):
    finished = pyqtSignal(object)
    failed = pyqtSignal(str)

    def run(self, fn):
        def target():
            try:
                self.finished.emit(fn())
            except Exception as e:  # noqa: BLE001  (shown to the user)
                self.failed.emit(str(e) or type(e).__name__)
        threading.Thread(target=target, daemon=True).start()


class CalibrationPage(QWidget):
    def __init__(self, window):
        super().__init__()
        self.window = window
        self.flow = None
        self.busy = False
        self.worker = Worker()
        self.worker.finished.connect(self._done)
        self.worker.failed.connect(self._failed)
        layout = QHBoxLayout(self)

        left = QVBoxLayout()
        options = QGroupBox('Modo')
        o = QVBoxLayout(options)
        self.manual = QRadioButton('Manual: el robot se mueve y tú mides con una cinta métrica')
        self.auto = QRadioButton('Automático: la cámara mide con el marcador ArUco (a ~1 m)')
        self.manual.setChecked(True)
        group = QButtonGroup(self)
        group.addButton(self.manual)
        group.addButton(self.auto)
        o.addWidget(self.manual)
        o.addWidget(self.auto)
        row = QHBoxLayout()
        row.addWidget(QLabel('Rondas recta + giro:'))
        self.rounds = QSpinBox()
        self.rounds.setRange(1, 3)
        self.rounds.setValue(2)
        row.addWidget(self.rounds)
        row.addStretch(1)
        o.addLayout(row)
        self.dead_band = QCheckBox('Medir la zona muerta')
        self.dead_band.setChecked(True)
        self.battery = QCheckBox('Compensar la batería (robot real)')
        self.verify = QCheckBox('Comprobación final')
        self.verify.setChecked(True)
        self.virtual_tape = QCheckBox('Cinta métrica virtual (solo simulación)')
        for c in (self.dead_band, self.battery, self.verify, self.virtual_tape):
            o.addWidget(c)
        self.start_button = QPushButton('Empezar calibración')
        self.start_button.clicked.connect(self.start)
        o.addWidget(self.start_button)
        left.addWidget(options)
        left.addWidget(QLabel('Pasos:'))
        self.steps = QListWidget()
        left.addWidget(self.steps, 1)
        layout.addLayout(left, 2)

        right = QVBoxLayout()
        self.step_box = QGroupBox('Paso')
        s = QVBoxLayout(self.step_box)
        self.instructions = QLabel('Elige el modo y pulsa «Empezar calibración».')
        self.instructions.setWordWrap(True)
        s.addWidget(self.instructions)
        buttons = QHBoxLayout()
        self.run_button = QPushButton('Mover el robot')
        self.run_button.clicked.connect(self.run_step)
        self.moving_button = QPushButton('¡Se mueve!')
        self.moving_button.setStyleSheet('font-weight: bold;')
        self.moving_button.clicked.connect(lambda: self.flow and self.flow.moving.set())
        self.skip_button = QPushButton('Saltar paso')
        self.skip_button.clicked.connect(self.skip)
        self.cancel_button = QPushButton('Cancelar')
        self.cancel_button.clicked.connect(self.cancel)
        for b in (self.run_button, self.moving_button, self.skip_button, self.cancel_button):
            buttons.addWidget(b)
        s.addLayout(buttons)
        self.form_box = QWidget()
        self.form = QFormLayout(self.form_box)
        s.addWidget(self.form_box)
        form_buttons = QHBoxLayout()
        self.tape_button = QPushButton('Usar la medida de la simulación')
        self.tape_button.clicked.connect(self.fill_from_sim)
        self.submit_button = QPushButton('Aplicar medidas')
        self.submit_button.clicked.connect(self.submit)
        form_buttons.addWidget(self.tape_button)
        form_buttons.addWidget(self.submit_button)
        s.addLayout(form_buttons)
        right.addWidget(self.step_box)

        self.table = QTableWidget(len(PARAMS), 3)
        self.table.setHorizontalHeaderLabels(['Parámetro', 'Antes', 'Ahora'])
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        for i, name in enumerate(PARAMS):
            label, unit, _ = LABELS[name]
            self.table.setItem(i, 0, QTableWidgetItem(f'{label} ({unit})' if unit else label))
        self.table.resizeColumnsToContents()
        self.table.resizeRowsToContents()
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.setFixedHeight(self.table.horizontalHeader().height() + 4
                                  + sum(self.table.rowHeight(i) for i in range(len(PARAMS))))
        right.addWidget(self.table)
        end = QHBoxLayout()
        self.save_button = QPushButton('Guardar en el robot')
        self.save_button.clicked.connect(self.save)
        self.discard_button = QPushButton('Descartar (volver a los de antes)')
        self.discard_button.clicked.connect(self.discard)
        self.reset_button = QPushButton('Restaurar robot.yaml')
        self.reset_button.clicked.connect(self.reset)
        for b in (self.save_button, self.discard_button, self.reset_button):
            end.addWidget(b)
        right.addLayout(end)
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        right.addWidget(self.log, 1)
        layout.addLayout(right, 3)
        self.inputs = {}
        self._update()

    # ------------------------------------------------------------ helpers

    def session(self):
        link = self.window.link
        if link is None:
            raise RuntimeError('no hay conexión con ROS 2')
        return link.session()

    def say(self, text):
        self.log.appendPlainText(text)

    def _confirm_motion(self):
        if not self.window.real_robot():
            return True
        return QMessageBox.question(
            self, 'Robot real', 'El robot real se va a mover. ¿Hay espacio libre y tienes a mano '
            'el interruptor de la batería?') == QMessageBox.Yes

    def _show_params(self):
        try:
            now = self.session().params()
        except Exception:  # noqa: BLE001  (not connected yet)
            return
        before = self.flow.original if self.flow and self.flow.original else now
        for i, name in enumerate(PARAMS):
            digits = LABELS[name][2]
            self.table.setItem(i, 1, QTableWidgetItem(f'{getattr(before, name):.{digits}f}'))
            item = QTableWidgetItem(f'{getattr(now, name):.{digits}f}')
            if abs(getattr(now, name) - getattr(before, name)) > 1e-9:
                item.setForeground(self.palette().highlight())
            self.table.setItem(i, 2, item)

    def _update(self):
        flow, busy = self.flow, self.busy
        step = flow.step if flow else None
        self.steps.clear()
        for st in (flow.steps if flow else []):
            self.steps.addItem(f'{ICONS[st.status]}  {st.title}')
        if flow and not flow.done:
            self.steps.setCurrentRow(flow.index)
        waiting = bool(flow and flow._pending is not None)
        self.start_button.setEnabled(not busy)
        self.run_button.setEnabled(bool(step) and not busy and not waiting)
        self.moving_button.setVisible(bool(step) and step.kind == 'dead_band'
                                      and not flow.auto and busy)
        self.skip_button.setEnabled(bool(step) and not busy)
        self.cancel_button.setEnabled(busy)
        self.form_box.setVisible(waiting)
        self.submit_button.setVisible(waiting)
        sim_values = waiting and any(f.sim_value is not None for f in step.fields)
        self.tape_button.setVisible(bool(sim_values))
        finished = bool(flow and flow.done)
        self.save_button.setEnabled(finished and not busy)
        self.discard_button.setEnabled(bool(flow) and not busy)
        self.reset_button.setEnabled(not busy)
        if step:
            self.step_box.setTitle(f'Paso {flow.index + 1} de {len(flow.steps)}: {step.title}')
            self.instructions.setText(step.instructions)
        elif finished:
            self.step_box.setTitle('Calibración terminada')
            self.instructions.setText('Revisa la tabla: «Guardar en el robot» la conserva al '
                                      'reiniciar; «Descartar» vuelve a los valores de antes.')

    def refresh(self, snap):
        sim = snap.driver == 'fake'
        self.virtual_tape.setEnabled(sim and not self.busy)
        if not sim:
            self.virtual_tape.setChecked(False)

    # ------------------------------------------------------------ actions

    def start(self):
        from rasptank_calibration.flow import Flow
        try:
            session = self.session()
        except RuntimeError as e:
            QMessageBox.warning(self, 'Calibración', str(e))
            return
        session.abort.clear()
        self.flow = Flow(session, auto=self.auto.isChecked(), rounds=self.rounds.value(),
                         dead_band=self.dead_band.isChecked(), battery=self.battery.isChecked(),
                         verify=self.verify.isChecked(),
                         virtual_tape=self.virtual_tape.isChecked())
        self.log.clear()
        self._work(self.flow.start, 'Leyendo la calibración actual...')

    def run_step(self):
        if not self._confirm_motion():
            return
        self.flow.s.abort.clear()
        self._work(self.flow.run, f'{self.flow.step.title}: moviendo...')

    def _work(self, fn, text):
        self.busy = True
        self.say(text)
        self._update()
        self.worker.run(fn)

    def _done(self, outcome):
        self.busy = False
        if isinstance(outcome, str):
            self.say('  ' + outcome)
        if outcome is not None and getattr(outcome, 'report', ''):
            self.say('  ' + outcome.report.replace('\n', '\n  '))
        if outcome is not None and getattr(outcome, 'needs_input', False):
            self._build_form()
            self.say('  Mide y escribe las medidas.')
        if self.flow and self.flow.done:
            self.say('Calibración terminada.')
        self._show_params()
        self._update()

    def _failed(self, text):
        self.busy = False
        if self.flow and self.flow.step:
            self.flow.step.status = 'error'
        self.say(f'  ERROR: {text}')
        self._show_params()
        self._update()

    def _build_form(self):
        while self.form.rowCount():
            self.form.removeRow(0)
        self.inputs = {}
        for f in self.flow.step.fields:
            box = QDoubleSpinBox()
            box.setRange(-1000.0, 1000.0)
            box.setDecimals(1)
            box.setSuffix(' ' + f.unit)
            self.form.addRow(f.label + ':', box)
            self.inputs[f.key] = box
            if f.sim_value is not None and self.flow.virtual_tape:
                box.setValue(f.sim_value)

    def fill_from_sim(self):
        for f in self.flow.step.fields:
            if f.sim_value is not None:
                self.inputs[f.key].setValue(f.sim_value)

    def submit(self):
        values = {k: box.value() for k, box in self.inputs.items()}
        if self.flow.step.kind == 'straight' and values['measured'] <= 0:
            QMessageBox.warning(self, 'Medidas', 'La distancia avanzada tiene que ser positiva.')
            return
        if self.flow.step.kind == 'straight' and not self._confirm_motion():
            return     # it drives back to the start mark after the measurement
        self._work(lambda: self.flow.submit(values), 'Aplicando las medidas...')

    def skip(self):
        self.say(f'{self.flow.step.title}: saltado')
        self.flow.skip()
        self._update()

    def cancel(self):
        if self.flow is not None:
            self.flow.s.abort.set()
            self.flow.moving.set()

    def save(self):
        self._work(self.flow.save, 'Guardando en el robot...')

    def discard(self):
        self._work(self.flow.discard, 'Volviendo a los valores de antes...')

    def reset(self):
        if QMessageBox.question(self, 'Restaurar', 'Se borra la calibración guardada y se vuelve '
                                'a los valores de robot.yaml. ¿Seguro?') != QMessageBox.Yes:
            return
        try:
            session = self.session()
        except RuntimeError as e:
            QMessageBox.warning(self, 'Calibración', str(e))
            return
        self._work(session.reset, 'Restaurando robot.yaml...')
