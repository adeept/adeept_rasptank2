"""Step by step calibration flow for a GUI (the RaspTank app), without Qt.

    flow = Flow(session, auto=False)
    flow.start()
    while not flow.done:
        step = flow.step                 # title, instructions, fields to fill in
        out = flow.run()                 # moves the robot (blocking: use a thread)
        if out.needs_input:              # manual: the user measures...
            flow.submit({'measured': 87.0, 'lateral': -2.5})
    flow.save()   or   flow.discard()

Manual steps move the robot and then ask for tape measurements; automatic
steps measure with the marker and finish by themselves. Each finished step
is already applied to base_node (the next step needs it).
"""
import math
import threading
from dataclasses import dataclass, field
from typing import Optional

from . import procedure as proc


@dataclass
class Field:
    key: str
    label: str
    unit: str
    sim_value: Optional[float] = None     # virtual tape measure (simulation)


@dataclass
class Step:
    kind: str
    title: str
    instructions: str
    fields: list = field(default_factory=list)
    report: str = ''
    status: str = 'pendiente'             # pendiente, midiendo, hecho, error, saltado


@dataclass
class Outcome:
    needs_input: bool
    report: str = ''


MANUAL = {
    'dead_band': ('Zona muerta',
                  'El robot acelerará muy despacio hacia delante. Pulsa «¡Se mueve!» en cuanto '
                  'las orugas empiecen a moverse.'),
    'straight': ('Recta',
                 f'Marca con cinta la parte delantera del robot. Avanzará '
                 f'{proc.MANUAL_DISTANCE * 100:.0f} cm. Después mide cuánto ha avanzado en la '
                 'dirección inicial y cuánto se ha desviado hacia un lado.'),
    'rotation': ('Giro',
                 'Marca con cinta la dirección del robot (a lo largo de una oruga). Dará una '
                 'vuelta completa a la izquierda. Después mide cuántos grados se ha pasado (+) '
                 'o se ha quedado corto (-).'),
    'battery': ('Batería',
                'Guarda el voltaje actual como referencia: con la batería más baja el robot '
                'subirá la potencia para ir a la misma velocidad.'),
    'verify': ('Comprobación',
               'Marca la posición del robot. Hará un cuadrado de 50 cm y volverá; mide a '
               'cuántos cm del punto de partida termina.'),
}
AUTO = {
    'dead_band': ('Zona muerta', 'Acelera muy despacio hasta que la cámara ve que el marcador '
                                 'se acerca.'),
    'straight': ('Recta', f'Avanza {proc.AUTO_DISTANCE * 100:.0f} cm hacia el marcador y vuelve; '
                          'la cámara mide la distancia y el desvío.'),
    'rotation': ('Giro', 'Oscila a los lados y da vueltas completas; la cámara mide cuánto ha '
                         'girado de verdad.'),
    'battery': MANUAL['battery'],
    'verify': ('Comprobación', 'Ida, vuelta completa y regreso: comprueba que vuelve al '
                               'mismo sitio.'),
}
FIELDS = {
    'straight': [Field('measured', 'Distancia avanzada', 'cm'),
                 Field('lateral', 'Desvío a la izquierda (+) o derecha (-)', 'cm')],
    'rotation': [Field('error', 'Grados de más (+) o de menos (-)', '°')],
    'verify': [Field('error', 'Distancia al punto de partida', 'cm')],
}


class Flow:
    def __init__(self, session, auto=False, rounds=2, dead_band=True, battery=False,
                 verify=True, virtual_tape=False):
        self.s = session
        self.auto = auto
        self.virtual_tape = virtual_tape
        self.moving = threading.Event()       # the "it moves!" button (manual dead band)
        kinds = (['dead_band'] if dead_band else []) + ['straight', 'rotation'] * rounds
        kinds += (['battery'] if battery else []) + (['verify'] if verify else [])
        texts = AUTO if auto else MANUAL
        self.steps = []
        rounds_done = 0
        for kind in kinds:
            title, text = texts[kind]
            if kind == 'straight':
                rounds_done += 1
                if rounds > 1:
                    title += f' ({rounds_done}/{rounds})'
            elif kind == 'rotation' and rounds > 1:
                title += f' ({rounds_done}/{rounds})'
            fields = [] if auto else [Field(f.key, f.label, f.unit) for f in FIELDS.get(kind, [])]
            self.steps.append(Step(kind, title, text, fields))
        self.index = 0
        self.original = None
        self._pending = None                  # what a manual step needs to finish

    # ------------------------------------------------------------ state

    @property
    def done(self):
        return self.index >= len(self.steps)

    @property
    def step(self):
        return None if self.done else self.steps[self.index]

    def start(self):
        self.original = self.s.params()
        if self.auto:
            self.s.camera_tilt(0.0)

    def _finish(self, report, status='hecho'):
        self.step.report = report
        self.step.status = status
        self.index += 1
        self._pending = None
        return Outcome(False, report)

    def skip(self):
        return self._finish('saltado', 'saltado')

    # ------------------------------------------------------------ running

    def run(self):
        """Do the movement of the current step (blocking)."""
        step = self.step
        step.status = 'midiendo'
        kind = step.kind
        if kind == 'battery':
            return self._finish(proc.battery(self.s, True).report)
        if self.auto:
            if kind == 'dead_band':
                return self._finish(proc.dead_band_auto(self.s).report)
            if kind == 'straight':
                return self._finish(proc.straight_auto(self.s).report)
            if kind == 'rotation':
                return self._finish(proc.rotation_auto(self.s).report)
            _, _, text = proc.verify_auto(self.s)
            return self._finish(text)
        start = self.s.truth_mark()
        if kind == 'dead_band':
            self.moving.clear()
            if self.virtual_tape and start is not None:
                def moving():
                    t = self.s.truth_since(start)
                    return self.moving.is_set() or (t is not None and math.hypot(t[0], t[1]) > 0.5)
            else:
                moving = self.moving.is_set
            result = proc.dead_band_manual(self.s, moving)
            self.s.move(-proc.STRAIGHT_SPEED / 2, 0.0, 1.0)
            return self._finish(result.report)
        if kind == 'straight':
            commanded = proc.straight_run_manual(self.s)
            t = self.s.truth_since(start)
            self._set_sim(t and {'measured': t[0], 'lateral': t[1]})
            self._pending = commanded
        elif kind == 'rotation':
            commanded = proc.rotation_run_manual(self.s)
            t = self.s.truth_since(start)
            self._set_sim(t and {'error': t[2] - commanded})
            self._pending = commanded
        else:   # verify
            proc.verify_run_manual(self.s)
            t = self.s.truth_since(start)
            self._set_sim(t and {'error': math.hypot(t[0], t[1])})
            self._pending = True
        return Outcome(True)

    def _set_sim(self, values):
        for f in self.step.fields:
            f.sim_value = None if not values else values.get(f.key)

    def submit(self, values):
        """Finish a manual step with the user's measurements (cm, degrees)."""
        step = self.step
        if self._pending is None:
            raise RuntimeError('primero hay que mover el robot')
        if step.kind == 'straight':
            commanded = self._pending
            result = proc.straight_manual(self.s, commanded, values['measured'] / 100.0,
                                          values['lateral'] / 100.0)
            report = result.report
            self.s.move(-proc.STRAIGHT_SPEED, 0.0, commanded / proc.STRAIGHT_SPEED)  # back
        elif step.kind == 'rotation':
            report = proc.rotation_manual(self.s, self._pending, values['error']).report
        else:
            report = f'cuadrado de 50 cm: termina a {values["error"]:.1f} cm del inicio'
        return self._finish(report)

    # ------------------------------------------------------------ end

    def current(self):
        return self.s.params()

    def save(self):
        return self.s.save()

    def discard(self):
        if self.original is not None:
            self.s.apply(self.original)
