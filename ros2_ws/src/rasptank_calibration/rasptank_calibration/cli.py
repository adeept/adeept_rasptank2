"""Calibrate the odometry of the RaspTank from the terminal.

    ros2 run rasptank_calibration calibrar            # manual: tape measure
    ros2 run rasptank_calibration calibrar --auto     # automatic: ArUco marker

Manual: you need ~1.5 m of free floor, a tape measure and some tape to mark
where the robot starts. Automatic: the goal marker (scripts/generar_marcador.py)
standing ~1 m in front of the robot, and the detector running (pc.launch.py
or sim.launch.py).

In simulation, --medidas-sim fills the measurements in from the true pose
(/sim/ground_truth): a virtual tape measure, to practise or to test.
"""
import argparse
import math
import sys
import threading
import time

from . import procedure as proc
from .session import Aborted, CalibrationError, Session


class Terminal:
    def __init__(self, args, session):
        self.args = args
        self.s = session

    def say(self, text=''):
        print(text, flush=True)

    def confirm(self, text):
        if self.args.si:
            self.say(f'{text} [s]')
            return True
        return input(f'{text} [S/n] ').strip().lower() in ('', 's', 'si', 'sí', 'y')

    def go(self, text):
        """Ask before every movement."""
        self.say('\n' + text)
        if not self.args.si:
            input('  Pulsa Enter para mover el robot (Ctrl+C cancela)... ')

    def number(self, text, sim_value=None):
        if sim_value is not None and self.args.medidas_sim:
            self.say(f'{text} {sim_value:.1f}   (cinta métrica virtual)')
            return sim_value
        hint = f' [simulación: {sim_value:.1f}]' if sim_value is not None else ''
        while True:
            raw = input(f'{text}{hint} ').strip().replace(',', '.')
            try:
                return float(raw)
            except ValueError:
                self.say('  Escribe un número, por ejemplo 87.5')

    # ------------------------------------------------------------ truth helpers (simulation)

    def truth(self):
        return self.s.truth if self.s.simulated else None

    def truth_since(self, start):
        """(forward cm, left cm, turned deg) since `start`, in the start frame."""
        now = self.truth()
        if start is None or now is None:
            return None
        dx, dy = now[0] - start[0], now[1] - start[1]
        c, s = math.cos(-start[2]), math.sin(-start[2])
        return (100 * (c * dx - s * dy), 100 * (s * dx + c * dy),
                math.degrees(now[2] - start[2]))

    # ------------------------------------------------------------ steps

    def dead_band(self):
        if self.args.auto:
            self.go('ZONA MUERTA: el robot acelerará muy despacio hasta que la cámara vea que se mueve.')
            return proc.dead_band_auto(self.s)
        self.go('ZONA MUERTA: el robot acelerará muy despacio. Pulsa Enter en cuanto las orugas '
                'empiecen a moverse.')
        start = self.truth()
        pressed = threading.Event()
        if self.args.medidas_sim and start is not None:
            def moving():
                t = self.truth_since(start)
                return t is not None and math.hypot(t[0], t[1]) > 0.5
        else:
            threading.Thread(target=lambda: (input(), pressed.set()), daemon=True).start()
            moving = pressed.is_set
        result = proc.dead_band_manual(self.s, moving)
        self.s.move(-proc.STRAIGHT_SPEED / 2, 0.0, 1.0)
        return result

    def rotation(self):
        if self.args.auto:
            self.go('GIRO: oscilará a izquierda y derecha y dará una vuelta completa.')
            return proc.rotation_auto(self.s)
        self.go('GIRO: marca con cinta la dirección del robot. Dará una vuelta completa a la '
                'izquierda.')
        start = self.truth()
        commanded = proc.rotation_run_manual(self.s)
        t = self.truth_since(start)
        sim = None if t is None else t[2] - commanded
        error = self.number('¿Cuántos grados ha girado DE MÁS (+) o DE MENOS (-) respecto a la '
                            'marca?', sim)
        return proc.rotation_manual(self.s, commanded, error)

    def straight(self):
        if self.args.auto:
            self.go(f'RECTA: avanzará {proc.AUTO_DISTANCE * 100:.0f} cm hacia el marcador y volverá.')
            return proc.straight_auto(self.s)
        self.go(f'RECTA: marca la posición de la parte delantera del robot. Avanzará '
                f'{proc.MANUAL_DISTANCE * 100:.0f} cm.')
        start = self.truth()
        commanded = proc.straight_run_manual(self.s)
        t = self.truth_since(start)
        measured = self.number('¿Cuántos cm ha avanzado (en la dirección inicial)?',
                               None if t is None else t[0])
        lateral = self.number('¿Cuántos cm se ha desviado a la IZQUIERDA (+) o DERECHA (-)?',
                              None if t is None else t[1])
        result = proc.straight_manual(self.s, commanded, measured / 100.0, lateral / 100.0)
        self.go('Vuelvo marcha atrás al punto de partida.')
        self.s.move(-proc.STRAIGHT_SPEED, 0.0, commanded / proc.STRAIGHT_SPEED)
        return result

    def verify(self):
        if self.args.auto:
            self.go('COMPROBACIÓN: ida, vuelta completa y regreso.')
            _, _, text = proc.verify_auto(self.s)
            return text
        self.go('COMPROBACIÓN: marca la posición y hará un cuadrado de 50 cm.')
        start = self.truth()
        proc.verify_run_manual(self.s)
        t = self.truth_since(start)
        error = self.number('¿A cuántos cm del punto de partida ha terminado?',
                            None if t is None else math.hypot(t[0], t[1]))
        return f'cuadrado de 50 cm: termina a {error:.1f} cm del inicio'

    # ------------------------------------------------------------ flow

    def run(self):
        s = self.s
        time.sleep(1.0)
        mode = 'AUTOMÁTICA (marcador ArUco)' if self.args.auto else 'MANUAL (cinta métrica)'
        where = {'fake': 'SIMULACIÓN', 'pca9685': 'ROBOT REAL'}.get(s.driver, 'desconocido')
        original = s.params()
        self.say(f'Calibración {mode} — {where}')
        self.say(f'Valores actuales: {self.format(original)}')
        if s.driver == 'pca9685':
            self.say('\n¡ROBOT REAL! Deja espacio libre, ten a mano el interruptor de la batería.')
        if not self.confirm('¿Empezamos?'):
            return 0
        if self.args.auto:
            s.camera_tilt(0.0)
        try:
            if self.args.dead_band:
                self.say('  ' + self.dead_band().report)
            for i in range(self.args.rondas):
                if self.args.rondas > 1:
                    self.say(f'\n--- Ronda {i + 1} de {self.args.rondas}')
                self.say('  ' + self.straight().report.replace('\n', '\n  '))
                self.say('  ' + self.rotation().report.replace('\n', '\n  '))
            use_battery = s.voltage is not None and s.driver == 'pca9685' and \
                self.confirm(f'\n¿Compensar la batería (ahora {s.voltage:.2f} V)?')
            self.say('  ' + proc.battery(s, use_battery).report)
            if self.args.comprobar:
                self.say('  ' + self.verify())
        except (Aborted, CalibrationError, ValueError, KeyboardInterrupt) as e:
            s.stop_robot()
            self.say(f'\nCalibración interrumpida: {e or "Ctrl+C"}. Vuelvo a los valores de antes.')
            s.apply(original)
            return 1
        final = s.params()
        self.say(f'\nAntes:   {self.format(original)}\nDespués: {self.format(final)}')
        if self.args.no_guardar:
            self.say('No se guarda (--no-guardar): los valores siguen aplicados hasta reiniciar.')
        elif self.confirm('¿Guardar en el robot?'):
            self.say(s.save())
        else:
            s.apply(original)
            self.say('Descartado: vuelvo a los valores de antes.')
        return 0

    @staticmethod
    def format(cal):
        return (f'velocidad {cal.max_track_speed:.3f} m/s · separación {cal.track_separation:.3f} m'
                f' · zona muerta {cal.min_throttle:.3f} · ganancias {cal.left_gain:.3f}/'
                f'{cal.right_gain:.3f} · batería {cal.battery_reference:.2f} V')


def main(argv=None):
    parser = argparse.ArgumentParser(description='Calibración de la odometría del RaspTank')
    parser.add_argument('--auto', action='store_true', help='con el marcador ArUco y la cámara')
    parser.add_argument('--rondas', type=int, default=2, help='repeticiones de giro + recta')
    parser.add_argument('--sin-zona-muerta', dest='dead_band', action='store_false',
                        help='no medir la zona muerta')
    parser.add_argument('--sin-comprobar', dest='comprobar', action='store_false',
                        help='no hacer la prueba final')
    parser.add_argument('--si', action='store_true', help='no preguntar (¡el robot se mueve solo!)')
    parser.add_argument('--medidas-sim', action='store_true',
                        help='simulación: medidas tomadas de /sim/ground_truth')
    parser.add_argument('--no-guardar', action='store_true')
    parser.add_argument('--detalle', action='store_true', help='muestra cada medida')
    args, _ = parser.parse_known_args(argv)
    session = Session()
    session.verbose = args.detalle
    try:
        end = time.monotonic() + 5.0
        while session.driver is None and time.monotonic() < end:
            time.sleep(0.1)                  # /base/driver is latched: it arrives soon
        if session.driver is None:
            print('AVISO: no sé si es el robot real o la simulación (¿está base_node en marcha?)')
        if args.medidas_sim:
            if not session.simulated:
                print('--medidas-sim solo funciona en simulación')
                return 2
        return Terminal(args, session).run()
    except CalibrationError as e:
        print(f'Error: {e}')
        return 1
    finally:
        session.close()


if __name__ == '__main__':
    sys.exit(main())
