"""Calibration steps, shared by the CLI (cli.py) and the app.

Each step has a manual and an automatic version:

  manual     the robot moves and the user measures with a tape measure
             (a floor mark at the start, the distance, the sideways shift,
             the angle), and types the numbers in
  automatic  the robot faces the ArUco goal marker (about 1 m away, with
             ~0.5 m free in front) and the camera measures the motion

Order: dead band -> straight -> rotation (-> repeat) -> battery -> verify.
The rotation runs at low track speeds, so it needs the speed scale first.
Every step applies its result to base_node straight away, because the
next step depends on it; nothing is saved until Session.save().
"""
import math
from dataclasses import dataclass

from rasptank_core import calibration as calc

from .session import CalibrationError

STRAIGHT_SPEED = 0.10          # m/s
TURN_SPEED = 0.8               # rad/s
MANUAL_DISTANCE = 1.0          # m
MANUAL_TURNS = 1               # full turns (more: the error may pass 180 deg)
AUTO_DISTANCE = 0.30           # m towards the marker and back
AUTO_SWING = 20.0              # deg, keeps the marker in view (half FOV ~31 deg)


@dataclass
class Result:
    calibration: calc.BaseCalibration
    report: str


def center_on_marker(session, tolerance=2.0, tries=4):
    """Turn until the marker is straight ahead (automatic mode, between tests)."""
    for _ in range(tries):
        m = session.marker()
        if m is None:
            raise CalibrationError('no veo el marcador')
        bearing = _bearing(m)
        if abs(bearing) <= tolerance:
            return m
        speed = TURN_SPEED / 2
        session.move(0.0, math.copysign(speed, bearing), math.radians(abs(bearing)) / speed)
    return session.marker()


# ------------------------------------------------------------ dead band

def dead_band_manual(session, moving):
    """Ramp the throttle until the user says the tracks move (`moving()`)."""
    cal = session.params()
    throttle = session.ramp(moving)
    new = calc.dead_band(throttle, cal)
    session.apply(new)
    return Result(new, f'empieza a moverse con {throttle:.0%} de potencia: '
                       f'min_throttle {cal.min_throttle:.3f} -> {new.min_throttle:.3f}')


def dead_band_auto(session, threshold=0.006):
    center_on_marker(session, tolerance=5.0)
    cal = session.params()
    start = session.marker(samples=20)
    if start is None:
        raise CalibrationError('no veo el marcador: ponlo a ~1 m delante del robot')
    recent = []

    def moved():
        # mean of the last 3 frames: one noisy frame must not trigger it
        m = session.latest_marker(max_age=0.3)
        if m is not None and (not recent or m is not recent[-1]):
            recent.append(m)
            del recent[:-3]
        return len(recent) == 3 and start[0] - sum(r[0] for r in recent) / 3 > threshold

    throttle = session.ramp(moved)
    new = calc.dead_band(throttle, cal)
    session.apply(new)
    session.move(-STRAIGHT_SPEED / 2, 0.0, 1.0)     # back towards the start
    return Result(new, f'el marcador se acerca con {throttle:.0%} de potencia: '
                       f'min_throttle {cal.min_throttle:.3f} -> {new.min_throttle:.3f}')


# ------------------------------------------------------------ rotation

def rotation_run_manual(session):
    """Turn MANUAL_TURNS full turns to the left; returns the commanded degrees."""
    _, angle = session.move(0.0, TURN_SPEED, math.radians(360.0 * MANUAL_TURNS) / TURN_SPEED)
    return angle


def rotation_manual(session, commanded_deg, error_deg):
    """`error_deg`: how much it turned too much (+) or too little (-)."""
    cal = session.params()
    new = calc.rotation(commanded_deg, commanded_deg + error_deg, cal)
    session.apply(new)
    return Result(new, f'giró {commanded_deg + error_deg:.0f}° de {commanded_deg:.0f}°: '
                       f'track_separation {cal.track_separation:.4f} -> {new.track_separation:.4f}')


def _bearing(m):
    return math.degrees(math.atan2(m[1], m[0]))


def rotation_auto(session):
    """Small swings that keep the marker in view, then a full turn."""
    cal = session.params()
    commanded = measured = 0.0
    for swing in (AUTO_SWING, -AUTO_SWING, -AUTO_SWING, AUTO_SWING):
        before = center_on_marker(session) if swing > 0 else session.marker()
        speed = math.copysign(TURN_SPEED / 2, swing)
        _, angle = session.move(0.0, speed, math.radians(abs(swing)) / abs(speed))
        after = session.marker()
        if before is None or after is None:
            # turned too much: undo it and skip this swing
            session.move(0.0, -speed, math.radians(abs(swing)) / abs(speed))
            continue
        commanded += abs(angle)
        measured += abs(calc.turned_from_bearings(angle, _bearing(before), _bearing(after)))
    if commanded == 0.0:
        raise CalibrationError('he perdido el marcador en todas las oscilaciones')
    new = calc.rotation(commanded, measured, cal)
    session.apply(new)
    report = (f'oscilaciones: giró {measured:.1f}° de {commanded:.1f}°; '
              f'track_separation {cal.track_separation:.4f} -> {new.track_separation:.4f}')
    # full turns (left and right) with the provisional value, measured
    # precisely by the bearing, at the speed the robot turns at normally
    turns = []
    for direction in (1.0, -1.0):
        before = center_on_marker(session)
        _, angle = session.move(0.0, direction * TURN_SPEED, 2 * math.pi / TURN_SPEED)
        after = session.marker()
        if before is None or after is None:
            report += '\nvuelta completa: no veo el marcador al acabar'
            continue
        real = calc.turned_from_bearings(angle, _bearing(before), _bearing(after))
        session.log(f'vuelta {angle:+.0f}°: marcador {_bearing(before):+.1f}° -> '
                    f'{_bearing(after):+.1f}°, giro real {real:+.1f}°')
        turns.append(calc.rotation(angle, real, new))
        report += f'\nvuelta completa: giró {abs(real):.1f}° de {abs(angle):.0f}°'
    if turns:
        new = calc.average(turns)
        session.apply(new)
        report += f'\ntrack_separation -> {new.track_separation:.4f}'
    return Result(new, report)


# ------------------------------------------------------------ straight

def straight_run_manual(session, distance=MANUAL_DISTANCE):
    commanded, _ = session.move(STRAIGHT_SPEED, 0.0, distance / STRAIGHT_SPEED)
    return commanded


def straight_manual(session, commanded, measured, lateral):
    """`measured` and `lateral` in metres (lateral + = to the left)."""
    cal = session.params()
    new = calc.straight(commanded, measured, lateral, cal)
    session.apply(new)
    return Result(new, _straight_report(cal, new, commanded, measured, lateral))


def _straight_report(cal, new, commanded, measured, lateral):
    return (f'avanzó {measured * 100:.1f} cm de {commanded * 100:.0f} cm, desvío '
            f'{lateral * 100:+.1f} cm: max_track_speed {cal.max_track_speed:.4f} -> '
            f'{new.max_track_speed:.4f}, ganancias {new.left_gain:.3f} / {new.right_gain:.3f}')


def straight_auto(session):
    """Towards the marker and back, measuring both legs with the camera.

    Both legs are computed from the same starting calibration and averaged:
    each marker measurement is only good to ~2 % of its distance.
    """
    center_on_marker(session)
    cal = session.params()
    results, reports = [], []
    for direction in (1.0, -1.0):
        before = session.marker()
        commanded, _ = session.move(direction * STRAIGHT_SPEED, 0.0,
                                    AUTO_DISTANCE / STRAIGHT_SPEED)
        after = session.marker()
        if before is None or after is None:
            raise CalibrationError('he perdido el marcador en la recta ('
                                   + ('antes' if before is None else 'después') + ')')
        dx, dy = calc.straight_from_marker(before[:2], after[:2])
        # the same imbalance shifts the robot to the same side forwards and
        # backwards (left track slower: always to the left)
        results.append(calc.straight(abs(commanded), abs(dx), dy, cal))
        reports.append(_straight_report(cal, results[-1], abs(commanded), abs(dx), dy))
    new = calc.average(results)
    session.apply(new)
    reports.append(f'media: max_track_speed {new.max_track_speed:.4f}, ganancias '
                   f'{new.left_gain:.3f} / {new.right_gain:.3f}')
    return Result(new, '\n'.join(reports))


# ------------------------------------------------------------ battery and checks

def battery(session, enable=True):
    cal = session.params()
    voltage = session.voltage if enable else None
    new = calc.battery(voltage, cal)
    session.apply(new)
    if new.battery_reference:
        return Result(new, f'compensación de batería con referencia {new.battery_reference:.2f} V')
    return Result(new, 'sin compensación de batería')


def verify_run_manual(session, side=0.5):
    """A square of `side` m turning left; the user measures the closing error."""
    for _ in range(4):
        session.move(STRAIGHT_SPEED, 0.0, side / STRAIGHT_SPEED)
        session.move(0.0, TURN_SPEED / 2, math.radians(90.0) / (TURN_SPEED / 2))


def verify_auto(session):
    """There and back plus a full turn: does it end where it started?

    Returns (distance error m, heading error deg, text): measured as the
    change of the marker's distance and bearing from the start.
    """
    before = center_on_marker(session)
    if before is None:
        raise CalibrationError('no veo el marcador')
    session.move(STRAIGHT_SPEED, 0.0, AUTO_DISTANCE / STRAIGHT_SPEED)
    session.move(0.0, TURN_SPEED, 2 * math.pi / TURN_SPEED)
    session.move(-STRAIGHT_SPEED, 0.0, AUTO_DISTANCE / STRAIGHT_SPEED)
    after = session.marker()
    if after is None:
        return None, None, 'al terminar no veo el marcador: el error de giro es grande'
    distance = math.hypot(*after[:2]) - math.hypot(*before[:2])
    turn = calc.normalize_deg(_bearing(before) - _bearing(after))
    return distance, turn, (f'ida {AUTO_DISTANCE * 100:.0f} cm, vuelta completa y regreso: '
                            f'error de distancia {distance * 100:+.1f} cm, de giro {turn:+.1f}°')
