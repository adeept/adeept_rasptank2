"""State machine of the mission "pick up the cube and take it to the goal".

Pure logic, without ROS: the mission node gives it, at every control step,
what the robot knows (odometry pose, last cube and goal positions in the
robot frame, state of the arm) and publishes what it returns (twist and
named arm pose). That way it can be tested, and reused, without a robot.

States:

    ESPERA -> PREPARAR -> BUSCAR_CUBO -> APROXIMAR_CUBO -> AVANCE_FINAL
           -> COGER -> COMPROBAR -> BUSCAR_META -> APROXIMAR_META
           -> SOLTAR -> RETROCEDER -> FIN

COMPROBAR backs up and looks: if the cube is still on the floor the grasp
failed and it goes back to APROXIMAR_CUBO (up to `max_retries` times).
Any timeout ends in FALLO. `stop()` goes back to ESPERA from any state.
"""
import math
from dataclasses import dataclass, field
from typing import Optional

from .kinematics import Pose2D, clamp

ESPERA = 'ESPERA'
PREPARAR = 'PREPARAR'
BUSCAR_CUBO = 'BUSCAR_CUBO'
APROXIMAR_CUBO = 'APROXIMAR_CUBO'
AVANCE_FINAL = 'AVANCE_FINAL'
COGER = 'COGER'
COMPROBAR = 'COMPROBAR'
BUSCAR_META = 'BUSCAR_META'
APROXIMAR_META = 'APROXIMAR_META'
SOLTAR = 'SOLTAR'
RETROCEDER = 'RETROCEDER'
FIN = 'FIN'
FALLO = 'FALLO'

GRAB_SEQUENCE = ['pre_coger', 'cerrar', 'transportar']
DROP_SEQUENCE = ['soltar', 'abrir', 'buscar']


@dataclass
class Target:
    """Object position in the robot frame (base_footprint), x forward, y left."""
    x: float
    y: float
    stamp: float

    @property
    def distance(self):
        return math.hypot(self.x, self.y)

    @property
    def bearing(self):
        return math.atan2(self.y, self.x)


@dataclass
class MissionInput:
    now: float
    pose: Pose2D
    cube: Optional[Target] = None
    goal: Optional[Target] = None
    arm_pose: str = ''        # last named pose the arm reached
    arm_moving: bool = False


@dataclass
class MissionOutput:
    v: float = 0.0
    w: float = 0.0
    arm_pose: Optional[str] = None   # named pose requested (None: keep)
    state: str = ESPERA
    info: str = ''


@dataclass
class MissionParams:
    target_max_age: float = 0.7      # s, older detections are ignored
    lost_timeout: float = 3.0        # s without the target -> search again
    search_speed: float = 0.6        # rad/s
    search_turn_time: float = 0.5    # s turning, then...
    search_look_time: float = 0.7    # s stopped, looking (slow detector)
    search_timeout: float = 45.0     # s
    approach_timeout: float = 60.0   # s
    k_angular: float = 1.5
    k_linear: float = 0.6
    max_linear: float = 0.12         # m/s
    max_angular: float = 0.8         # rad/s
    min_angular: float = 0.15        # rad/s when only turning to align
    align_tolerance: float = 0.06    # rad
    grasp_reach: float = 0.165       # m, cube x when the lowered gripper is on it
    final_gap: float = 0.12          # m, blind approach from this gap
    final_speed: float = 0.05        # m/s
    arm_settle: float = 0.3          # s after a pose is reached
    arm_timeout: float = 6.0         # s per pose
    check_backup: float = 0.12       # m
    check_time: float = 1.2          # s looking for a cube left on the floor
    max_retries: int = 2
    drop_distance: float = 0.26      # m, goal x where the cube is released
    drop_tolerance: float = 0.03     # m
    retreat: float = 0.15            # m


class Mission:
    def __init__(self, params=None):
        self.p = params or MissionParams()
        self.state = ESPERA
        self.info = ''
        self.arm_request = None
        self.retries = 0
        self._entered = 0.0
        self._last_seen = None
        self._seq = []
        self._seq_index = 0
        self._seq_reached = None
        self._seq_started = 0.0
        self._drive = None
        self._after_seq = None
        self._now = 0.0
        self.history = []

    # ------------------------------------------------------------ control

    def start(self):
        if self.state in (ESPERA, FIN, FALLO):
            self.retries = 0
            self._go(PREPARAR)
            self._sequence(['buscar'], BUSCAR_CUBO)

    def stop(self):
        self._go(ESPERA)

    @property
    def finished(self):
        return self.state in (FIN, FALLO)

    def step(self, inp):
        self._now = inp.now
        v, w = 0.0, 0.0
        handler = {
            PREPARAR: self._run_sequence,
            BUSCAR_CUBO: lambda i: self._search(i, i.cube, APROXIMAR_CUBO),
            APROXIMAR_CUBO: self._approach_cube,
            AVANCE_FINAL: self._run_drive,
            COGER: self._run_sequence,
            COMPROBAR: self._check,
            BUSCAR_META: lambda i: self._search(i, i.goal, APROXIMAR_META),
            APROXIMAR_META: self._approach_goal,
            SOLTAR: self._run_sequence,
            RETROCEDER: self._run_drive,
        }.get(self.state)
        if handler is not None:
            v, w = handler(inp) or (0.0, 0.0)
        return MissionOutput(v, w, self.arm_request, self.state, self.info)

    # ------------------------------------------------------------ helpers

    def _go(self, state, info=''):
        if state != self.state:
            self.history.append(state)
        self.state = state
        self.info = info
        self._entered = self._now
        self._drive = None

    def _elapsed(self):
        return self._now - self._entered

    def _fresh(self, inp, target):
        return target is not None and inp.now - target.stamp <= self.p.target_max_age

    def _sequence(self, poses, then):
        self._seq = list(poses)
        self._seq_index = 0
        self._seq_reached = None
        self._seq_started = self._now
        self._after_seq = then
        self.arm_request = self._seq[0]

    def _run_sequence(self, inp):
        pose = self._seq[self._seq_index]
        self.info = f'brazo: {pose}'
        if inp.arm_pose == pose and not inp.arm_moving:
            if self._seq_reached is None:
                self._seq_reached = inp.now
            if inp.now - self._seq_reached >= self.p.arm_settle:
                self._seq_index += 1
                self._seq_reached = None
                self._seq_started = inp.now
                if self._seq_index == len(self._seq):
                    then = self._after_seq
                    self._go(then)
                    if callable(getattr(self, '_on_' + then.lower(), None)):
                        getattr(self, '_on_' + then.lower())(inp)
                    return None
                self.arm_request = self._seq[self._seq_index]
        elif inp.now - self._seq_started > self.p.arm_timeout:
            self._go(FALLO, f'el brazo no llega a {pose!r}')
        return None

    def _drive_distance(self, inp, distance, speed, then):
        self._drive = (inp.pose.x, inp.pose.y, distance, abs(speed), then, inp.now)

    def _run_drive(self, inp):
        if self._drive is None:
            return None
        x0, y0, distance, speed, then, t0 = self._drive
        done = math.hypot(inp.pose.x - x0, inp.pose.y - y0)
        limit = abs(distance) / max(speed, 1e-3) * 2.0 + 2.0
        if done >= abs(distance) - 1e-3 or inp.now - t0 > limit:
            self._go(then)
            hook = getattr(self, '_on_' + then.lower(), None)
            if callable(hook):
                hook(inp)
            return 0.0, 0.0
        self.info = f'{done:.2f}/{abs(distance):.2f} m'
        return math.copysign(speed, distance), 0.0

    # ------------------------------------------------------------ states

    def _search(self, inp, target, approach_state):
        if self._fresh(inp, target):
            self._last_seen = inp.now
            self._go(approach_state)
            return 0.0, 0.0
        if self._elapsed() > self.p.search_timeout:
            what = 'el cubo' if approach_state == APROXIMAR_CUBO else 'la meta'
            self._go(FALLO, f'no encuentro {what}')
            return 0.0, 0.0
        period = self.p.search_turn_time + self.p.search_look_time
        phase = self._elapsed() % period
        self.info = 'girando' if phase < self.p.search_turn_time else 'mirando'
        return (0.0, self.p.search_speed) if phase < self.p.search_turn_time else (0.0, 0.0)

    def _track(self, inp, target, search_state):
        """Common part of the approaches: returns the target or None."""
        if self._fresh(inp, target):
            self._last_seen = inp.now
            return target
        if self._last_seen is None or inp.now - self._last_seen > self.p.lost_timeout:
            self._go(search_state, 'objetivo perdido')
        elif self._elapsed() > self.p.approach_timeout:
            self._go(FALLO, 'la aproximación tarda demasiado')
        return None

    def _steer(self, target, remaining):
        p = self.p
        w = clamp(p.k_angular * target.bearing, -p.max_angular, p.max_angular)
        v = clamp(p.k_linear * remaining, -p.max_linear / 2, p.max_linear)
        v *= max(0.0, 1.0 - abs(target.bearing) / 0.6)
        return v, w

    def _align(self, target):
        p = self.p
        w = clamp(p.k_angular * target.bearing, -p.max_angular, p.max_angular)
        if abs(w) < p.min_angular:
            w = math.copysign(p.min_angular, w)
        return 0.0, w

    def _approach_cube(self, inp):
        t = self._track(inp, inp.cube, BUSCAR_CUBO)
        if t is None:
            return 0.0, 0.0
        remaining = t.x - self.p.grasp_reach
        self.info = f'cubo a {t.distance:.2f} m, {math.degrees(t.bearing):.0f}°'
        if remaining <= self.p.final_gap:
            if abs(t.bearing) > self.p.align_tolerance:
                return self._align(t)
            self._go(AVANCE_FINAL)
            self._drive_distance(inp, remaining, self.p.final_speed, COGER)
            return 0.0, 0.0
        return self._steer(t, remaining)

    def _on_coger(self, inp):
        self._sequence(GRAB_SEQUENCE, COMPROBAR)
        self._go(COGER)

    def _on_comprobar(self, inp):
        self._go(COMPROBAR)
        self._drive_distance(inp, -self.p.check_backup, self.p.final_speed, COMPROBAR)
        self._checking_since = None

    def _check(self, inp):
        if self._drive is not None:
            # backing up first; _run_drive re-enters COMPROBAR when done
            x0, y0, distance, speed, then, t0 = self._drive
            if math.hypot(inp.pose.x - x0, inp.pose.y - y0) < abs(distance) - 1e-3 \
                    and inp.now - t0 < abs(distance) / speed * 2 + 2:
                return -speed, 0.0
            self._drive = None
            self._checking_since = inp.now
        if self._checking_since is None:
            self._checking_since = inp.now
        cube = inp.cube
        if self._fresh(inp, cube) and cube.stamp >= self._checking_since and 0.05 < cube.x < 0.8:
            self.retries += 1
            if self.retries > self.p.max_retries:
                self._go(FALLO, 'no consigo coger el cubo')
                return 0.0, 0.0
            self._go(PREPARAR, f'reintento {self.retries}')
            self._sequence(['buscar'], APROXIMAR_CUBO)
            self._last_seen = inp.now
            return 0.0, 0.0
        if inp.now - self._checking_since >= self.p.check_time:
            self._go(BUSCAR_META)
        else:
            self.info = 'comprobando el agarre'
        return 0.0, 0.0

    def _approach_goal(self, inp):
        t = self._track(inp, inp.goal, BUSCAR_META)
        if t is None:
            return 0.0, 0.0
        remaining = t.x - self.p.drop_distance
        self.info = f'meta a {t.distance:.2f} m, {math.degrees(t.bearing):.0f}°'
        if abs(remaining) <= self.p.drop_tolerance:
            if abs(t.bearing) > self.p.align_tolerance:
                return self._align(t)
            self._go(SOLTAR)
            self._sequence(DROP_SEQUENCE, RETROCEDER)
            return 0.0, 0.0
        return self._steer(t, remaining)

    def _on_retroceder(self, inp):
        self._go(RETROCEDER)
        self._drive_distance(inp, -self.p.retreat, self.p.final_speed * 2, FIN)
