"""What the app shows about the robot, computed from a snapshot (no Qt, no ROS)."""
from dataclasses import dataclass, field
from typing import Optional

SIM_DOMAIN = 41
STALE = 1.5          # s without a heartbeat: the base is not running


@dataclass
class Snapshot:
    """Latest values received, with the time (monotonic s) they arrived."""
    now: float = 0.0
    driver: Optional[str] = None
    heartbeat_at: Optional[float] = None
    stopped: Optional[bool] = None
    voltage: Optional[float] = None
    battery_pct: Optional[float] = None
    distance: Optional[float] = None
    line: Optional[tuple] = None
    pose: Optional[tuple] = None              # odometry (x, y, deg)
    truth: Optional[tuple] = None             # simulation only
    mission: str = ''
    arm: str = ''
    governor: Optional[bool] = None
    camera_hz: float = 0.0
    nodes: frozenset = field(default_factory=frozenset)


def base_alive(s):
    return s.heartbeat_at is not None and s.now - s.heartbeat_at < STALE


def mode(s):
    """(text, colour) of the header."""
    if not base_alive(s):
        return 'SIN CONEXIÓN', '#7f8c8d'
    if s.stopped:
        return 'PARADA DE EMERGENCIA', '#c0392b'
    if s.driver == 'fake':
        return 'SIMULACIÓN', '#2980b9'
    if s.driver == 'pca9685':
        return 'ROBOT REAL', '#d35400'
    return 'CONECTADO', '#16a085'


def diagnosis(s):
    """One sentence about what is missing, for the status page."""
    if not base_alive(s):
        return ('No llega el latido de la base. ¿Está arrancada la simulación o el robot, y en '
                'el mismo dominio ROS?')
    if s.stopped:
        return 'Parada de emergencia activa: pulsa «Rearmar» cuando sea seguro.'
    if s.camera_hz < 1.0:
        return 'La base responde, pero no llega la cámara.'
    if s.distance is None:
        return 'La base y la cámara responden, pero no llega el ultrasonido.'
    return 'Todo responde.'


def fmt(value, pattern, missing='—'):
    return missing if value is None else pattern.format(value)
