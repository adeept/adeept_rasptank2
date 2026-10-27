"""Arm and camera-tilt servos of the RaspTank.

Joint angles are in radians, as in the URDF and /joint_states. The servos
are driven in degrees (0-180) through the PCA9685: `ServoMap` converts
between both. The servos give no feedback, so `ArmModel` estimates where
they are by moving towards the target at a limited speed; that estimate is
what the arm node publishes on /joint_states.
"""
import math
from dataclasses import dataclass

JOINTS = [
    'shoulder_joint',      # servo A (channel 0): arm up/down
    'elbow_joint',         # servo B (channel 1): hand up/down
    'wrist_roll_joint',    # servo C (channel 2): gripper rotation
    'gripper_joint',       # servo D (channel 3): open/close
    'camera_tilt_joint',   # servo E (channel 4): camera up/down
]


@dataclass
class ServoMap:
    channel: int
    center_deg: float = 90.0   # servo angle for joint angle 0
    sign: float = 1.0          # +1 or -1, servo direction
    min_deg: float = 0.0
    max_deg: float = 180.0

    def to_deg(self, angle):
        deg = self.center_deg + self.sign * math.degrees(angle)
        return max(self.min_deg, min(self.max_deg, deg))

    def to_angle(self, deg):
        return math.radians((deg - self.center_deg) * self.sign)

    def limits(self):
        """Joint limits (rad) allowed by the servo range."""
        a, b = self.to_angle(self.min_deg), self.to_angle(self.max_deg)
        return min(a, b), max(a, b)


DEFAULT_SERVOS = {
    'shoulder_joint': ServoMap(0),
    'elbow_joint': ServoMap(1),
    'wrist_roll_joint': ServoMap(2),
    'gripper_joint': ServoMap(3),
    'camera_tilt_joint': ServoMap(4),
}


def servos_from_params(channels, centers, signs, mins, maxs):
    """Build the ServoMaps from parallel lists ordered as JOINTS."""
    servos = {}
    for i, name in enumerate(JOINTS):
        servos[name] = ServoMap(int(channels[i]), float(centers[i]), float(signs[i]),
                                float(mins[i]), float(maxs[i]))
    return servos


def parse_poses(flat):
    """Named poses from parameters.

    `flat` maps 'pose.joint' -> angle (that is how ROS 2 parameters given
    as nested YAML arrive). Returns {pose: {joint: angle}}. A pose may set
    only some joints: 'cerrar' only moves the gripper.
    """
    poses = {}
    for key, value in flat.items():
        pose, _, joint = key.partition('.')
        if joint not in JOINTS:
            raise ValueError(f'pose {pose!r}: unknown joint {joint!r}')
        poses.setdefault(pose, {})[joint] = float(value)
    return poses


class ArmModel:
    """Servo positions moving at a limited speed towards their targets."""

    def __init__(self, servos, max_speed, initial=None):
        self.servos = servos
        self.max_speed = max_speed          # rad/s
        self.position = {j: 0.0 for j in JOINTS}
        if initial:
            self.position.update(initial)
        self.position = {j: self.clamp(j, a) for j, a in self.position.items()}
        self.target = dict(self.position)

    def clamp(self, joint, angle):
        low, high = self.servos[joint].limits()
        return max(low, min(high, angle))

    def set_target(self, angles):
        for joint, angle in angles.items():
            if joint not in self.target:
                raise ValueError(f'unknown joint {joint!r}')
            self.target[joint] = self.clamp(joint, angle)

    def hold(self):
        """Stop where the servos are now (emergency stop)."""
        self.target = dict(self.position)

    def step(self, dt):
        max_step = self.max_speed * dt
        for joint, goal in self.target.items():
            err = goal - self.position[joint]
            self.position[joint] += max(-max_step, min(max_step, err))
        return self.position

    def moving(self, tolerance=1e-3):
        return any(abs(self.target[j] - self.position[j]) > tolerance for j in JOINTS)

    def servo_degrees(self):
        return {j: self.servos[j].to_deg(a) for j, a in self.position.items()}


@dataclass
class ArmGeometry:
    """Planar geometry of the arm (metres), as in rasptank.urdf.xacro.

    The shoulder is at (shoulder_x, shoulder_z) from base_footprint;
    `reach` goes from the elbow to the centre between the fingers.
    """
    shoulder_x: float = 0.03
    shoulder_z: float = 0.11
    upper_arm: float = 0.08
    reach: float = 0.10

    def gripper_center(self, shoulder, elbow):
        """(x, z) of the gripper centre in base_footprint. Positive = up."""
        x = self.shoulder_x + self.upper_arm * math.cos(shoulder)
        z = self.shoulder_z + self.upper_arm * math.sin(shoulder)
        x += self.reach * math.cos(shoulder + elbow)
        z += self.reach * math.sin(shoulder + elbow)
        return x, z


@dataclass
class ArmStatus:
    """What /arm/state says (std_msgs/String with JSON, no custom message:
    the workspace builds without a C compiler, also on Windows)."""
    pose: str = ''           # last named pose asked for ('' after joint commands)
    moving: bool = False     # the servos are still going towards the command
    stopped: bool = False    # emergency stop
    received: bool = False   # False until the first message arrives

    def to_json(self):
        import json
        return json.dumps({'pose': self.pose, 'moving': self.moving, 'stopped': self.stopped})

    @classmethod
    def from_json(cls, text):
        import json
        try:
            d = json.loads(text)
            return cls(str(d.get('pose', '')), bool(d.get('moving', False)),
                       bool(d.get('stopped', False)), True)
        except (ValueError, AttributeError):
            return cls(received=True)
