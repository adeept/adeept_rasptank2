"""Arm and camera tilt of the RaspTank (servos A-E on the PCA9685).

Subscribes:
  /arm/pose            std_msgs/String        named pose from `poses`
  /arm/joint_command   sensor_msgs/JointState  name + position (rad)
  /emergency_stop      std_msgs/Bool          True holds the servos
Publishes:
  /joint_states        sensor_msgs/JointState  estimated servo positions
  /arm/state           rasptank_interfaces/ArmState

The servos have no feedback: the published position moves towards the
command at `max_speed` (rad/s), and the servos follow that same profile,
so they do not jump.

Poses come from parameters, e.g. in YAML:
  poses:
    abrir: {gripper_joint: 0.6}
    buscar: {shoulder_joint: 1.0, elbow_joint: -0.6, gripper_joint: 0.6}
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from rasptank_core.arm import JOINTS, ArmModel, parse_poses, servos_from_params
from rasptank_interfaces.msg import ArmState

from .drivers import FakeServos, PcaServos


class ArmNode(Node):
    def __init__(self, **kwargs):
        super().__init__('arm_node', allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True, **kwargs)

        def p(name, default):
            if not self.has_parameter(name):
                self.declare_parameter(name, default)
            return self.get_parameter(name).value

        driver = p('driver', 'fake')
        channels = p('servo_channels', [0, 1, 2, 3, 4])
        servos = servos_from_params(channels,
                                    p('servo_centers', [90.0] * 5),
                                    p('servo_signs', [1.0, -1.0, 1.0, -1.0, 1.0]),
                                    p('servo_min', [0.0] * 5),
                                    p('servo_max', [180.0] * 5))
        rate = p('rate', 50.0)
        self.poses = parse_poses({k: v.value for k, v in
                                  self.get_parameters_by_prefix('poses').items()})
        initial_pose = p('initial_pose', 'plegado')
        initial = {'camera_tilt_joint': p('camera_tilt', 0.35)}
        initial.update(self.poses.get(initial_pose, {}))

        if driver == 'pca9685':
            self.servos = PcaServos(p('i2c_address', 0x5f), p('pwm_frequency', 50),
                                    [s.channel for s in servos.values()])
        elif driver == 'fake':
            self.servos = FakeServos()
        else:
            raise ValueError(f'unknown driver {driver!r} (pca9685 or fake)')

        self.arm = ArmModel(servos, p('max_speed', 1.5), initial)
        self.pose_name = initial_pose if initial_pose in self.poses else ''
        self.stopped = False
        self.dt = 1.0 / rate
        self.write_servos()

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.state_pub = self.create_publisher(ArmState, 'arm/state', latched)
        self.create_subscription(String, 'arm/pose', self.on_pose, 10)
        self.create_subscription(JointState, 'arm/joint_command', self.on_joints, 10)
        self.create_subscription(Bool, 'emergency_stop', self.on_stop, latched)
        self.create_timer(self.dt, self.loop)
        self._last_state = None
        self.get_logger().info(f'arm ready (driver {driver}), poses: {sorted(self.poses)}')

    def on_pose(self, msg):
        if self.stopped:
            self.get_logger().warn(f'emergency stop: pose {msg.data!r} ignored')
            return
        if msg.data not in self.poses:
            self.get_logger().error(f'unknown pose {msg.data!r}; known: {sorted(self.poses)}')
            return
        self.arm.set_target(self.poses[msg.data])
        self.pose_name = msg.data

    def on_joints(self, msg):
        if self.stopped:
            return
        try:
            self.arm.set_target(dict(zip(msg.name, msg.position)))
        except ValueError as e:
            self.get_logger().error(str(e))
            return
        self.pose_name = ''

    def on_stop(self, msg):
        self.stopped = msg.data
        if self.stopped:
            self.arm.hold()

    def write_servos(self):
        for joint, deg in self.arm.servo_degrees().items():
            self.servos.set_deg(self.arm.servos[joint].channel, deg)

    def loop(self):
        positions = self.arm.step(self.dt)
        self.write_servos()
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(JOINTS)
        js.position = [positions[j] for j in JOINTS]
        self.joint_pub.publish(js)
        state = (self.pose_name, self.arm.moving(), self.stopped)
        if state != self._last_state:
            self._last_state = state
            msg = ArmState(pose=state[0], moving=state[1], stopped=state[2])
            msg.header.stamp = js.header.stamp
            self.state_pub.publish(msg)

    def destroy_node(self):
        self.servos.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArmNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise       # a real error; otherwise a callback was cut by the shutdown
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
