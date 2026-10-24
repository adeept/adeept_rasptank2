"""Connection to the robot for the calibration (used by the CLI and the app).

A ROS 2 node spinning in a background thread, with blocking helpers:
read/apply/save the base parameters, drive open-loop movements, ramp the
throttle, and measure the ArUco marker. Every movement checks the
emergency stop and an `abort` event, and always ends sending zeros.
"""
import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import BatteryState, JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection2DArray

from rasptank_core.calibration import PARAMS, BaseCalibration
from rasptank_core.geometry import Transform, quat_to_yaw


class CalibrationError(RuntimeError):
    pass


class Aborted(CalibrationError):
    pass


class Session:
    """`node`: an existing node already spun by someone else (the app);
    without it the session creates its own node and spinning thread."""

    def __init__(self, base_node='base_node', node=None):
        self.own_context = False
        self.own_node = node is None
        if node is None:
            if not rclpy.ok():
                try:
                    from rclpy.signals import SignalHandlerOptions
                    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
                except (ImportError, TypeError):
                    rclpy.init()
                self.own_context = True
            node = Node('calibracion')
        self.node = n = node
        self.base = base_node.strip('/')
        self.lock = threading.Lock()
        self.abort = threading.Event()
        self.verbose = False
        self.stopped = False
        self.driver = None
        self.voltage = None
        self.truth = None
        self.marker_msg = None        # (time, x, y, yaw) of the last marker, robot frame
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cmd_pub = n.create_publisher(Twist, 'cmd_vel', 10)
        self.joint_pub = n.create_publisher(JointState, 'arm/joint_command', 10)
        self.get_cli = n.create_client(GetParameters, f'/{self.base}/get_parameters')
        self.set_cli = n.create_client(SetParameters, f'/{self.base}/set_parameters')
        self.save_cli = n.create_client(Trigger, 'base/save_calibration')
        self.reset_cli = n.create_client(Trigger, 'base/reset_calibration')
        n.create_subscription(Bool, 'base/stopped', self._on_stopped, latched)
        n.create_subscription(String, 'base/driver', self._on_driver, latched)
        n.create_subscription(BatteryState, 'battery', self._on_battery, 10)
        n.create_subscription(Odometry, 'sim/ground_truth', self._on_truth, 10)
        n.create_subscription(Detection2DArray, 'detections', self._on_detections, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, n)
        self.executor = None
        if self.own_node:
            self.executor = SingleThreadedExecutor()
            self.executor.add_node(n)
            self.thread = threading.Thread(target=self._spin, daemon=True)
            self.thread.start()

    def _spin(self):
        try:
            self.executor.spin()
        except Exception:      # shutdown while spinning
            pass

    def close(self):
        self.stop_robot()
        if self.own_node:
            self.executor.shutdown(timeout_sec=1.0)
            self.node.destroy_node()
        if self.own_context and rclpy.ok():
            rclpy.shutdown()

    # ------------------------------------------------------------ subscriptions

    def _on_stopped(self, msg):
        self.stopped = msg.data

    def _on_driver(self, msg):
        self.driver = msg.data

    def _on_battery(self, msg):
        self.voltage = msg.voltage

    def _on_truth(self, msg):
        p = msg.pose.pose
        yaw = quat_to_yaw(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        if self.truth is not None:
            # unwrapped: several turns add up instead of wrapping at +-180 deg
            yaw = self.truth[2] + math.atan2(math.sin(yaw - self.truth[2]),
                                             math.cos(yaw - self.truth[2]))
        self.truth = (p.position.x, p.position.y, yaw)

    def _on_detections(self, msg):
        for d in msg.detections:
            if not d.results or d.results[0].hypothesis.class_id != 'meta':
                continue
            try:
                tf = self.tf_buffer.lookup_transform('base_footprint', msg.header.frame_id, Time())
            except TransformException:
                return
            t, q = tf.transform.translation, tf.transform.rotation
            T = Transform.from_quat([t.x, t.y, t.z], [q.x, q.y, q.z, q.w])
            pose = d.results[0].pose.pose
            o = pose.orientation
            marker = Transform.from_quat([pose.position.x, pose.position.y, pose.position.z],
                                         [o.x, o.y, o.z, o.w])
            M = T @ marker
            normal = M.R[:, 2]                      # the marker's z axis points out of it
            with self.lock:
                self.marker_msg = (time.monotonic(), M.t[0], M.t[1],
                                   math.atan2(normal[1], normal[0]))
            return

    def log(self, text):
        self.node.get_logger().debug(text)
        if self.verbose:
            print('    ' + text, flush=True)

    @property
    def simulated(self):
        return self.driver == 'fake'

    # ------------------------------------------------------------ virtual tape (simulation)

    def truth_mark(self):
        """True pose now (simulation only), to measure from it later."""
        return self.truth if self.simulated else None

    def truth_since(self, start):
        """(forward cm, left cm, turned deg) since `start`, in the start frame."""
        now = self.truth_mark()
        if start is None or now is None:
            return None
        dx, dy = now[0] - start[0], now[1] - start[1]
        c, s = math.cos(-start[2]), math.sin(-start[2])
        return (100 * (c * dx - s * dy), 100 * (s * dx + c * dy),
                math.degrees(now[2] - start[2]))

    # ------------------------------------------------------------ parameters

    def _call(self, client, request, timeout=3.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise CalibrationError(f'no responde {client.srv_name}: ¿está arrancado el robot?')
        future = client.call_async(request)
        end = time.monotonic() + timeout
        while not future.done() and time.monotonic() < end:
            time.sleep(0.01)
        if not future.done():
            raise CalibrationError(f'{client.srv_name} no contesta')
        return future.result()

    def params(self):
        res = self._call(self.get_cli, GetParameters.Request(names=PARAMS))
        values = {}
        for name, value in zip(PARAMS, res.values):
            if value.type == ParameterType.PARAMETER_DOUBLE:
                values[name] = value.double_value
            elif value.type == ParameterType.PARAMETER_INTEGER:
                values[name] = float(value.integer_value)
        return BaseCalibration.from_dict(values)

    def apply(self, cal):
        req = SetParameters.Request()
        for name, value in cal.to_dict().items():
            req.parameters.append(ParameterMsg(
                name=name, value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                                double_value=float(value))))
        res = self._call(self.set_cli, req)
        bad = [f'{p.name}: {r.reason}' for p, r in zip(req.parameters, res.results)
               if not r.successful]
        if bad:
            raise CalibrationError('el robot rechaza ' + ', '.join(bad))

    def save(self):
        res = self._call(self.save_cli, Trigger.Request())
        if not res.success:
            raise CalibrationError(res.message)
        return res.message

    def reset(self):
        return self._call(self.reset_cli, Trigger.Request()).message

    # ------------------------------------------------------------ movement

    def stop_robot(self):
        for _ in range(3):
            self.cmd_pub.publish(Twist())
            time.sleep(0.03)

    def _check(self):
        if self.abort.is_set():
            raise Aborted('cancelado')
        if self.stopped:
            raise Aborted('parada de emergencia activa')

    def move(self, v, w, seconds):
        """Drive (v m/s, w rad/s) for `seconds`, open loop. Returns what the
        robot believes it did: (distance m, angle deg)."""
        self._check()
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = float(v), float(w)
        end = time.monotonic() + seconds
        try:
            while time.monotonic() < end:
                self._check()
                self.cmd_pub.publish(cmd)
                time.sleep(0.05)
        finally:
            self.stop_robot()
        time.sleep(0.5)                     # let it settle before measuring
        return v * seconds, math.degrees(w * seconds)

    def ramp(self, until, rate=0.015, limit=0.5):
        """Raise the throttle slowly (driving forward) until `until()` is true.

        min_throttle is set to 0 meanwhile, so the throttle is exactly
        speed / max_track_speed. Returns the throttle when it stopped.
        """
        self._check()
        original = self.params()
        self.apply(original.copy(min_throttle=0.0))
        throttle = 0.0
        start = time.monotonic()
        try:
            while throttle < limit:
                self._check()
                if until():
                    break
                throttle = rate * (time.monotonic() - start)
                cmd = Twist()
                cmd.linear.x = throttle * original.max_track_speed
                self.cmd_pub.publish(cmd)
                time.sleep(0.05)
            else:
                raise CalibrationError(f'no se ha movido ni con el {limit:.0%} de potencia')
        finally:
            self.stop_robot()
            self.apply(original)
        return throttle

    def camera_tilt(self, radians):
        js = JointState(name=['camera_tilt_joint'], position=[float(radians)])
        self.joint_pub.publish(js)
        time.sleep(1.0)

    # ------------------------------------------------------------ marker

    def latest_marker(self, max_age=0.5):
        with self.lock:
            m = self.marker_msg
        if m is None or time.monotonic() - m[0] > max_age:
            return None
        return m[1:]

    def marker(self, samples=12, timeout=4.0):
        """Average (x, y, yaw) of the marker in the robot frame, or None."""
        seen, last = [], None
        end = time.monotonic() + timeout
        while len(seen) < samples and time.monotonic() < end:
            self._check()
            with self.lock:
                m = self.marker_msg
            if m is not None and m is not last and time.monotonic() - m[0] < 0.5:
                seen.append(m[1:])
                last = m
            time.sleep(0.02)
        if len(seen) < max(3, samples // 3):
            return None
        x = sum(s[0] for s in seen) / len(seen)
        y = sum(s[1] for s in seen) / len(seen)
        yaw = math.atan2(sum(math.sin(s[2]) for s in seen), sum(math.cos(s[2]) for s in seen))
        return x, y, yaw
