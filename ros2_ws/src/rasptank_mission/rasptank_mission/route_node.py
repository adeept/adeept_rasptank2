"""Follow a route of points, given as a parameter or clicked in RViz.

Subscribes:
  /odom            nav_msgs/Odometry
  /clicked_point   geometry_msgs/PointStamped  RViz "Publish Point": adds a point
  /goal_pose       geometry_msgs/PoseStamped   RViz "2D Goal Pose": go only there
Publishes:
  /cmd_vel         geometry_msgs/Twist   (only while following)
  /route/planned   nav_msgs/Path         the points to visit
  /route/traveled  nav_msgs/Path         where the robot has been
  /route/markers   visualization_msgs/MarkerArray  numbered points
Services:
  /route/start  /route/stop  /route/clear   std_srvs/Trigger

Parameters: `waypoints` [x1, y1, x2, y2, ...] in the odom frame,
`autostart`, and the RouteParams of rasptank_core.route.
"""
import dataclasses
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from rasptank_core.geometry import quat_to_yaw
from rasptank_core.kinematics import Pose2D
from rasptank_core.route import RouteFollower, RouteParams


class RouteNode(Node):
    def __init__(self):
        super().__init__('route_node')
        values = {f.name: self.declare_parameter(f.name, f.default).value
                  for f in dataclasses.fields(RouteParams)}
        self.route = RouteFollower(RouteParams(**values))
        flat = list(self.declare_parameter('waypoints', [0.0]).value)
        if len(flat) >= 2:
            self.route.set_route(zip(flat[0::2], flat[1::2]))
        self.frame = self.declare_parameter('odom_frame', 'odom').value
        self.running = self.declare_parameter('autostart', False).value and not self.route.done
        self.pose = None
        self.traveled = Path()
        self.traveled.header.frame_id = self.frame

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.planned_pub = self.create_publisher(Path, 'route/planned', 2)
        self.traveled_pub = self.create_publisher(Path, 'route/traveled', 2)
        self.marker_pub = self.create_publisher(MarkerArray, 'route/markers', 2)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_subscription(PointStamped, 'clicked_point', self.on_point, 10)
        self.create_subscription(PoseStamped, 'goal_pose', self.on_goal, 10)
        self.create_service(Trigger, 'route/start', self.on_start)
        self.create_service(Trigger, 'route/stop', self.on_stop)
        self.create_service(Trigger, 'route/clear', self.on_clear)
        self.create_timer(0.05, self.loop)
        self.create_timer(0.5, self.publish_route)
        self.get_logger().info('route ready: click points in RViz (Publish Point) and call '
                               '/route/start')

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        self.pose = Pose2D(msg.pose.pose.position.x, msg.pose.pose.position.y,
                           quat_to_yaw(q.x, q.y, q.z, q.w))
        last = self.traveled.poses[-1].pose.position if self.traveled.poses else None
        if last is None or math.hypot(last.x - self.pose.x, last.y - self.pose.y) > 0.01:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose = msg.pose.pose
            self.traveled.poses.append(ps)
            self.traveled.poses = self.traveled.poses[-3000:]

    def on_point(self, msg):
        self.route.add(msg.point.x, msg.point.y)
        self.get_logger().info(f'point {len(self.route.waypoints)}: '
                               f'({msg.point.x:.2f}, {msg.point.y:.2f})')
        self.publish_route()

    def on_goal(self, msg):
        self.route.set_route([(msg.pose.position.x, msg.pose.position.y)])
        self.running = True
        self.get_logger().info(f'going to ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')

    def on_start(self, request, response):
        if self.route.done and self.route.waypoints:
            self.route.index = 0     # run the same route again
        self.running = bool(self.route.waypoints)
        response.success = self.running
        response.message = 'following the route' if self.running else 'no points: click some in RViz'
        return response

    def on_stop(self, request, response):
        self.running = False
        self.cmd_pub.publish(Twist())
        response.success, response.message = True, 'stopped'
        return response

    def on_clear(self, request, response):
        self.running = False
        self.route.clear()
        self.traveled.poses = []
        self.cmd_pub.publish(Twist())
        self.publish_route()
        response.success, response.message = True, 'route cleared'
        return response

    def loop(self):
        if not self.running or self.pose is None:
            return
        v, w = self.route.step(self.pose)
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = float(v), float(w)
        self.cmd_pub.publish(cmd)
        if self.route.done:
            self.running = False
            self.cmd_pub.publish(Twist())
            self.get_logger().info('route finished')

    def publish_route(self):
        stamp = self.get_clock().now().to_msg()
        path = Path()
        path.header.frame_id = self.frame
        path.header.stamp = stamp
        markers = MarkerArray()
        clear = Marker(ns='route', action=Marker.DELETEALL)
        markers.markers.append(clear)
        for i, (x, y) in enumerate(self.route.waypoints):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
            done = i < self.route.index
            for kind, mtype in ((0, Marker.SPHERE), (1, Marker.TEXT_VIEW_FACING)):
                m = Marker(ns='route', id=2 * i + kind, type=mtype, action=Marker.ADD)
                m.header = path.header
                m.pose.position.x, m.pose.position.y = float(x), float(y)
                m.pose.position.z = 0.02 if kind == 0 else 0.08
                m.pose.orientation.w = 1.0
                m.scale.x = m.scale.y = m.scale.z = 0.04
                m.color = ColorRGBA(r=0.3, g=0.9, b=0.3, a=0.9) if done else \
                    ColorRGBA(r=1.0, g=0.6, b=0.1, a=0.9)
                if kind == 1:
                    m.text = str(i + 1)
                    m.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
                markers.markers.append(m)
        self.planned_pub.publish(path)
        self.traveled.header.stamp = stamp
        self.traveled_pub.publish(self.traveled)
        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = RouteNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise       # a real error; otherwise a callback was cut by the shutdown
    finally:
        if rclpy.ok():
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
