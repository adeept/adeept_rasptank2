"""Plantilla de un nodo de ROS 2 "de verdad" (sin la API Robot).

    ros2 run rasptank_alumno plantilla_nodo

Un nodo se suscribe a topics (callbacks) y publica en otros. Aquí: lee el
ultrasonido y avanza mientras no haya nada a menos de 30 cm. Cópiala para
tus propios nodos y añádelos a setup.py (entry_points).
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range


class MiNodo(Node):
    def __init__(self):
        super().__init__('mi_nodo')
        self.declare_parameter('distancia_segura', 0.30)   # ros2 run ... --ros-args -p distancia_segura:=0.5
        self.distancia = None
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Range, 'ultrasonic', self.on_range, qos_profile_sensor_data)
        self.create_timer(0.1, self.control)                # 10 veces por segundo

    def on_range(self, msg):
        self.distancia = msg.range

    def control(self):
        if self.distancia is None:
            return
        cmd = Twist()
        if self.distancia > self.get_parameter('distancia_segura').value:
            cmd.linear.x = 0.1
        else:
            cmd.angular.z = 0.8          # gira a la izquierda
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    nodo = MiNodo()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise       # a real error; otherwise a callback was cut by the shutdown
    finally:
        if rclpy.ok():
            nodo.pub.publish(Twist())
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
