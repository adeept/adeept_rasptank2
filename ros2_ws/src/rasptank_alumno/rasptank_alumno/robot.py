"""API sencilla para programar el RaspTank sin saber ROS 2.

    from rasptank_alumno.robot import Robot

    robot = Robot()
    robot.avanzar(0.3)              # metros
    robot.girar(90)                 # grados, positivo = izquierda
    print(robot.distancia())        # ultrasonido, metros
    robot.cerrar()

Funciona igual con el simulador (sim.launch.py) y con el robot real
(robot.launch.py en la Raspberry). Por debajo es un nodo de ROS 2 que
publica /cmd_vel y /arm/pose y escucha /odom, /ultrasonic,
/line_sensors, /detections, /arm/state y /battery: cuando quieras ver
cómo se hace "de verdad", lee este fichero y plantilla_nodo.py.
"""
import math
import threading
import time

import rclpy
import rclpy.time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, JointState, Range
from std_msgs.msg import ColorRGBA, String
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection2DArray

from rasptank_interfaces.msg import ArmState, LineSensors

ARTICULACIONES = ['shoulder_joint', 'elbow_joint', 'wrist_roll_joint', 'gripper_joint',
                  'camera_tilt_joint']


ALTURA_CUBO = 0.02     # m, altura del centro del cubo
ALCANCE_PINZA = 0.165  # m delante del centro del robot donde baja la pinza (pre_coger)


def _transformar(tf, p):
    """Origen y punto p (x, y, z) del sistema de la cámara en el del robot."""
    t, q = tf.transform.translation, tf.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
         [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
         [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    origen = (t.x, t.y, t.z)
    punto = [origen[i] + sum(R[i][j] * p[j] for j in range(3)) for i in range(3)]
    return origen, punto


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Robot:
    """El RaspTank. Todas las distancias en metros y los ángulos en grados."""

    def __init__(self, nombre='alumno'):
        if not rclpy.ok():
            try:
                # Ctrl+C llega como KeyboardInterrupt al programa del alumno, que
                # así puede parar el robot (robot.cerrar()) antes de salir
                from rclpy.signals import SignalHandlerOptions
                rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
            except (ImportError, TypeError):
                rclpy.init()
        self.node = Node(nombre)
        self._lock = threading.Lock()
        self._odom = None
        self._range = None
        self._line = None
        self._detections = {}
        self._arm = None
        self._battery = None
        self._cmd = (0.0, 0.0)
        self._send_until = 0.0

        n = self.node
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, n)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._cmd_pub = n.create_publisher(Twist, 'cmd_vel', 10)
        self._pose_pub = n.create_publisher(String, 'arm/pose', 10)
        self._joint_pub = n.create_publisher(JointState, 'arm/joint_command', 10)
        self._leds_pub = n.create_publisher(ColorRGBA, 'leds', 10)
        self._buzzer_pub = n.create_publisher(String, 'buzzer', 10)
        n.create_subscription(Odometry, 'odom', lambda m: self._set('_odom', m), 10)
        n.create_subscription(Range, 'ultrasonic', lambda m: self._set('_range', m),
                              qos_profile_sensor_data)
        n.create_subscription(LineSensors, 'line_sensors', lambda m: self._set('_line', m), 10)
        n.create_subscription(ArmState, 'arm/state', lambda m: self._set('_arm', m), latched)
        n.create_subscription(BatteryState, 'battery', lambda m: self._set('_battery', m), 10)
        n.create_subscription(Detection2DArray, 'detections', self._on_detections, 10)
        # se repite la orden de velocidad para que no salte el "watchdog" de la base
        n.create_timer(0.1, self._send_cmd)

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(n)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()
        self._esperar_datos()

    # ------------------------------------------------------------ interno

    def _set(self, name, msg):
        with self._lock:
            setattr(self, name, msg)

    def _on_detections(self, msg):
        now = time.monotonic()
        try:
            tf = self._tf_buffer.lookup_transform('base_footprint', msg.header.frame_id,
                                                  rclpy.time.Time())
        except TransformException:
            return
        for d in msg.detections:
            if not d.results:
                continue
            cls = d.results[0].hypothesis.class_id
            p = d.results[0].pose.pose.position
            origen, punto = _transformar(tf, (p.x, p.y, p.z))
            if cls == 'cubo':
                # el tamaño aparente da una distancia aproximada; mejor cortar el
                # rayo de la cámara con el plano del centro del cubo (2 cm)
                rayo = [punto[i] - origen[i] for i in range(3)]
                if rayo[2] >= -1e-6:
                    continue
                k = (ALTURA_CUBO - origen[2]) / rayo[2]
                punto = [origen[i] + k * rayo[i] for i in range(3)]
            with self._lock:
                self._detections[cls] = (now, d, punto[0], punto[1])

    def _send_cmd(self):
        v, w = self._cmd
        if time.monotonic() < self._send_until:
            t = Twist()
            t.linear.x, t.angular.z = float(v), float(w)
            self._cmd_pub.publish(t)

    def _esperar_datos(self, segundos=10.0):
        fin = time.monotonic() + segundos
        while self._odom is None and time.monotonic() < fin:
            time.sleep(0.05)
        if self._odom is None:
            print('AVISO: no llega /odom. ¿Está arrancado el robot o la simulación? '
                  '¿Mismo ROS_DOMAIN_ID?')

    # ------------------------------------------------------------ movimiento

    def mover(self, velocidad, giro=0.0):
        """Mueve el robot sin parar: velocidad en m/s, giro en grados/s (+ = izquierda).

        Sigue moviéndose hasta que llames a parar() u otra orden.
        """
        self._cmd = (velocidad, math.radians(giro))
        self._send_until = float('inf')

    def parar(self):
        self._cmd = (0.0, 0.0)
        self._send_until = time.monotonic() + 0.5    # unos ceros y deja de publicar

    def esperar(self, segundos):
        time.sleep(segundos)

    def avanzar(self, distancia, velocidad=0.1):
        """Avanza (o retrocede si es negativa) una distancia y se para."""
        x0, y0, _ = self.posicion()
        self.mover(math.copysign(abs(velocidad), distancia))
        limite = time.monotonic() + abs(distancia) / abs(velocidad) * 2 + 2
        while time.monotonic() < limite:
            x, y, _ = self.posicion()
            if math.hypot(x - x0, y - y0) >= abs(distancia):
                break
            time.sleep(0.02)
        self.parar()

    def girar(self, grados, velocidad=45.0):
        """Gira sobre sí mismo (grados, + = izquierda) y se para."""
        girado = 0.0
        anterior = self.posicion()[2]
        limite = time.monotonic() + abs(grados) / abs(velocidad) * 2 + 3
        while abs(girado) < abs(grados) - 0.5 and time.monotonic() < limite:
            # frena al final para no pasarse (el robot tarda en reaccionar)
            falta = abs(grados) - abs(girado)
            self.mover(0.0, math.copysign(max(8.0, min(abs(velocidad), 2.0 * falta)), grados))
            time.sleep(0.02)
            actual = self.posicion()[2]
            girado += (actual - anterior + 180) % 360 - 180
            anterior = actual
        self.parar()

    def ir_a(self, x, y, velocidad=0.1):
        """Va al punto (x, y) del mapa de odometría: gira hacia él y avanza."""
        px, py, angulo = self.posicion()
        rumbo = math.degrees(math.atan2(y - py, x - px))
        self.girar((rumbo - angulo + 180) % 360 - 180)
        px, py, _ = self.posicion()
        self.avanzar(math.hypot(x - px, y - py), velocidad)

    # ------------------------------------------------------------ sensores

    def posicion(self):
        """(x, y, ángulo en grados) según la odometría. Empieza en (0, 0, 0)."""
        with self._lock:
            m = self._odom
        if m is None:
            return 0.0, 0.0, 0.0
        p = m.pose.pose
        return p.position.x, p.position.y, math.degrees(_yaw(p.orientation))

    def distancia(self):
        """Distancia del ultrasonido en metros (2.0 = no hay nada delante)."""
        with self._lock:
            m = self._range
        return m.range if m is not None else float('nan')

    def linea(self):
        """(izquierda, centro, derecha): True si ese sensor ve la línea negra."""
        with self._lock:
            m = self._line
        return (m.left, m.middle, m.right) if m is not None else (False, False, False)

    def bateria(self):
        """Voltaje de la batería."""
        with self._lock:
            m = self._battery
        return m.voltage if m is not None else float('nan')

    def ver(self, objeto='cubo', antiguedad=0.5):
        """Lo que ve la cámara de un objeto ("cubo" o "meta"), o None.

        Devuelve un diccionario con:
          distancia  metros desde el centro del robot, sobre el suelo
          angulo     grados desde el frente del robot, + = a la izquierda
          adelante, izquierda  la misma posición en metros (x, y del robot)
          px, py     centro en la imagen (píxeles)
          ancho, alto  tamaño en la imagen (píxeles)
        Necesita el detector (lo lanzan sim.launch.py y pc.launch.py).
        """
        with self._lock:
            item = self._detections.get(objeto)
        if item is None or time.monotonic() - item[0] > antiguedad:
            return None
        _, d, x, y = item
        return {'distancia': math.hypot(x, y), 'angulo': math.degrees(math.atan2(y, x)),
                'adelante': x, 'izquierda': y,
                'px': d.bbox.center.position.x, 'py': d.bbox.center.position.y,
                'ancho': d.bbox.size_x, 'alto': d.bbox.size_y}

    # ------------------------------------------------------------ brazo

    def brazo(self, postura, esperar=True):
        """Lleva el brazo a una postura con nombre (robot.yaml): plegado,
        buscar, pre_coger, cerrar, abrir, transportar, soltar."""
        self._pose_pub.publish(String(data=postura))
        if esperar:
            self._esperar_brazo(postura)

    def articulaciones(self, angulos, esperar=True):
        """Mueve articulaciones sueltas, en grados: {'gripper_joint': 30}."""
        js = JointState()
        js.name = list(angulos)
        js.position = [math.radians(a) for a in angulos.values()]
        self._joint_pub.publish(js)
        if esperar:
            self._esperar_brazo('')

    def camara(self, grados, esperar=True):
        """Inclina la cámara: 0 = al frente, positivo = hacia abajo."""
        self.articulaciones({'camera_tilt_joint': grados}, esperar)

    def abrir_pinza(self):
        self.brazo('abrir')

    def cerrar_pinza(self):
        self.brazo('cerrar')

    def _esperar_brazo(self, postura, segundos=6.0):
        time.sleep(0.2)
        fin = time.monotonic() + segundos
        while time.monotonic() < fin:
            with self._lock:
                a = self._arm
            if a is not None and not a.moving and (not postura or a.pose == postura):
                return
            time.sleep(0.05)
        print(f'AVISO: el brazo no confirma la postura {postura!r}')

    # ------------------------------------------------------------ luces y sonido

    def luces(self, rojo, verde, azul):
        """Color de los LEDs, cada componente de 0 a 255: robot.luces(0, 255, 0)."""
        self._leds_pub.publish(ColorRGBA(r=rojo / 255.0, g=verde / 255.0, b=azul / 255.0, a=1.0))

    def apagar_luces(self):
        self.luces(0, 0, 0)

    def pitar(self, nota='A5', segundos=0.2):
        """Una nota (C4, D#5, A5...) durante unos segundos, sin esperar."""
        self.melodia(f'{nota}:{segundos}')

    def melodia(self, notas):
        """Varias notas "nota:segundos" separadas por espacios; "-" es un silencio:
        robot.melodia('C5:0.2 E5:0.2 G5:0.4'). No espera a que termine."""
        self._buzzer_pub.publish(String(data=notas))

    # ------------------------------------------------------------ fin

    def cerrar(self):
        """Para el robot y cierra la conexión con ROS."""
        self._cmd = (0.0, 0.0)
        self._send_until = 0.0
        if rclpy.ok():
            for _ in range(3):
                self._cmd_pub.publish(Twist())
                time.sleep(0.05)
        self._executor.shutdown(timeout_sec=1.0)
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.cerrar()
