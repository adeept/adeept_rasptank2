# RaspTank con ROS 2

Workspace de ROS 2 Humble para el Adeept RaspTank V4 (Raspberry Pi 4): nodos
del robot, simulador ligero con RViz, rutas por puntos, visión (cubo rojo y
meta ArUco, o un modelo YOLO entrenado) y la misión **coger el cubo y
llevarlo a la meta**. Pensado para clase: el alumno programa con una API
sencilla en Python o con nodos de ROS 2, primero en simulación y después en
el robot real, sin cambiar su código.

## Índice

1. [Arquitectura](#1-arquitectura)
2. [Instalación en el PC](#2-instalación-en-el-pc)
3. [Simulación](#3-simulación)
4. [Programar el robot (alumnos)](#4-programar-el-robot-alumnos)
5. [Rutas en RViz](#5-rutas-en-rviz)
6. [La misión: coger el cubo y llevarlo a la meta](#6-la-misión-coger-el-cubo-y-llevarlo-a-la-meta)
7. [Visión y entrenamiento de un modelo](#7-visión-y-entrenamiento-de-un-modelo)
8. [El robot real](#8-el-robot-real)
9. [Calibración del hardware](#9-calibración-del-hardware)
10. [Calibración de la odometría](#10-calibración-de-la-odometría)
11. [Seguridad](#11-seguridad)
12. [La aplicación](#12-la-aplicación)
13. [Referencia de topics y servicios](#13-referencia-de-topics-y-servicios)
14. [Tests](#14-tests)

## 1. Arquitectura

```
          Raspberry Pi 4 (Docker)                          PC del alumno
 ┌──────────────────────────────────────┐   WiFi   ┌──────────────────────────────┐
 │ base_node    /cmd_vel → motores      │  (DDS)   │ detector_node  cubo + ArUco  │
 │              /odom (odometría)       │◄────────►│ mission_node   máquina estados│
 │ arm_node     /arm/pose → 5 servos    │          │ route_node     rutas         │
 │ sensors_node /ultrasonic /line_...   │          │ RViz2                         │
 │ camera_node  /camera/image/compressed│          │ código del alumno             │
 └──────────────────────────────────────┘          └──────────────────────────────┘
```

La Raspberry Pi 4 solo mueve el hardware y envía la imagen en JPEG. Todo lo
pesado (visión, misión, RViz) corre en el PC.

En **simulación** se usan los mismos `base_node` y `arm_node` con
`driver: fake`, y `world_node` sustituye a los sensores y a la cámara: dibuja
una imagen sintética (cubo rojo, marcador ArUco de verdad, paredes, línea),
de modo que el detector y la misión son exactamente los del robot real.

| Paquete | Qué contiene |
|---|---|
| `rasptank_core` | Lógica sin ROS: cinemática, odometría, brazo, visión, máquina de estados de la misión, rutas. Con tests. |
| `rasptank_description` | URDF/xacro del tanque, el brazo y la cámara. `display.launch.py` para verlo en RViz. |
| `rasptank_hw` | Nodos del hardware: `base_node`, `arm_node`, `sensors_node`, `camera_node` (driver real o `fake`), `motor_watchdog_node` y `governor_node` (seguridad). |
| `rasptank_calibration` | Calibración de la odometría, manual o automática: `ros2 run rasptank_calibration calibrar`. |
| `rasptank_sim` | Simulador ligero (`world_node`) y generador de datasets sintéticos (`dataset_sim`). |
| `rasptank_perception` | `detector_node` (cubo por color o YOLO, meta por ArUco) y `capture_node` (fotos para el dataset). |
| `rasptank_mission` | `mission_node` (coger y llevar el cubo) y `route_node` (rutas por puntos). |
| `rasptank_bringup` | Launch (`sim`, `robot`, `pc`), configuración, mundos y RViz. |
| `rasptank_alumno` | API `Robot` en español y ejemplos `ej01`…`ej07`, plantilla de nodo. |

Y fuera del workspace, la **aplicación** del PC: `./scripts/app.sh` ([tools/app](tools/app/README.md)).

El código antiguo de Adeept (`web/`, `examples/`, `Client/`) sigue igual y
se puede usar sin ROS.

## 2. Instalación en el PC

Ubuntu 22.04 con ROS 2 Humble (o WSL2 en Windows):

```bash
sudo apt install ros-humble-desktop ros-humble-xacro ros-humble-vision-msgs \
    ros-humble-teleop-twist-keyboard python3-opencv python3-colcon-common-extensions
git clone https://github.com/gbellido-dev/RaspTank2.git
cd RaspTank2
./scripts/build_ros2.sh
source scripts/ros_env.sh        # en cada terminal nuevo
```

Los paquetes son Python puro (más un paquete de mensajes), así que también
funcionan con ROS 2 Humble de [RoboStack](https://robostack.github.io/)
(conda/pixi) en Windows, como el pack de RobotMK2.

## 3. Simulación

```bash
./scripts/sim.sh                 # mundo de la misión: cubo rojo, meta, un obstáculo
./scripts/sim.sh linea           # circuito con línea negra
./scripts/sim.sh vacio           # arena vacía
```

En RViz se ven el robot, el mundo, el haz del ultrasonido, la imagen de la
cámara con las detecciones y las rutas. Para moverlo con el teclado, en otro
terminal:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

El simulador no tiene física: el robot atraviesa las paredes, pero se avisa
en el registro y en `/sim/collision`. Los mundos son ficheros YAML en
`ros2_ws/src/rasptank_bringup/worlds/`; copia uno para crear el tuyo
(`./scripts/sim.sh mi_mundo`).

## 4. Programar el robot (alumnos)

Con la API `Robot` no hace falta saber ROS:

```python
from rasptank_alumno.robot import Robot

robot = Robot()
robot.avanzar(0.5)                  # metros (negativo = atrás)
robot.girar(90)                     # grados, + = izquierda
robot.mover(0.1, 30)                # m/s y grados/s, hasta otra orden
robot.parar()
d = robot.distancia()               # ultrasonido, m (2.0 = nada delante)
izq, centro, der = robot.linea()    # sensores de línea
x, y, angulo = robot.posicion()     # odometría
cubo = robot.ver('cubo')            # {'distancia', 'angulo', 'adelante', ...} o None
robot.brazo('pre_coger')            # posturas de config/robot.yaml
robot.cerrar_pinza()
robot.camara(20)                    # inclinación de la cámara, grados
robot.luces(0, 255, 0)              # LEDs (0-255)
robot.melodia('C5:0.2 E5:0.2 G5:0.4')   # zumbador: nota:segundos, "-" = silencio
robot.cerrar()
```

Ejemplos, de menos a más (con la simulación abierta):

| Ejemplo | Mundo | Qué enseña |
|---|---|---|
| `ros2 run rasptank_alumno ej01_avanzar` | cualquiera | moverse |
| `ej02_cuadrado` | `vacio` | bucles, error de odometría |
| `ej03_evitar_obstaculos` | `mision` | sensores, decisiones |
| `ej04_seguir_linea` | `linea` | control reactivo |
| `ej05_ruta` | `vacio` | puntos y ángulos |
| `ej06_brazo` | cualquiera | brazo, pinza, cámara |
| `ej07_coger_cubo` | `mision` | visión + control proporcional + brazo |
| `ej08_luces_sonido` | cualquiera | LEDs y zumbador como semáforo de distancia |
| `plantilla_nodo` | cualquiera | un nodo de ROS 2 de verdad |

Los ejemplos están en `ros2_ws/src/rasptank_alumno/rasptank_alumno/ejemplos/`.
Un script propio se puede ejecutar directamente con
`python3 mi_programa.py` después de `source scripts/ros_env.sh`.

## 5. Rutas en RViz

```bash
./scripts/sim.sh vacio route:=true
```

1. En RViz, herramienta **Publish Point** y clic en el suelo: cada clic añade
   un punto numerado.
2. `ros2 service call /route/start std_srvs/srv/Trigger`
3. El robot recorre los puntos; en naranja la ruta planificada y en verde el
   recorrido real.

Con **2D Goal Pose** va directamente a un punto. Otros servicios:
`/route/stop` y `/route/clear`. Los puntos también se pueden dar como
parámetro (`waypoints: [x1, y1, x2, y2, ...]`).

En el robot real la posición sale solo de las órdenes de velocidad (no hay
encoders): las rutas largas acumulan error. Es una buena discusión para
clase.

## 6. La misión: coger el cubo y llevarlo a la meta

```bash
./scripts/sim.sh mision mission:=true
ros2 service call /mission/start std_srvs/srv/Trigger     # /mission/stop para pararla
```

o directamente `./scripts/sim.sh mision mission:=true autostart:=true`.

Máquina de estados (`rasptank_core/mission.py`):

```
ESPERA → PREPARAR → BUSCAR_CUBO → APROXIMAR_CUBO → AVANCE_FINAL → COGER
       → COMPROBAR → BUSCAR_META → APROXIMAR_META → SOLTAR → RETROCEDER → FIN
                │ (el cubo sigue en el suelo: reintenta, hasta max_retries)
                └→ APROXIMAR_CUBO                     cualquier tiempo agotado → FALLO
```

- **Buscar:** gira a saltos y se para a mirar (el detector y la WiFi tienen
  retraso).
- **Aproximar:** control proporcional hacia el objetivo. La posición del cubo
  se calcula cortando el rayo de la cámara con el plano del suelo (TF), más
  fiable que el tamaño aparente.
- **Avance final:** los últimos 12 cm el cubo sale de la imagen; avanza a
  ciegas con la odometría.
- **Coger:** posturas `pre_coger` → `cerrar` → `transportar`.
- **Comprobar:** retrocede y mira; si el cubo sigue en el suelo, el agarre
  falló.
- **Soltar:** delante del marcador, `soltar` → `abrir` → `buscar`.

El estado se ve en RViz (texto sobre el robot) y en `/mission/state`. Los
parámetros están en `config/mission.yaml`.

Prueba automática, sin pantalla: `./scripts/prueba_mision_sim.sh` (termina
en unos 45 s con el cubo dentro de la meta).

### Variante: clasificar

Varios cubos de dos colores y dos metas: los rojos al marcador ArUco 0 y los
azules al 1 (`config/sort.yaml`). El robot elige el cubo más cercano, lo sigue
hasta cogerlo sin confundirlo con otro, lo lleva a la meta de su clase y
repite; ignora los que ya están en una meta.

```bash
./scripts/sim.sh clasificar sort:=true mission:=true autostart:=true
./scripts/prueba_mision_sim.sh clasificar 600 sort:=true      # prueba automática
```

Con el robot real: `pc.launch.py sort:=true mission:=true`, dos marcadores
impresos (`generar_marcador.py --id 0` y `--id 1`) y cubos de los colores de
`sort.yaml` (o las clases de un modelo YOLO propio en `sort_classes`).

## 7. Visión y entrenamiento de un modelo

El detector (`config/perception.yaml`) tiene tres niveles:

1. **Color** (por defecto): el cubo rojo en HSV. No necesita entrenamiento,
   pero depende de la luz.
2. **ArUco** (siempre, para la meta): marcador impreso; da distancia y
   ángulo precisos. Para imprimirlo:
   `python3 scripts/generar_marcador.py --id 0 --lado 8`.
3. **YOLO**: un modelo entrenado por los alumnos para el cubo (u otros
   objetos). Se ejecuta en el PC con OpenCV (no hace falta PyTorch).

Proceso de entrenamiento: ver [tools/yolo/README.md](tools/yolo/README.md).
En resumen: capturar fotos con `capture_node`, etiquetarlas, entrenar con
`tools/yolo/entrenar_yolo.py` (PC con GPU o Google Colab), y lanzar con
`cube_detector:=yolo yolo_model:=/ruta/cubo_meta.onnx`. El simulador genera también
datasets sintéticos ya etiquetados (`ros2 run rasptank_sim dataset_sim`).

## 8. El robot real

> **Seguridad:** el robot real se mueve de verdad. Levántalo del suelo (las
> orugas al aire) la primera vez, ten a mano `ros2 topic pub --once
> /emergency_stop std_msgs/msg/Bool "{data: true}"` y empieza con velocidades
> bajas. Si deja de recibir `/cmd_vel` durante 0,5 s, se para solo.

En la Raspberry Pi 4 (Raspberry Pi OS 64 bits, con I2C activado) ROS 2 corre
en Docker; ver [docker/README.md](docker/README.md):

```bash
python3 scripts/pi/camara_host.py &                 # cámara (fuera de Docker)
ROS_DOMAIN_ID=10 docker compose -f docker/compose.yaml up -d
```

En el PC, **mismo `ROS_DOMAIN_ID`** y misma red:

```bash
export ROS_DOMAIN_ID=10
source scripts/ros_env.sh
ros2 launch rasptank_bringup pc.launch.py                 # detector + RViz
ros2 launch rasptank_bringup pc.launch.py mission:=true   # + misión
ros2 run rasptank_alumno ej01_avanzar                     # programas del alumno
```

En RViz pon *Fixed Frame* `odom`. Con varios robots en el aula, **un
`ROS_DOMAIN_ID` distinto por robot** (por ejemplo 10, 11, 12…). Exporta el
dominio *antes* de `source scripts/ros_env.sh`; si no hay ninguno,
`ros_env.sh` pone 41, el de la simulación, para que la simulación y los
programas del alumno se vean sin configurar nada. Si la WiFi del aula bloquea el multicast y los nodos no
se ven entre PC y robot, configura un servidor de descubrimiento de Fast DDS
o `ROS_STATIC_PEERS`/`ROS_DISCOVERY_SERVER`.

## 9. Calibración del hardware

En `ros2_ws/src/rasptank_bringup/config/robot.yaml`:

| Qué | Cómo |
|---|---|
| Sentido de las orugas | `ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}}"`: si una oruga va hacia atrás, `left_sign`/`right_sign: -1`. Si al pedir giro a la izquierda gira a la derecha, intercambia `left_channels` y `right_channels`. |
| Velocidad, zona muerta, giro | Con la calibración de la odometría ([sección 10](#10-calibración-de-la-odometría)), desde la app o con `calibrar`. |
| Servos | `servo_centers`: grados del servo con la articulación a 0 rad (brazo horizontal, pinza cerrada, cámara al frente). `servo_signs: -1` si va al revés. Ajusta después las posturas (`poses`), sobre todo `pre_coger`, para que la pinza quede a 1–2 cm del suelo y 16,5 cm delante del centro del robot. |
| Cámara | Pasa la imagen a raw con `ros2 run image_transport republish compressed raw --ros-args -r in/compressed:=/camera/image/compressed -r out:=/camera/image_raw`, calibra con un tablero de ajedrez (`ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 image:=/camera/image_raw`) y copia `camera_matrix` y `distortion` a `camera_node`. Sin calibrar se usa `hfov`. |
| Medidas | Si el robot no coincide con el URDF (`rasptank_description/urdf/rasptank.urdf.xacro`), corrige las medidas allí y en `ArmGeometry` (un test comprueba que coinciden). |

## 10. Calibración de la odometría

Sin encoders, el robot calcula dónde está a partir de lo que pide a los
motores, y se equivoca de forma **sistemática**: una oruga más lenta (se
desvía en recto), menos velocidad de la supuesta, una zona muerta donde no se
mueve, y deslizamiento al girar (gira menos de lo que cree). La calibración
mide esos errores con unos movimientos y los corrige en `base_node`:

| Prueba | Qué mide | Qué ajusta |
|---|---|---|
| Zona muerta | potencia a la que arrancan las orugas | `min_throttle` |
| Recta (1 m manual, 30 cm ida y vuelta automática) | distancia real y desvío lateral | `max_track_speed`, `left_gain`, `right_gain` |
| Giro (una vuelta manual; oscilaciones y vueltas automáticas) | ángulo real | `track_separation` (efectiva) |
| Batería (opcional) | voltaje al calibrar | `battery_reference`: sube la potencia cuando la batería baja |
| Comprobación | cuadrado de 50 cm (manual) o ida + vuelta + regreso (automática) | nada: dice cuánto error queda |

- **Manual:** cinta métrica y cinta adhesiva para marcar la posición inicial;
  1,5 m de suelo libre.
- **Automática:** el marcador ArUco de la meta de pie a ~1 m delante del robot,
  con ~50 cm libres, y el detector en marcha (`pc.launch.py` o la simulación).
  La cámara mide los movimientos: la distancia por la posición del marcador
  (no por su orientación, que con un marcador pequeño es imprecisa) y el giro
  por el ángulo con el que se ve.

Desde la **app** (pestaña Calibración) o en el terminal:

```bash
ros2 run rasptank_calibration calibrar            # manual
ros2 run rasptank_calibration calibrar --auto     # automática
```

El resultado se aplica en caliente y, al guardarlo, queda en
`~/.rasptank/calibracion.yaml` **del robot** (en Docker, en un volumen), que
`base_node` carga al arrancar por encima de `robot.yaml`. Calibra sobre el
suelo donde se va a usar el robot; los errores aleatorios (cada giro patina
distinto) no se corrigen, así que las rutas largas siguen acumulando error.

**En simulación**, el mundo `calibracion` esconde errores de ese tipo
(`robot_errors` en el YAML del mundo: ningún otro mundo los tiene). En RViz,
la caja naranja es dónde está el robot de verdad y el modelo, dónde cree que
está. Con la **cinta métrica virtual** (`--medidas-sim` o la casilla de la app)
las medidas salen de `/sim/ground_truth`. Ejercicio: calibrar y comprobar que
se recuperan los errores escondidos. En una prueba típica, sin calibrar el
cuadrado de 50 cm acaba a más de 1 m del inicio; calibrado, a unos pocos cm.

## 11. Seguridad

- **Vigilante de órdenes** en `base_node`: sin `/cmd_vel` durante 0,5 s, para.
- **Vigilante de motores** (`motor_watchdog_node`, proceso aparte): si
  `base_node` deja de dar señales de vida (`/base/heartbeat`), apaga
  directamente los canales de los motores del PCA9685, que si no seguirían con
  la última potencia. `base_node` se reinicia solo (`respawn`) y lo primero que
  hace es parar los motores.
- **Gobernador** (`governor_node`, activo por defecto, `governor:=false` para
  quitarlo): limita la velocidad hacia delante con el ultrasonido (frena desde
  45 cm y no avanza a menos de 12 cm). Girar y retroceder siempre se puede. No
  ve objetos bajos (el cubo) ni fuera de su cono. Todos publican en `/cmd_vel`
  como siempre; el gobernador pasa la orden a `base_node` por `/cmd_vel_safe`.
- **Parada de emergencia:** `/emergency_stop` (la app tiene un botón siempre
  visible). Y, en el suelo, el interruptor de la batería a mano.

## 12. La aplicación

`./scripts/app.sh` (`--dominio 10` para un robot real): estado en vivo,
parada de emergencia, mover, calibración y simulación. Ver
[tools/app/README.md](tools/app/README.md).

## 13. Referencia de topics y servicios

| Topic | Tipo | Quién |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | alumno, misión, rutas, app → gobernador → base |
| `/cmd_vel_safe` | `geometry_msgs/Twist` | gobernador → base |
| `/base/heartbeat`, `/base/throttle`, `/base/driver` | | base (vigilante, simulador, app) |
| `/governor/active` | `std_msgs/Bool` | gobernador: true mientras frena |
| `/odom` + TF `odom→base_footprint` | `nav_msgs/Odometry` | base |
| `/emergency_stop` | `std_msgs/Bool` (latched) | cualquiera → base, brazo, misión |
| `/arm/pose` | `std_msgs/String` | postura con nombre → brazo |
| `/arm/joint_command` | `sensor_msgs/JointState` | ángulos sueltos → brazo |
| `/arm/state` | `std_msgs/String` (JSON: `pose`, `moving`, `stopped`) | brazo |
| `/joint_states` | `sensor_msgs/JointState` | brazo (estimado: los servos no tienen realimentación) |
| `/ultrasonic` | `sensor_msgs/Range` | sensores / simulador |
| `/line_sensors` | `std_msgs/Int8MultiArray` (`[izq, centro, der]`, 1 = línea) | sensores / simulador |
| `/battery` | `sensor_msgs/BatteryState` | sensores / simulador |
| `/leds`, `/leds/state` | `std_msgs/ColorRGBA` | color de los LEDs → `signals_node` |
| `/buzzer` | `std_msgs/String` | melodía → `signals_node` |
| `/sim/sorted` | `std_msgs/String` | simulador: "bien/total" cubos clasificados |
| `/camera/image/compressed`, `/camera/camera_info` | JPEG + calibración | cámara / simulador |
| `/detections` | `vision_msgs/Detection2DArray` (`cubo`, `meta`) | detector |
| `/detections/image` | `sensor_msgs/Image` | detector (para RViz) |
| `/mission/state` | `std_msgs/String` | misión |
| `/route/planned`, `/route/traveled` | `nav_msgs/Path` | rutas |
| `/sim/cube_pose`, `/sim/cube_in_goal`, `/sim/collision`, `/sim/ground_truth` | | simulador |

Servicios (`std_srvs/Trigger`): `/mission/start`, `/mission/stop`,
`/route/start`, `/route/stop`, `/route/clear`, `/base/save_calibration`,
`/base/reset_calibration`.

## 14. Tests

```bash
./scripts/test.sh                  # tests unitarios (lógica, simulador, visión, nodos, calibración)
./scripts/test.sh --e2e            # y además la misión completa en simulación
./scripts/test.sh --calibracion    # y la calibración manual y automática simuladas (~10 min)
(cd tools/app && QT_QPA_PLATFORM=offscreen python3 -m pytest tests)   # la app
```
