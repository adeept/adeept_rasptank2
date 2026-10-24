# Práctica 1 — Primeros pasos

**Objetivo:** arrancar la simulación, ver el robot en RViz y entender qué es
un *topic* de ROS 2.

## 1. La simulación

```bash
./scripts/sim.sh mision
```

Se abre RViz: el tanque, una arena con paredes, un cubo rojo y una meta con
un marcador ArUco. Arriba a la izquierda, la imagen de la cámara.

1. Gira la vista con el ratón (arrastrar), acerca con la rueda.
2. Activa la casilla **TF** en *Displays*: ¿qué son esos ejes? Busca
   `base_footprint`, `camera_optical_frame` y `gripper_center_link`.

## 2. La app

En otro terminal: `./scripts/app.sh`. La cabecera debe decir **SIMULACIÓN**.

- Pestaña **Mover**: mueve el robot con las flechas y el brazo con las posturas.
- Pulsa **PARADA DE EMERGENCIA** y luego intenta moverlo. Pulsa **Rearmar**.

## 3. Los topics

En otro terminal:

```bash
ros2 topic list
ros2 topic echo /ultrasonic --field range
ros2 topic hz /camera/image/compressed
ros2 topic pub --once /arm/pose std_msgs/msg/String "{data: buscar}"
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"    # Ctrl+C para parar
ros2 node list
```

## Preguntas

1. ¿Qué topic publica la distancia del ultrasonido? ¿Cada cuánto?
2. Cuando paras `ros2 topic pub` del último comando, el robot se para en
   medio segundo. ¿Por qué? (Pista: `base_node`, `cmd_timeout`.)
3. En RViz hay una caja naranja transparente además del robot. Lee la
   sección 10 de ROS2.md: ¿qué representa cada uno?
4. Dibuja un esquema: qué nodos hay y qué topics los unen (`rqt_graph` ayuda
   si está instalado).

## Entrega

Las respuestas y una captura de RViz con el robot mirando al cubo.
