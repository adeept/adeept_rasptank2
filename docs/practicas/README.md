# Prácticas con el RaspTank y ROS 2

Diez prácticas de unas 2 horas, de menos a más. Todas se hacen primero en
**simulación** (no hace falta el robot) y, cuando hay robot, se repiten en el
**robot real** cambiando solo el dominio ROS. Instalación y referencia:
[ROS2.md](../../ROS2.md).

| # | Práctica | Aprendes | Mundo |
|---|---|---|---|
| 1 | [Primeros pasos](p01_primeros_pasos.md) | simulación, RViz, topics, la app | `mision` |
| 2 | [Moverse](p02_moverse.md) | API `Robot`, bucles, odometría | `vacio` |
| 3 | [Sensores y reacciones](p03_sensores.md) | ultrasonido, LEDs, zumbador, decisiones | `mision` |
| 4 | [Seguir una línea](p04_linea.md) | control todo o nada y proporcional | `linea` |
| 5 | [Rutas](p05_rutas.md) | coordenadas, ángulos, rutas en RViz | `vacio` |
| 6 | [Calibrar la odometría](p06_calibracion.md) | errores sistemáticos, medir, corregir | `calibracion` |
| 7 | [Visión: ver el cubo y la meta](p07_vision.md) | color HSV, ArUco, coger el cubo | `mision` |
| 8 | [La misión](p08_mision.md) | máquinas de estados, parámetros, clasificar | `mision`, `clasificar` |
| 9 | [Entrenar un detector](p09_entrenar.md) | datos, etiquetas, entrenar, evaluar | `mision` |
| 10 | [Tu propio nodo ROS 2](p10_nodo.md) | publicar, suscribirse, parámetros | cualquiera |

**Antes de cada práctica**, en cada terminal:

```bash
cd RaspTank2
source scripts/ros_env.sh
```

**Normas con el robot real:** el robot levantado (orugas al aire) la primera
vez que se prueba un programa, velocidades bajas, la app abierta con la
**PARADA DE EMERGENCIA** a mano, y el interruptor de la batería cerca.

Cada práctica termina con **entrega**: el código, las medidas o capturas y
las respuestas a las preguntas.
