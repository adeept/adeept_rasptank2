# Práctica 3 — Sensores y reacciones

**Objetivo:** leer el ultrasonido y reaccionar: esquivar obstáculos y avisar
con luces y sonido.

```bash
./scripts/sim.sh mision
ros2 run rasptank_alumno ej03_evitar_obstaculos
ros2 run rasptank_alumno ej08_luces_sonido      # mueve el robot con la app
```

## Ejercicios

1. Modifica `ej03` para que, antes de girar, **mire** a izquierda y derecha
   (gira 45°, mide, gira -90°, mide) y elija el lado con más espacio.
2. Añade luces: verde avanzando, amarillo mirando, rojo cuando está cerca.
3. Haz un "radar": el robot gira una vuelta en pasos de 15° y apunta la
   distancia en cada ángulo. Imprime la tabla y di dónde está lo más cercano.

## Preguntas

1. El cubo de la arena mide 4 cm. ¿Lo ve el ultrasonido? ¿Por qué?
   (Mira la altura del sensor en el URDF: `ultrasonic_link`.)
2. El **gobernador** (ROS2.md, sección 11) frena aunque tu programa diga
   avanzar. Prueba a lanzar con `governor:=false`: ¿qué cambia? ¿Por qué es
   buena idea tenerlo con alumnos?
3. ¿Qué valor da `robot.distancia()` cuando no hay nada delante?

## Entrega

El programa del ejercicio 1, la tabla del radar y las respuestas.
