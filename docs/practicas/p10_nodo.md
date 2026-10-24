# Práctica 10 — Tu propio nodo ROS 2

**Objetivo:** pasar de la API `Robot` a un nodo de ROS 2 de verdad:
suscriptores, publicadores, temporizadores y parámetros.

Lee `ros2_ws/src/rasptank_alumno/rasptank_alumno/ejemplos/plantilla_nodo.py` y
ejecútalo: `ros2 run rasptank_alumno plantilla_nodo`.

## Ejercicios

1. Cambia el parámetro sin tocar el código:
   `ros2 run rasptank_alumno plantilla_nodo --ros-args -p distancia_segura:=0.5`.
2. Escribe un nodo `vigilante_bateria` que se suscriba a `/battery` y, si baja
   de 7 V, publique luces rojas en `/leds` y una melodía en `/buzzer`.
3. Escribe un nodo `contador_distancia` que, con `/odom`, publique en
   `/distancia_total` (`std_msgs/Float32`) los metros recorridos.
4. Añádelos a `setup.py` (`entry_points`) y compila con
   `./scripts/build_ros2.sh rasptank_alumno`.
5. Reto: un nodo sigue-paredes que mantenga 20 cm a la derecha con el
   ultrasonido mirando de lado (gira la "cabeza" con la cámara... ¿se puede?
   ¿qué sensor te falta?).

## Preguntas

1. ¿Qué diferencia hay entre un topic y un servicio? Busca un ejemplo de
   cada en el RaspTank.
2. ¿Por qué `/emergency_stop` usa QoS *transient local* (latched)?
3. ¿Qué pasa si dos nodos publican a la vez en `/cmd_vel`?

## Entrega

Los nodos de los ejercicios 2 y 3 y las respuestas.
