# Práctica 7 — Visión: ver el cubo y la meta

**Objetivo:** entender cómo el robot ve el cubo (color) y la meta (ArUco), y
usarlo para cogerlo.

```bash
./scripts/sim.sh mision
ros2 topic echo /detections --field detections     # lo que ve el detector
ros2 run rasptank_alumno ej07_coger_cubo
```

## Ejercicios

1. Cambia los rangos HSV del rojo en `config/perception.yaml` (por ejemplo,
   `S` mínimo de 120 a 40), vuelve a compilar (`./scripts/build_ros2.sh
   rasptank_bringup`) y mira la imagen de RViz: ¿detecta más cosas?
2. En `ej07`, imprime `robot.ver('meta')` mientras te acercas al marcador.
   Compara la distancia con la de `/sim/ground_truth`.
3. Completa `ej07` para que, después de coger el cubo, lo lleve a la meta.
4. Con el robot real: imprime el marcador (`python3 scripts/generar_marcador.py`)
   y comprueba la distancia que da con una cinta métrica.

## Preguntas

1. ¿Por qué el rojo necesita **dos** rangos en HSV?
2. ¿Por qué la distancia a la meta (ArUco) es más precisa que la del cubo?
3. ¿Qué haría fallar al detector por color en el aula?

## Entrega

`ej07` completo (cubo a la meta) y las respuestas.
