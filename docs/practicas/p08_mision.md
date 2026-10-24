# Práctica 8 — La misión: máquinas de estados

**Objetivo:** entender la misión completa como una máquina de estados y
adaptarla.

```bash
./scripts/sim.sh mision mission:=true autostart:=true
```

En RViz, el texto sobre el robot dice el estado. Lee
`ros2_ws/src/rasptank_core/rasptank_core/mission.py` y el diagrama de ROS2.md
(sección 6).

## Ejercicios

1. Dibuja la máquina de estados con todas las transiciones y qué las provoca.
2. Cambia en `config/mission.yaml` `max_linear` y `search_turn_time`. ¿Qué
   pasa con el tiempo total? Mide 3 ejecuciones de cada.
3. Mueve el cubo en `worlds/mision.yaml` detrás del robot. ¿Lo encuentra?
4. **Clasificar:** `./scripts/sim.sh clasificar sort:=true mission:=true
   autostart:=true`. Lee `rasptank_core/sorting.py`: ¿cómo evita coger otra
   vez un cubo que ya está en una meta?
5. Reto: añade un estado nuevo, por ejemplo **CELEBRAR** al final (luces y una
   melodía), y su test en `test/test_mission.py`.

## Preguntas

1. ¿Por qué en la búsqueda el robot gira y se para, en vez de girar sin parar?
2. ¿Para qué sirve el estado **COMPROBAR**?
3. ¿Qué pasaría si la misión no tuviera tiempos máximos (timeouts)?

## Entrega

El diagrama, la tabla de tiempos, el estado nuevo con su test y las respuestas.
