# Práctica 2 — Moverse

**Objetivo:** programar movimientos con la API `Robot` y descubrir el error
de la odometría.

```bash
./scripts/sim.sh vacio
ros2 run rasptank_alumno ej01_avanzar
ros2 run rasptank_alumno ej02_cuadrado
```

Abre los ficheros en `ros2_ws/src/rasptank_alumno/rasptank_alumno/ejemplos/`
y léelos: `robot.avanzar()`, `robot.girar()`, `robot.posicion()`.

## Ejercicios

1. Copia `ej02_cuadrado.py` como `mi_poligono.py` y haz que dibuje un polígono
   de N lados (N lo eliges al empezar). ¿Cuánto hay que girar en cada vértice?
   Ejecútalo con `python3 mi_poligono.py`.
2. Haz un "ocho" con `robot.mover(velocidad, giro)` y `robot.esperar()`.
3. Imprime la posición después de cada lado. ¿Vuelve exactamente al inicio?

## En el robot real

Repite el cuadrado en el suelo y marca con cinta dónde empieza y dónde
acaba. Mide la distancia.

## Preguntas

1. En simulación, ¿cuánto error tiene el cuadrado? ¿Y en el robot real?
2. El robot no tiene encoders: ¿cómo calcula `robot.posicion()`?
3. ¿Qué pasa si subes mucho la velocidad? ¿Y el error?

## Entrega

`mi_poligono.py`, la tabla de errores (simulación y robot) y las respuestas.
