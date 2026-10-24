# Práctica 4 — Seguir una línea

**Objetivo:** controlar el robot con tres sensores de línea, primero "todo o
nada" y después proporcional.

```bash
./scripts/sim.sh linea
ros2 run rasptank_alumno ej04_seguir_linea
```

`robot.linea()` devuelve `(izquierda, centro, derecha)`: `True` si ese sensor
ve la línea negra.

## Ejercicios

1. Mide cuánto tarda en dar una vuelta al circuito. Prueba 3 valores de
   `VELOCIDAD` y `GIRO`: ¿cuál es el más rápido que no se sale?
2. **Control proporcional:** calcula un error con los sensores (por ejemplo
   izquierda = -1, centro = 0, derecha = +1, la media de los que ven línea) y
   gira `K * error`. Compara con el todo o nada.
3. Si pierde la línea, que vuelva a buscarla girando hacia el último lado
   donde la vio.

## Preguntas

1. ¿Por qué oscila el control todo o nada?
2. ¿Qué pasa si K es muy grande? ¿Y muy pequeña?
3. Con el robot real: los sensores pueden dar `True` sobre blanco (al revés).
   ¿Qué parámetro lo cambia? (`config/robot.yaml`, `line_active_high`.)

## Entrega

Los dos controladores, la tabla de tiempos por vuelta y las respuestas.
