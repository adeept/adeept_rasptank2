# Práctica 6 — Calibrar la odometría

**Objetivo:** medir los errores sistemáticos del robot y corregirlos.

## 1. El problema

```bash
./scripts/sim.sh calibracion
```

Este mundo esconde errores como los de un robot real. Ejecuta el cuadrado
(`ej02_cuadrado`) y mira en RViz la diferencia entre el robot (donde **cree**
estar) y la caja naranja (donde **está**). Apunta el error.

## 2. Calibración manual

En la app, pestaña **Calibración**, modo **Manual**, con la casilla **cinta
métrica virtual** desactivada. Para medir en simulación, usa la posición real:

```bash
ros2 topic echo /sim/ground_truth --field pose.pose.position
```

Sigue los pasos: zona muerta, recta, giro, otra ronda y comprobación. No
guardes todavía.

## 3. Calibración automática

Pulsa **Descartar**, elige **Automático** y repítela. Compara los resultados
de las dos (tabla de la app).

## 4. En el robot real

Marcas con cinta en el suelo y cinta métrica, calibración manual. Después la
automática con el marcador impreso a 1 m. **Guarda** la mejor.

## Preguntas

1. ¿Qué corrige cada paso? (ROS2.md, sección 10.)
2. ¿Por qué hay que hacer la recta antes del giro?
3. Antes y después de calibrar, ¿cuánto error tiene el cuadrado?
4. ¿Qué errores no puede corregir la calibración?

## Entrega

Tabla de parámetros (antes, manual, automática), error del cuadrado antes y
después, y las respuestas.
