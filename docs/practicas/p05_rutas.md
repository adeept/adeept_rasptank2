# Práctica 5 — Rutas

**Objetivo:** ir de un punto a otro con coordenadas y planificar rutas en RViz.

## 1. Programando

```bash
./scripts/sim.sh vacio
ros2 run rasptank_alumno ej05_ruta
```

El robot empieza en (0, 0) mirando hacia +x. `robot.ir_a(x, y)` gira hacia el
punto y avanza.

1. Escribe una ruta que pase por 5 puntos y vuelva al inicio.
2. Programa tú `ir_a` sin usar el de la API: con `robot.posicion()`,
   `math.atan2` y `math.hypot`, gira hacia el punto y avanza.

## 2. Con el ratón

```bash
./scripts/sim.sh vacio route:=true
```

Herramienta **Publish Point** de RViz: haz clic en varios puntos. Luego:
`ros2 service call /route/start std_srvs/srv/Trigger`. En naranja la ruta, en
verde el recorrido.

## 3. En el robot real

Haz la misma ruta y marca con cinta dónde termina.

## Preguntas

1. En el robot real, ¿dónde termina comparado con la simulación? ¿Por qué?
2. ¿Qué es `atan2` y por qué no basta `atan`?
3. ¿Cómo sabría el robot dónde está de verdad? (Piensa en sensores que no tiene.)

## Entrega

Tu `ir_a`, la ruta y las medidas en el robot real.
