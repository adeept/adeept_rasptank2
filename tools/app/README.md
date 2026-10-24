# Aplicación RaspTank

Panel para clase, en el PC: estado del robot en vivo, **parada de emergencia
siempre visible**, mover las orugas, el brazo y la cámara, **calibración de
la odometría** (manual y automática) y abrir la simulación.

```bash
./scripts/app.sh                  # simulación (dominio 41)
./scripts/app.sh --dominio 10     # el robot real con ROS_DOMAIN_ID=10
```

Necesita ROS 2 Humble, el workspace compilado (`./scripts/build_ros2.sh`) y
PyQt5 (`sudo apt install python3-pyqt5`). El dominio se puede cambiar en la
propia ventana (botón **Conectar**).

## Cabecera

| Modo | Cuándo |
| --- | --- |
| **SIN CONEXIÓN** | No llega el latido de `base_node` en ese dominio |
| **SIMULACIÓN** | `base_node` con `driver: fake` |
| **ROBOT REAL** | `base_node` con `driver: pca9685`: los motores se mueven de verdad |
| **PARADA DE EMERGENCIA** | Alguien ha publicado `/emergency_stop` = true |

El modo lo dice el propio robot (`/base/driver`, `/base/heartbeat`): la app no
se fía de cómo se ha abierto la sesión. **PARADA DE EMERGENCIA** para las orugas,
el brazo, la misión, la teleoperación y la calibración; **Rearmar** (pide
confirmación con el robot real) lo deshace.

## Pestañas

- **Estado:** base, batería, ultrasonido, sensores de línea, odometría (lo que
  el robot *cree*), posición real (solo en simulación), brazo, gobernador,
  cámara (con la imagen) y misión, y una frase de diagnóstico.
- **Mover:** orugas mientras se mantiene pulsado un botón (al soltar, o si la
  ventana pierde el foco, se para), posturas del brazo e inclinación de la cámara.
- **Calibración:** ver abajo.
- **Simulación:** abre o cierra `sim.sh` con el mundo y las opciones elegidas,
  en el dominio de la cabecera. Desactivada si hay un robot real conectado.

## Calibración

El robot no tiene encoders: calcula dónde está a partir de lo que pide a los
motores. La calibración mide cuánto se equivoca y lo corrige (ver
[ROS2.md](../../ROS2.md#10-calibración-de-la-odometría)).

1. Elige **Manual** (cinta métrica) o **Automático** (marcador ArUco de la
   meta, de pie a ~1 m delante del robot, con el detector en marcha).
2. **Empezar calibración**: aparece la lista de pasos (zona muerta, recta,
   giro, otra ronda de recta y giro, comprobación).
3. En cada paso: lee las instrucciones, **Mover el robot**, y en manual escribe
   las medidas y **Aplicar medidas**. En la zona muerta manual, pulsa **¡Se
   mueve!** en cuanto arranquen las orugas.
4. Al terminar, la tabla compara antes y ahora. **Guardar en el robot** lo
   conserva al reiniciar (fichero `~/.rasptank/calibracion.yaml` del robot);
   **Descartar** vuelve a los valores de antes; **Restaurar robot.yaml** borra
   la calibración guardada.

Cada paso se aplica al robot en cuanto termina (el siguiente lo necesita).
**Cancelar** o la parada de emergencia detienen el movimiento al momento. Con
el robot real, la app pide confirmación antes de cada movimiento.

En simulación, el mundo `calibracion` esconde errores como los de un robot
real (una oruga más lenta, zona muerta, deslizamiento al girar). La
**cinta métrica virtual** rellena las medidas con la posición real del
simulador: sirve para practicar y para comprobar que la calibración recupera
los errores escondidos.

## Tests

```bash
cd tools/app && QT_QPA_PLATFORM=offscreen python3 -m pytest tests
```
