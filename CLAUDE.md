# RaspTank2 — guía para Claude

Adeept RaspTank V4 (Raspberry Pi 4) con ROS 2 Humble. La documentación está
en español; escribe en español. El código y sus comentarios, en inglés.

## Mapa del repo

- `ros2_ws/src/rasptank_core`: lógica sin ROS (cinemática, brazo, visión,
  máquina de estados de la misión, rutas). Aquí van los tests de la lógica.
- `ros2_ws/src/rasptank_hw`: nodos del hardware (`driver: pca9685` o `fake`), el
  vigilante de motores y el gobernador de velocidad (ultrasonido).
- `ros2_ws/src/rasptank_calibration`: calibración de la odometría (manual y
  automática con ArUco): sesión ROS, pasos (`procedure.py`), asistente para la
  app (`flow.py`) y CLI `calibrar`.
- `ros2_ws/src/rasptank_sim`: simulador ligero, cámara sintética y errores de
  motor simulados (`robot_errors`, `/sim/ground_truth`).
- `ros2_ws/src/rasptank_perception`: detector (color/YOLO + ArUco).
- `ros2_ws/src/rasptank_mission`: misión coger y llevar el cubo, rutas.
- `ros2_ws/src/rasptank_bringup`: launch, `config/*.yaml`, mundos, RViz.
- `ros2_ws/src/rasptank_alumno`: API `Robot` en español y ejemplos.
- `tools/app/rasptank_app`: app PyQt5 del PC (estado, parada, mover, calibración, simulación).
- `docker/`: ROS 2 en la Pi; `scripts/pi/camara_host.py`: cámara fuera de Docker.
- `tools/windows/pixi.toml` (+ `pixi.lock`) y `scripts/*_windows.bat`: Windows con
  RoboStack, ver WINDOWS.md. `scripts/tests.py`: tests en Linux y Windows.
- `web/`, `examples/`, `Client/`: código original de Adeept (sin ROS), no tocar.

## Comandos

```bash
./scripts/build_ros2.sh                 # compilar
./scripts/test.sh                       # tests unitarios
./scripts/test.sh --e2e                 # + misión completa en simulación (~1 min)
./scripts/test.sh --calibracion         # + calibración manual y automática simuladas (~10 min)
(cd tools/app && QT_QPA_PLATFORM=offscreen python3 -m pytest tests)   # app
./scripts/app.sh [--dominio N]          # la app
./scripts/sim.sh [mision|linea|vacio]   # simulación con RViz
python3 -m flake8                       # estilo (.flake8)
```

## Reglas

- **Seguridad física:** nunca muevas el robot real (cmd_vel, poses del brazo,
  `docker compose up`) sin que el usuario lo confirme.
- Los cambios de la misión o del simulador se validan con `./scripts/test.sh --e2e`;
  los de la calibración o del modelo de motores, con `--calibracion`.
- Seguridad de la base: no quitar el vigilante de `base_node`, el
  `motor_watchdog_node` ni el `respawn`; el gobernador solo reduce velocidad.
- Las medidas del brazo están en el URDF y en `ArmGeometry`: cambia ambos
  (test_description.py lo comprueba).
- El código debe funcionar con Python 3.10 y OpenCV 4.5 (Ubuntu 22.04) y con
  versiones nuevas (RoboStack).
- No versionar `ros2_ws/{build,install,log}`, datasets ni modelos (`*.onnx`, `*.pt`),
  ni `.pixi/` ni `windows/`.
- Todo el workspace es Python (mensajes estándar, paquetes ament_python): no
  añadir mensajes propios ni código C++, o Windows necesitaría Visual Studio.
- Dominios ROS: simulación 41 por defecto; cada robot real, uno distinto.
