# RaspTank2 — guía para Claude

Adeept RaspTank V4 (Raspberry Pi 4) con ROS 2 Humble. La documentación está
en español; escribe en español. El código y sus comentarios, en inglés.

## Mapa del repo

- `ros2_ws/src/rasptank_core`: lógica sin ROS (cinemática, brazo, visión,
  máquina de estados de la misión, rutas). Aquí van los tests de la lógica.
- `ros2_ws/src/rasptank_hw`: nodos del hardware (`driver: pca9685` o `fake`).
- `ros2_ws/src/rasptank_sim`: simulador ligero y cámara sintética.
- `ros2_ws/src/rasptank_perception`: detector (color/YOLO + ArUco).
- `ros2_ws/src/rasptank_mission`: misión coger y llevar el cubo, rutas.
- `ros2_ws/src/rasptank_bringup`: launch, `config/*.yaml`, mundos, RViz.
- `ros2_ws/src/rasptank_alumno`: API `Robot` en español y ejemplos.
- `docker/`: ROS 2 en la Pi; `scripts/pi/camara_host.py`: cámara fuera de Docker.
- `web/`, `examples/`, `Client/`: código original de Adeept (sin ROS), no tocar.

## Comandos

```bash
./scripts/build_ros2.sh                 # compilar
./scripts/test.sh                       # tests unitarios
./scripts/test.sh --e2e                 # + misión completa en simulación (~1 min)
./scripts/sim.sh [mision|linea|vacio]   # simulación con RViz
python3 -m flake8                       # estilo (.flake8)
```

## Reglas

- **Seguridad física:** nunca muevas el robot real (cmd_vel, poses del brazo,
  `docker compose up`) sin que el usuario lo confirme.
- Los cambios de la misión o del simulador se validan con `./scripts/test.sh --e2e`.
- Las medidas del brazo están en el URDF y en `ArmGeometry`: cambia ambos
  (test_description.py lo comprueba).
- El código debe funcionar con Python 3.10 y OpenCV 4.5 (Ubuntu 22.04) y con
  versiones nuevas (RoboStack).
- No versionar `ros2_ws/{build,install,log}`, datasets ni modelos (`*.onnx`, `*.pt`).
- Dominios ROS: simulación 41 por defecto; cada robot real, uno distinto.
