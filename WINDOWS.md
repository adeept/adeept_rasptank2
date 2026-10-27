# RaspTank en Windows

ROS 2 Humble nativo en Windows 10/11 (64 bits) con
[RoboStack](https://robostack.github.io/) y [Pixi](https://pixi.sh), como en
RobotMK2. El workspace del RaspTank es todo Python: **no hace falta Visual
Studio ni compilar nada en C++**.

## Instalar (una vez, con internet)

```bat
scripts\instalar_windows.bat
```

Descarga `pixi.exe` en `windows\` si no lo tienes, instala el entorno de
`tools\windows\pixi.toml` (versiones fijadas en `pixi.lock`, unos 3 GB en
`tools\windows\.pixi`) y compila el workspace. Si mueves la carpeta del
proyecto, vuelve a ejecutar `scripts\compilar_windows.bat`.

## Usar

O todo desde el menú: **`menu_rasptank.bat`**.

| Qué | Cómo |
|---|---|
| Aplicación (estado, parada, mover, calibración, simulación) | `scripts\app_windows.bat` |
| Simulación con RViz | `scripts\sim_windows.bat mision` (o `linea`, `vacio`, `calibracion`, `clasificar`) |
| Misión automática | `scripts\sim_windows.bat mision mission:=true autostart:=true` |
| Terminal con ROS 2 (programas del alumno, `ros2 topic list`...) | `scripts\terminal_ros2_windows.bat` |
| Calibrar la odometría en el terminal | `scripts\calibrar_windows.bat [--auto]` |
| Compilar después de cambiar código | `scripts\compilar_windows.bat` |
| Tests | `scripts\tests_windows.bat [--e2e]` |

En el terminal de ROS 2, los programas del alumno se ejecutan igual que en
Linux: `ros2 run rasptank_alumno ej01_avanzar` o `python mi_programa.py`.

## Con el robot real

1. El robot (Raspberry Pi) y el PC en la misma red WiFi.
2. El mismo dominio ROS en los dos. En el PC, antes de lanzar nada:
   `set ROS_DOMAIN_ID=10` (o en la app, campo **Dominio ROS**).
3. Permite ROS 2 en el cortafuegos de Windows, una vez y como administrador:
   `scripts\abrir_firewall_windows.bat` (`/preview` para ver qué hará).
4. Detector, misión y RViz en el PC: en el terminal de ROS 2,
   `ros2 launch rasptank_bringup pc.launch.py`.

## Problemas habituales

| Síntoma | Qué hacer |
|---|---|
| `pixi` no se descarga | Descarga `pixi-x86_64-pc-windows-msvc.exe` de github.com/prefix-dev/pixi/releases, renómbralo a `windows\pixi.exe` y repite. |
| `AVISO: el workspace no esta compilado` | `scripts\compilar_windows.bat` |
| No se ven los topics del robot | Mismo `ROS_DOMAIN_ID`, misma red, cortafuegos (paso 3). |
| RViz no abre o se ve negro | Actualiza el controlador de la tarjeta gráfica. |
