# ROS 2 en la Raspberry Pi 4 (Docker)

Raspberry Pi OS (Bookworm, 64 bits) no tiene paquetes de ROS 2, así que ROS 2
Humble corre en un contenedor y el sistema se queda como está: el código de
Adeept (`web/`, `setup.py`) sigue funcionando si no está el contenedor.

> No arranques a la vez el servidor web de Adeept (`webServer.service`) y el
> contenedor: los dos usan los mismos motores y servos.

## Preparar la Pi (una vez)

```bash
sudo raspi-config nonint do_i2c 0          # activa I2C (PCA9685 y ADS7830)
curl -fsSL https://get.docker.com | sh
sudo usermod -aG docker $USER              # y vuelve a entrar en la sesión
sudo systemctl disable --now webServer     # si instalaste el servidor de Adeept
git clone https://github.com/gbellido-dev/RaspTank2.git && cd RaspTank2
docker compose -f docker/compose.yaml build      # unos 15-20 min la primera vez
```

## Arrancar

```bash
python3 scripts/pi/camara_host.py &                     # cámara: picamera2 fuera de Docker
ROS_DOMAIN_ID=10 docker compose -f docker/compose.yaml up -d
docker logs -f rasptank                                  # ver qué pasa
```

El contenedor lanza `robot.launch.py` (base, brazo, sensores, cámara y
robot_state_publisher) y se reinicia solo con la Pi (`restart:
unless-stopped`). Para pararlo: `docker compose -f docker/compose.yaml down`.

**¿Por qué la cámara va fuera?** picamera2 necesita la libcamera de Raspberry
Pi OS, que no está en el contenedor (Ubuntu 22.04). `camara_host.py` lee la
cámara y pasa los JPEG por un socket local (`127.0.0.1:8555`) al
`camera_node` del contenedor (`backend: tcp`). Para que arranque solo, crea
un servicio de systemd o añádelo a `crontab -e` con
`@reboot python3 /home/pi/RaspTank2/scripts/pi/camara_host.py`.

## Comprobar

```bash
docker exec -it rasptank bash
ros2 topic list
ros2 topic hz /camera/image/compressed
ros2 topic echo /ultrasonic --once
ros2 topic pub --once /arm/pose std_msgs/msg/String "{data: buscar}"
```

Mover las orugas por primera vez **con el robot levantado**:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}}"
```

## Problemas habituales

| Síntoma | Causa probable |
|---|---|
| `ValueError: No I2C device at address: 0x5f` | I2C desactivado o la placa de Adeept sin alimentación (interruptor de la batería). |
| El PC no ve los topics del robot | Distinto `ROS_DOMAIN_ID`, otra red, o la WiFi bloquea multicast. |
| `camera stream ... retrying` | `camara_host.py` no está en marcha. |
| Los servos tiemblan o se van al extremo al arrancar | Revisa `servo_centers` y las posturas en `config/robot.yaml`. |

Para cambiar la configuración sin reconstruir la imagen, monta el YAML:
añade en `compose.yaml`
`- ../ros2_ws/src/rasptank_bringup/config/robot.yaml:/ws/install/rasptank_bringup/share/rasptank_bringup/config/robot.yaml`.
