# Carga ROS 2 Humble y el workspace del RaspTank. Uso: source scripts/ros_env.sh
# Busca ROS en /opt/ros/humble (Ubuntu 22.04) o en un entorno de conda/pixi
# ya activado (RoboStack).
_RT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
if [[ -z "${ROS_DISTRO:-}" ]]; then
  if [[ -f /opt/ros/humble/setup.bash ]]; then
    source /opt/ros/humble/setup.bash
  else
    echo "ROS 2 Humble no encontrado: instala ros-humble-desktop o activa tu entorno RoboStack" >&2
    return 1 2>/dev/null || exit 1
  fi
fi
if [[ -f "$_RT_ROOT/ros2_ws/install/setup.bash" ]]; then
  source "$_RT_ROOT/ros2_ws/install/setup.bash"
fi
set -u 2>/dev/null || true
unset _RT_ROOT
