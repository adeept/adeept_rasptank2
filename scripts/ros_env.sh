# Carga ROS 2 Humble y el workspace del RaspTank.   source scripts/ros_env.sh
# Busca ROS en /opt/ros/humble (Ubuntu 22.04) o usa el entorno ya activado
# (RoboStack con conda/pixi).
_rt_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
_rt_nounset=0
[[ $- == *u* ]] && _rt_nounset=1 && set +u      # los setup.bash de ROS no soportan -u
if [[ -z "${AMENT_PREFIX_PATH:-}" && -f /opt/ros/humble/setup.bash ]]; then
  source /opt/ros/humble/setup.bash
fi
if [[ -z "${AMENT_PREFIX_PATH:-}" ]]; then
  echo "ROS 2 Humble no encontrado: instala ros-humble-desktop o activa tu entorno RoboStack" >&2
else
  [[ -f "$_rt_root/ros2_ws/install/setup.bash" ]] && source "$_rt_root/ros2_ws/install/setup.bash"
  # simulación: 41 (como RobotMK2). Con el robot real, exporta su dominio antes.
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-41}"
fi
[[ $_rt_nounset == 1 ]] && set -u
unset _rt_root _rt_nounset
