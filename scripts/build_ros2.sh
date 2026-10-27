#!/usr/bin/env bash
# Compila el workspace ROS 2 del RaspTank.  ./scripts/build_ros2.sh [paquetes...]
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
if [[ -z "${AMENT_PREFIX_PATH:-}" ]]; then
  [[ -f /opt/ros/humble/setup.bash ]] || { echo "ROS 2 Humble no encontrado" >&2; exit 1; }
  source /opt/ros/humble/setup.bash
fi
set -u
cd "$ROOT/ros2_ws"
if [[ $# -gt 0 ]]; then
  colcon build --symlink-install --packages-select "$@"
else
  colcon build --symlink-install
fi
echo "Listo. En cada terminal:  source $ROOT/scripts/ros_env.sh"
