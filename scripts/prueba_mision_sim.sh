#!/usr/bin/env bash
# Prueba de extremo a extremo, sin pantalla: lanza la simulación con la
# misión y comprueba que el cubo acaba en la meta.
#   ./scripts/prueba_mision_sim.sh [mundo] [segundos]
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORLD="${1:-mision}"
TIMEOUT="${2:-180}"
source "$ROOT/scripts/ros_env.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"

LOG="$(mktemp -t rasptank_sim.XXXXXX.log)"
ros2 launch rasptank_bringup sim.launch.py world:="$WORLD" rviz:=false mission:=true \
  autostart:=true >"$LOG" 2>&1 &
LAUNCH_PID=$!
trap 'kill -INT $LAUNCH_PID 2>/dev/null; wait $LAUNCH_PID 2>/dev/null' EXIT

python3 "$ROOT/scripts/comprobar_mision.py" "$TIMEOUT"
RESULT=$?
if [[ $RESULT -ne 0 ]]; then
  echo "--- registro de la simulación: $LOG"
  grep -a -E "ERROR|WARN|died|Traceback" "$LOG" | tail -20
fi
exit $RESULT
