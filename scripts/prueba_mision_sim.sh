#!/usr/bin/env bash
# Prueba de extremo a extremo, sin pantalla: lanza la simulación con la
# misión y comprueba que el cubo acaba en la meta.
#   ./scripts/prueba_mision_sim.sh [mundo] [segundos] [argumentos de launch...]
#   ./scripts/prueba_mision_sim.sh mision 180 cube_detector:=yolo yolo_model:=$HOME/cubo_meta.onnx
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORLD="${1:-mision}"
TIMEOUT="${2:-180}"
shift 2 2>/dev/null || shift $#
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"     # no choca con una simulación abierta (41)
source "$ROOT/scripts/ros_env.sh"

LOG="$(mktemp -t rasptank_sim.XXXXXX.log)"
setsid ros2 launch rasptank_bringup sim.launch.py world:="$WORLD" rviz:=false mission:=true \
  autostart:=true "$@" >"$LOG" 2>&1 &
LAUNCH_PID=$!

stop_sim() {
  kill -INT -- -"$LAUNCH_PID" 2>/dev/null
  for _ in $(seq 20); do
    kill -0 "$LAUNCH_PID" 2>/dev/null || return
    sleep 0.5
  done
  kill -KILL -- -"$LAUNCH_PID" 2>/dev/null
}
trap stop_sim EXIT

python3 "$ROOT/scripts/comprobar_mision.py" "$TIMEOUT"
RESULT=$?
if [[ $RESULT -ne 0 ]]; then
  echo "--- registro de la simulación: $LOG"
  grep -a -E "ERROR|WARN|died|Traceback" "$LOG" | tail -20
fi
exit $RESULT
