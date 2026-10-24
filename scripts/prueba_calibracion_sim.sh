#!/usr/bin/env bash
# Prueba de extremo a extremo de la calibración, sin pantalla, en el mundo
# "calibracion" (robot con errores escondidos):
#   ./scripts/prueba_calibracion_sim.sh manual   # cinta métrica virtual
#   ./scripts/prueba_calibracion_sim.sh auto     # marcador ArUco
# Pasa si, después de calibrar, la prueba final cierra con poco error.
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODE="${1:-manual}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-78}"
source "$ROOT/scripts/ros_env.sh"

LOG="$(mktemp -t rasptank_calib.XXXXXX.log)"
setsid ros2 launch rasptank_bringup sim.launch.py world:=calibracion rviz:=false \
  calibration_file:=none >"$LOG" 2>&1 &
LAUNCH_PID=$!
stop_sim() {
  kill -INT -- -"$LAUNCH_PID" 2>/dev/null
  for _ in $(seq 20); do kill -0 "$LAUNCH_PID" 2>/dev/null || return; sleep 0.5; done
  kill -KILL -- -"$LAUNCH_PID" 2>/dev/null
}
trap stop_sim EXIT
sleep 6

if [[ "$MODE" == "auto" ]]; then
  ARGS=(--auto); LIMIT=8
else
  ARGS=(--medidas-sim); LIMIT=15
fi
OUT="$(timeout 600 ros2 run rasptank_calibration calibrar "${ARGS[@]}" --si --no-guardar 2>&1)"
echo "$OUT" | grep -E "^  |Antes|Después|Error|interrump"
ERROR="$(echo "$OUT" | grep -oE "(termina a|está a) [0-9.]+ cm" | grep -oE "[0-9.]+" | tail -1)"
if [[ -n "$ERROR" ]] && awk -v e="$ERROR" -v l="$LIMIT" 'BEGIN { exit !(e < l) }'; then
  echo "RESULTADO: calibración $MODE correcta (error final $ERROR cm < $LIMIT cm)"
  exit 0
fi
echo "RESULTADO: calibración $MODE FALLIDA (error final ${ERROR:-desconocido} cm, límite $LIMIT cm)"
echo "--- registro de la simulación: $LOG"
exit 1
