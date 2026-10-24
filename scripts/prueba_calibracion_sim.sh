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
  ARGS=(--auto)
else
  ARGS=(--medidas-sim)
fi
OUT="$(timeout 600 ros2 run rasptank_calibration calibrar "${ARGS[@]}" --si --no-guardar --detalle 2>&1)"
echo "$OUT" | grep -E "^  |Antes|Después|Error|interrump"
if [[ "$MODE" == "auto" ]]; then
  # error de distancia (cm) y de giro (grados) de ida + vuelta completa + regreso
  LINE="$(echo "$OUT" | grep -E "error de distancia" | tail -1)"
  DIST="$(echo "$LINE" | sed -E 's/.*distancia ([-+0-9.]+) cm.*/\1/')"
  TURN="$(echo "$LINE" | sed -E 's/.*giro ([-+0-9.]+)°.*/\1/')"
  OK=$(awk -v d="$DIST" -v t="$TURN" 'BEGIN { print (d != "" && d*d < 16 && t*t < 25) ? 1 : 0 }')
  SUMMARY="distancia ${DIST:-?} cm (límite 4), giro ${TURN:-?}° (límite 5)"
else
  ERROR="$(echo "$OUT" | grep -oE "termina a [0-9.]+ cm" | grep -oE "[0-9.]+" | tail -1)"
  OK=$(awk -v e="$ERROR" 'BEGIN { print (e != "" && e < 15) ? 1 : 0 }')
  SUMMARY="cuadrado de 50 cm cierra a ${ERROR:-?} cm (límite 15)"
fi
if [[ "$OK" == 1 ]]; then
  echo "RESULTADO: calibración $MODE correcta: $SUMMARY"
  exit 0
fi
echo "RESULTADO: calibración $MODE FALLIDA: $SUMMARY"
echo "--- registro de la simulación: $LOG"
exit 1
