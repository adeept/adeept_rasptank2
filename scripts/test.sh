#!/usr/bin/env bash
# Todos los tests: unitarios de cada paquete; con --e2e, la misión simulada;
# con --calibracion, la calibración manual y automática simuladas (~10 min).
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT/scripts/ros_env.sh"
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1     # evita plugins de launch_testing que no hacen falta
FAILED=0
for pkg in rasptank_core rasptank_description rasptank_sim rasptank_perception rasptank_hw rasptank_calibration; do
  echo "== $pkg"
  (cd "$ROOT/ros2_ws/src/$pkg" && python3 -m pytest -q test) || FAILED=1
done
echo "== app"
(cd "$ROOT/tools/app" && QT_QPA_PLATFORM=offscreen python3 -m pytest -q tests) || FAILED=1
if [[ " $* " == *" --e2e "* ]]; then
  echo "== misión simulada"
  "$ROOT/scripts/prueba_mision_sim.sh" mision 180 || FAILED=1
fi
if [[ " $* " == *" --calibracion "* ]]; then
  for mode in manual auto; do
    echo "== calibración $mode simulada"
    "$ROOT/scripts/prueba_calibracion_sim.sh" "$mode" || FAILED=1
  done
fi
exit $FAILED
