#!/usr/bin/env bash
# Todos los tests: unitarios de cada paquete y, con --e2e, la misión simulada.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT/scripts/ros_env.sh"
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1     # evita plugins de launch_testing que no hacen falta
FAILED=0
for pkg in rasptank_core rasptank_description rasptank_sim rasptank_perception rasptank_hw; do
  echo "== $pkg"
  (cd "$ROOT/ros2_ws/src/$pkg" && python3 -m pytest -q test) || FAILED=1
done
if [[ "${1:-}" == "--e2e" ]]; then
  echo "== misión simulada"
  "$ROOT/scripts/prueba_mision_sim.sh" mision 180 || FAILED=1
fi
exit $FAILED
