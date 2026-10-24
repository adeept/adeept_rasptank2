#!/usr/bin/env bash
# Abre la simulación con RViz.  ./scripts/sim.sh [mision|linea|vacio] [argumentos de launch...]
#   ./scripts/sim.sh                     mundo de la misión, sin misión automática
#   ./scripts/sim.sh mision mission:=true autostart:=true
#   ./scripts/sim.sh vacio route:=true
#   ./scripts/sim.sh linea
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT/scripts/ros_env.sh"
WORLD="${1:-mision}"
shift || true
exec ros2 launch rasptank_bringup sim.launch.py world:="$WORLD" "$@"
