#!/usr/bin/env bash
# Aplicación del RaspTank: estado, parada de emergencia, mover, calibración y simulación.
#   ./scripts/app.sh                 # dominio 41 (simulación)
#   ./scripts/app.sh --dominio 10    # robot real con ROS_DOMAIN_ID=10
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT/scripts/ros_env.sh"
cd "$ROOT/tools/app" && exec python3 -m rasptank_app "$@"
