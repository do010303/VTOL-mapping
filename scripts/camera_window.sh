#!/bin/bash
# The nadir camera's view, live, in a window of its own, next to a simulation already running
# (./run_sitl.sh or scripts/static_run.sh). q or Esc closes it; the simulation goes on.
#   scripts/camera_window.sh                 # the flight run_sitl.sh started last (run/last_vehicle: world, model)
#   scripts/camera_window.sh forest cw30e_mapping_0
#   scripts/camera_window.sh forest_static vtol_rig
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
. "$HERE/env.sh"
read -r LAST_WORLD LAST_MODEL < "$HERE/run/last_vehicle" 2>/dev/null
WORLD="${1:-${LAST_WORLD:-forest}}"
MODEL="${2:-${LAST_MODEL:-cw30e_mapping_0}}"
CONFIG="$HERE/config/sensors_flight.json"
case "$WORLD" in *_static) CONFIG="$HERE/config/sensors.json" ;; esac
# System python (protobuf 3 with the C++ backend, OpenCV with GTK windows).
exec env PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=cpp python3 -s "$HERE/scripts/camera_view.py" --window \
	--world "$WORLD" --model "$MODEL" --config "$CONFIG"
