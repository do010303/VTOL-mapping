#!/bin/bash
# The nadir camera's view, live, in a window of its own, next to a simulation already running
# (./run_sitl.sh or scripts/static_run.sh). q or Esc closes it; the simulation goes on.
#   scripts/camera_window.sh                 # flight (model standard_vtol_mapping_0, world forest)
#   scripts/camera_window.sh forest_static vtol_rig
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
. "$HERE/env.sh"
WORLD="${1:-forest}"
MODEL="${2:-standard_vtol_mapping_0}"
CONFIG="$HERE/config/sensors_flight.json"
[ "$WORLD" = forest ] || CONFIG="$HERE/config/sensors.json"
# System python (protobuf 3 with the C++ backend, OpenCV with GTK windows).
exec env PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=cpp python3 -s "$HERE/scripts/camera_view.py" --window \
	--world "$WORLD" --model "$MODEL" --config "$CONFIG"
