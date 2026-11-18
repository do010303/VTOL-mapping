#!/bin/bash
# Timing check (spec 9.3): the rig at a constant 25 m/s, 500 m up, flies over a 20 m step whose edge is y = 0.
# Its true position is start + 25 t, so the lag of the pose stream, of the LiDAR lines and of the images against
# their own timestamps can each be measured (scripts/sync_check.py).
#
#   scripts/sync_run.sh            # OUT=<dir> (default out/sync_<date>), DURATION=<sim s> (default 22)
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
. "$HERE/scripts/sim_up.sh"
OUT="${OUT:-$HERE/out/sync_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$OUT"
cp "$HERE/config/sensors_sync.json" "$OUT/sensors.json"
# Nodes first, so they see the run from its start: they wait for the topics.
$NODE_PY "$HERE/scripts/lidar_node.py" --world sync_test --model vtol_rig_moving --out "$OUT" \
	--config "$HERE/config/sensors_sync.json" --duration "${DURATION:-22}" > "$OUT/lidar_node.log" 2>&1 &
LP=$!; track $LP
$NODE_PY "$HERE/scripts/camera_node.py" --world sync_test --model vtol_rig_moving --out "$OUT" \
	--config "$HERE/config/sensors_sync.json" --duration "${DURATION:-22}" > "$OUT/camera_node.log" 2>&1 &
CP=$!; track $CP
sim_start sync_test
wait $LP; wait $CP
python3 "$HERE/scripts/sync_check.py" "$OUT"
