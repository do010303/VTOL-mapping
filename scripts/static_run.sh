#!/bin/bash
# Static test (spec 9.1-9.5 with a fixed vehicle): the VTOL rig held at a height over the forest,
# camera and LiDAR nodes recording at the same time, then the report.
#
#   scripts/static_run.sh                       # forest, 500 m, 10 s of sim time
#   HEIGHTS="500 600 880 1000 1200 1300" scripts/static_run.sh     # range threshold sweep, one server
#   WORLD=flat_static scripts/static_run.sh     # bare ground: range accuracy against H / cos(beta)
#   WORLD=forest_slope_static scripts/static_run.sh   # the sloping forest (H is above z = 0, not above the terrain)
#   GUI=1 scripts/static_run.sh                 # with the Gazebo GUI (and its camera view panel)
#   VIEW=0 ...                                  # no camera-view video (h<H>/camera_view.mp4)
#
#   DURATION=<sim s> per height (default 10), OUT=<dir> (default out/<world>_<date>)
#
# The rig is moved between heights with the world's set_pose service, so the forest loads once.
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
. "$HERE/scripts/sim_up.sh"

WORLD="${WORLD:-forest_static}"
HEIGHTS="${HEIGHTS:-500}"
DURATION="${DURATION:-10}"
OUT="${OUT:-$HERE/out/${WORLD}_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$OUT"
case "$WORLD" in
	forest_slope*) SCENE=forest_slope ;;
	forest_1400*) SCENE=forest_1400 ;;
	*) SCENE=forest ;;
esac
python3 -c '
import json, sys
c = json.load(open(sys.argv[1]))
c.update(scene=sys.argv[3], world=sys.argv[4])
json.dump(c, open(sys.argv[2], "w"), indent=2)' "$HERE/config/sensors.json" "$OUT/sensors.json" "$SCENE" "$WORLD"

sim_start "$WORLD"

for H in $HEIGHTS; do
	dir="$OUT/h$H"
	mkdir -p "$dir"
	# Body at H; yaw 90 deg: nose north, the scan line and the image's long side east-west.
	gz service -s "/world/$WORLD/set_pose" --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 5000 \
		--req "name: 'vtol_rig', position: {x: 0, y: 0, z: $H}, orientation: {x: 0, y: 0, z: 0.7071068, w: 0.7071068}" > /dev/null
	sleep 1
	echo "$(date +%T) height $H m: recording $DURATION s of sim time -> $dir"
	$NODE_PY "$HERE/scripts/lidar_node.py" --world "$WORLD" --model vtol_rig --out "$dir" --duration "$DURATION" \
		> "$dir/lidar_node.log" 2>&1 &
	LP=$!; track $LP
	$NODE_PY "$HERE/scripts/camera_node.py" --world "$WORLD" --model vtol_rig --out "$dir" --duration "$DURATION" \
		> "$dir/camera_node.log" 2>&1 &
	CP=$!; track $CP
	VP=
	if [ "${VIEW:-1}" != 0 ]; then
		$NODE_PY "$HERE/scripts/camera_view.py" --world "$WORLD" --model vtol_rig --out "$dir/camera_view.mp4" \
			--duration "$DURATION" > "$dir/camera_view.log" 2>&1 &
		VP=$!; track $VP
	fi
	wait $LP || echo "lidar_node exit $?"
	wait $CP || echo "camera_node exit $?"
	[ -n "$VP" ] && { wait $VP || true; }
	echo "{\"height_m\": $H, \"world\": \"$WORLD\"}" > "$dir/run.json"
	tail -1 "$dir/lidar_node.log" | cut -c1-400
	tail -1 "$dir/camera_node.log" | cut -c1-300
	ps -o rss= -p $GZ_PID | awk '{printf "gz server RSS %.0f MB\n", $1/1024}'
done

python3 "$HERE/scripts/static_report.py" "$OUT" || true
echo "results: $OUT"
