#!/bin/bash
# PX4 SITL: standard_vtol with the mapping payload (models/standard_vtol_mapping, airframe 4004)
# in the forest world, plus the LiDAR and camera nodes.
#
#   ./run_sitl.sh                      headless; then fly: scripts/vtol_mission.py
#   GUI=1 ./run_sitl.sh                with the Gazebo GUI
#   NODES=0 ./run_sitl.sh              without the sensor nodes
#   VIEW=0 ./run_sitl.sh               without the camera-view video (out/<run>/camera_view.mp4); with GUI=1 the
#                                      camera view is also a panel in the Gazebo window
#   OUT=<dir> ./run_sitl.sh            where the nodes record (default out/flight_<date>)
#   SPAWN="0,-1900,0.3,0,0,1.5708"     vehicle spawn pose x,y,z,roll,pitch,yaw (ENU; yaw 90 deg: nose north)
#   AIRSPEED=17                        FW_AIRSPD_TRIM, m/s (above 18: FW_AIRSPD_MAX = AIRSPEED + 5)
#   THR_MAX=0.6                        FW_THR_MAX (the airframe's default 0.6)
#   MIN_ALT=450                        the nodes record (LiDAR) and trigger (camera) above this altitude
#   TRIGGER_BOX="-700 700 -700 700"    the camera fires only over this area (world x, y)
#
# PX4 runs as a daemon with its log in run/px4.log. Gazebo runs in partition vtol_mapping under the process
# title vtol_mapping_gazebo; only what this script starts is stopped on exit (Ctrl+C or scripts/stop_sim.sh).
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
. "$HERE/scripts/sim_up.sh"

[ -x "$BUILD/bin/px4" ] || { echo "PX4 SITL is not built ($BUILD)" >&2; exit 1; }
OUT="${OUT:-$HERE/out/flight_$(date +%Y%m%d_%H%M%S)}"
MIN_ALT="${MIN_ALT:-450}"
# TRIGGER_BOX="xmin xmax ymin ymax": the camera fires only inside it (default: everywhere above MIN_ALT)
TRIGGER_BOX_ARG=
[ -n "$TRIGGER_BOX" ] && TRIGGER_BOX_ARG="--trigger-box $TRIGGER_BOX"

sim_start forest

# Own rootfs: parameters start from firmware defaults on every start.
ROOTFS="$RUN_DIR/rootfs"
mkdir -p "$ROOTFS"
rm -f "$ROOTFS/parameters.bson" "$ROOTFS/parameters_backup.bson"
cp "$BUILD/rootfs/gz_env.sh" "$ROOTFS/gz_env.sh"
cat >> "$ROOTFS/gz_env.sh" <<EOT
export PX4_GZ_MODELS="$HERE/models"
export PX4_GZ_WORLDS="$HERE/worlds"
export GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH"
EOT

export PX4_SYS_AUTOSTART=4004
export PX4_SIM_MODEL=gz_standard_vtol_mapping
export PX4_GZ_WORLD=forest
export PX4_GZ_MODEL_POSE="${SPAWN:-0,-1900,0.3,0,0,1.5708}"
[ -z "$GUI" ] && export PX4_GZ_NO_FOLLOW=1
# No RC and no ground station: no fail-safe on their absence.
export PX4_PARAM_NAV_DLL_ACT=0 PX4_PARAM_NAV_RCL_ACT=0 PX4_PARAM_COM_RC_IN_MODE=4
export PX4_PARAM_FW_AIRSPD_TRIM="${AIRSPEED:-17}"
# PX4's FW_AIRSPD_MAX is 20 m/s: raise it above the trim for faster surveys (the spec's 25 m/s), and let
# THR_MAX=<0..1> lift the airframe's 0.6 throttle limit if the speed is not reached.
if [ "${AIRSPEED:-17}" -gt 18 ] 2>/dev/null; then
	export PX4_PARAM_FW_AIRSPD_MAX=$(( ${AIRSPEED} + 5 ))
fi
[ -n "$THR_MAX" ] && export PX4_PARAM_FW_THR_MAX="$THR_MAX"
# The survey line starts 1 km from home.
export PX4_PARAM_MIS_DIST_1WP=0

cd "$ROOTFS"
"$BUILD/bin/px4" -d "$BUILD/etc" -w "$ROOTFS" > "$RUN_DIR/px4.log" 2>&1 &
PX4_PID=$!
track $PX4_PID
until grep -q "Startup script returned successfully" "$RUN_DIR/px4.log"; do
	kill -0 $PX4_PID 2>/dev/null || { tail -30 "$RUN_DIR/px4.log"; exit 1; }
	sleep 1
done

if [ "${NODES:-1}" != 0 ]; then
	mkdir -p "$OUT"
	cp "$HERE/config/sensors_flight.json" "$OUT/sensors.json"
	[ -f "$HERE/config/lidar_angles.json" ] && cp "$HERE/config/lidar_angles.json" "$OUT/"
	$NODE_PY "$HERE/scripts/lidar_node.py" --world forest --model standard_vtol_mapping_0 --out "$OUT" \
		--config "$HERE/config/sensors_flight.json" --min-alt "$MIN_ALT" > "$OUT/lidar_node.log" 2>&1 &
	track $!
	$NODE_PY "$HERE/scripts/camera_node.py" --world forest --model standard_vtol_mapping_0 --out "$OUT" \
		--config "$HERE/config/sensors_flight.json" --trigger-every 60 --min-alt "$MIN_ALT" $TRIGGER_BOX_ARG > "$OUT/camera_node.log" 2>&1 &
	track $!
	# The camera view (preview camera) as a video: VIEW=0 to leave it out.
	if [ "${VIEW:-1}" != 0 ]; then
		$NODE_PY "$HERE/scripts/camera_view.py" --world forest --model standard_vtol_mapping_0 \
			--config "$HERE/config/sensors_flight.json" --out "$OUT/camera_view.mp4" > "$OUT/camera_view.log" 2>&1 &
		track $!
	fi
	echo "$OUT" > "$RUN_DIR/last_flight"
fi

echo "PX4 SITL ready (world forest, model standard_vtol_mapping, log $RUN_DIR/px4.log${NODES:+}; nodes -> $OUT)"
while kill -0 $PX4_PID 2>/dev/null && kill -0 $GZ_PID 2>/dev/null; do sleep 2; done
echo "PX4 or Gazebo exited"
