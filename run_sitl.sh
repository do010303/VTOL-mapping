#!/bin/bash
# PX4 SITL: the CW-30E-sized quadplane with the mapping payload (models/cw30e_mapping, PX4 airframe 4004 +
# config/cw30e_px4_params.json) in the forest world, plus the LiDAR and camera nodes.
#
#   ./run_sitl.sh                      headless; then fly: scripts/vtol_mission.py
#   WORLD=forest_slope ./run_sitl.sh   the sloping forest (default forest: flat, 2 x 2 km; forest_1400: phase 1/2)
#   VEHICLE=standard_vtol ./run_sitl.sh   PX4's standard_vtol (5 kg, 2.1 m span) instead, as in R1-R6
#   GUI=1 ./run_sitl.sh                with the Gazebo GUI
#   NODES=0 ./run_sitl.sh              without the sensor nodes
#   VIEW=0 ./run_sitl.sh               without the camera-view video (out/<run>/camera_view.mp4); with GUI=1 the
#                                      camera view is also a panel in the Gazebo window
#   OUT=<dir> ./run_sitl.sh            where the nodes record (default out/flight_<date>)
#   SPAWN="0,-1900,0.6,0,0,1.5708"     vehicle spawn pose x,y,z,roll,pitch,yaw (ENU; yaw 90 deg: nose north)
#   AIRSPEED=25                        FW_AIRSPD_TRIM, m/s (cw30e default 25, standard_vtol 17; above 18 the
#                                      standard_vtol gets FW_AIRSPD_MAX = AIRSPEED + 5)
#   THR_MAX=0.6                        FW_THR_MAX (standard_vtol's default 0.6, cw30e 0.85)
#   MIN_ALT=450                        the nodes record (LiDAR) and trigger (camera) above this altitude
#   TRIGGER_BOX="-700 700 -700 700"    the camera fires only over this area (world x, y)
#   IMG=jpg                            images as JPEG (IMG_QUALITY, default 85: ~3.4 MB) instead of PNG (~16 MB)
#   IMG_SCALE=2                        write images 2x smaller than rendered (test runs: 2376 x 1584, ~1 MB);
#                                      out/<run>/sensors.json then holds the scaled K
#   ON_LINE=0                          trigger the camera on turns and run-ins too (default: only on the survey
#                                      lines: course N/S within 10 deg, roll < 8 deg)
#   TRIGGER_LINES="-95 0 95"           with ON_LINE: x of the planned lines; photos only within 20 m of one (else
#                                      the straight leg back to the landing counts as a line; main_run sets it)
#
# PX4 runs as a daemon with its log in run/px4.log. Gazebo runs in partition vtol_mapping under the process
# title vtol_mapping_gazebo; only what this script starts is stopped on exit (Ctrl+C or scripts/stop_sim.sh).
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
. "$HERE/scripts/sim_up.sh"

[ -x "$BUILD/bin/px4" ] || { echo "PX4 SITL is not built ($BUILD)" >&2; exit 1; }
OUT="$(realpath -m "${OUT:-$HERE/out/flight_$(date +%Y%m%d_%H%M%S)}")"      # absolute: PX4 runs in run/rootfs
MIN_ALT="${MIN_ALT:-450}"
# TRIGGER_BOX="xmin xmax ymin ymax": the camera fires only inside it (default: everywhere above MIN_ALT)
TRIGGER_BOX_ARG=
[ -n "$TRIGGER_BOX" ] && TRIGGER_BOX_ARG="--trigger-box $TRIGGER_BOX"

WORLD="${WORLD:-forest}"
VEHICLE="${VEHICLE:-cw30e}"
case "$VEHICLE" in
	cw30e) MODEL=cw30e_mapping; SPAWN_Z=0.6 ;;
	standard_vtol) MODEL=standard_vtol_mapping; SPAWN_Z=0.3 ;;
	*) echo "VEHICLE: cw30e or standard_vtol" >&2; exit 1 ;;
esac
case "$WORLD" in
	forest_slope*) SCENE=forest_slope ;;
	forest_1400*) SCENE=forest_1400 ;;
	*) SCENE=forest ;;
esac

sim_start "$WORLD"
echo "$WORLD ${MODEL}_0" > "$RUN_DIR/last_vehicle"           # for scripts/camera_window.sh

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
export PX4_SIM_MODEL=gz_$MODEL
export PX4_GZ_WORLD="$WORLD"
export PX4_GZ_MODEL_POSE="${SPAWN:-0,-1900,$SPAWN_Z,0,0,1.5708}"
[ -z "$GUI" ] && export PX4_GZ_NO_FOLLOW=1
# No RC and no ground station: no fail-safe on their absence.
export PX4_PARAM_NAV_DLL_ACT=0 PX4_PARAM_NAV_RCL_ACT=0 PX4_PARAM_COM_RC_IN_MODE=4
if [ "$VEHICLE" = cw30e ]; then
	# The CW-30E-sized airframe: geometry, hover thrust, rate gains, 25 m/s (scripts/make_models.py).
	eval "$(python3 -c '
import json, sys
for k, v in json.load(open(sys.argv[1])).items():
    print(f"export PX4_PARAM_{k}={v}")' "$HERE/config/cw30e_px4_params.json")"
	[ -n "$AIRSPEED" ] && export PX4_PARAM_FW_AIRSPD_TRIM="$AIRSPEED"
else
	export PX4_PARAM_FW_AIRSPD_TRIM="${AIRSPEED:-17}"
	# PX4's FW_AIRSPD_MAX is 20 m/s: raise it above the trim for faster surveys (the spec's 25 m/s), and let
	# THR_MAX=<0..1> lift the airframe's 0.6 throttle limit if the speed is not reached.
	if [ "${AIRSPEED:-17}" -gt 18 ] 2>/dev/null; then
		export PX4_PARAM_FW_AIRSPD_MAX=$(( ${AIRSPEED} + 5 ))
	fi
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
	# The run's sensors + which scene (ground truth) and vehicle it used.
	python3 -c '
import json, sys
c = json.load(open(sys.argv[1]))
c.update(scene=sys.argv[3], world=sys.argv[4], vehicle=sys.argv[5])
s, cam = int(sys.argv[6]), c["camera"]
if s > 1:                     # images written s times smaller (camera_node.py --scale): the K of those files
    cam.update(width=cam["width"] // s, height=cam["height"] // s, fx=cam["fx"] / s, fy=cam["fy"] / s,
               cx=(cam["cx"] + 0.5) / s - 0.5, cy=(cam["cy"] + 0.5) / s - 0.5, written_scale=s)
cam.update(jpeg_quality=int(sys.argv[7]), trigger_on_line=sys.argv[8] != "0")
json.dump(c, open(sys.argv[2], "w"), indent=2)' "$HERE/config/sensors_flight.json" "$OUT/sensors.json" "$SCENE" "$WORLD" "$MODEL" \
		"${IMG_SCALE:-1}" "${IMG_QUALITY:-85}" "${ON_LINE:-1}"
	[ -f "$HERE/config/lidar_angles.json" ] && cp "$HERE/config/lidar_angles.json" "$OUT/"
	$NODE_PY "$HERE/scripts/lidar_node.py" --world "$WORLD" --model ${MODEL}_0 --out "$OUT" \
		--config "$HERE/config/sensors_flight.json" --min-alt "$MIN_ALT" > "$OUT/lidar_node.log" 2>&1 &
	track $!
	$NODE_PY "$HERE/scripts/camera_node.py" --world "$WORLD" --model ${MODEL}_0 --out "$OUT" \
		--config "$HERE/config/sensors_flight.json" --trigger-every 60 --min-alt "$MIN_ALT" $TRIGGER_BOX_ARG --format "${IMG:-png}" \
		--quality "${IMG_QUALITY:-85}" --scale "${IMG_SCALE:-1}" $([ "${ON_LINE:-1}" != 0 ] && echo --on-line) \
		$([ -n "$TRIGGER_LINES" ] && echo --line-x $TRIGGER_LINES) > "$OUT/camera_node.log" 2>&1 &
	track $!
	# The camera view (preview camera) as a video: VIEW=0 to leave it out.
	if [ "${VIEW:-1}" != 0 ]; then
		$NODE_PY "$HERE/scripts/camera_view.py" --world "$WORLD" --model ${MODEL}_0 \
			--config "$HERE/config/sensors_flight.json" --out "$OUT/camera_view.mp4" > "$OUT/camera_view.log" 2>&1 &
		track $!
	fi
	echo "$OUT" > "$RUN_DIR/last_flight"
fi

echo "PX4 SITL ready (world $WORLD, model $MODEL, log $RUN_DIR/px4.log${NODES:+}; nodes -> $OUT)"
while kill -0 $PX4_PID 2>/dev/null && kill -0 $GZ_PID 2>/dev/null; do sleep 2; done
echo "PX4 or Gazebo exited"
