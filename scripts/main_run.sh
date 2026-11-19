#!/bin/bash
# The spec's main configuration in one go (spec 7): 600 m AGL, 25 m/s, 10 lines 95 m apart, camera every 60 m over
# the forest, VTOL landing at home; then the per-ray LiDAR, the reports, the SfM check, the 3D map, the export.
#
#   scripts/main_run.sh                                   # forest (flat), CW-30E-sized airframe
#   WORLD=forest_slope OUT=out/main_slope scripts/main_run.sh
#   EXPORT=0 IMG=png VEHICLE=standard_vtol scripts/main_run.sh
#
#   WORLD    forest | forest_slope | forest_1400           OUT     result folder (default out/main_<world>_<time>)
#   VEHICLE  cw30e | standard_vtol                          IMG     jpg (default, quality 95) | png
#   LINES, SPACING, ALT (600), EXPORT (1: LAS + COLMAP into <OUT>/export), GUI=1 (watch it)
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$HERE"
WORLD="${WORLD:-forest}"
OUT="$(realpath -m "${OUT:-out/main_${WORLD}_$(date +%Y%m%d_%H%M%S)}")"
LINES="${LINES:-10}"
SPACING="${SPACING:-95}"
ALT="${ALT:-600}"
mkdir -p "$OUT"
PLANNED=$(python3 -c "n, s = $LINES, $SPACING; print(' '.join(f'{-(n - 1) / 2 * s + k * s:g}' for k in range(n)))")

echo "$(date +%T) flight: $WORLD, $LINES lines x $SPACING m at $ALT m -> $OUT"
WORLD="$WORLD" OUT="$OUT" VIEW="${VIEW:-0}" IMG="${IMG:-jpg}" MIN_ALT=$(( ALT - 50 )) TRIGGER_BOX="-700 700 -700 700" \
	./run_sitl.sh > "$OUT/sitl.log" 2>&1 &
SITL=$!
trap 'kill -TERM $SITL 2>/dev/null; wait $SITL 2>/dev/null' EXIT
until grep -q "PX4 SITL ready" "$OUT/sitl.log"; do
	kill -0 $SITL 2>/dev/null || { echo "run_sitl.sh exited"; tail -20 "$OUT/sitl.log"; exit 1; }
	sleep 2
done
timeout 4000 python3 scripts/vtol_mission.py --alt "$ALT" --lines "$LINES" --spacing "$SPACING" > "$OUT/mission.log" 2>&1 \
	|| echo "mission: exit $?"
tail -2 "$OUT/mission.log"
kill -TERM $SITL; wait $SITL 2>/dev/null || true
trap - EXIT
cp "$(ls -t run/rootfs/log/*/*.ulg | head -1)" "$OUT/px4.ulg" 2>/dev/null || true

echo "$(date +%T) per-ray LiDAR"
python3 scripts/lidar_exact.py "$OUT" > "$OUT/lidar_exact.log" 2>&1
echo "$(date +%T) reports"
python3 scripts/flight_report.py "$OUT" --lidar gz --planned-x $PLANNED > /dev/null 2>&1 || echo "flight_report (gz) failed"
python3 scripts/flight_report.py "$OUT" --lidar exact --planned-x $PLANNED > /dev/null 2>&1 || echo "flight_report (exact) failed"
echo "$(date +%T) SfM check"
python3 scripts/sfm_check.py "$OUT" > "$OUT/sfm_check.log" 2>&1 || echo "sfm_check failed"
echo "$(date +%T) 3D map"
python3 scripts/lidar_map.py "$OUT" --every 2 --min-alt $(( ALT - 50 )) > "$OUT/lidar_map.log" 2>&1 || echo "lidar_map failed"
if [ "${EXPORT:-1}" != 0 ]; then
	echo "$(date +%T) export"
	python3 scripts/export_dataset.py "$OUT" --planned-x $PLANNED > "$OUT/export.log" 2>&1 || echo "export failed"
fi
echo "$(date +%T) done: $OUT"
grep -E "Survey|Tracking|Returns vs|DSM 1 m|Ground reached|Images" "$OUT/flight_report_exact.md" 2>/dev/null || true
