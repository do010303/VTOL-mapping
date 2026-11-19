#!/bin/bash
# Tie-point check of a scene's texture with the static rig (spec 6, 9.5): one image at each of 6 positions,
# x in {0, 95} (the line spacing) and y in {-60, 0, 60} (the 60 m trigger spacing), at the survey height, then
# scripts/sfm_check.py on the pairs (forward: 60 m apart on one line; side: 95 m apart across lines).
#
#   scripts/texture_pairs.sh                         # forest_static, 600 m
#   WORLD=forest_1400_static scripts/texture_pairs.sh
#   H=600 X0=200 Y0=-300 WORLD=forest_slope_static scripts/texture_pairs.sh
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
. "$HERE/scripts/sim_up.sh"

WORLD="${WORLD:-forest_static}"
H="${H:-600}"
X0="${X0:-0}"
Y0="${Y0:-0}"
OUT="${OUT:-$HERE/out/texture_${WORLD}_$(date +%Y%m%d_%H%M%S)}"
case "$WORLD" in
	forest_slope*) SCENE=forest_slope ;;
	forest_1400*) SCENE=forest_1400 ;;
	*) SCENE=forest ;;
esac
mkdir -p "$OUT/camera"
python3 -c '
import json, sys
c = json.load(open(sys.argv[1]))
c.update(scene=sys.argv[3], world=sys.argv[4])
json.dump(c, open(sys.argv[2], "w"), indent=2)' "$HERE/config/sensors.json" "$OUT/sensors.json" "$SCENE" "$WORLD"

sim_start "$WORLD"
sleep 3                                   # let the server run (and render) before the first image
k=0
for dx in 0 95; do
	for dy in -60 0 60; do
		x=$(( X0 + dx )); y=$(( Y0 + dy ))
		gz service -s "/world/$WORLD/set_pose" --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 5000 \
			--req "name: 'vtol_rig', position: {x: $x, y: $y, z: $H}, orientation: {x: 0, y: 0, z: 0.7071068, w: 0.7071068}" > /dev/null
		sleep 1
		d="$OUT/p$k"
		mkdir -p "$d"
		$NODE_PY "$HERE/scripts/camera_node.py" --world "$WORLD" --model vtol_rig --out "$d" --max-frames 3 --duration 6 \
			> "$d/camera_node.log" 2>&1 &
		CP=$!; track $CP
		wait $CP || echo "camera_node exit $?"
		echo "$(date +%T) image $k at ($x, $y, $H)"
		k=$((k + 1))
	done
done

# One images.csv for sfm_check.py, ordered along x then y.
python3 - "$OUT" <<'EOF'
import csv, glob, os, shutil, sys
out = sys.argv[1]
rows = []
for k, d in enumerate(sorted(glob.glob(os.path.join(out, "p*")), key=lambda p: int(os.path.basename(p)[1:]))):
    # The last frame with a pose: the first can still show the previous position (or come before any pose).
    r = [x for x in csv.DictReader(open(os.path.join(d, "camera", "images.csv"))) if x["x_m"] not in ("nan", "")][-1]
    src = os.path.join(d, "camera", "img_%05d.png" % int(r["image_id"]))
    r["image_id"] = str(k)
    if "file" in r:
        r["file"] = "img_%05d.png" % k
    shutil.copy(src, os.path.join(out, "camera", "img_%05d.png" % k))
    rows.append(r)
with open(os.path.join(out, "camera", "images.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
    w.writeheader()
    w.writerows(rows)
EOF
python3 "$HERE/scripts/sfm_check.py" "$OUT" --y-range -10000 10000
echo "results: $OUT"
