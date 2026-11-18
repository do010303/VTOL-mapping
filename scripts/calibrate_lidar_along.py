#!/usr/bin/env python3
"""Along-track angle of each simulated LiDAR ray, from timing runs over the slanted step (scripts/sync_run.sh).

Gazebo's gpu_lidar rays are not exactly in the scan plane: they lean back by up to ~1 mrad (about 0.5 m
along track at 500 m), the same at 50, 100 and 200 lines/s, so it is geometry, not timing. Ray j reaches the
step's slanted edge when the rig (25 m/s, known start) is at the edge's y for that ray; the first line that
shows it bounds the along-track offset of that ray to one line period. The bounds of several runs (different
line rates, so different phases) are intersected, then smoothed over the swath (polynomial in beta).

Adds gamma_rad (ray direction d_L = normalize(gamma, sin beta, -cos beta)) to config/lidar_angles.json.

  python3 scripts/calibrate_lidar_along.py out/sync_slant out/sync_slant_lr50 out/sync_slant_lr100
"""
import glob
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
from static_report import load_scans  # noqa: E402

path = os.path.join(ROOT, "config", "lidar_angles.json")
cal = json.load(open(path))
beta = np.array(cal["beta_rad"])
lo = np.full(beta.size, -np.inf)
hi = np.full(beta.size, np.inf)
for run in sys.argv[1:]:
    cfg = json.load(open(os.path.join(run, "sensors.json")))
    s, lc = cfg["sync"], cfg["lidar"]
    tl, r = load_scans(run)
    z_l = s["height_m"] + lc["t_BL"][2]
    hs = s["slant_step_m"]
    x = (z_l - hs) * np.tan(-beta)                       # beta > 0: left of the nose (north) = west
    y_edge = s["slant_y0_m"] + s["slant_tan"] * x
    zhit = z_l - r * np.cos(beta)[None, :]
    for j in range(beta.size):
        on = np.nonzero(np.abs(zhit[:, j] - hs) < 0.3)[0]      # the top only: left rays also hit the slanted face
        if len(on) == 0 or on[0] == 0:
            continue
        k = on[0]
        # Rig y when the ray first lands on the step lies between its y at the two lines' times.
        y_prev = s["start_y_m"] + s["speed_mps"] * tl[k - 1]
        y_now = s["start_y_m"] + s["speed_mps"] * tl[k]
        # The hit is at rig_y + dy_j, and it lands on the step once rig_y + dy_j >= y_edge.
        lo[j] = max(lo[j], y_edge[j] - y_now)
        hi[j] = min(hi[j], y_edge[j] - y_prev)
ok = np.isfinite(lo) & np.isfinite(hi) & (lo <= hi)
mid = 0.5 * (lo + hi)
z_top = None
cfg = json.load(open(os.path.join(sys.argv[1], "sensors.json")))
z_top = cfg["sync"]["height_m"] + cfg["lidar"]["t_BL"][2] - cfg["sync"]["slant_step_m"]
# Along-track offset dy of the hit at height drop z_top: dy = gamma * range = gamma * z_top / cos(beta).
g_raw = mid * np.cos(beta) / z_top
coef = np.polyfit(beta[ok], g_raw[ok], 6)
gamma = np.polyval(coef, beta)
resid = (np.polyval(coef, beta[ok]) - g_raw[ok]) * z_top / np.cos(beta[ok])
cal["gamma_rad"] = gamma.round(10).tolist()
cal["gamma_source"] = [os.path.relpath(os.path.abspath(r), ROOT) for r in sys.argv[1:]]
cal["gamma_fit_resid_m"] = {"rms": float(np.sqrt(np.mean(resid ** 2))), "max_abs": float(np.abs(resid).max())}
json.dump(cal, open(path, "w"))
print(f"{ok.sum()} rays bounded (mean bound width {np.mean(hi[ok] - lo[ok]) * 100:.1f} cm); gamma "
      f"{gamma.min() * 1e3:+.3f}..{gamma.max() * 1e3:+.3f} mrad (nadir {gamma[beta.size // 2] * 1e3:+.3f}); "
      f"fit residual rms {cal['gamma_fit_resid_m']['rms'] * 100:.1f} cm")
