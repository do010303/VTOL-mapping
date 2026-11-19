#!/usr/bin/env python3
"""Timing of the pose stream, the LiDAR lines and the images against the true motion (scripts/sync_run.sh).

The rig moves north at a constant speed from a known start, so its true position at sim time t is
y(t) = start_y + speed x t. A 20 m step has its edge on y = 0.

  pose stream   the y of each pose message against y(its stamp)
  LiDAR         the line on which the nadir ray first lands on the step: the lidar is then over y = 0
  camera        the row of the step's edge in each image taken after passing it: where the camera was

A positive lag means the data shows the scene as it was that long BEFORE the message's timestamp.
Writes <run>/sync.json.

  python3 scripts/sync_check.py out/sync_<date>
"""

import json
import os
import sys

import cv2
import numpy as np

from static_report import load_scans

run = sys.argv[1]
cfg = json.load(open(os.path.join(run, "sensors.json")))
s, lc, cc = cfg["sync"], cfg["lidar"], cfg["camera"]
v, y0, edge, step = s["speed_mps"], s["start_y_m"], s["edge_y_m"], s["step_m"]
out = {}

traj = np.loadtxt(os.path.join(run, "trajectory.csv"), delimiter=",", skiprows=1)
t, y = traj[:, 0], traj[:, 2]
fit_v, fit_y0 = np.polyfit(t, y, 1)
lag = (y0 + v * t - y) / v
out["pose_stream"] = {"messages": int(len(t)), "rate_hz": float(1 / np.median(np.diff(t))), "speed_fit_mps": float(fit_v),
                      "lag_ms": {"mean": float(lag.mean() * 1e3), "min": float(lag.min() * 1e3), "max": float(lag.max() * 1e3)},
                      "x_drift_m": float(np.abs(traj[:, 1]).max()), "z_drift_m": float(np.ptp(traj[:, 3]))}

# LiDAR: nadir ray, ground -> top of the step.
tl, r = load_scans(run)
nadir = r[:, r.shape[1] // 2]
z_l = s["height_m"] + lc["t_BL"][2]
k = int(np.argmax(nadir < z_l - step / 2))
t_true = (edge - y0) / v
out["lidar"] = {"edge_between_stamps_s": [float(tl[k - 1]), float(tl[k])], "true_crossing_s": float(t_true),
                "lag_ms": [float((tl[k - 1] - t_true) * 1e3), float((tl[k] - t_true) * 1e3)],
                "nadir_range_before_after_m": [float(nadir[k - 1]), float(nadir[k])]}
# Whole line: the across-track rays cross the edge at the same time (edge parallel to the scan line).
first = np.array([np.argmax(r[:, j] < (z_l - step / 2) / np.cos(np.radians(lc["scan_min_deg"] + j * lc["angle_step_deg"])))
                  for j in range(0, r.shape[1], 50)])
out["lidar"]["edge_line_index_spread"] = int(first.max() - first.min())

# LiDAR, slanted edge: ray j (ground offset x_j across track) reaches the 10 m step when the rig is at
# y = slant_y0 + tan * x_j, i.e. at t_j = (that - start_y) / speed. The first line showing it, with stamp t_k,
# says lag in [t_(k-1) - t_j, t_k - t_j]; the rays' times are 1.1 ms apart, so together they pin the lag.
beta = np.radians(lc["scan_min_deg"] + np.arange(r.shape[1]) * lc["angle_step_deg"])
cal = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config", "lidar_angles.json")
if os.path.exists(cal):
    beta = np.array(json.load(open(cal))["beta_rad"])
hs = s["slant_step_m"]
x_top = (z_l - hs) * np.tan(-beta)       # ray to the left (beta > 0) lands west (x < 0): the rig's nose is north
y_e = s["slant_y0_m"] + s["slant_tan"] * x_top
t_j = (y_e - y0) / v
gamma = np.zeros_like(beta)
if os.path.exists(cal) and "gamma_rad" in json.load(open(cal)):
    gamma = np.array(json.load(open(cal))["gamma_rad"])
# The rays lean back by gamma (scripts/calibrate_lidar_along.py): that alone puts the hit gamma * range behind the
# lidar, which looks like a lag of gamma * range / speed. It is subtracted, so what remains is timing.
geom = -gamma * (z_l - hs) / np.cos(beta) / v
zhit = z_l - r * np.cos(beta)[None, :]
lo, hi = [], []
for j in range(r.shape[1]):
    on = np.nonzero(np.abs(zhit[:, j] - hs) < 0.3)[0]      # the top only: left rays also hit the slanted face
    if len(on) == 0 or on[0] == 0:
        continue
    k = on[0]
    lo.append(tl[k - 1] - t_j[j] - geom[j])
    hi.append(tl[k] - t_j[j] - geom[j])
lo, hi = np.array(lo), np.array(hi)
out["lidar_slant"] = {"rays": int(len(lo)), "geometric_lean_ms_removed": [float(geom.min() * 1e3), float(geom.max() * 1e3)], "lag_ms_interval": [float(lo.max() * 1e3), float(hi.min() * 1e3)],
                      "consistent": bool(lo.max() <= hi.min() + 1.5e-3)}   # 1.5 ms: the smoothed lean table is good to ~2 cm

# Camera: the edge's row in the images taken north of the edge.
im = np.genfromtxt(os.path.join(run, "camera", "images.csv"), delimiter=",", names=True, dtype=None, encoding=None)
rows = []
for i in range(len(im)):
    ti = float(im["t_s"][i])
    y_true = y0 + v * ti
    z_c = s["height_m"] + cc["t_BC"][2]
    if not 8 < y_true - edge < 0.42 * cc["height"] / cc["fy"] * (z_c - step):
        continue
    img = cv2.imread(os.path.join(run, "camera", "img_%05d.png" % im["image_id"][i]), cv2.IMREAD_GRAYSCALE)
    band = img[:, cc["width"] // 2 - 200:cc["width"] // 2 + 200].astype(float).mean(axis=1)
    g = np.abs(np.diff(band))
    j = int(np.argmax(g))
    # Sub-pixel: where the profile crosses the mid level between the two sides.
    lo, hi = band[min(j + 6, len(band) - 1)], band[max(j - 5, 0)]
    mid = (lo + hi) / 2
    a, b = band[j], band[j + 1]
    row = j + (a - mid) / (a - b) if a != b else j + 0.5
    # Image down = backward: the edge (behind the camera) is below the centre by fy * dy / dz.
    y_meas = edge + (row - cc["cy"]) * (z_c - step) / cc["fy"]
    rows.append({"t_s": ti, "edge_row_px": float(row), "camera_y_from_image_m": float(y_meas), "true_y_m": float(y_true),
                 "lag_ms": float((y_true - y_meas) / v * 1e3), "pose_stream_y_m": float(im["y_m"][i])})
out["camera"] = {"images_used": len(rows), "per_image": rows,
                 "lag_ms": {"mean": float(np.mean([q["lag_ms"] for q in rows])) if rows else None,
                            "min": float(np.min([q["lag_ms"] for q in rows])) if rows else None,
                            "max": float(np.max([q["lag_ms"] for q in rows])) if rows else None},
                 "ms_per_pixel": float((s["height_m"] - step) / cc["fy"] / v * 1e3)}
json.dump(out, open(os.path.join(run, "sync.json"), "w"), indent=2)
p, l, c = out["pose_stream"], out["lidar"], out["camera"]
print(f"pose stream: {p['rate_hz']:.0f} Hz, speed {p['speed_fit_mps']:.4f} m/s, lag {p['lag_ms']['mean']:+.2f} ms "
      f"({p['lag_ms']['min']:+.2f}..{p['lag_ms']['max']:+.2f})")
print(f"lidar: edge between line stamps {l['edge_between_stamps_s'][0]:.3f} and {l['edge_between_stamps_s'][1]:.3f} s, "
      f"true crossing {l['true_crossing_s']:.3f} s -> lag {l['lag_ms'][0]:+.1f}..{l['lag_ms'][1]:+.1f} ms")
sl = out["lidar_slant"]
print(f"lidar (slanted edge, {sl['rays']} rays, ray lean removed): timing lag {sl['lag_ms_interval'][0]:+.2f} .. {sl['lag_ms_interval'][1]:+.2f} ms"
      f"{'' if sl['consistent'] else '  (INCONSISTENT: the rays disagree)'}")
if rows:
    print(f"camera: {len(rows)} images, lag {c['lag_ms']['mean']:+.1f} ms ({c['lag_ms']['min']:+.1f}..{c['lag_ms']['max']:+.1f}), "
          f"{c['ms_per_pixel']:.1f} ms per pixel")
