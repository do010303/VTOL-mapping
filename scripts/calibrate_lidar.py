#!/usr/bin/env python3
"""Per-ray scan angles of the simulated LiDAR, measured on bare ground (out/<flat run>/h<H>).

Gazebo's gpu_lidar samples its depth texture at angles up to about one step (0.03 deg) off the
nominal beta_k = min + k * step; on flat ground that is a deterministic range error of up to
3e-4 x R (16 cm at 500 m, 35 cm at 1100 m at the swath edge). The real angle of ray k follows from
its range to a plane at known height: cos(beta_k) = z_lidar / R_k. Writes config/lidar_angles.json,
which scripts/static_report.py and scripts/flight_report.py use instead of the nominal angles.

  python3 scripts/calibrate_lidar.py out/flat_accuracy/h500
"""
import glob
import json
import os
import sys

import numpy as np

import scan_codec

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

d = sys.argv[1]
cfg = json.load(open(os.path.join(ROOT, "config", "sensors.json")))["lidar"]
run = json.load(open(os.path.join(d, "run.json")))
assert run["world"] == "flat_static", "calibrate on flat_static"
r = np.concatenate([scan_codec.load(f)["ranges_m"] for f in sorted(glob.glob(os.path.join(d, "lidar", "scans_*.npz")))])
assert np.allclose(r, r[0]), "the lines differ: is the rig static?"
z_l = run["height_m"] + cfg["t_BL"][2]
nominal = np.radians(np.linspace(cfg["scan_min_deg"], cfg["scan_max_deg"], cfg["rays_per_line"]))
beta = np.sign(nominal) * np.arccos(np.clip(z_l / r[0], -1, 1))
out = {"source": os.path.relpath(os.path.abspath(d), ROOT), "z_lidar_m": z_l, "rays": int(beta.size),
       "max_offset_deg": float(np.degrees(np.abs(beta - nominal)).max()),
       "beta_rad": beta.round(9).tolist()}
json.dump(out, open(os.path.join(ROOT, "config", "lidar_angles.json"), "w"))
print(f"config/lidar_angles.json: {beta.size} rays, max offset {out['max_offset_deg'] * 1000:.1f} mdeg from nominal")
