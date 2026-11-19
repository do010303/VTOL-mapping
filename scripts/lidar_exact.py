#!/usr/bin/env python3
"""LiDAR with the spec's per-ray timing, by ray casting on the scene mesh (spec 2 and 5).

Gazebo's gpu_lidar renders a whole scan line at one instant. The spec's scanner emits ray k of a line at
t_s + k * pulse_spacing_s (1/600 000 s: the 1333 rays take 2.22 ms of the 5 ms line period), and each ray must use
the vehicle pose at its own emission time. This script casts exactly those rays against the scene's own meshes
(the OBJ files Gazebo renders: terrain and tree tiles), with the recorded 500 Hz trajectory:

  ray k of line l:  t = t_l + k * pulse_spacing_s,  beta_k = scan_min + k * step (k = 0: -20 deg, right of track)
                    origin  o = p(t) + R(t) t_BL,  direction  R(t) R_BL d_L(beta_k),  d_L = (0, sin beta, -cos beta)
                    range   = distance to the first triangle hit, if within [range_min, range_max], else inf

The pose at t is interpolated linearly (position) and normalised-linearly (quaternion) between the trajectory
samples. Thin geometric rays, first hit, no noise: the spec's ideal model. Line times t_l are those of the Gazebo
recording (same scan lines, so the two can be compared ray by ray); --synth makes lines at lines_per_s instead.

  <run>/lidar_exact/scans_<k>.npz   t_s (line start), ranges_m (lines x rays, float32, inf = no return)
  <run>/lidar_exact/stats.json      counts, timing, comparison with the Gazebo recording

  python3 scripts/lidar_exact.py out/r5_main                    # scene from <run>/sensors.json
  python3 scripts/lidar_exact.py out/<run> --compare-only       # just the comparison with lidar/
"""

import argparse
import json
import os
import time

import numpy as np

from flight_report import interp_poses
from scene import Truth, scene_of_run
from static_report import load_scans


def load_obj_triangles(path):
    """Triangles of an OBJ written by make_forest.py: one vertex per triangle corner, faces in vertex order."""
    with open(path, "rb") as f:
        data = f.read()
    lines = [ln[2:] for ln in data.split(b"\n") if ln.startswith(b"v ")]
    v = np.array(b" ".join(lines).split(), dtype=np.float32).reshape(-1, 3)
    if len(v) % 3:
        raise ValueError(f"{path}: {len(v)} vertices, not one per triangle corner")
    return v


def scene_mesh(truth, ground_only=False):
    """All scene triangles as (vertices (3n, 3) float32, indices (n, 3)), cached in models/<scene>/mesh_cache.npz."""
    objs = truth.meshes()
    if ground_only:
        objs = [p for p in objs if os.path.basename(p) == "ground.obj"]
        v = np.concatenate([load_obj_triangles(p) for p in objs])
        return v, np.arange(len(v), dtype=np.uint32).reshape(-1, 3)
    cache = os.path.join(os.path.dirname(truth.mesh_dir), "mesh_cache.npz")
    if os.path.exists(cache) and os.path.getmtime(cache) > max(os.path.getmtime(p) for p in objs):
        v = np.load(cache)["v"]
    else:
        v = np.concatenate([load_obj_triangles(p) for p in objs])
        np.savez(cache, v=v)
    return v, np.arange(len(v), dtype=np.uint32).reshape(-1, 3)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run")
    ap.add_argument("--scene", help="default: from <run>/sensors.json")
    ap.add_argument("--synth", action="store_true", help="scan lines at lines_per_s over the trajectory, not Gazebo's")
    ap.add_argument("--min-alt", type=float, default=None, help="with --synth: only above this altitude, m")
    ap.add_argument("--chunk", type=int, default=400, help="scan lines per ray-casting batch")
    ap.add_argument("--ground-only", action="store_true", help="cast against the terrain only (flat_static world)")
    ap.add_argument("--compare-only", action="store_true")
    args = ap.parse_args()
    import open3d as o3d

    run = args.run
    cfg_dir = run if os.path.exists(os.path.join(run, "sensors.json")) else os.path.dirname(run.rstrip("/"))  # static: h<H>/
    cfg = json.load(open(os.path.join(cfg_dir, "sensors.json")))
    lc = cfg["lidar"]
    truth = Truth(args.scene or scene_of_run(cfg_dir))
    out_dir = os.path.join(run, "lidar_exact")
    os.makedirs(out_dir, exist_ok=True)
    traj = np.loadtxt(os.path.join(run, "trajectory.csv"), delimiter=",", skiprows=1)
    traj = traj[np.concatenate([[True], np.diff(traj[:, 0]) > 0])]

    n_rays = lc["rays_per_line"]
    beta = np.radians(np.linspace(lc["scan_min_deg"], lc["scan_max_deg"], n_rays))
    d_l = np.stack([np.zeros(n_rays), np.sin(beta), -np.cos(beta)], axis=1)
    d_b = d_l @ np.array(lc["R_BL"], dtype=float).T
    t_bl = np.array(lc["t_BL"], dtype=float)
    dt = lc["pulse_spacing_s"] * np.arange(n_rays)
    r_min, r_max = lc["range_min_m"], lc["range_max_m"]

    t_gz, r_gz = load_scans(run, "lidar")
    if args.synth or t_gz is None:
        t_lines = np.arange(traj[0, 0], traj[-1, 0] - dt[-1], 1.0 / lc["lines_per_s"])
        if args.min_alt is not None:
            p, _ = interp_poses(traj, t_lines)
            t_lines = t_lines[p[:, 2] >= args.min_alt]
        src = "synthetic"
    else:
        t_lines = t_gz
        src = "gazebo line times"
    # Rays past the end of the trajectory cannot be posed.
    t_lines = t_lines[t_lines + dt[-1] <= traj[-1, 0]]

    stats = {"scene": truth.scene, "line_times": src, "lines": int(len(t_lines)), "rays_per_line": n_rays,
             "pulse_spacing_s": lc["pulse_spacing_s"], "scan_window_s": float(dt[-1]),
             "range_m": [r_min, r_max], "ray_order": "k = 0 at scan_min_deg (right of track) ... k = n-1 at scan_max_deg",
             "pose_interpolation": "linear position, normalised-linear quaternion between trajectory samples",
             "trajectory_rate_hz": float(1 / np.median(np.diff(traj[:, 0])))}

    if not args.compare_only:
        t0 = time.time()
        v, tri = scene_mesh(truth, args.ground_only)
        sc = o3d.t.geometry.RaycastingScene()
        sc.add_triangles(o3d.core.Tensor(v), o3d.core.Tensor(tri))
        stats["triangles"] = int(len(tri))
        stats["mesh_load_s"] = time.time() - t0
        t0 = time.time()
        buf_t, buf_r, k_file, done = [], [], 0, 0
        for f in os.listdir(out_dir):
            if f.startswith("scans_") or f == "stats.json":     # stats.json marks a finished run
                os.remove(os.path.join(out_dir, f))
        for a in range(0, len(t_lines), args.chunk):
            tl = t_lines[a:a + args.chunk]
            tk = (tl[:, None] + dt[None, :]).ravel()
            p, rot = interp_poses(traj, tk)
            o = p + rot @ t_bl
            d = np.einsum("mij,mj->mi", rot, np.tile(d_b, (len(tl), 1)))
            rays = np.concatenate([o, d], axis=1).astype(np.float32)
            hit = sc.cast_rays(o3d.core.Tensor(rays), nthreads=0)["t_hit"].numpy()
            rng = np.where((hit >= r_min) & (hit <= r_max), hit, np.inf).astype(np.float32)
            buf_t.append(tl)
            buf_r.append(rng.reshape(len(tl), n_rays))
            done += len(tl)
            if sum(len(x) for x in buf_t) >= 2000 or done == len(t_lines):
                np.savez_compressed(os.path.join(out_dir, f"scans_{k_file:04d}.npz"), t_s=np.concatenate(buf_t),
                         ranges_m=np.concatenate(buf_r))
                k_file += 1
                buf_t, buf_r = [], []
            if a // args.chunk % 50 == 0:
                el = time.time() - t0
                print(f"{done}/{len(t_lines)} lines, {done * n_rays / max(el, 1e-9) / 1e6:.1f} M rays/s", flush=True)
        stats["cast_s"] = time.time() - t0

    t_ex, r_ex = load_scans(run, "lidar_exact")
    hit = np.isfinite(r_ex)
    stats["returns"] = int(hit.sum())
    stats["return_ratio"] = float(hit.mean())
    if t_gz is not None and not args.synth:
        # Ray-by-ray against the Gazebo recording (same lines): where both have a return.
        j = np.searchsorted(t_gz, t_ex)
        j = np.clip(j, 0, len(t_gz) - 1)
        same = np.abs(t_gz[j] - t_ex) < 1e-6
        a, b = r_ex[same], r_gz[j[same]]
        both = np.isfinite(a) & np.isfinite(b)
        diff = (a - b)[both]
        stats["vs_gazebo"] = {
            "lines_compared": int(same.sum()), "both_return": float(both.mean()),
            "only_exact": float((np.isfinite(a) & ~np.isfinite(b)).mean()),
            "only_gazebo": float((~np.isfinite(a) & np.isfinite(b)).mean()),
            "range_diff_m": {"median": float(np.median(diff)), "p05": float(np.percentile(diff, 5)),
                             "p95": float(np.percentile(diff, 95)),
                             "within_5cm": float(np.mean(np.abs(diff) < 0.05)),
                             "within_50cm": float(np.mean(np.abs(diff) < 0.5))},
        }
    json.dump(stats, open(os.path.join(out_dir, "stats.json"), "w"), indent=2)
    print(json.dumps({k: stats[k] for k in stats if k not in ("ray_order", "pose_interpolation")}, indent=1))


if __name__ == "__main__":
    main()
