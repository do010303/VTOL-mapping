#!/usr/bin/env python3
"""Export a survey flight as the dataset of spec section 8, plus the checks of 9.4 (density, coverage) and
9.5 (images: spacing, overlap from the saved poses).

  <run>/export/
    config.json              sensors, mount transforms, LiDAR angle calibration, frames, strips, crop, what each file is
    trajectory.csv           t_s, x_m, y_m, z_m, qx, qy, qz, qw, strip_id, segment (survey | transit), 500 Hz
    lidar/strip_<s>.npz      raw measurements per strip: t_s and scanline_id per line, beta_rad / gamma_rad per ray,
                             range_m (lines x rays, inf = no return)
    las/strip_<s>.las        point cloud, LAS 1.2 point format 1: gps_time = sim time (s), point_source_id = strip,
                             scan_angle_rank = beta (deg); local ENU metres, no CRS; classification 0 (not classified)
    truth/labels_strip_<s>.npz   per LAS point, same order: label (1 ground, 2 vegetation) and tree_id (-1 = none),
                             from the scene geometry; kept apart from the LAS (spec 8: labels not given to the algorithm)
    colmap/sparse/0/         cameras.txt (PINHOLE), images.txt (world-to-camera pose of every survey image),
                             points3D.txt (empty); images in ../camera; reference_positions.txt (name x y z) for
                             colmap model_aligner
    checks.json, coverage.png    density and coverage of the evaluation area, image overlap counts

  python3 scripts/export_dataset.py out/r5_main --planned-x -427.5 -332.5 ... 427.5
  python3 scripts/export_dataset.py out/r5_main --lines 10 --spacing 95 --no-crop
"""

import argparse
import datetime
import glob
import json
import math
import os
import struct

import numpy as np

from flight_report import interp_poses
from static_report import ROOT, Truth, lidar_dirs, load_scans, quat_to_matrix
from strips import segment, strip_of_time

LAS_SCALE = 0.001


def write_las(path, xyz, gps_time, source_id, scan_angle_deg):
    """LAS 1.2, point data record format 1 (28 bytes), no VLRs."""
    n = len(xyz)
    lo, hi = xyz.min(axis=0), xyz.max(axis=0)
    offset = np.floor(lo)
    rec = np.zeros(n, dtype=[("x", "<i4"), ("y", "<i4"), ("z", "<i4"), ("intensity", "<u2"), ("flags", "u1"),
                             ("cls", "u1"), ("angle", "i1"), ("user", "u1"), ("src", "<u2"), ("t", "<f8")])
    q = np.round((xyz - offset) / LAS_SCALE).astype(np.int64)
    rec["x"], rec["y"], rec["z"] = q[:, 0], q[:, 1], q[:, 2]
    rec["flags"] = 0b00001001                      # return 1 of 1
    rec["angle"] = np.clip(np.round(scan_angle_deg), -90, 90).astype(np.int8)
    rec["src"] = source_id
    rec["t"] = gps_time
    today = datetime.date.today()
    header = struct.pack(
        "<4sHH16sBB32s32sHHHIIBHI5I3d3d6d",
        b"LASF", 0, 0, b"\0" * 16, 1, 2,
        b"VTOL-mapping simulation".ljust(32, b"\0"), b"export_dataset.py".ljust(32, b"\0"),
        today.timetuple().tm_yday, today.year, 227, 227, 0, 1, 28, n, n, 0, 0, 0, 0,
        LAS_SCALE, LAS_SCALE, LAS_SCALE, *offset,
        hi[0], lo[0], hi[1], lo[1], hi[2], lo[2])
    assert len(header) == 227
    with open(path, "wb") as f:
        f.write(header)
        f.write(rec.tobytes())


def read_las(path):
    """Reads back what write_las wrote (header fields we use + the points), to check the file."""
    with open(path, "rb") as f:
        h = f.read(227)
        (_, _, _, _, vmaj, vmin, _, _, _, _, hsize, off, nvlr, fmt, rlen, n, *_rest) = struct.unpack(
            "<4sHH16sBB32s32sHHHIIBHI5I3d3d6d", h)
        vals = struct.unpack("<3d3d6d", h[131:227])
        scale, offs = np.array(vals[0:3]), np.array(vals[3:6])
        f.seek(off)
        rec = np.frombuffer(f.read(n * rlen), dtype=[("x", "<i4"), ("y", "<i4"), ("z", "<i4"), ("intensity", "<u2"),
                                                      ("flags", "u1"), ("cls", "u1"), ("angle", "i1"), ("user", "u1"),
                                                      ("src", "<u2"), ("t", "<f8")])
    xyz = np.stack([rec["x"], rec["y"], rec["z"]], axis=1) * scale + offs
    return {"version": f"{vmaj}.{vmin}", "format": fmt, "record_length": rlen, "points": n,
            "bounds": [float(vals[7]), float(vals[6]), float(vals[9]), float(vals[8]), float(vals[11]), float(vals[10])]}, \
        xyz, rec


def rot_to_quat(r):
    """(qw, qx, qy, qz) of a rotation matrix."""
    w = math.sqrt(max(0.0, 1 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + r[0, 0] - r[1, 1] - r[2, 2])) / 2, r[2, 1] - r[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - r[0, 0] + r[1, 1] - r[2, 2])) / 2, r[0, 2] - r[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - r[0, 0] - r[1, 1] + r[2, 2])) / 2, r[1, 0] - r[0, 1])
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run")
    ap.add_argument("--planned-x", type=float, nargs="*", help="x of the planned lines")
    ap.add_argument("--lines", type=int, help="instead of --planned-x: N lines centred on x = 0 ...")
    ap.add_argument("--spacing", type=float, default=95.0, help="... this far apart")
    ap.add_argument("--y-range", type=float, nargs=2, default=(-650, 650))
    ap.add_argument("--eval", type=float, nargs=4, default=(-400, 400, -400, 400))
    ap.add_argument("--margin", type=float, default=100.0, help="LAS: keep points within eval area + margin, m")
    ap.add_argument("--no-crop", action="store_true", help="LAS: every point of the strips")
    ap.add_argument("--decimate", type=int, default=1, help="LAS: every n-th scan line")
    args = ap.parse_args()
    run = args.run
    planned = args.planned_x
    if planned is None and args.lines:
        planned = [-(args.lines - 1) / 2 * args.spacing + k * args.spacing for k in range(args.lines)]
    ex = os.path.join(run, "export")
    for d in ("lidar", "las", "truth", "colmap/sparse/0"):
        os.makedirs(os.path.join(ex, d), exist_ok=True)

    cfg = json.load(open(os.path.join(run, "sensors.json")))
    lc, cc = cfg["lidar"], cfg["camera"]
    cal_path = os.path.join(ROOT, "config", "lidar_angles.json")
    cal = json.load(open(cal_path)) if os.path.exists(cal_path) else None
    d_l, beta = lidar_dirs(cfg)
    gamma = d_l[:, 0] / np.hypot(d_l[:, 1], d_l[:, 2])
    truth = Truth()

    # Trajectory with strips.
    traj = np.loadtxt(os.path.join(run, "trajectory.csv"), delimiter=",", skiprows=1)
    traj = traj[np.concatenate([[True], np.diff(traj[:, 0]) > 0])]
    tp, tr = interp_poses(traj, traj[:, 0])
    _, strips = segment(traj[:, 0], tp, tr, args.y_range, planned_x=planned)
    sid_traj = strip_of_time(strips, traj[:, 0])
    with open(os.path.join(ex, "trajectory.csv"), "w") as f:
        f.write("t_s,x_m,y_m,z_m,qx,qy,qz,qw,strip_id,segment\n")
        for row, s in zip(traj, sid_traj):
            f.write(",".join(f"{v:.6f}" if i >= 4 else f"{v:.4f}" for i, v in enumerate(row[:8]))
                    + f",{s},{'survey' if s else 'transit'}\n")
    pose_dt = np.diff(traj[:, 0])

    # LiDAR per strip.
    t, r = load_scans(run)
    sid_lines = strip_of_time(strips, t)
    x0, x1, y0, y1 = args.eval
    m = 0 if args.no_crop else args.margin
    las_info, totals = [], {"points": 0, "rays": 0}
    cnt = np.zeros((len(truth.y), len(truth.x)), dtype=np.int64)
    kd = None
    for st in strips:
        sel = np.nonzero(sid_lines == st["id"])[0]
        np.savez_compressed(os.path.join(ex, "lidar", f"strip_{st['id']:02d}.npz"), t_s=t[sel], scanline_id=sel,
                            strip_id=st["id"], beta_rad=beta, gamma_rad=gamma, range_m=r[sel],
                            pulse_spacing_s=lc["pulse_spacing_s"])
        totals["rays"] += r[sel].size
        sel = sel[::args.decimate]
        p, rot = interp_poses(traj, t[sel])
        o = p + np.einsum("nij,j->ni", rot, np.array(lc["t_BL"]))
        xyz_l, tt_l, ang_l = [], [], []
        for a in range(0, len(sel), 2000):
            sl = slice(a, a + 2000)
            rr = r[sel][sl]
            hit = np.isfinite(rr)
            pts = o[sl, None, :] + np.where(hit, rr, 0)[..., None] * np.einsum("nij,kj->nki", rot[sl], d_l)
            xyz_l.append(pts[hit])
            tt_l.append(np.repeat(t[sel][sl][:, None], rr.shape[1], axis=1)[hit])
            ang_l.append(np.repeat(np.degrees(beta)[None, :], rr.shape[0], axis=0)[hit])
        xyz, tt, ang = np.concatenate(xyz_l), np.concatenate(tt_l), np.concatenate(ang_l)
        if not args.no_crop:
            keep = (xyz[:, 0] >= x0 - m) & (xyz[:, 0] <= x1 + m) & (xyz[:, 1] >= y0 - m) & (xyz[:, 1] <= y1 + m)
            xyz, tt, ang = xyz[keep], tt[keep], ang[keep]
        path = os.path.join(ex, "las", f"strip_{st['id']:02d}.las")
        write_las(path, xyz, tt, st["id"], ang)
        # Truth labels from the scene.
        ground = np.abs(xyz[:, 2]) < 0.05
        if kd is None:
            from scipy.spatial import cKDTree
            kd = cKDTree(np.stack([truth.trees["x_m"], truth.trees["y_m"]], axis=1))
        dist, idx = kd.query(xyz[:, :2])
        tree = np.where(~ground & (dist <= truth.trees["crown_radius_m"][idx] + 0.3), truth.trees["tree_id"][idx], -1)
        np.savez_compressed(os.path.join(ex, "truth", f"labels_strip_{st['id']:02d}.npz"),
                            label=np.where(ground, 1, 2).astype(np.uint8), tree_id=tree.astype(np.int32))
        # Density on the 1 m truth grid.
        j = np.round((xyz[:, 0] - truth.x[0]) / truth.cell).astype(int)
        i = np.round((xyz[:, 1] - truth.y[0]) / truth.cell).astype(int)
        ok = (i >= 0) & (i < cnt.shape[0]) & (j >= 0) & (j < cnt.shape[1])
        np.add.at(cnt, (i[ok], j[ok]), 1)
        las_info.append({"strip": st["id"], "file": os.path.relpath(path, ex), "points": int(len(xyz)),
                         "ground_ratio": float(ground.mean()), "size_MB": os.path.getsize(path) / 1e6})
        totals["points"] += len(xyz)
        print(f"strip {st['id']:2d} {st['dir']} x {st['x_nominal']:+7.1f}: {len(sel)} lines, {len(xyz):,} points -> {path}",
              flush=True)

    # LAS read-back check (first strip).
    hdr, xyz_back, rec = read_las(os.path.join(ex, las_info[0]["file"]))
    las_check = {"header": hdr, "src_ids": sorted(set(rec["src"].tolist())),
                 "gps_time_range": [float(rec["t"].min()), float(rec["t"].max())]}

    # COLMAP.
    im = np.genfromtxt(os.path.join(run, "camera", "images.csv"), delimiter=",", names=True, dtype=None, encoding=None)
    si = strip_of_time(strips, im["t_s"])
    r_bc, t_bc = np.array(cc["R_BC"]), np.array(cc["t_BC"])
    col = os.path.join(ex, "colmap", "sparse", "0")
    with open(os.path.join(col, "cameras.txt"), "w") as f:
        f.write("# CAMERA_ID MODEL WIDTH HEIGHT PARAMS[]\n")
        f.write(f"1 PINHOLE {cc['width']} {cc['height']} {cc['fx']:.6f} {cc['fy']:.6f} {cc['cx']} {cc['cy']}\n")
    centres, rows = [], []
    with open(os.path.join(col, "images.txt"), "w") as f, \
            open(os.path.join(ex, "colmap", "reference_positions.txt"), "w") as g:
        f.write("# IMAGE_ID QW QX QY QZ TX TY TZ CAMERA_ID NAME (world-to-camera; world = local ENU, m)\n"
                "# POINTS2D[] (empty)\n")
        for k in np.nonzero(si > 0)[0]:
            r_wb = quat_to_matrix(im["qx"][k], im["qy"][k], im["qz"][k], im["qw"][k])
            p_wb = np.array([im["x_m"][k], im["y_m"][k], im["z_m"][k]])
            r_wc = r_wb @ r_bc
            c_w = p_wb + r_wb @ t_bc
            r_cw = r_wc.T
            tvec = -r_cw @ c_w
            q = rot_to_quat(r_cw)
            name = "img_%05d.png" % im["image_id"][k]
            f.write(f"{im['image_id'][k] + 1} {q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f} "
                    f"{tvec[0]:.4f} {tvec[1]:.4f} {tvec[2]:.4f} 1 {name}\n\n")
            g.write(f"{name} {c_w[0]:.4f} {c_w[1]:.4f} {c_w[2]:.4f}\n")
            centres.append(c_w)
            rows.append((k, r_cw, tvec))
    open(os.path.join(col, "points3D.txt"), "w").write("# empty: poses are given, run point_triangulator\n")
    # Consistency: tree tops projected with the COLMAP pose land where the overlay projection puts them.
    k, r_cw, tvec = rows[len(rows) // 2]
    tw = np.stack([truth.trees["x_m"], truth.trees["y_m"], truth.trees["height_m"]], axis=1)
    pc = tw @ r_cw.T + tvec
    vis = pc[:, 2] > 0
    u = cc["fx"] * pc[vis, 0] / pc[vis, 2] + cc["cx"]
    v = cc["fy"] * pc[vis, 1] / pc[vis, 2] + cc["cy"]
    from static_report import project
    r_wb = quat_to_matrix(im["qx"][k], im["qy"][k], im["qz"][k], im["qw"][k])
    u2, v2, _ = project(cfg, tw[vis], np.array([im["x_m"][k], im["y_m"][k], im["z_m"][k]]), r_wb)
    colmap_check = {"image": "img_%05d.png" % im["image_id"][k], "max_px_diff_vs_overlay": float(max(np.abs(u - u2).max(), np.abs(v - v2).max()))}

    # Image overlap (9.5): how many survey images see each 5 m cell of the eval area at the canopy top (30 m)
    # and on the ground.
    g5 = np.arange(x0 + 2.5, x1, 5.0), np.arange(y0 + 2.5, y1, 5.0)
    gx, gy = np.meshgrid(*g5)
    seen = {}
    for zc in (30.0, 0.0):
        pts = np.stack([gx.ravel(), gy.ravel(), np.full(gx.size, zc)], axis=1)
        n_img = np.zeros(gx.size, dtype=int)
        for _, r_cw, tvec in rows:
            pc = pts @ r_cw.T + tvec
            u = cc["fx"] * pc[:, 0] / pc[:, 2] + cc["cx"]
            v = cc["fy"] * pc[:, 1] / pc[:, 2] + cc["cy"]
            n_img += (pc[:, 2] > 0) & (u >= 0) & (u < cc["width"]) & (v >= 0) & (v < cc["height"])
        seen[zc] = n_img.reshape(gx.shape)
    # Spacing and overlaps from the poses.
    sp = np.concatenate([np.hypot(np.diff(im["x_m"][si == s["id"]]), np.diff(im["y_m"][si == s["id"]]))
                         for s in strips if (si == s["id"]).sum() > 1])
    h_mean = float(np.mean([c[2] for c in centres]))
    along = 2 * (h_mean - 30) * math.tan(math.radians(cc["vfov_deg"] / 2))
    across = 2 * (h_mean - 30) * math.tan(math.radians(cc["hfov_deg"] / 2))
    xs = sorted(s["x_nominal"] for s in strips)
    lidar_half = (h_mean + lc["t_BL"][2] - cc["t_BC"][2]) * math.tan(math.radians(lc["scan_max_deg"]))
    box = (truth.y[:, None] >= y0) & (truth.y[:, None] <= y1) & (truth.x[None, :] >= x0) & (truth.x[None, :] <= x1)
    c_box = cnt[box]
    checks = {
        "strips": [{k: s[k] for k in ("id", "dir", "x_nominal", "t0", "t1")} for s in strips],
        "pose_rate_hz": float(1 / np.median(pose_dt)), "pose_max_gap_s": float(pose_dt.max()),
        "lidar": {"rays_on_strips": totals["rays"], "las_points": totals["points"], "decimate": args.decimate,
                  "crop": None if args.no_crop else [x0 - m, x1 + m, y0 - m, y1 + m],
                  "eval_density_pts_m2": {"mean": float(c_box.mean()), "p05": float(np.percentile(c_box, 5)),
                                          "min": int(c_box.min())},
                  "eval_coverage_1m": float((c_box > 0).mean()),
                  "side_overlap_lidar": float(1 - np.mean(np.diff(xs)) / (2 * lidar_half)) if len(xs) > 1 else None},
        "las": las_info, "las_readback": las_check,
        "images": {"survey": len(rows), "spacing_m": {"mean": float(sp.mean()), "min": float(sp.min()), "max": float(sp.max())},
                   "forward_overlap_canopy": float(1 - sp.mean() / along),
                   "side_overlap_canopy": float(1 - np.mean(np.diff(xs)) / across) if len(xs) > 1 else None,
                   "images_per_5m_cell_canopy": {"min": int(seen[30.0].min()), "median": float(np.median(seen[30.0])),
                                                 "p05": float(np.percentile(seen[30.0], 5))},
                   "images_per_5m_cell_ground": {"min": int(seen[0.0].min()), "median": float(np.median(seen[0.0]))}},
        "colmap": colmap_check,
    }
    json.dump(checks, open(os.path.join(ex, "checks.json"), "w"), indent=2)

    meta = {
        "run": os.path.abspath(run), "created": datetime.datetime.now().isoformat(timespec="seconds"),
        "frames": cfg["frames"], "units": "metres, seconds (sim time from the start of the run), radians",
        "sensors": cfg, "lidar_angle_calibration": None if cal is None else
        {k: cal[k] for k in cal if k not in ("beta_rad", "gamma_rad")},
        "lidar_ray_direction": "d_L = normalize(gamma, sin(beta), -cos(beta)); p_W = T_WB(t) (t_BL + range d_L)",
        "lidar_timing": "Gazebo renders each scan line at one instant: t_s of the line is the time of all its rays "
                        "(pulse_spacing_s is the real scanner's schedule, kept for reference, not applied)",
        "strips": checks["strips"], "planned_x": planned, "y_range": args.y_range, "eval_area": args.eval,
        "files": {"trajectory.csv": "vehicle pose (body FLU -> world ENU), strip_id 0 = transit",
                  "lidar/": "raw per-strip measurements", "las/": "LAS 1.2 PDRF 1 point clouds, local ENU, no CRS",
                  "truth/": "labels from the scene, same point order as las/", "colmap/": "COLMAP text model, images in ../camera",
                  "truth_rasters": os.path.join(ROOT, "models", "forest", "truth.npz")},
        "ideal_assumptions": ["no wind", "exact pose", "pinhole camera, instantaneous exposure", "thin rays, single return",
                              "hard range limit, no reflectance model", "flat terrain"],
    }
    json.dump(meta, open(os.path.join(ex, "config.json"), "w"), indent=2)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 2, figsize=(14, 6.5))
    ext = [truth.x[0], truth.x[-1], truth.y[0], truth.y[-1]]
    mm = ax[0].imshow(np.where(cnt > 0, cnt, np.nan), origin="lower", extent=ext, cmap="magma",
                      vmax=np.percentile(c_box, 99))
    fig.colorbar(mm, ax=ax[0], shrink=0.7, label="LiDAR points per 1 m cell")
    mi = ax[1].imshow(seen[30.0], origin="lower", extent=[x0, x1, y0, y1], cmap="viridis")
    fig.colorbar(mi, ax=ax[1], shrink=0.7, label="images seeing the cell (canopy top, 5 m cells)")
    for a in ax:
        a.add_patch(plt.Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, ec="w", lw=1.5, ls="--"))
        a.set_xlim(x0 - 150, x1 + 150); a.set_ylim(y0 - 150, y1 + 150)
        a.set_xlabel("x east (m)"); a.set_ylabel("y north (m)")
    ax[0].set_title("LiDAR density (spec 9.4); dashed: evaluation area")
    ax[1].set_title("Image overlap count (spec 9.5)")
    fig.savefig(os.path.join(ex, "coverage.png"), dpi=90, bbox_inches="tight")
    L, I = checks["lidar"], checks["images"]
    print(json.dumps({"strips": len(strips), "pose_hz": round(checks["pose_rate_hz"]), "las_points": totals["points"],
                      "eval_density": L["eval_density_pts_m2"], "eval_coverage_1m": L["eval_coverage_1m"],
                      "lidar_side_overlap": L["side_overlap_lidar"], "images": I["survey"],
                      "image_overlap": [I["forward_overlap_canopy"], I["side_overlap_canopy"]],
                      "images_per_cell_canopy": I["images_per_5m_cell_canopy"], "las_readback": hdr,
                      "colmap_px_diff": colmap_check["max_px_diff_vs_overlay"]}, indent=1))


if __name__ == "__main__":
    main()
