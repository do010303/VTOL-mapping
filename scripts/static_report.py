#!/usr/bin/env python3
"""Report of a static run (scripts/static_run.sh): LiDAR range check, returns vs height, camera geometry.

For each out/<run>/h<H>/:
  * LiDAR rays -> world points (p_W = T_WB x T_BL x p_L, spec 5) with the rig pose from trajectory.csv.
  * flat_static: range error against the exact slant range (z_lidar / cos beta, spec 9.1).
  * forest_static: return ratio per scan angle, ground vs vegetation returns, point z against the DSM.
  * camera: size and K against config/sensors.json; tree tops (trees.csv) and the LiDAR line projected into
    the image (overlay_h<H>.jpg), which checks mount, orientation and the shared clock in one picture.
Writes <run>/report.json, <run>/report.md and figures.

  python3 scripts/static_report.py out/forest_static_<date>
"""

import argparse
import glob
import json
import math
import os

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)


def quat_to_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def load_scans(d):
    t, r = [], []
    for f in sorted(glob.glob(os.path.join(d, "lidar", "scans_*.npz"))):
        z = np.load(f)
        t.append(z["t_s"])
        r.append(z["ranges_m"])
    if not t:
        return None, None
    return np.concatenate(t), np.concatenate(r)


def rig_pose(d):
    a = np.loadtxt(os.path.join(d, "trajectory.csv"), delimiter=",", skiprows=1, ndmin=2)
    p, q = a[-1, 1:4], a[-1, 4:8]
    return p, quat_to_matrix(*q)


def lidar_angles(cfg, calibrated=True):
    """Scan angle of each ray: the measured table (scripts/calibrate_lidar.py) if there is one."""
    lc = cfg["lidar"]
    path = os.path.join(ROOT, "config", "lidar_angles.json")
    if calibrated and os.path.exists(path):
        cal = json.load(open(path))
        if cal["rays"] == lc["rays_per_line"]:
            return np.array(cal["beta_rad"])
    return np.radians(np.linspace(lc["scan_min_deg"], lc["scan_max_deg"], lc["rays_per_line"]))


def lidar_dirs(cfg, calibrated=True):
    """Unit ray directions in the LiDAR frame: d_L = (0, sin beta, -cos beta) (spec 5), with the calibrated
    along-track lean gamma (scripts/calibrate_lidar_along.py) as x component when there is one."""
    beta = lidar_angles(cfg, calibrated)
    gamma = np.zeros_like(beta)
    path = os.path.join(ROOT, "config", "lidar_angles.json")
    if calibrated and os.path.exists(path):
        cal = json.load(open(path))
        if "gamma_rad" in cal and len(cal["gamma_rad"]) == beta.size:
            gamma = np.array(cal["gamma_rad"])
    d = np.stack([gamma, np.sin(beta), -np.cos(beta)], axis=1)
    return d / np.linalg.norm(d, axis=1, keepdims=True), beta


def lidar_points(cfg, ranges, p_wb, r_wb):
    lc = cfg["lidar"]
    d_l, beta = lidar_dirs(cfg)
    t_bl = np.array(lc["t_BL"])
    r_bl = np.array(lc["R_BL"])
    with np.errstate(invalid="ignore"):              # inf (no return) x 0
        p_b = t_bl + (ranges[..., None] * (d_l @ r_bl.T))
    return p_b @ r_wb.T + p_wb, beta


class Truth:
    def __init__(self):
        z = np.load(os.path.join(ROOT, "models", "forest", "truth.npz"))
        self.x, self.y, self.dsm = z["x"], z["y"], z["dsm"]
        self.cell = self.x[1] - self.x[0]
        self.trees = np.genfromtxt(os.path.join(ROOT, "models", "forest", "trees.csv"), delimiter=",",
                                   names=True, dtype=None, encoding=None)

    def surface_at(self, x, y):
        """Exact height of the highest crown surface above each (x, y), from the tree list (0: open ground)."""
        from scipy.spatial import cKDTree
        tr = self.trees
        if not hasattr(self, "kd"):
            self.kd = cKDTree(np.stack([tr["x_m"], tr["y_m"]], axis=1))
        sides, out = 7, np.zeros(len(x))
        sector = 2 * np.pi / sides
        _, idx = self.kd.query(np.stack([x, y], axis=1), k=12, distance_upper_bound=5.0)
        for c in range(idx.shape[1]):
            i = idx[:, c]
            ok = i < len(tr)
            ii = i[ok]
            dx, dy = x[ok] - tr["x_m"][ii], y[ok] - tr["y_m"][ii]
            d = np.hypot(dx, dy)
            a = np.mod(np.arctan2(dy, dx) - tr["yaw_rad"][ii], sector) - sector / 2
            edge = tr["crown_radius_m"][ii] * np.cos(sector / 2) / np.cos(a)
            h, low = tr["height_m"][ii], tr["crown_low_m"][ii]
            z = np.where(d <= edge, h - (h - low) * d / edge, 0.0)
            out[ok] = np.maximum(out[ok], z)
        return out

    def dsm_at(self, x, y):
        j = np.round((x - self.x[0]) / self.cell).astype(int)
        i = np.round((y - self.y[0]) / self.cell).astype(int)
        ok = (i >= 0) & (i < len(self.y)) & (j >= 0) & (j < len(self.x))
        out = np.full(x.shape, np.nan)
        out[ok] = self.dsm[i[ok], j[ok]]
        return out


def project(cfg, p_w, p_wb, r_wb):
    c = cfg["camera"]
    r_bc, t_bc = np.array(c["R_BC"]), np.array(c["t_BC"])
    p_b = (p_w - p_wb) @ r_wb               # R_WB^T (p_W - t_WB)
    p_c = (p_b - t_bc) @ r_bc               # R_BC^T (p_B - t_BC)
    z = p_c[:, 2]
    u = c["fx"] * p_c[:, 0] / z + c["cx"]
    v = c["fy"] * p_c[:, 1] / z + c["cy"]
    return u, v, z


def analyse(run, d, cfg, truth, world):
    out = {"dir": os.path.basename(d)}
    meta = json.load(open(os.path.join(d, "run.json")))
    h = meta["height_m"]
    out["height_m"] = h
    ls = json.load(open(os.path.join(d, "lidar", "stats.json")))
    cs = json.load(open(os.path.join(d, "camera", "stats.json")))
    out["lidar_node"] = ls
    out["camera_node"] = {k: cs[k] for k in ("frames", "interval_mean_s", "interval_max_s", "write_mean_s")}
    t, r = load_scans(d)
    p_wb, r_wb = rig_pose(d)
    lc = cfg["lidar"]
    z_l = p_wb[2] + (r_wb @ np.array(lc["t_BL"]))[2]
    out["lidar_height_m"] = float(z_l)
    pts, beta = lidar_points(cfg, r, p_wb, r_wb)
    hit = np.isfinite(r)
    out["return_ratio"] = float(hit.mean())
    edge = np.abs(np.degrees(beta)) > abs(lc["scan_max_deg"]) - 1
    nadir = np.abs(np.degrees(beta)) < 1
    out["return_ratio_nadir"] = float(hit[:, nadir].mean())
    out["return_ratio_edge"] = float(hit[:, edge].mean())
    # Theoretical: the ground is in range where z_l / cos(beta) <= max range.
    out["ground_in_range_ratio_theory"] = float(np.mean(z_l / np.cos(beta) <= lc["range_max_m"]))
    out["ground_slant_edge_m"] = float(z_l / math.cos(math.radians(lc["scan_max_deg"])))
    per_angle = hit.mean(axis=0)
    out["per_angle_return"] = per_angle[::37].round(3).tolist()

    if hit.any():
        z = pts[..., 2][hit]
        if world == "flat_static":
            nominal = lidar_angles(cfg, calibrated=False)
            err0 = (r - (z_l / np.cos(nominal))[None, :])[hit]
            out["range_error_nominal_angles_m"] = {"std": float(err0.std()), "max_abs": float(np.abs(err0).max())}
            err = (r - (z_l / np.cos(beta))[None, :])[hit]
            out["range_error_m"] = {"mean": float(err.mean()), "std": float(err.std()),
                                    "max_abs": float(np.abs(err).max()),
                                    "p99_abs": float(np.percentile(np.abs(err), 99))}
            out["ground_z_m"] = {"mean": float(z.mean()), "std": float(z.std())}
        else:
            ground = z < 0.5
            out["ground_return_ratio"] = float(ground.mean())
            xy = pts[..., :2][hit]
            ds = truth.dsm_at(xy[:, 0], xy[:, 1])
            veg = ~ground & np.isfinite(ds)
            res = z[veg] - ds[veg]
            out["veg_z_minus_dsm_m"] = {"median": float(np.median(res)), "p05": float(np.percentile(res, 5)),
                                        "p95": float(np.percentile(res, 95)),
                                        "above_dsm_0.3m": float(np.mean(res > 0.3))} if veg.any() else None
            sub = np.random.default_rng(0).choice(len(z), min(len(z), 400000), replace=False)
            ex = z[sub] - truth.surface_at(xy[sub, 0], xy[sub, 1])
            out["returns_vs_surface_m"] = {"above_by_5cm": float(np.mean(ex > 0.05)), "max_above": float(ex.max()),
                                           "on_top_surface_within_5cm": float(np.mean(np.abs(ex) < 0.05))}
            out["veg_height_m"] = {"mean": float(z[~ground].mean()), "max": float(z[~ground].max())} \
                if (~ground).any() else None
            out["swath_m"] = float(np.ptp(xy[:, 1 if abs(r_wb[0, 0]) < 0.5 else 0]))

    # Camera geometry and overlay.
    c = cfg["camera"]
    info = cs.get("camera_info") or {}
    out["camera_K_ok"] = bool(info) and np.allclose(np.array(info["K"]),
                                                     [[c["fx"], 0, c["cx"]], [0, c["fy"], c["cy"]], [0, 0, 1]], atol=1e-2)
    out["camera_size_ok"] = info.get("width") == c["width"] and info.get("height") == c["height"]
    cam_h = p_wb[2] + (r_wb @ np.array(c["t_BC"]))[2]
    out["gsd_ground_cm"] = float(cam_h * 35.7e-3 / 35e-3 / c["width"] * 100)
    out["footprint_ground_m"] = [float(2 * cam_h * math.tan(math.radians(c["hfov_deg"] / 2))),
                                 float(2 * cam_h * math.tan(math.radians(c["vfov_deg"] / 2)))]
    imgs = sorted(glob.glob(os.path.join(d, "camera", "img_*.png")))
    if imgs:
        img = cv2.imread(imgs[-1])
        out["image_mean_rgb"] = img.reshape(-1, 3)[:, ::-1].mean(axis=0).round(1).tolist()
        # Tree tops in view.
        if truth is not None and world != "flat_static":
            tw = np.stack([truth.trees["x_m"], truth.trees["y_m"], truth.trees["height_m"]], axis=1)
            u, v, zc = project(cfg, tw, p_wb, r_wb)
            inside = (zc > 0) & (u >= 0) & (u < c["width"]) & (v >= 0) & (v < c["height"])
            out["trees_in_image"] = int(inside.sum())
            # How well the image agrees with the tree tops: mean green-ness at the tops vs. random points.
            ui, vi = u[inside].astype(int), v[inside].astype(int)
            g = img[vi, ui].astype(float)
            rng = np.random.default_rng(0)
            ru, rv = rng.integers(0, c["width"], 4000), rng.integers(0, c["height"], 4000)
            gr = img[rv, ru].astype(float)
            out["crown_contrast"] = {"tops_mean_bgr": g.mean(axis=0).round(1).tolist(),
                                     "random_mean_bgr": gr.mean(axis=0).round(1).tolist()}
            for a, b in zip(ui, vi):
                cv2.circle(img, (int(a), int(b)), 6, (0, 0, 255), 2)
        # LiDAR line, coloured by height (blue ground -> yellow top of canopy).
        last = np.isfinite(r[-1])
        if last.any():
            u, v, zc = project(cfg, pts[-1][last], p_wb, r_wb)
            zz = pts[-1][last][:, 2]
            col = cv2.applyColorMap(np.clip(zz / 30 * 255, 0, 255).astype(np.uint8).reshape(-1, 1), cv2.COLORMAP_VIRIDIS)
            for a, b, cc in zip(u, v, col[:, 0]):
                if 0 <= a < c["width"] and 0 <= b < c["height"]:
                    cv2.circle(img, (int(a), int(b)), 3, tuple(int(x) for x in cc), -1)
            out["lidar_line_v_px"] = {"mean": float(np.mean(v)), "cy": c["cy"]}
            out["lidar_line_u_px"] = [float(u.min()), float(u.max())]
        small = cv2.resize(img, (c["width"] // 3, c["height"] // 3), interpolation=cv2.INTER_AREA)
        cv2.imwrite(os.path.join(run, f"overlay_h{h}.jpg"), small, [cv2.IMWRITE_JPEG_QUALITY, 85])
        crop = img[int(c["cy"]) - 300:int(c["cy"]) + 300, int(c["cx"]) - 500:int(c["cx"]) + 500]
        cv2.imwrite(os.path.join(run, f"overlay_h{h}_centre.jpg"), crop, [cv2.IMWRITE_JPEG_QUALITY, 90])
    return out, per_angle, beta


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run")
    args = ap.parse_args()
    cfg = json.load(open(os.path.join(args.run, "sensors.json")))
    dirs = sorted(glob.glob(os.path.join(args.run, "h*")), key=lambda p: float(os.path.basename(p)[1:]))
    world = json.load(open(os.path.join(dirs[0], "run.json")))["world"]
    truth = Truth() if world != "flat_static" else None
    rows, curves = [], []
    for d in dirs:
        o, pa, beta = analyse(args.run, d, cfg, truth, world)
        rows.append(o)
        curves.append((o["height_m"], pa))

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 2, figsize=(11, 4))
    for h, pa in curves:
        ax[0].plot(np.degrees(beta), 100 * pa, label=f"{h:g} m")
    ax[0].set_xlabel("scan angle beta (deg, + left)")
    ax[0].set_ylabel("rays with a return (%)")
    ax[0].set_ylim(-5, 105)
    ax[0].legend(fontsize=8)
    ax[0].grid(alpha=0.3)
    hs = [o["height_m"] for o in rows]
    ax[1].plot(hs, [100 * o["return_ratio"] for o in rows], "o-", label="measured, all rays")
    ax[1].plot(hs, [100 * o["ground_in_range_ratio_theory"] for o in rows], "k--", label="ground within max range (theory)")
    if world != "flat_static":
        ax[1].plot(hs, [100 * o.get("ground_return_ratio", 0) * o["return_ratio"] for o in rows], "s-",
                   label="ground returns, all rays")
    ax[1].axvline(cfg["lidar"]["range_max_m"], color="r", alpha=0.4, lw=1)
    ax[1].set_xlabel("height above ground (m)")
    ax[1].set_ylabel("%")
    ax[1].set_ylim(-5, 105)
    ax[1].legend(fontsize=8)
    ax[1].grid(alpha=0.3)
    fig.suptitle(f"{world}: LiDAR returns, max range {cfg['lidar']['range_max_m']:g} m")
    fig.tight_layout()
    fig.savefig(os.path.join(args.run, "lidar_returns.png"), dpi=110)

    with open(os.path.join(args.run, "report.json"), "w") as f:
        json.dump(rows, f, indent=2)
    lines = [f"# Static run {os.path.basename(args.run)} ({world})", "",
             "| H (m) | lines/sim-s | gaps | RTF | returns | nadir | edge ±20° | ground | range min–max (m) | "
             "camera frames | K ok |", "|---:|---:|---:|---:|---:|---:|---:|---:|---|---:|:-:|"]
    for o in rows:
        ls = o["lidar_node"]
        rng = f"{ls['range_min_m']:.2f}–{ls['range_max_m']:.2f}" if ls["range_min_m"] else "–"
        g = f"{100 * o['ground_return_ratio']:.1f} %" if "ground_return_ratio" in o else "–"
        lines.append(f"| {o['height_m']:g} | {ls['lines_per_sim_s']:.1f} | {ls['gaps']} | {ls['real_time_factor']:.2f} | "
                     f"{100 * o['return_ratio']:.1f} % | {100 * o['return_ratio_nadir']:.0f} % | "
                     f"{100 * o['return_ratio_edge']:.0f} % | {g} | {rng} | {o['camera_node']['frames']} | "
                     f"{'✓' if o['camera_K_ok'] and o['camera_size_ok'] else '✗'} |")
    if world == "flat_static":
        lines += ["", "Range error against z_lidar / cos(beta); nominal angles vs. the calibrated table (config/lidar_angles.json).",
                  "", "| H (m) | nominal: std (m) | nominal: max abs | calibrated: mean | std | max abs |",
                  "|---:|---:|---:|---:|---:|---:|"]
        for o in rows:
            e, e0 = o.get("range_error_m"), o.get("range_error_nominal_angles_m")
            if e:
                lines.append(f"| {o['height_m']:g} | {e0['std']:.4f} | {e0['max_abs']:.4f} | {e['mean']:+.4f} | "
                             f"{e['std']:.4f} | {e['max_abs']:.4f} |")
    with open(os.path.join(args.run, "report.md"), "w") as f:
        f.write("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
