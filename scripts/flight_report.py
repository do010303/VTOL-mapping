#!/usr/bin/env python3
"""Report of a survey flight (./run_sitl.sh + scripts/vtol_mission.py): point cloud and images against the scene.

  * Every LiDAR line is georeferenced with the vehicle pose interpolated to the line's time
    (p_W = T_WB(t) x T_BL x p_L, spec 5). Gazebo renders a whole line at one instant, so all rays of a
    line share that time (the 1/600000 s pulse spacing of the spec is not applied here).
  * On the survey line (inside the forest, wings level): ground returns against the DTM, all returns against
    the DSM, DSM from the cloud against the DSM of the scene, density, swath, speed, attitude, cross-track.
  * Images: spacing, forward overlap, and tree tops projected into one image (overlay_flight.jpg).
Writes <run>/flight_report.json, flight_report.md, cloud_line.npz (x, y, z, t of the on-line returns), figures.

  python3 scripts/flight_report.py out/flight_500
"""

import argparse
import json
import math
import os

import cv2
import numpy as np

from scene import Truth, scene_of_run
from static_report import image_file, lidar_dirs, load_scans, project, quat_to_matrix
from strips import segment, strip_of_time


def interp_poses(traj, t):
    """Position and rotation matrices at times t (linear / normalised-linear between ~1 ms samples)."""
    i = np.clip(np.searchsorted(traj[:, 0], t), 1, len(traj) - 1)
    t0, t1 = traj[i - 1, 0], traj[i, 0]
    a = ((t - t0) / np.maximum(t1 - t0, 1e-9))[:, None]
    p = (1 - a) * traj[i - 1, 1:4] + a * traj[i, 1:4]
    q0, q1 = traj[i - 1, 4:8], traj[i, 4:8]
    q1 = np.where((np.sum(q0 * q1, axis=1) < 0)[:, None], -q1, q1)
    q = (1 - a) * q0 + a * q1
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    x, y, z, w = q.T
    r = np.empty((len(t), 3, 3))
    r[:, 0, 0] = 1 - 2 * (y * y + z * z); r[:, 0, 1] = 2 * (x * y - w * z); r[:, 0, 2] = 2 * (x * z + w * y)
    r[:, 1, 0] = 2 * (x * y + w * z); r[:, 1, 1] = 1 - 2 * (x * x + z * z); r[:, 1, 2] = 2 * (y * z - w * x)
    r[:, 2, 0] = 2 * (x * z - w * y); r[:, 2, 1] = 2 * (y * z + w * x); r[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return p, r


def lidar_source(run, choice="auto"):
    """(sub-directory, exact): lidar_exact/ (scripts/lidar_exact.py: per-ray emission times, ray casting on the
    scene mesh) when it exists or is asked for, else lidar/ (the Gazebo gpu_lidar recording)."""
    d = os.path.join(run, "lidar_exact")
    complete = os.path.exists(os.path.join(d, "stats.json"))       # written last: a stopped run has none
    if choice == "exact" and not complete:
        raise SystemExit(f"{d} is missing or incomplete: run  python3 scripts/lidar_exact.py {run}  to the end")
    if choice == "auto" and os.path.isdir(d) and not complete:
        print(f"warning: {d} is incomplete (lidar_exact.py stopped early), using the Gazebo recording", flush=True)
    if choice == "exact" or (choice == "auto" and complete):
        return "lidar_exact", True
    return "lidar", False


def world_points(cfg, t, r, traj, exact, calibrated=True, p=None, rot=None, chunk=None):
    """Yields (sl, xyz, t_ray, hit) per chunk of scan lines: the returns in world coordinates,
    p_W = T_WB(t) T_BL p_L (spec 5). Gazebo recording: every ray of a line at the line's time t_s and the line's
    pose (p, rot), with the calibrated directions. Exact recording: ray k at t_s + k * pulse_spacing_s with the
    pose interpolated at that time, nominal directions (the ray caster used exactly those)."""
    lc = cfg["lidar"]
    d_l, _ = lidar_dirs(cfg, calibrated=calibrated and not exact)
    d_b = d_l @ np.array(lc["R_BL"]).T
    t_bl = np.array(lc["t_BL"])
    n = r.shape[1]
    dt = lc["pulse_spacing_s"] * np.arange(n) if exact else np.zeros(n)
    chunk = chunk or (300 if exact else 2000)
    for a in range(0, len(t), chunk):
        sl = slice(a, a + chunk)
        rr = r[sl]
        hit = np.isfinite(rr)
        tk = t[sl, None] + dt[None, :]
        if exact:
            pk, rk = interp_poses(traj, tk.ravel())
            o = pk + rk @ t_bl
            d = np.einsum("mij,mj->mi", rk, np.tile(d_b, (len(rr), 1)))
            pts = (o + np.where(hit, rr, 0.0).reshape(-1, 1) * d).reshape(len(rr), n, 3)
        else:
            pp, ro = (p[sl], rot[sl]) if p is not None else interp_poses(traj, t[sl])
            o = pp + np.einsum("nij,j->ni", ro, t_bl)
            d_w = np.einsum("nij,kj->nki", ro, d_b)
            pts = o[:, None, :] + np.where(hit, rr, 0.0)[..., None] * d_w
        yield sl, pts[hit], tk[hit], hit


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run")
    ap.add_argument("--y-range", type=float, nargs=2, default=(-650, 650), help="the survey lines' measured part")
    ap.add_argument("--planned-x", type=float, nargs="*", help="x of the planned lines (cross-track reference)")
    ap.add_argument("--eval", type=float, nargs=4, default=(-400, 400, -400, 400), metavar=("XMIN", "XMAX", "YMIN", "YMAX"),
                    help="evaluation area for coverage and DSM (spec 6/7: central 800 x 800 m)")
    ap.add_argument("--no-calibration", action="store_true", help="nominal scan angles instead of lidar_angles.json")
    ap.add_argument("--tag", default="", help="appended to the output names (flight_report_exact_<tag>.md ...), "
                                              "to keep several reports of one flight side by side")
    ap.add_argument("--lidar", choices=("auto", "gz", "exact"), default="auto",
                    help="auto: lidar_exact/ if it exists (scripts/lidar_exact.py), else the Gazebo recording")
    args = ap.parse_args()
    run = args.run
    cfg = json.load(open(os.path.join(run, "sensors.json")))
    cc = cfg["camera"]
    truth = Truth(scene_of_run(run))
    traj = np.loadtxt(os.path.join(run, "trajectory.csv"), delimiter=",", skiprows=1)
    traj = traj[np.concatenate([[True], np.diff(traj[:, 0]) > 0])]
    lidar_sub, exact = lidar_source(run, args.lidar)
    sfx = ("_exact" if exact else "") + (f"_{args.tag}" if args.tag else "")   # Gazebo report: plain names
    t, r = load_scans(run, lidar_sub)
    out = {"scene": truth.scene, "lidar_source": lidar_sub, "lines_total": int(len(t)),
           "pose_rate_hz": float(1 / np.median(np.diff(traj[:, 0])))}

    p, rot = interp_poses(traj, t)
    sid, strips = segment(t, p, rot, args.y_range, planned_x=args.planned_x)
    if not strips:
        raise SystemExit("no survey strip found")
    for st in strips:
        idx = np.arange(st["first"], st["last"] + 1)
        pp, rr = p[idx], rot[idx]
        ct = pp[:, 0] - st["x_nominal"]
        roll = np.degrees(np.arcsin(rr[:, 2, 1]))
        st.update({"lines": int(len(idx)), "duration_s": float(t[idx[-1]] - t[idx[0]]),
                   "length_m": float(abs(pp[-1, 1] - pp[0, 1])),
                   "ground_speed_mps": float(abs(pp[-1, 1] - pp[0, 1]) / (t[idx[-1]] - t[idx[0]])),
                   "altitude_m": [float(pp[:, 2].min()), float(pp[:, 2].max())],
                   "cross_track_m": {"mean": float(ct.mean()), "max_abs": float(np.abs(ct).max())},
                   "roll_max_abs_deg": float(np.abs(roll).max())})
    out["strips"] = strips
    on = sid > 0
    t, r, p, rot = t[on], r[on], p[on], rot[on]
    dur = sum(st["duration_s"] for st in strips)
    out["line"] = {
        "strips": len(strips), "lines": int(on.sum()), "duration_s": dur,
        "length_m": float(sum(st["length_m"] for st in strips)),
        "ground_speed_mps": float(np.mean([st["ground_speed_mps"] for st in strips])),
        "line_spacing_m": float(np.mean([st["length_m"] / (st["lines"] - 1) for st in strips])),
        "altitude_m": {"mean": float(p[:, 2].mean()), "min": float(p[:, 2].min()), "max": float(p[:, 2].max())},
        "cross_track_m": {"max_abs": float(max(st["cross_track_m"]["max_abs"] for st in strips))},
        "roll_deg": {"max_abs": float(np.abs(np.degrees(np.arcsin(rot[:, 2, 1]))).max())},
        "pitch_deg": {"mean": float(np.degrees(-np.arcsin(rot[:, 2, 0])).mean())},
    }
    out["calibrated_angles"] = bool(not exact and not args.no_calibration and
                                    os.path.exists(os.path.join(os.path.dirname(__file__), "..", "config", "lidar_angles.json")))
    hit_n = int(np.isfinite(r).sum())
    xyz = np.empty((hit_n, 3), dtype=np.float32)                   # 10 strips: ~130 M points
    tt = np.empty(hit_n)
    k0 = 0
    for sl, pts, tk, hit in world_points(cfg, t, r, traj, exact, not args.no_calibration, p, rot):
        xyz[k0:k0 + len(pts)] = pts
        tt[k0:k0 + len(pts)] = tk
        k0 += len(pts)
    hit = np.isfinite(r)
    out["returns"] = {"rays": int(hit.size), "returns": int(hit.sum()), "ratio": float(hit.mean()),
                      "per_s": float(hit.sum() / dur)}

    inside = (np.abs(xyz[:, 0]) < truth.x[-1]) & (np.abs(xyz[:, 1]) < truth.y[-1])
    xyz, tt = xyz[inside], tt[inside]
    z = xyz[:, 2]
    # Per point, in blocks (10 strips: ~120 M points; whole-array float64 temporaries need > 10 GB).
    dsm = np.empty(len(z), np.float32)
    zr = np.empty(len(z), np.float32)                                 # height above the terrain
    for a in range(0, len(z), 5_000_000):
        b = slice(a, a + 5_000_000)
        dsm[b] = truth.dsm_at(xyz[b, 0], xyz[b, 1])
        zr[b] = z[b] - truth.dtm_at(xyz[b, 0], xyz[b, 1])
    ground = zr < 0.5
    out["ground"] = {"ratio": float(ground.mean()), "z_minus_dtm_mean_m": float(zr[ground].mean()),
                     "z_minus_dtm_std_m": float(zr[ground].std()),
                     "z_minus_dtm_abs_p99_m": float(np.percentile(np.abs(zr[ground]), 99)),
                     "z_minus_dtm_abs_max_m": float(np.abs(zr[ground]).max())}
    res = z - dsm
    out["returns_vs_dsm"] = {"above_dsm_by_0.5m": float(np.mean(res > 0.5)), "above_dsm_by_1m": float(np.mean(res > 1.0)),
                             "veg_median_m": float(np.median(res[~ground]))}
    sub = np.random.default_rng(0).choice(len(z), min(len(z), 1000000), replace=False)
    ex = z[sub] - truth.surface_at(xyz[sub, 0], xyz[sub, 1])
    top = np.abs(ex) < 0.5                               # returns from the top surface (the rest: crown sides, trunks)
    out["returns_vs_surface"] = {"above_by_5cm": float(np.mean(ex > 0.05)), "above_by_20cm": float(np.mean(ex > 0.20)),
                                 "max_above_m": float(ex.max()), "top_surface_ratio": float(top.mean()),
                                 "top_surface_mean_m": float(ex[top].mean()), "top_surface_std_m": float(ex[top].std())}
    first = (tt >= strips[0]["t0"]) & (tt <= strips[0]["t1"])
    out["swath_m"] = float(np.percentile(xyz[first, 0], 99.9) - np.percentile(xyz[first, 0], 0.1))

    # DSM / DTM from the cloud on the truth's 1 m grid.
    cell = truth.cell
    n = len(truth.x)
    flat = np.empty(len(z), np.int32)               # cell index i * n + j (int32: 10 strips ~ 120 M points)
    for a in range(0, len(z), 5_000_000):
        b = slice(a, a + 5_000_000)
        jj = np.round((xyz[b, 0] - truth.x[0]) / cell).astype(np.int32)
        ii = np.round((xyz[b, 1] - truth.y[0]) / cell).astype(np.int32)
        flat[b] = ii * n + jj
    dsm_c = np.full(n * n, -np.inf)
    np.maximum.at(dsm_c, flat, z)
    dtm_c = np.full(n * n, np.inf)                 # lowest return above the terrain, per cell
    np.minimum.at(dtm_c, flat, zr)
    cnt = np.bincount(flat, minlength=n * n)
    seen = cnt > 0
    ex0, ex1, ey0, ey1 = args.eval
    xs = [st["x_nominal"] for st in strips]
    half = out["swath_m"] / 2 - 5
    # Evaluation area: the spec's central area, cut to what the strips can cover.
    bx0, bx1 = max(ex0, min(xs) - half), min(ex1, max(xs) + half)
    by0, by1 = max(ey0, args.y_range[0] + 10), min(ey1, args.y_range[1] - 10)
    box = np.zeros((n, n), bool)
    box[np.ix_((truth.y >= by0) & (truth.y <= by1), (truth.x >= bx0) & (truth.x <= bx1))] = True
    out["eval_area_m"] = [float(bx0), float(bx1), float(by0), float(by1)]
    out["density_pts_m2"] = float(cnt.reshape(n, n)[box].mean())
    out["coverage_1m"] = float((cnt.reshape(n, n)[box] > 0).mean())
    out["points_per_cell_p05"] = float(np.percentile(cnt.reshape(n, n)[box], 5))
    dens = np.where(box, cnt.reshape(n, n), np.nan)
    box = box.ravel()
    e = (dsm_c - truth.dsm.ravel())[seen & box]
    out["dsm_1m"] = {"cells_with_data": float((seen & box).sum() / box.sum()), "bias_m": float(e.mean()),
                     "rmse_m": float(np.sqrt((e ** 2).mean())), "median_abs_m": float(np.median(np.abs(e))),
                     "within_1m": float(np.mean(np.abs(e) < 1.0))}
    g = dtm_c[seen & box]
    out["dtm_lowest_point_1m"] = {"cells_reaching_ground": float(np.mean(g < 0.5)),
                                  "rmse_of_those_m": float(np.sqrt((g[g < 0.5] ** 2).mean()))}
    # 5 m cells: is there a ground return in each (enough for a DTM under canopy)?
    k = 5
    g5 = (dtm_c.reshape(n, n) < 0.5)[:n // k * k, :n // k * k].reshape(n // k, k, n // k, k).any(axis=(1, 3))
    b5 = box.reshape(n, n)[:n // k * k, :n // k * k].reshape(n // k, k, n // k, k).all(axis=(1, 3))
    out["dtm_5m_cells_with_ground"] = float(g5[b5].mean())
    if len(xyz) <= 30_000_000:                     # bigger runs: use export_dataset.py (LAS per strip)
        np.savez_compressed(os.path.join(run, f"cloud_line{sfx}.npz"), xyz=xyz.astype(np.float32), t=tt)

    # Images.
    im = np.genfromtxt(os.path.join(run, "camera", "images.csv"), delimiter=",", names=True, dtype=None, encoding=None)
    si = strip_of_time(strips, im["t_s"])
    on_i = si > 0
    sp = np.concatenate([np.hypot(np.diff(im["x_m"][si == st["id"]]), np.diff(im["y_m"][si == st["id"]]))
                         for st in strips if (si == st["id"]).sum() > 1])
    agl = im["z_m"][on_i] - truth.dtm_at(im["x_m"][on_i], im["y_m"][on_i])
    h_canopy = agl.mean() - 30
    along = 2 * h_canopy * math.tan(math.radians(cc["vfov_deg"] / 2))
    lat = im["t_s"] - im["trigger_t_s"]
    across = 2 * h_canopy * math.tan(math.radians(cc["hfov_deg"] / 2))
    side = float(1 - np.mean(np.diff(sorted(xs))) / across) if len(xs) > 1 else None
    out["images"] = {"total": int(len(im)), "on_line": int(on_i.sum()), "side_overlap_at_30m_canopy": side,
                     "spacing_m": {"mean": float(sp.mean()), "min": float(sp.min()), "max": float(sp.max())},
                     "forward_overlap_at_30m_canopy": float(1 - sp.mean() / along),
                     "trigger_to_image_s": {"mean": float(np.nanmean(lat)), "max": float(np.nanmax(lat))},
                     "gsd_ground_cm": float(agl.mean() * 35.7 / 35.0 / cc["width"] * 100),
                     "agl_m": {"mean": float(agl.mean()), "min": float(agl.min()), "max": float(agl.max())}}
    k0 = np.nonzero(si == strips[0]["id"])[0]
    k = k0[len(k0) // 2]
    img = cv2.imread(os.path.join(run, "camera", image_file(im, k)))
    p_wb = np.array([im["x_m"][k], im["y_m"][k], im["z_m"][k]])
    r_wb = quat_to_matrix(im["qx"][k], im["qy"][k], im["qz"][k], im["qw"][k])
    tw = np.stack([truth.trees["x_m"], truth.trees["y_m"], truth.trees["height_m"] + truth.tree_z0], axis=1)
    u, v, zc = project(cfg, tw, p_wb, r_wb)
    ok = (zc > 0) & (u >= 0) & (u < cc["width"]) & (v >= 0) & (v < cc["height"])
    for a, b in zip(u[ok], v[ok]):
        cv2.circle(img, (int(a), int(b)), 6, (0, 0, 255), 2)
    # The LiDAR returns of the 0.25 s around the exposure, by height.
    near = np.abs(tt - im["t_s"][k]) < 0.125
    u, v, zc = project(cfg, xyz[near].astype(np.float64), p_wb, r_wb)
    col = cv2.applyColorMap(np.clip(zr[near] / 30 * 255, 0, 255).astype(np.uint8).reshape(-1, 1), cv2.COLORMAP_VIRIDIS)
    for a, b, c in zip(u[::3], v[::3], col[::3, 0]):
        if 0 <= a < cc["width"] and 0 <= b < cc["height"]:
            cv2.circle(img, (int(a), int(b)), 2, tuple(int(x) for x in c), -1)
    out["overlay_image"] = {"file": image_file(im, k), "trees_in_image": int(ok.sum())}
    cy, cx = int(cc["cy"]), int(cc["cx"])
    cv2.imwrite(os.path.join(run, f"overlay_flight_centre{sfx}.jpg"), img[cy - 350:cy + 350, cx - 600:cx + 600],
                [cv2.IMWRITE_JPEG_QUALITY, 90])
    cv2.imwrite(os.path.join(run, f"overlay_flight{sfx}.jpg"),
                cv2.resize(img, (cc["width"] // 3, cc["height"] // 3), interpolation=cv2.INTER_AREA))

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 4, figsize=(21, 6.5))
    ext = [truth.x[0], truth.x[-1], truth.y[0], truth.y[-1]]
    a = np.where(seen, dsm_c, np.nan).reshape(n, n)
    for axx, data, title in ((ax[0], truth.dsm, "DSM of the scene (m)"), (ax[1], a, "DSM from the LiDAR cloud (m)")):
        m = axx.imshow(data, origin="lower", extent=ext, vmin=0, vmax=30, cmap="viridis")
        axx.set_title(title)
    m2 = ax[2].imshow(dens, origin="lower", extent=ext, vmin=0, vmax=np.nanpercentile(dens, 99), cmap="magma")
    ax[2].set_title("LiDAR points per 1 m cell (eval area)")
    fig.colorbar(m2, ax=ax[2], shrink=0.6)
    for axx in ax[:3]:
        axx.set_xlim(bx0 - 30, bx1 + 30); axx.set_ylim(by0 - 30, by1 + 30)
        axx.set_xlabel("x east (m)"); axx.set_ylabel("y north (m)")
    fig.colorbar(m, ax=ax[:2], shrink=0.6)
    ax[3].hist(e, bins=np.linspace(-5, 5, 101), color="#3b7dd8")
    ax[3].set_yscale("log"); ax[3].set_xlabel("DSM cloud − DSM scene, 1 m cells (m)"); ax[3].set_title("DSM error")
    fig.savefig(os.path.join(run, f"flight_dsm{sfx}.png"), dpi=90, bbox_inches="tight")

    json.dump(out, open(os.path.join(run, f"flight_report{sfx}.json"), "w"), indent=2)
    L, G, D, I = out["line"], out["ground"], out["dsm_1m"], out["images"]
    md = f"""# Flight {os.path.basename(os.path.abspath(run))}

Scene `{truth.scene}`; LiDAR from `{lidar_sub}/` ({'per-ray emission times, ray casting on the scene mesh' if exact else 'Gazebo gpu_lidar, one instant per line'}).

| | |
|---|---|
| Survey | {L['strips']} strip(s), {L['length_m']:.0f} m in {L['duration_s']:.1f} s, {L['ground_speed_mps']:.2f} m/s, altitude {L['altitude_m']['min']:.1f}–{L['altitude_m']['max']:.1f} m |
| Tracking | cross-track max {L['cross_track_m']['max_abs']:.2f} m, roll max {L['roll_deg']['max_abs']:.2f}°, pitch mean {L['pitch_deg']['mean']:.2f}° |
| LiDAR lines | {L['lines']} ({L['lines'] / L['duration_s']:.1f}/s), {L['line_spacing_m'] * 100:.1f} cm apart along track |
| Returns | {out['returns']['returns']:,} of {out['returns']['rays']:,} rays ({100 * out['returns']['ratio']:.2f} %), {out['returns']['per_s']:,.0f}/s |
| Swath / density | {out['swath_m']:.1f} m; {out['density_pts_m2']:.1f} points/m² over the eval area {out['eval_area_m']}, {100 * out['coverage_1m']:.2f} % of its 1 m cells hit, 5 % of cells < {out['points_per_cell_p05']:.0f} points |
| Ground returns | {100 * G['ratio']:.1f} % of returns; z − DTM = {G['z_minus_dtm_mean_m'] * 100:+.1f} ± {G['z_minus_dtm_std_m'] * 100:.1f} cm, p99 {G['z_minus_dtm_abs_p99_m'] * 100:.1f} cm (truth: 0) |
| Returns vs. exact scene surface | {100 * out['returns_vs_surface']['above_by_5cm']:.3f} % above it by > 5 cm, {100 * out['returns_vs_surface']['above_by_20cm']:.3f} % by > 20 cm (max {out['returns_vs_surface']['max_above_m']:.2f} m); on the top surface: {out['returns_vs_surface']['top_surface_mean_m'] * 100:+.1f} ± {out['returns_vs_surface']['top_surface_std_m'] * 100:.1f} cm |
| Returns above the 1 m DSM (cell maximum) | {100 * out['returns_vs_dsm']['above_dsm_by_0.5m']:.2f} % by > 0.5 m, {100 * out['returns_vs_dsm']['above_dsm_by_1m']:.2f} % by > 1 m |
| DSM 1 m | {100 * D['cells_with_data']:.1f} % of cells; bias {D['bias_m']:+.2f} m, RMSE {D['rmse_m']:.2f} m, median abs {D['median_abs_m']:.2f} m, {100 * D['within_1m']:.1f} % within 1 m |
| Ground reached | {100 * out['dtm_lowest_point_1m']['cells_reaching_ground']:.1f} % of 1 m cells, {100 * out['dtm_5m_cells_with_ground']:.1f} % of 5 m cells |
| Images | {I['on_line']} on the line, spacing {I['spacing_m']['mean']:.2f} m ({I['spacing_m']['min']:.2f}–{I['spacing_m']['max']:.2f}), forward overlap {100 * I['forward_overlap_at_30m_canopy']:.1f} %{'' if I['side_overlap_at_30m_canopy'] is None else f", side overlap {100 * I['side_overlap_at_30m_canopy']:.1f} %"} at a 30 m canopy, GSD {I['gsd_ground_cm']:.1f} cm |
| Trigger → image | {I['trigger_to_image_s']['mean'] * 1000:.1f} ms mean, {I['trigger_to_image_s']['max'] * 1000:.1f} ms max |

| Strip | Dir | x (m) | Lines | Length (m) | Speed (m/s) | Altitude (m) | Cross-track mean / max (m) | Roll max (°) |
|---:|:-:|---:|---:|---:|---:|---|---|---:|
""" + "\n".join(f"| {st['id']} | {st['dir']} | {st['x_nominal']:+.1f} | {st['lines']} | {st['length_m']:.0f} | "
                 f"{st['ground_speed_mps']:.2f} | {st['altitude_m'][0]:.1f}–{st['altitude_m'][1]:.1f} | "
                 f"{st['cross_track_m']['mean']:+.2f} / {st['cross_track_m']['max_abs']:.2f} | {st['roll_max_abs_deg']:.2f} |"
                 for st in strips) + "\n"
    open(os.path.join(run, f"flight_report{sfx}.md"), "w").write(md)
    print(md)


if __name__ == "__main__":
    main()
