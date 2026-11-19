#!/usr/bin/env python3
"""Spec 9.6: ground filtering (CSF and PTD), DTM / DSM / CHM from the point cloud, scored against the scene's truth,
for one run or several (flat and sloping scenes side by side in one report).

  python3 scripts/ground_eval.py out/main_flat out/main_slope          # both scenes, one report
  python3 scripts/ground_eval.py out/main_flat --filter-density 5      # faster

Per run (the survey lines' returns inside the evaluation area, spec 6/7: central 800 x 800 m):
  truth labels    ground = within 5 cm of the exact terrain (scripts/scene.py dtm_at), the rest vegetation
  CSF             cloth simulation filter (Zhang et al. 2016; package cloth-simulation-filter)
  PTD             progressive TIN densification (Axelsson 2000): lowest point per seed cell, then per TIN triangle
                  the candidate closest to it is added while its distance and angles stay under the thresholds
  filters run on a random subsample (--filter-density points/m^2; the spec asks to record any decimation), so
  precision / recall / F1 of the ground class are on that subsample
  DTM             1 m grid, linear TIN of each filter's ground points (nearest outside their hull)
  DSM             1 m grid, highest return per cell (all returns)
  CHM             DSM - DTM
  truth DSM       highest exact surface (crowns from the tree list, terrain) over 4 x 4 samples per cell
                  (a single sample at the cell centre would sit ~0.5 m under the cloud's per-cell maximum)
Scores: bias, RMSE, MAE, |error| p95, share within 0.5 m, share of cells with data (cells holding >= 1 ground point
for the DTM, >= 1 return for the DSM).

Writes <run>/ground_eval.json, <run>/ground_eval.png (error maps), and --report (default out/ground_eval.md).
"""
import argparse
import json
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from flight_report import interp_poses, lidar_source, world_points  # noqa: E402
from scene import Truth, scene_of_run  # noqa: E402
from static_report import ROOT, load_scans  # noqa: E402
from strips import segment  # noqa: E402
import fastcore  # noqa: E402

GROUND_TOL = 0.05


# ---------------------------------------------------------------- data

def load_points(run, a):
    cfg = json.load(open(os.path.join(run, "sensors.json")))
    traj = np.loadtxt(os.path.join(run, "trajectory.csv"), delimiter=",", skiprows=1)
    traj = traj[np.concatenate([[True], np.diff(traj[:, 0]) > 0])]
    sub, exact = lidar_source(run, "auto")
    t, r = load_scans(run, sub)
    p, rot = interp_poses(traj, t)
    sid, strips = segment(t, p, rot, a.y_range, planned_x=a.planned_x)
    on = sid > 0
    t, r, p, rot = t[on], r[on], p[on], rot[on]
    x0, x1, y0, y1 = a.eval
    m = a.margin
    parts = []
    for _, pts, _, _ in world_points(cfg, t, r, traj, exact, True, p, rot):
        k = (pts[:, 0] >= x0 - m) & (pts[:, 0] <= x1 + m) & (pts[:, 1] >= y0 - m) & (pts[:, 1] <= y1 + m)
        parts.append(pts[k].astype(np.float32))
    return np.concatenate(parts), {"lidar_source": sub, "strips": len(strips)}


def truth_dsm(truth, xs, ys, k=4):
    """Highest exact surface in each 1 m cell (cells centred on xs, ys). Crowns are cones (up to 16 m tall on a
    2.5 m radius): a grid of samples misses their peaks, so per tree and per cell within its crown the point of the
    cell closest to the apex is sampled too (where a cone is highest in the cell), plus k x k samples per cell for
    the terrain and the crown edges."""
    out = np.full((len(ys), len(xs)), -np.inf)
    off = (np.arange(k) + 0.5) / k - 0.5
    gx, gy = np.meshgrid(xs, ys)
    for dx in off:
        for dy in off:
            z = truth.surface_at((gx + dx).ravel(), (gy + dy).ravel()).reshape(gx.shape)
            np.maximum(out, z, out=out)
    tr = truth.trees
    sel = (tr["x_m"] > xs[0] - 6) & (tr["x_m"] < xs[-1] + 6) & (tr["y_m"] > ys[0] - 6) & (tr["y_m"] < ys[-1] + 6)
    ax, ay, rad = tr["x_m"][sel], tr["y_m"][sel], tr["crown_radius_m"][sel]
    R = int(np.ceil(rad.max())) + 1
    for di in range(-R, R + 1):
        for dj in range(-R, R + 1):
            j = np.round(ax - xs[0]).astype(np.int64) + dj
            i = np.round(ay - ys[0]).astype(np.int64) + di
            ok = (i >= 0) & (i < len(ys)) & (j >= 0) & (j < len(xs)) & (np.hypot(di, dj) <= rad + 1.5)
            if not ok.any():
                continue
            cx, cy = xs[j[ok]], ys[i[ok]]
            px = np.clip(ax[ok], cx - 0.4999, cx + 0.4999)          # the cell's point closest to the apex
            py = np.clip(ay[ok], cy - 0.4999, cy + 0.4999)
            z = truth.surface_at(px, py)
            np.maximum.at(out, (i[ok], j[ok]), z)
    return out


# ---------------------------------------------------------------- filters

def csf(xyz, slope):
    import CSF
    f = CSF.CSF()
    f.params.bSloopSmooth = True
    f.params.cloth_resolution = 1.0
    f.params.rigidness = 2 if slope else 3
    f.params.class_threshold = 0.5
    f.params.time_step = 0.65
    f.params.interations = 500
    f.setPointCloud(xyz.astype(np.float64))
    g, ng = CSF.VecInt(), CSF.VecInt()
    f.do_filtering(g, ng, False)
    lab = np.zeros(len(xyz), bool)
    lab[np.asarray(g, dtype=np.int64)] = True
    return lab


def ptd(xyz, seed_cell=25.0, max_dist=0.5, max_angle_deg=10.0, max_iter=30):
    """Progressive TIN densification (Axelsson style), batch form: per iteration every point whose distance to its
    TIN triangle and angles to the triangle's corners pass the thresholds joins the ground."""
    from scipy.spatial import Delaunay
    n = len(xyz)
    xy, z = xyz[:, :2].astype(np.float64), xyz[:, 2].astype(np.float64)
    # Seeds: the lowest point per seed cell.
    ij = np.floor((xy - xy.min(0)) / seed_cell).astype(np.int64)
    key = ij[:, 0] * 100000 + ij[:, 1]
    order = np.lexsort((z, key))
    first = np.concatenate([[True], key[order][1:] != key[order][:-1]])
    ground = np.zeros(n, bool)
    ground[order[first]] = True
    # The TIN only reaches the seeds' hull: seed the area's rim densely too (lowest per 5 m in a 5 m band).
    lo, hi = xy.min(0), xy.max(0)
    rim = np.any((xy - lo < 5) | (hi - xy < 5), axis=1)
    ri = np.nonzero(rim)[0]
    kr = np.floor((xy[ri] - lo) / 5).astype(np.int64)
    key = kr[:, 0] * 100000 + kr[:, 1]
    o = np.lexsort((z[ri], key))
    ground[ri[o][np.concatenate([[True], key[o][1:] != key[o][:-1]])]] = True
    tan_max = np.tan(np.radians(max_angle_deg))
    for it in range(max_iter):
        gi = np.nonzero(ground)[0]
        tri = Delaunay(xy[gi])
        cand = np.nonzero(~ground)[0]
        s = tri.find_simplex(xy[cand])
        ok = s >= 0
        cand, s = cand[ok], s[ok]
        v = gi[tri.simplices[s]]                                   # (m, 3) point indices of the triangle
        p0, p1, p2 = xyz[v[:, 0]].astype(np.float64), xyz[v[:, 1]].astype(np.float64), xyz[v[:, 2]].astype(np.float64)
        nrm = np.cross(p1 - p0, p2 - p0)
        nrm /= np.linalg.norm(nrm, axis=1, keepdims=True)
        q = xyz[cand].astype(np.float64)
        d = np.abs(np.einsum("ij,ij->i", q - p0, nrm))             # distance to the triangle's plane
        hd = np.stack([np.hypot(*(q[:, :2] - pk[:, :2]).T) for pk in (p0, p1, p2)], axis=1)
        dz = np.stack([np.abs(q[:, 2] - pk[:, 2]) for pk in (p0, p1, p2)], axis=1)
        ang_ok = np.all(dz <= tan_max * np.maximum(hd, 1e-6) + 0.05, axis=1)
        good = (d <= max_dist) & ang_ok
        if not good.any():
            break
        add = cand[good]
        ground[add] = True
        if len(add) < 0.001 * len(gi):
            break
    return ground


# ---------------------------------------------------------------- rasters and scores

def dtm_from(xyz, lab, xs, ys):
    from scipy.interpolate import LinearNDInterpolator, NearestNDInterpolator
    g = xyz[lab]
    # Thin to the lowest ground point per 0.5 m cell (the TIN of millions of points is slow and adds nothing).
    ij = np.floor((g[:, :2] - [xs[0] - 0.5, ys[0] - 0.5]) / 0.5).astype(np.int64)
    key = ij[:, 0] * 100000 + ij[:, 1]
    o = np.lexsort((g[:, 2], key))
    first = np.concatenate([[True], key[o][1:] != key[o][:-1]])
    g = g[o][first].astype(np.float64)
    gx, gy = np.meshgrid(xs, ys)
    lin = LinearNDInterpolator(g[:, :2], g[:, 2])(gx, gy)
    miss = ~np.isfinite(lin)
    if miss.any():
        lin[miss] = NearestNDInterpolator(g[:, :2], g[:, 2])(gx[miss], gy[miss])
    cells = cell_index(xyz[lab], xs, ys)
    has = np.zeros(gx.size, bool)
    has[cells[cells >= 0]] = True
    return lin, has.reshape(gx.shape)


def cell_index(xyz, xs, ys):
    j = np.round(xyz[:, 0] - xs[0]).astype(np.int64)
    i = np.round(xyz[:, 1] - ys[0]).astype(np.int64)
    ok = (i >= 0) & (i < len(ys)) & (j >= 0) & (j < len(xs))
    return np.where(ok, i * len(xs) + j, -1)


def dsm_from(xyz, xs, ys):
    c = cell_index(xyz, xs, ys)
    k = c >= 0
    out = fastcore.cell_reduce(c[k], xyz[k, 2], len(xs) * len(ys), "max").reshape(len(ys), len(xs))
    return out, np.isfinite(out)


def scores(est, ref, mask=None):
    e = (est - ref)
    e = e[mask] if mask is not None else e.ravel()
    e = e[np.isfinite(e)]
    return {"bias_m": float(e.mean()), "rmse_m": float(np.sqrt(np.mean(e ** 2))), "mae_m": float(np.abs(e).mean()),
            "abs_p95_m": float(np.percentile(np.abs(e), 95)), "within_0.5m": float(np.mean(np.abs(e) <= 0.5)),
            "cells": int(e.size)}


def classification(pred, truth):
    tp = int(np.sum(pred & truth))
    fp = int(np.sum(pred & ~truth))
    fn = int(np.sum(~pred & truth))
    tn = int(np.sum(~pred & ~truth))
    p = tp / max(tp + fp, 1)
    r = tp / max(tp + fn, 1)
    n = tp + fp + fn + tn
    po = (tp + tn) / n
    pe = ((tp + fp) * (tp + fn) + (fn + tn) * (fp + tn)) / n ** 2
    return {"precision": p, "recall": r, "f1": 2 * p * r / max(p + r, 1e-12),
            "type1_ground_rejected": fn / max(tp + fn, 1), "type2_veg_accepted": fp / max(fp + tn, 1),
            "kappa": (po - pe) / max(1 - pe, 1e-12), "tp": tp, "fp": fp, "fn": fn, "tn": tn}


# ---------------------------------------------------------------- one run

def evaluate(run, a):
    t0 = time.time()
    truth = Truth(scene_of_run(run))
    xyz, info = load_points(run, a)
    print(f"{run}: scene {truth.scene}, {len(xyz):,} returns in the area (+{a.margin:g} m), "
          f"{time.time() - t0:.0f} s", flush=True)
    x0, x1, y0, y1 = a.eval
    xs = np.arange(x0 + 0.5, x1, 1.0)
    ys = np.arange(y0 + 0.5, y1, 1.0)
    gx, gy = np.meshgrid(xs, ys)
    dtm_true = truth.dtm_at(gx.ravel(), gy.ravel()).reshape(gx.shape)
    dsm_true = truth_dsm(truth, xs, ys)
    chm_true = dsm_true - dtm_true
    print(f"   truth rasters {time.time() - t0:.0f} s", flush=True)

    dsm, dsm_has = dsm_from(xyz, xs, ys)
    area = (x1 - x0 + 2 * a.margin) * (y1 - y0 + 2 * a.margin)
    n_sub = min(len(xyz), int(a.filter_density * area))
    sub = xyz[np.sort(np.random.default_rng(0).choice(len(xyz), n_sub, replace=False))]
    lab_true = np.abs(sub[:, 2] - truth.dtm_at(sub[:, 0], sub[:, 1])) < GROUND_TOL
    del xyz
    out = {"run": os.path.basename(run.rstrip("/")), "scene": truth.scene, "flat": truth.flat, **info,
           "eval_area_m": list(a.eval), "margin_m": a.margin, "filter_subsample": {
               "points": int(n_sub), "points_per_m2": n_sub / area, "ground_share_truth": float(lab_true.mean())},
           "dsm": {**scores(dsm, dsm_true, dsm_has), "cells_with_data": float(dsm_has.mean())},
           "methods": {}}
    maps = {}
    for name, fn in (("truth labels", None), ("CSF", lambda: csf(sub, not truth.flat)), ("PTD", lambda: ptd(sub))):
        t1 = time.time()
        lab = lab_true if fn is None else fn()
        dtm, has = dtm_from(sub, lab, xs, ys)
        dtm_s = scores(dtm, dtm_true)
        dtm_s["cells_with_data"] = float(has.mean())
        dtm_s["with_data"] = scores(dtm, dtm_true, has)
        chm_s = scores(dsm - dtm, chm_true, dsm_has)
        res = {"dtm": dtm_s, "chm": chm_s, "seconds": time.time() - t1}
        if fn is not None:
            res["ground_class"] = classification(lab, lab_true)
        out["methods"][name] = res
        maps[name] = dtm - dtm_true
        cl = res.get("ground_class")
        print(f"   {name:12s} " + (f"P {cl['precision']:.3f} R {cl['recall']:.3f} F1 {cl['f1']:.3f}  " if cl else "")
              + f"DTM RMSE {dtm_s['rmse_m']:.2f} m bias {dtm_s['bias_m']:+.2f}  CHM RMSE {chm_s['rmse_m']:.2f} m  "
              f"{res['seconds']:.0f} s", flush=True)
    json.dump(out, open(os.path.join(run, "ground_eval.json"), "w"), indent=1)
    chm_err = np.where(dsm_has, dsm - maps["CSF"] - dtm_true - chm_true, np.nan)     # CSF CHM - truth CHM
    figure(run, out, maps, chm_err, (x0, x1, y0, y1))
    return out


def figure(run, out, maps, chm_err, ext):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 3, figsize=(17, 5.6))
    for a_, (title, m, lim) in zip(ax, (("DTM error, CSF (m)", maps["CSF"], 1.0), ("DTM error, PTD (m)", maps["PTD"], 1.0),
                                       ("CHM error, CSF DTM (m)", chm_err, 3.0))):
        im = a_.imshow(m, origin="lower", extent=ext, cmap="RdBu_r", vmin=-lim, vmax=lim)
        a_.set_title(title)
        a_.set_xlabel("x east (m)")
        a_.set_ylabel("y north (m)")
        fig.colorbar(im, ax=a_, shrink=0.8)
    fig.suptitle(f"{out['run']} ({out['scene']})")
    fig.tight_layout()
    fig.savefig(os.path.join(run, "ground_eval.png"), dpi=100)
    plt.close(fig)


def report(results, path):
    pct = lambda v: f"{100 * v:.1f} %"  # noqa: E731
    lines = ["# Spec 9.6: ground filtering, DTM / DSM / CHM", "",
             "Written by `scripts/ground_eval.py` " + " ".join(f"`out/{r['run']}`" for r in results) + ".", ""]
    hdr = "| | " + " | ".join(f"{r['run']} ({r['scene']})" for r in results) + " |"
    sep = "|---|" + "---:|" * len(results)
    row = lambda name, f: "| " + name + " | " + " | ".join(f(r) for r in results) + " |"  # noqa: E731
    lines += ["## Data", "", hdr, sep,
              row("LiDAR, survey lines", lambda r: f"{r['lidar_source']}, {r['strips']} lines"),
              row("Evaluation area", lambda r: f"{r['eval_area_m']} m (+{r['margin_m']:g} m margin)"),
              row("Filter subsample", lambda r: f"{r['filter_subsample']['points']:,} points, "
                                                f"{r['filter_subsample']['points_per_m2']:.1f} /m²"),
              row("Ground share (truth)", lambda r: pct(r["filter_subsample"]["ground_share_truth"])), ""]
    lines += ["## Ground class (subsample)", "", hdr, sep]
    for m in ("CSF", "PTD"):
        for k, name in (("precision", "precision"), ("recall", "recall"), ("f1", "F1"), ("kappa", "kappa"),
                        ("type1_ground_rejected", "type I (ground rejected)"),
                        ("type2_veg_accepted", "type II (vegetation accepted)")):
            lines.append(row(f"{m} {name}", lambda r, m=m, k=k: f"{r['methods'][m]['ground_class'][k]:.3f}"))
    lines += ["", "## DTM (1 m grid, all evaluation cells)", "", hdr, sep]
    for m in ("truth labels", "CSF", "PTD"):
        lines.append(row(f"{m}: bias / RMSE", lambda r, m=m: f"{r['methods'][m]['dtm']['bias_m']:+.3f} / "
                                                              f"{r['methods'][m]['dtm']['rmse_m']:.3f} m"))
        lines.append(row(f"{m}: \\|e\\| p95, within 0.5 m", lambda r, m=m: f"{r['methods'][m]['dtm']['abs_p95_m']:.2f} m, "
                                                                          f"{pct(r['methods'][m]['dtm']['within_0.5m'])}"))
        lines.append(row(f"{m}: cells with ground points", lambda r, m=m: pct(r["methods"][m]["dtm"]["cells_with_data"])))
    lines += ["", "## DSM and CHM", "", hdr, sep,
              row("DSM bias / RMSE", lambda r: f"{r['dsm']['bias_m']:+.3f} / {r['dsm']['rmse_m']:.3f} m"),
              row("DSM within 0.5 m", lambda r: pct(r["dsm"]["within_0.5m"])),
              row("DSM cells with data", lambda r: pct(r["dsm"]["cells_with_data"]))]
    for m in ("truth labels", "CSF", "PTD"):
        lines.append(row(f"CHM ({m} DTM) bias / RMSE", lambda r, m=m: f"{r['methods'][m]['chm']['bias_m']:+.3f} / "
                                                                       f"{r['methods'][m]['chm']['rmse_m']:.3f} m"))
    lines += ["", "Error maps: " + ", ".join(f"`out/{r['run']}/ground_eval.png`" for r in results) + ".", ""]
    open(path, "w").write("\n".join(lines))
    print(f"wrote {path}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--eval", type=float, nargs=4, default=(-400, 400, -400, 400))
    ap.add_argument("--margin", type=float, default=20.0, help="points this far outside the area too (edges), m")
    ap.add_argument("--filter-density", type=float, default=10.0, help="filter subsample, points per m^2")
    ap.add_argument("--planned-x", type=float, nargs="*",
                    default=[-427.5 + 95 * k for k in range(10)], help="survey lines (default: the spec's 10)")
    ap.add_argument("--y-range", type=float, nargs=2, default=(-650, 650))
    ap.add_argument("--report", default=os.path.join(ROOT, "out", "ground_eval.md"))
    a = ap.parse_args()
    results = [evaluate(r, a) for r in a.runs]
    report(results, a.report)


if __name__ == "__main__":
    sys.exit(main())
