#!/usr/bin/env python3
"""Do the survey images have enough tie points for SfM? (spec 6: varied, structured texture; spec 9.5)

For image pairs along a strip (forward overlap) and across neighbouring strips (side overlap): SIFT features,
ratio-test matches, RANSAC fundamental matrix. Because the true poses are known, every match can also be checked
against the true epipolar geometry (Sampson distance to F_true): the share of correct matches and how evenly they
cover the image (3 x 3 grid) say whether SfM/MVS can work on this scene, independently of any SfM program.

  python3 scripts/sfm_check.py out/r5_main                 # 8 forward + 8 side pairs, images at 1/2 scale
  python3 scripts/sfm_check.py out/<run> --pairs 12 --scale 0.5

Writes <run>/sfm_check.json and <run>/sfm_check.jpg (one pair with its correct matches).
"""

import argparse
import json
import os

import cv2
import numpy as np

from static_report import image_file, quat_to_matrix


def camera_pose(cfg, row):
    """World-to-camera rotation and translation (p_C = R p_W + t) of an images.csv row."""
    c = cfg["camera"]
    r_wb = quat_to_matrix(row["qx"], row["qy"], row["qz"], row["qw"])
    p_wb = np.array([row["x_m"], row["y_m"], row["z_m"]])
    r_wc = r_wb @ np.array(c["R_BC"], dtype=float)
    c_w = p_wb + r_wb @ np.array(c["t_BC"], dtype=float)
    r_cw = r_wc.T
    return r_cw, -r_cw @ c_w


def skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def sampson(f, a, b):
    """Sampson distance (px) of matches a <-> b (n x 2) under F (b^T F a = 0)."""
    ah = np.hstack([a, np.ones((len(a), 1))])
    bh = np.hstack([b, np.ones((len(b), 1))])
    fa = ah @ f.T
    ftb = bh @ f
    num = np.sum(bh * fa, axis=1) ** 2
    den = fa[:, 0] ** 2 + fa[:, 1] ** 2 + ftb[:, 0] ** 2 + ftb[:, 1] ** 2
    return np.sqrt(num / np.maximum(den, 1e-12))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run")
    ap.add_argument("--pairs", type=int, default=8, help="forward pairs and side pairs each")
    ap.add_argument("--scale", type=float, default=0.5, help="image scale for feature detection")
    ap.add_argument("--features", type=int, default=8000)
    ap.add_argument("--y-range", type=float, nargs=2, default=(-400, 400), help="pairs centred in this y range")
    args = ap.parse_args()
    run = args.run
    cfg = json.load(open(os.path.join(run, "sensors.json")))
    c = cfg["camera"]
    im = np.genfromtxt(os.path.join(run, "camera", "images.csv"), delimiter=",", names=True, dtype=None, encoding=None)
    # Strips from the images' own positions: runs of images with the same x (+-20 m).
    xs = np.round(im["x_m"] / 20.0)
    inside = (im["y_m"] > args.y_range[0]) & (im["y_m"] < args.y_range[1]) & (im["z_m"] > 300)
    k_mat = np.array([[c["fx"], 0, c["cx"]], [0, c["fy"], c["cy"]], [0, 0, 1]]) * args.scale
    k_mat[2, 2] = 1
    k_inv = np.linalg.inv(k_mat)

    fwd, side = [], []
    idx = np.nonzero(inside)[0]
    for a, b in zip(idx[:-1], idx[1:]):
        if xs[a] == xs[b] and abs(im["y_m"][b] - im["y_m"][a]) < 80:
            fwd.append((a, b))
    for a in idx:
        # nearest image on the neighbouring line (95 m east), at about the same y
        cand = idx[(np.abs(im["x_m"][idx] - im["x_m"][a] - 95) < 25) & (np.abs(im["y_m"][idx] - im["y_m"][a]) < 35)]
        if len(cand):
            side.append((a, cand[np.argmin(np.abs(im["y_m"][cand] - im["y_m"][a]))]))
    rng = np.random.default_rng(0)
    pick = lambda lst: [lst[i] for i in sorted(rng.choice(len(lst), min(args.pairs, len(lst)), replace=False))] if lst else []
    sift = cv2.SIFT_create(nfeatures=args.features)
    cache = {}

    def feats(k):
        if k not in cache:
            img = cv2.imread(os.path.join(run, "camera", image_file(im, k)), cv2.IMREAD_GRAYSCALE)
            img = cv2.resize(img, None, fx=args.scale, fy=args.scale, interpolation=cv2.INTER_AREA)
            kp, des = sift.detectAndCompute(img, None)
            cache[k] = (img, np.array([p.pt for p in kp], dtype=np.float64).reshape(-1, 2), des)
        return cache[k]

    results = {}
    best = None
    for kind, pairs in (("forward", pick(fwd)), ("side", pick(side))):
        rows = []
        for a, b in pairs:
            ia, pa, da = feats(a)
            ib, pb, db = feats(b)
            if da is None or db is None or len(pa) < 8 or len(pb) < 8:
                rows.append({"pair": [int(a), int(b)], "features": [len(pa), len(pb)], "matches": 0, "correct": 0})
                continue
            mm = cv2.BFMatcher(cv2.NORM_L2).knnMatch(da, db, k=2)
            good = [m for m, n in (x for x in mm if len(x) == 2) if m.distance < 0.75 * n.distance]
            qa = pa[[m.queryIdx for m in good]]
            qb = pb[[m.trainIdx for m in good]]
            # True fundamental matrix from the saved poses.
            ra, ta = camera_pose(cfg, im[a])
            rb, tb = camera_pose(cfg, im[b])
            r_ab = rb @ ra.T
            t_ab = tb - r_ab @ ta
            f_true = k_inv.T @ skew(t_ab) @ r_ab @ k_inv
            d = sampson(f_true, qa, qb) if len(qa) else np.array([])
            ok = d < 1.5
            n_ransac = 0
            if len(qa) >= 8:
                _, mask = cv2.findFundamentalMat(qa, qb, cv2.FM_RANSAC, 1.0, 0.999)
                n_ransac = int(mask.sum()) if mask is not None else 0
            # Coverage of the correct matches: 3 x 3 grid of image a.
            gx = np.clip((qa[ok, 0] / ia.shape[1] * 3).astype(int), 0, 2)
            gy = np.clip((qa[ok, 1] / ia.shape[0] * 3).astype(int), 0, 2)
            cells = len(set(zip(gx.tolist(), gy.tolist())))
            row = {"pair": [int(im["image_id"][a]), int(im["image_id"][b])], "features": [len(pa), len(pb)],
                   "matches": int(len(qa)), "ransac_inliers": n_ransac, "correct": int(ok.sum()),
                   "correct_ratio": float(ok.mean()) if len(ok) else 0.0, "grid_cells_3x3": cells}
            rows.append(row)
            if kind == "forward" and (best is None or row["correct"] > best[0]):
                best = (row["correct"], ia, ib, qa[ok], qb[ok])
        c_ = np.array([r["correct"] for r in rows]) if rows else np.zeros(1)
        results[kind] = {"pairs": rows, "correct_matches": {"min": int(c_.min()), "median": float(np.median(c_))},
                         "correct_ratio_median": float(np.median([r.get("correct_ratio", 0) for r in rows])) if rows else 0,
                         "pairs_with_100_correct": float(np.mean(c_ >= 100)),
                         "grid_cells_median": float(np.median([r.get("grid_cells_3x3", 0) for r in rows])) if rows else 0}
        print(f"{kind}: {len(rows)} pairs, correct matches min {c_.min()} median {np.median(c_):.0f}, "
              f"correct ratio median {results[kind]['correct_ratio_median']:.2f}, "
              f"3x3 cells median {results[kind]['grid_cells_median']:.0f}", flush=True)
    results["settings"] = {"scale": args.scale, "sift_features": args.features, "ratio_test": 0.75,
                           "correct_if_sampson_px_below": 1.5, "image_size": [c["width"], c["height"]]}
    json.dump(results, open(os.path.join(run, "sfm_check.json"), "w"), indent=2)
    if best is not None:
        _, ia, ib, qa, qb = best
        canvas = np.hstack([cv2.cvtColor(ia, cv2.COLOR_GRAY2BGR), cv2.cvtColor(ib, cv2.COLOR_GRAY2BGR)])
        for (x1, y1), (x2, y2) in list(zip(qa, qb))[::max(1, len(qa) // 300)]:
            cv2.line(canvas, (int(x1), int(y1)), (int(x2) + ia.shape[1], int(y2)), (0, 255, 255), 1)
        cv2.imwrite(os.path.join(run, "sfm_check.jpg"), cv2.resize(canvas, None, fx=0.5, fy=0.5),
                    [cv2.IMWRITE_JPEG_QUALITY, 85])


if __name__ == "__main__":
    main()
