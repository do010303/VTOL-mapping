"""Storage of LiDAR scan lines (lidar/, lidar_exact/ scans_<k>.npz).

Ranges are kept in whole millimetres, each ray as the difference to the ray before it in the same line (neighbouring
rays see nearly the same range, so the differences are small and compress ~4x better than float32 metres), 0 mm = no
return. Rounding to 1 mm changes a range by at most 0.5 mm, well inside the LiDAR's own accuracy (spec: 10 mm) and the
ray caster's (+-0.3 cm against the scene). Files written before this (float32 `ranges_m`) are read as they are.

  save(path, t_s=..., ranges_m=..., scan_id=...)     -> npz with ranges_mm_delta (int32), zip-compressed
  load(path) -> dict with t_s, ranges_m (float32, inf = no return) and any other array
"""
import numpy as np


def encode(ranges_m):
    q = np.where(np.isfinite(ranges_m), np.round(np.asarray(ranges_m, np.float64) * 1000.0), 0).astype(np.int64)
    d = q.copy()
    d[:, 1:] -= q[:, :-1]
    return d.astype(np.int32)


def decode(delta):
    q = np.cumsum(delta.astype(np.int64), axis=1)
    return np.where(q > 0, q / 1000.0, np.inf).astype(np.float32)


def save(path, ranges_m, **arrays):
    np.savez_compressed(path, ranges_mm_delta=encode(ranges_m), **arrays)


def load(path):
    z = np.load(path)
    out = {k: z[k] for k in z.files if k != "ranges_mm_delta"}
    if "ranges_mm_delta" in z.files:
        out["ranges_m"] = decode(z["ranges_mm_delta"])
    return out
