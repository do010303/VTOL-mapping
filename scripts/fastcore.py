"""The LiDAR hot loops: the C++ module (cpp/vtol_core.cpp, built by cpp/build.sh) when it is built, else NumPy.

  world_points(t_lines, ranges, traj, d_b, t_bl, pulse_dt) -> (xyz float32 (N, 3), t (N,))
  cell_reduce(index, values, n_cells, "max" | "min" | "count") -> (n_cells,)
  VoxelMap(voxel, column)                                       (C++ only; None without it)
NATIVE tells which one runs; VTOL_NO_NATIVE=1 forces NumPy (tests).
"""
import os

import numpy as np

try:
    if os.environ.get("VTOL_NO_NATIVE") == "1":
        raise ImportError
    import vtol_core as _c
    NATIVE = True
except ImportError:
    _c = None
    NATIVE = False


def _interp(traj, t):
    from flight_report import interp_poses
    return interp_poses(traj, t)


def world_points(t_lines, ranges, traj, d_b, t_bl, pulse_dt):
    if NATIVE:
        return _c.world_points(np.ascontiguousarray(t_lines, np.float64), np.ascontiguousarray(ranges, np.float32),
                               np.ascontiguousarray(traj, np.float64), np.ascontiguousarray(d_b, np.float64),
                               np.ascontiguousarray(t_bl, np.float64), float(pulse_dt))
    n, m = ranges.shape
    hit = np.isfinite(ranges)
    tk = t_lines[:, None] + pulse_dt * np.arange(m)[None, :]
    if pulse_dt:
        pk, rk = _interp(traj, tk.ravel())
        o = pk + rk @ t_bl
        d = np.einsum("mij,mj->mi", rk, np.tile(d_b, (n, 1)))
        pts = (o + np.where(hit, ranges, 0.0).reshape(-1, 1) * d).reshape(n, m, 3)
    else:
        pp, ro = _interp(traj, t_lines)
        o = pp + np.einsum("nij,j->ni", ro, t_bl)
        d_w = np.einsum("nij,kj->nki", ro, d_b)
        pts = o[:, None, :] + np.where(hit, ranges, 0.0)[..., None] * d_w
    return pts[hit].astype(np.float32), tk[hit]


def cell_reduce(index, values, n_cells, op):
    index = np.asarray(index, np.int64)
    if NATIVE:
        v = np.zeros(0) if op == "count" else np.ascontiguousarray(values, np.float64)
        return _c.cell_reduce(np.ascontiguousarray(index), v, int(n_cells), op)
    ok = (index >= 0) & (index < n_cells)
    if op == "count":
        return np.bincount(index[ok], minlength=n_cells).astype(np.float64)
    out = np.full(n_cells, -np.inf if op == "max" else np.inf)
    (np.maximum if op == "max" else np.minimum).at(out, index[ok], np.asarray(values, np.float64)[ok])
    return out


VoxelMap = _c.VoxelMap if NATIVE else None
