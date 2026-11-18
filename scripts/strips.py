"""Survey strips of a flight: the stretches flown straight along a planned north-south line.

A LiDAR line (or any sample with a time and pose) belongs to strip s when the vehicle is not turning (|roll| < 8 deg:
turns bank 30 deg and more; R5 line 1 rolled up to 4 deg in a decaying oscillation with a straight track),
travelling north or south within heading_tol (course, not the nose's heading), inside the measured y range, and that stretch is continuous in time and
covers at least min_frac of the y range. Everything else (climb, turns, transits, descent) is "transit".
Strips are numbered 1, 2, ... in time order; transit is 0 (spec 8: strip_id and on-line / transition flag).
"""

import numpy as np


def segment(t, p, rot, y_range, heading_tol_deg=10.0, roll_max_deg=8.0, min_frac=0.8, gap_s=0.5, planned_x=None,
            max_offset=20.0):
    """Returns (strip_id per sample, list of strips). t: (n,), p: (n, 3), rot: (n, 3, 3) body to world.
    With planned_x, a straight stretch more than max_offset from every planned line (e.g. the way back to the
    landing over the forest) is transit, not a strip."""
    # Direction of travel (course) over +-0.5 s, not the nose: the VTOL yaws a few degrees about its track
    # at 25 m/s (seen in R4: +-5..13 deg with a straight track), which is no reason to break a strip.
    i_lo = np.searchsorted(t, t - 0.5)
    i_hi = np.minimum(np.searchsorted(t, t + 0.5), len(t) - 1)
    v = p[i_hi, :2] - p[i_lo, :2]
    sp = np.hypot(v[:, 0], v[:, 1])
    with np.errstate(invalid="ignore", divide="ignore"):
        ns = (np.abs(v[:, 0]) < np.sin(np.radians(heading_tol_deg)) * sp) & (sp > 0)
    level = np.abs(rot[:, 2, 1]) < np.sin(np.radians(roll_max_deg))
    inside = (p[:, 1] >= y_range[0]) & (p[:, 1] <= y_range[1])
    direction = np.sign(v[:, 1]).astype(int)
    ok = ns & level & inside
    ids = np.zeros(len(t), dtype=int)
    strips = []
    # Runs of ok with the same direction and no time gap.
    brk = np.ones(len(t), dtype=bool)
    brk[1:] = (~ok[1:]) | (~ok[:-1]) | (direction[1:] != direction[:-1]) | (np.diff(t) > gap_s)
    starts = np.nonzero(ok & brk)[0]
    for s in starts:
        e = s
        while e + 1 < len(t) and ok[e + 1] and not brk[e + 1]:
            e += 1
        idx = np.arange(s, e + 1)
        length = abs(p[e, 1] - p[s, 1])
        if length < min_frac * (y_range[1] - y_range[0]):
            continue
        x_med = float(np.median(p[idx, 0]))
        x_nom = x_med
        if planned_x is not None and len(planned_x):
            x_nom = float(planned_x[int(np.argmin(np.abs(np.asarray(planned_x) - x_med)))])
            if abs(x_nom - x_med) > max_offset:
                continue
        strips.append({"id": len(strips) + 1, "dir": "N" if direction[s] > 0 else "S", "x_nominal": x_nom,
                       "t0": float(t[s]), "t1": float(t[e]), "first": int(s), "last": int(e)})
        ids[idx] = len(strips)
    return ids, strips


def strip_of_time(strips, t):
    """Strip id (0 = transit) of arbitrary times t, from the strips' time spans."""
    out = np.zeros(len(t), dtype=int)
    for s in strips:
        out[(t >= s["t0"]) & (t <= s["t1"])] = s["id"]
    return out
