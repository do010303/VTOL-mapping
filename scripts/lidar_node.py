#!/usr/bin/env python3
"""LiDAR node: records every scan line of the nadir line scanner with its sim time.

Writes to <out>/lidar/:
  scans_<k>.npz     t_s (line time), scan_id, ranges_m (n_lines x rays, float32; inf = no return)
  stats.json        lines received, rate in sim and wall time, gaps, return ratio, range statistics

and <out>/trajectory.csv (the vehicle's pose stream, if --model is given). Per-ray times and
directions follow from config/sensors.json: beta_k = scan_min + k * step,
t_k = t_s + k * pulse_spacing (spec 2), d_L = (0, sin beta, -cos beta).

  scripts/lidar_node.py --world forest_static --model vtol_rig --out out/static_500 --duration 10
"""

import argparse
import json
import os
import signal
import sys
import threading
import time

from sensor_common import PoseFeed, load_config, stamp_s   # first: protobuf backend

import numpy as np
from gz.msgs10.laserscan_pb2 import LaserScan
from gz.transport13 import Node


class LidarRecorder:
    def __init__(self, cfg, out, chunk=2000, poses=None, min_alt=None):
        self.cfg = cfg["lidar"]
        self.dir = os.path.join(out, "lidar")
        os.makedirs(self.dir, exist_ok=True)
        self.chunk = chunk
        self.lock = threading.Lock()
        self.t, self.r = [], []
        self.n_chunks = 0
        self.lines = 0
        self.first_t = self.last_t = None
        self.first_wall = None
        self.gaps = 0
        self.max_gap = 0.0
        self.returns = 0
        self.rays = 0
        self.hist = np.zeros(1300, dtype=np.int64)       # 1 m bins of the range
        self.rmin, self.rmax = np.inf, 0.0
        self.width = None
        self.period = 1.0 / self.cfg["lines_per_s"]
        self.done = threading.Event()
        self.poses, self.min_alt = poses, min_alt
        self.skipped = 0

    def on_scan(self, msg):
        t = stamp_s(msg.header)
        if self.min_alt is not None:
            cur = self.poses.latest() if self.poses else None
            if cur is None or cur[1][2] < self.min_alt:
                self.skipped += 1
                return
        r = np.asarray(msg.ranges, dtype=np.float32)
        lo, hi = self.cfg["range_min_m"], self.cfg["range_max_m"]
        valid = np.isfinite(r) & (r >= lo) & (r <= hi)
        r = np.where(valid, r, np.inf).astype(np.float32)
        with self.lock:
            if self.first_t is None:
                self.first_t, self.first_wall = t, time.monotonic()
                self.width = r.size
            elif t <= self.last_t:
                return
            else:
                gap = t - self.last_t
                self.max_gap = max(self.max_gap, gap)
                if gap > 1.5 * self.period:
                    self.gaps += 1
            self.last_t = t
            self.lines += 1
            self.rays += r.size
            n = int(valid.sum())
            self.returns += n
            if n:
                rv = r[valid]
                self.rmin = min(self.rmin, float(rv.min()))
                self.rmax = max(self.rmax, float(rv.max()))
                self.hist += np.bincount(np.minimum(rv.astype(int), 1299), minlength=1300)
            self.t.append(t)
            self.r.append(r)
            if len(self.t) >= self.chunk:
                self._flush()

    def _flush(self):
        if not self.t:
            return
        np.savez(os.path.join(self.dir, f"scans_{self.n_chunks:04d}.npz"), t_s=np.array(self.t),
                 scan_id=np.arange(self.lines - len(self.t), self.lines), ranges_m=np.stack(self.r))
        self.n_chunks += 1
        self.t, self.r = [], []

    def stats(self):
        with self.lock:
            sim = (self.last_t - self.first_t) if self.lines > 1 else 0.0
            wall = time.monotonic() - self.first_wall if self.first_wall else 0.0
            cdf = np.cumsum(self.hist)
            med = int(np.searchsorted(cdf, cdf[-1] / 2)) if cdf[-1] else None
            return {
                "lines": self.lines, "rays_per_line": self.width,
                "sim_s": sim, "wall_s": wall, "real_time_factor": sim / wall if wall else None,
                "lines_per_sim_s": (self.lines - 1) / sim if sim else None,
                "lines_per_wall_s": self.lines / wall if wall else None,
                "gaps": self.gaps, "max_gap_s": self.max_gap, "skipped_below_min_alt": self.skipped,
                "return_ratio": self.returns / self.rays if self.rays else None,
                "returns_per_sim_s": self.returns / sim if sim else None,
                "range_min_m": self.rmin if self.returns else None,
                "range_median_m": med, "range_max_m": self.rmax if self.returns else None,
            }

    def close(self):
        with self.lock:
            self._flush()
        s = self.stats()
        with open(os.path.join(self.dir, "stats.json"), "w") as f:
            json.dump(s, f, indent=2)
        return s


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--world", required=True)
    ap.add_argument("--model", help="vehicle model whose pose to record (trajectory.csv)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--config", help="sensors.json (default config/sensors.json)")
    ap.add_argument("--duration", type=float, default=0, help="stop after this much sim time, s (0: on SIGINT)")
    ap.add_argument("--min-alt", type=float, help="record only while the vehicle is above this altitude, m")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    cfg = load_config(args.config)
    os.makedirs(args.out, exist_ok=True)
    node = Node()
    traj = poses = None
    if args.model:
        traj = open(os.path.join(args.out, "trajectory.csv"), "w", buffering=1)
        traj.write("t_s,x_m,y_m,z_m,qx,qy,qz,qw\n")
        poses = PoseFeed(node, args.world, args.model, log=traj)
    rec = LidarRecorder(cfg, args.out, poses=poses, min_alt=args.min_alt)

    topic = cfg["lidar"]["topic"]
    if not node.subscribe(LaserScan, topic, rec.on_scan):
        sys.exit(f"failed to subscribe to {topic}")

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    print(f"lidar_node: recording {topic} -> {rec.dir}", flush=True)
    last_print = time.monotonic()
    while not stop.wait(0.2):
        s = rec.stats()
        if args.duration and s["sim_s"] >= args.duration:
            break
        if not args.quiet and time.monotonic() - last_print >= 2 and s["lines"]:
            last_print = time.monotonic()
            print(f"lidar  sim {s['sim_s']:7.2f} s  lines {s['lines']:6d}  {s['lines_per_sim_s'] or 0:6.1f}/sim-s "
                  f"{s['lines_per_wall_s'] or 0:6.1f}/wall-s  RTF {s['real_time_factor'] or 0:.2f}  "
                  f"returns {100 * (s['return_ratio'] or 0):5.1f} %  range {s['range_min_m'] or 0:.1f}"
                  f"-{s['range_max_m'] or 0:.1f} m  gaps {s['gaps']}", flush=True)
    s = rec.close()
    if traj:
        traj.close()
    print("lidar_node: " + json.dumps(s), flush=True)
    # Skip the interpreter's teardown: gz-transport's callback threads abort it (pybind11 thread state).
    os._exit(0)


if __name__ == "__main__":
    main()
