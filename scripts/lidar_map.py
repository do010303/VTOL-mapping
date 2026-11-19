#!/usr/bin/env python3
"""LiDAR mapping node: builds the 3D point-cloud map of the forest from the scans and the vehicle's pose, live
or from a recorded flight, and shows it in RViz (like px4-mapping's ground_map.py for the radar).

The map is built from the LiDAR data alone (no ground truth):
  voxels      every return goes into a 3D voxel grid (--voxel, default 1 m); each voxel keeps its hit count
  ground      per 1 m column, the lowest return; the ground estimate is the lowest of those within 11 m
              (thin rays reach the ground between crowns: a first DTM, also under the crowns)
  surface     per 1 m column, the highest return (DSM); height above ground = z - ground estimate (a first CHM)

Sources:
  --live                  the running simulation (./run_sitl.sh or static_run.sh): /vtol/lidar/scan + vehicle pose
  <run>                   a recorded flight: <run>/lidar_exact/ (per-ray times, scripts/lidar_exact.py) if there is
                          one, else <run>/lidar/; --speed 1 replays at the flight's pace, 0 as fast as possible

RViz (--rviz; frame "map" = world ENU, metres; rviz2 -d rviz/lidar.rviz, or scripts/lidar_rviz.sh for both):
  /vtol/lidar/scan        the returns of the last second (fields: height above the ground estimate, z)
  /vtol/lidar/map         the voxel map (fields: height above ground, hits)
  /vtol/lidar/ground      the ground estimate, 1 m columns where a ray reached the ground
  /vtol/path              the flown path (nav_msgs/Path)
  /vtol/vehicle           the vehicle (Marker, CW-30E-sized) and /vtol/camera_footprint (the image's footprint)

At the end (Ctrl+C, or the replay's end) it writes <out>/lidar_map.npz (voxels, hits, ground and DSM grids),
<out>/lidar_map.ply (voxel centres, for CloudCompare / MeshLab: only with --ply) and <out>/lidar_map.png (top view).

  scripts/lidar_map.py out/r5_main --rviz --speed 20          # replay R5 20x faster, in RViz
  scripts/lidar_map.py out/r5_main                             # just build and save the map
  scripts/lidar_rviz.sh                                        # live: node + RViz next to ./run_sitl.sh
"""

import argparse
import json
import math
import os
import signal
import sys
import threading
import time
import warnings

warnings.filterwarnings("ignore", message="A NumPy version")
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import numpy as np  # noqa: E402

GROUND_CELL = 1.0
GROUND_WINDOW = 11          # 1 m columns: the ground under a crown is the lowest return within this window
KEEP_SCAN_S = 1.0


class LidarMap:
    """Voxel map plus per-column lowest / highest return. Accumulates in batches, merges with np.unique."""

    def __init__(self, voxel=1.0):
        self.v = voxel
        self.keys = np.zeros(0, np.int64)      # sorted voxel keys
        self.hits = np.zeros(0, np.int64)
        self.ckeys = np.zeros(0, np.int64)     # sorted 1 m column keys
        self.zmin = np.zeros(0)
        self.zmax = np.zeros(0)
        self.pending = []
        self.points = 0
        self.lock = threading.Lock()
        import fastcore
        # C++ voxel / column hash maps (cpp/vtol_core.cpp) when built: points go in as they come, no re-sorting
        self.native = fastcore.VoxelMap(voxel, GROUND_CELL) if fastcore.VoxelMap is not None else None

    @staticmethod
    def _key(i, j, k=None):
        o = 1 << 20
        if k is None:
            return (i + o) * (1 << 21) + (j + o)
        return ((i + o) * (1 << 21) + (j + o)) * (1 << 21) + (k + o)

    @staticmethod
    def _unkey(key, three):
        o = 1 << 20
        if three:
            k = key % (1 << 21) - o
            key = key // (1 << 21)
        j = key % (1 << 21) - o
        i = key // (1 << 21) - o
        return (i, j, k) if three else (i, j)

    def add(self, xyz):
        with self.lock:
            if self.native is not None:
                self.native.add(np.ascontiguousarray(xyz, dtype=np.float64))
            else:
                self.pending.append(np.asarray(xyz, dtype=np.float64))
            self.points += len(xyz)

    def merge(self):
        if self.native is not None:
            with self.lock:                      # the C++ add releases the GIL: no add while we read the maps
                keys, hits = self.native.voxels()
                ck, zmin, zmax = self.native.columns()
            ground = self._ground_grid(ck, zmin)
            with self.lock:
                self.keys, self.hits = keys, hits
                self.ckeys, self.zmin, self.zmax = ck, zmin, zmax
                self.ground = ground
            return
        with self.lock:
            if not self.pending:
                return
            xyz = np.concatenate(self.pending)
            self.pending = []
        q = np.floor(xyz / self.v).astype(np.int64)
        k, c = np.unique(self._key(q[:, 0], q[:, 1], q[:, 2]), return_counts=True)
        allk = np.concatenate([self.keys, k])
        allc = np.concatenate([self.hits, c])
        uk, inv = np.unique(allk, return_inverse=True)
        hits = np.bincount(inv, weights=allc).astype(np.int64)
        g = np.floor(xyz[:, :2] / GROUND_CELL).astype(np.int64)
        ck = self._key(g[:, 0], g[:, 1])
        allck = np.concatenate([self.ckeys, ck])
        allmin = np.concatenate([self.zmin, xyz[:, 2]])
        allmax = np.concatenate([self.zmax, xyz[:, 2]])
        order = np.argsort(allck, kind="stable")
        sk = allck[order]
        starts = np.concatenate([[0], np.nonzero(np.diff(sk))[0] + 1]) if len(sk) else np.zeros(0, int)
        zmin = np.minimum.reduceat(allmin[order], starts) if len(sk) else np.zeros(0)
        zmax = np.maximum.reduceat(allmax[order], starts) if len(sk) else np.zeros(0)
        ground = self._ground_grid(sk[starts], zmin)
        with self.lock:
            self.keys, self.hits = uk, hits
            self.ckeys, self.zmin, self.zmax = sk[starts], zmin, zmax
            self.ground = ground

    def n_voxels(self):
        return self.native.n_voxels if self.native is not None else len(self.keys)

    def n_columns(self):
        return self.native.n_columns if self.native is not None else len(self.ckeys)

    def _ground_grid(self, ckeys, zmin):
        """(i0, j0, grid) of the ground estimate: per 1 m cell, the lowest return within GROUND_WINDOW."""
        if not len(ckeys):
            return None
        from scipy.ndimage import minimum_filter
        i, j = self._unkey(ckeys, False)
        i0, j0 = i.min(), j.min()
        grid = np.full((j.max() - j0 + 1, i.max() - i0 + 1), np.inf)      # rows: y (j), columns: x (i)
        grid[j - j0, i - i0] = zmin
        g = minimum_filter(grid, size=GROUND_WINDOW, mode="constant", cval=np.inf)
        g[~np.isfinite(g)] = np.nan
        return i0, j0, g

    def ground_at(self, x, y):
        """Ground estimate at x, y (nan where no return within the window)."""
        gg = getattr(self, "ground", None)
        out = np.full(len(x), np.nan)
        if gg is None:
            return out
        i0, j0, g = gg
        i = np.floor(np.asarray(x) / GROUND_CELL).astype(np.int64) - i0
        j = np.floor(np.asarray(y) / GROUND_CELL).astype(np.int64) - j0
        ok = (i >= 0) & (j >= 0) & (j < g.shape[0]) & (i < g.shape[1])
        out[ok] = g[j[ok], i[ok]]
        return out

    def voxels(self):
        """(centres (n, 3), hits (n,), height above the column's lowest return (n,))."""
        with self.lock:
            keys, hits = self.keys.copy(), self.hits.copy()
        i, j, k = self._unkey(keys, True)
        c = (np.stack([i, j, k], axis=1) + 0.5) * self.v
        h = c[:, 2] - self.ground_at(c[:, 0], c[:, 1])
        return c, hits, np.nan_to_num(h, nan=0.0)

    def columns(self):
        """(x, y, lowest z, highest z) of every 1 m column with returns."""
        with self.lock:
            ck, zmin, zmax = self.ckeys.copy(), self.zmin.copy(), self.zmax.copy()
        i, j = self._unkey(ck, False)
        return (i + 0.5) * GROUND_CELL, (j + 0.5) * GROUND_CELL, zmin, zmax

    def save(self, out, ply=False):
        self.merge()
        c, hits, h = self.voxels()
        x, y, zmin, zmax = self.columns()
        np.savez_compressed(os.path.join(out, "lidar_map.npz"), voxel_m=self.v, centres=c.astype(np.float32),
                            hits=hits.astype(np.int32), height=h.astype(np.float32), col_x=x, col_y=y,
                            col_zmin=zmin.astype(np.float32), col_zmax=zmax.astype(np.float32), points=self.points)
        if ply:                         # the same voxels as the npz: only on request (disk)
            self._ply(out, c, h, hits)
        self._png(out, x, y, zmin, zmax)
        return self._info(c, x)

    def _ply(self, out, c, h, hits):
        with open(os.path.join(out, "lidar_map.ply"), "wb") as f:
            f.write((f"ply\nformat binary_little_endian 1.0\nelement vertex {len(c)}\nproperty float x\n"
                     "property float y\nproperty float z\nproperty float height\nproperty int hits\nend_header\n").encode())
            rec = np.zeros(len(c), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("h", "<f4"), ("n", "<i4")])
            rec["x"], rec["y"], rec["z"], rec["h"], rec["n"] = c[:, 0], c[:, 1], c[:, 2], h, hits
            f.write(rec.tobytes())

    def _png(self, out, x, y, zmin, zmax):
        if len(x):
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            fig, ax = plt.subplots(1, 2, figsize=(13, 6))
            gnd = self.ground_at(x, y)
            # The columns are a 1 m grid: an image of it (a scatter of millions of points took ~50 s).
            ix = np.floor(x / GROUND_CELL).astype(np.int64)
            iy = np.floor(y / GROUND_CELL).astype(np.int64)
            ext = (ix.min() * GROUND_CELL, (ix.max() + 1) * GROUND_CELL, iy.min() * GROUND_CELL, (iy.max() + 1) * GROUND_CELL)
            for a, val, title, cm, lim in ((ax[0], zmax - gnd, "canopy height (highest return - ground estimate), m",
                                            "viridis", (0, 30)),
                                           (ax[1], gnd, "ground estimate (lowest return within 11 m), m",
                                            "terrain", None)):
                img = np.full((iy.max() - iy.min() + 1, ix.max() - ix.min() + 1), np.nan)
                img[iy - iy.min(), ix - ix.min()] = val
                s = a.imshow(img, origin="lower", extent=ext, cmap=cm, interpolation="nearest",
                             vmin=lim[0] if lim else None, vmax=lim[1] if lim else None)
                a.set_aspect("equal")
                a.set_title(title, fontsize=10)
                a.set_xlabel("x east (m)")
                a.set_ylabel("y north (m)")
                fig.colorbar(s, ax=a, shrink=0.7)
            fig.tight_layout()
            fig.savefig(os.path.join(out, "lidar_map.png"), dpi=110)

    def _info(self, c, x):
        return {"points": int(self.points), "voxels": int(len(c)), "voxel_m": self.v, "columns_1m": int(len(x)),
                "area_m2": float(len(x) * GROUND_CELL ** 2)}


class Rviz:
    def __init__(self):
        import rclpy
        from geometry_msgs.msg import Point, PoseStamped
        from nav_msgs.msg import Path
        from sensor_msgs.msg import PointCloud2, PointField
        from visualization_msgs.msg import Marker
        rclpy.init()
        self.rclpy = rclpy
        self.node = rclpy.create_node("vtol_lidar_map")
        self.Point, self.PoseStamped, self.Path, self.PointCloud2, self.PointField, self.Marker = \
            Point, PoseStamped, Path, PointCloud2, PointField, Marker
        pub = lambda t, n: self.node.create_publisher(t, n, 1)  # noqa: E731
        self.scan_pub, self.map_pub = pub(PointCloud2, "/vtol/lidar/scan"), pub(PointCloud2, "/vtol/lidar/map")
        self.ground_pub = pub(PointCloud2, "/vtol/lidar/ground")
        self.path_pub, self.veh_pub = pub(Path, "/vtol/path"), pub(Marker, "/vtol/vehicle")
        self.fp_pub = pub(Marker, "/vtol/camera_footprint")
        self.path = Path()
        self.path.header.frame_id = "map"

    def stamp(self):
        return self.node.get_clock().now().to_msg()

    def cloud(self, xyz, extra, names):
        fields = [self.PointField(name=n, offset=4 * i, datatype=self.PointField.FLOAT32, count=1)
                  for i, n in enumerate(["x", "y", "z"] + names)]
        data = np.column_stack([xyz] + list(extra)).astype("<f4")
        c = self.PointCloud2()
        c.header.stamp, c.header.frame_id = self.stamp(), "map"
        c.height, c.width, c.fields = 1, len(data), fields
        c.point_step, c.row_step, c.is_dense = 4 * data.shape[1], 4 * data.shape[1] * len(data), True
        c.data = data.tobytes()
        return c

    def vehicle(self, p, rot, footprint):
        m = self.Marker()
        m.header.stamp, m.header.frame_id = self.stamp(), "map"
        m.ns, m.id, m.type, m.action = "vehicle", 0, self.Marker.CUBE, self.Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in p)
        qw = math.sqrt(max(0.0, 1 + rot[0, 0] + rot[1, 1] + rot[2, 2])) / 2
        m.pose.orientation.w = qw
        m.pose.orientation.x = (rot[2, 1] - rot[1, 2]) / (4 * qw)
        m.pose.orientation.y = (rot[0, 2] - rot[2, 0]) / (4 * qw)
        m.pose.orientation.z = (rot[1, 0] - rot[0, 1]) / (4 * qw)
        m.scale.x, m.scale.y, m.scale.z = 2.4 * 4, 4.4 * 4, 0.3 * 4          # 4x: visible from far away
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.3, 0.2, 1.0
        self.veh_pub.publish(m)
        if footprint is not None:
            f = self.Marker()
            f.header = m.header
            f.ns, f.id, f.type, f.action = "footprint", 0, self.Marker.LINE_STRIP, self.Marker.ADD
            f.scale.x = 1.5
            f.color.r, f.color.g, f.color.b, f.color.a = 1.0, 1.0, 1.0, 0.9
            f.pose.orientation.w = 1.0
            f.points = [self.Point(x=float(a), y=float(b), z=float(c)) for a, b, c in footprint]
            self.fp_pub.publish(f)
        ps = self.PoseStamped()
        ps.header = m.header
        ps.pose = m.pose
        self.path.poses.append(ps)
        if len(self.path.poses) > 20000:
            self.path.poses = self.path.poses[::2]
        self.path.header.stamp = m.header.stamp
        self.path_pub.publish(self.path)


def camera_footprint(cfg, p, rot, ground_z):
    """The camera image's corners on the plane z = ground_z (closed loop), from the pose."""
    c = cfg["camera"]
    r_wc = rot @ np.array(c["R_BC"], dtype=float)
    o = p + rot @ np.array(c["t_BC"], dtype=float)
    pts = []
    for u, v in ((0, 0), (c["width"], 0), (c["width"], c["height"]), (0, c["height"]), (0, 0)):
        d = r_wc @ np.array([(u - c["cx"]) / c["fx"], (v - c["cy"]) / c["fy"], 1.0])
        if d[2] >= -1e-6:
            return None
        s = (ground_z - o[2]) / d[2]
        pts.append(o + s * d)
    return pts


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run", nargs="?", help="recorded flight to replay (omit with --live)")
    ap.add_argument("--live", action="store_true")
    ap.add_argument("--world", help="live: gz world (default: run/last_vehicle)")
    ap.add_argument("--model", help="live: vehicle model (default: run/last_vehicle)")
    ap.add_argument("--config", help="live: sensors json (default config/sensors_flight.json)")
    ap.add_argument("--out", help="where to save the map (default <run>, live: out/live_map_<time>)")
    ap.add_argument("--rviz", action="store_true")
    ap.add_argument("--ply", action="store_true", help="also write lidar_map.ply (CloudCompare / MeshLab)")
    ap.add_argument("--voxel", type=float, default=1.0, help="voxel size, m")
    ap.add_argument("--every", type=int, default=1, help="use every n-th ray")
    ap.add_argument("--speed", type=float, default=0, help="replay: x real time (0: as fast as possible)")
    ap.add_argument("--min-alt", type=float, default=100, help="only lines while the vehicle is above this, m")
    ap.add_argument("--lidar", choices=("auto", "gz", "exact"), default="auto")
    ap.add_argument("--publish-period", type=float, default=2.0, help="s between map updates in RViz")
    ap.add_argument("--max-publish", type=int, default=1_000_000,
                    help="voxels sent to RViz at most (RViz dies on big clouds: 3 M crashed it)")
    args = ap.parse_args()
    if not args.live and not args.run:
        ap.error("a run directory, or --live")

    lmap = LidarMap(args.voxel)
    rv = Rviz() if args.rviz else None
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    recent = []                        # (t, xyz) of the last KEEP_SCAN_S
    state = {"pose": None, "t": None}
    lock = threading.Lock()

    def on_points(t, xyz, p, rot):
        lmap.add(xyz)
        with lock:
            recent.append((t, xyz[::2]))
            while recent and recent[0][0] < t - KEEP_SCAN_S:
                recent.pop(0)
            state["pose"], state["t"] = (p, rot), t

    if args.live:
        from sensor_common import PoseFeed, load_config, stamp_s   # gz bindings (python3 -s)
        from gz.msgs10.laserscan_pb2 import LaserScan
        from gz.transport13 import Node
        from static_report import lidar_dirs
        try:
            lw, lm = open(os.path.join(ROOT, "run", "last_vehicle")).read().split()
        except (OSError, ValueError):
            lw, lm = "forest", "cw30e_mapping_0"
        world, model = args.world or lw, args.model or lm
        cfg = load_config(args.config or os.path.join(ROOT, "config", "sensors_flight.json"))
        lc = cfg["lidar"]
        d_l, _ = lidar_dirs(cfg)
        d_l = d_l[::args.every]
        t_bl = np.array(lc["t_BL"])
        node = Node()
        poses = PoseFeed(node, world, model)
        from sensor_common import quat_to_matrix

        def on_scan(msg):
            t = stamp_s(msg.header)
            cur = poses.at(t)
            if cur is None:
                return
            p, q = np.array(cur[0]), cur[1]
            if p[2] < args.min_alt:
                return
            r = np.asarray(msg.ranges, dtype=np.float64)[::args.every]
            ok = np.isfinite(r) & (r >= lc["range_min_m"]) & (r <= lc["range_max_m"])
            rot = quat_to_matrix(*q)
            on_points(t, (t_bl + r[ok, None] * d_l[ok]) @ rot.T + p, p, rot)

        if not node.subscribe(LaserScan, lc["topic"], on_scan):
            sys.exit(f"failed to subscribe to {lc['topic']}")
        out = args.out or os.path.join(ROOT, "out", time.strftime("live_map_%Y%m%d_%H%M%S"))
        print(f"lidar_map: live, world {world}, model {model} -> {out}", flush=True)
        producer = None
    else:
        from flight_report import interp_poses, lidar_source, world_points
        from static_report import load_scans
        run = args.run
        cfg = json.load(open(os.path.join(run, "sensors.json")))
        sub, exact = lidar_source(run, args.lidar)
        t, r = load_scans(run, sub)
        traj = np.loadtxt(os.path.join(run, "trajectory.csv"), delimiter=",", skiprows=1)
        traj = traj[np.concatenate([[True], np.diff(traj[:, 0]) > 0])]
        p_l, rot_l = interp_poses(traj, t)
        keep = p_l[:, 2] >= args.min_alt
        t, r, p_l, rot_l = t[keep], r[keep][:, ::args.every] if args.every > 1 else r[keep], p_l[keep], rot_l[keep]
        if args.every > 1:
            cfg = dict(cfg, lidar=dict(cfg["lidar"], rays_per_line=r.shape[1],
                                       pulse_spacing_s=cfg["lidar"]["pulse_spacing_s"] * args.every,
                                       angle_step_deg=cfg["lidar"]["angle_step_deg"] * args.every))
        out = args.out or run
        print(f"lidar_map: replay {run} ({sub}, {len(t)} lines) -> {out}", flush=True)

        def producer():
            t_wall0, t_sim0 = time.monotonic(), t[0] if len(t) else 0
            chunk = 100
            for sl, pts, tk, hit in world_points(cfg, t, r, traj, exact, True, p_l, rot_l, chunk=chunk):
                if stop.is_set():
                    break
                i_last = min(sl.stop, len(t)) - 1
                on_points(t[i_last], pts, p_l[i_last], rot_l[i_last])
                if args.speed > 0:
                    wait = (t[i_last] - t_sim0) / args.speed - (time.monotonic() - t_wall0)
                    if wait > 0:
                        time.sleep(wait)
            state["done"] = True

        threading.Thread(target=producer, daemon=True).start()
    os.makedirs(out, exist_ok=True)

    last_pub, last_print = 0.0, 0.0
    while not stop.is_set():
        time.sleep(0.2)
        now = time.monotonic()
        if rv is not None and now - last_pub >= args.publish_period:
            # Only for RViz: rebuilding the whole map every 2 s cost most of a replay's time (saved at the end anyway).
            last_pub = now
            lmap.merge()
            if rv is not None:
                with lock:
                    sc = np.concatenate([x for _, x in recent]) if recent else np.zeros((0, 3))
                    pose = state["pose"]
                if len(sc):
                    g = lmap.ground_at(sc[:, 0], sc[:, 1])
                    rv.scan_pub.publish(rv.cloud(sc, [np.nan_to_num(sc[:, 2] - g)], ["height"]))
                c, hits, h = lmap.voxels()
                if len(c) > args.max_publish:
                    sel = np.random.default_rng(0).choice(len(c), args.max_publish, replace=False)
                    c, hits, h = c[sel], hits[sel], h[sel]
                if len(c):
                    rv.map_pub.publish(rv.cloud(c, [h, hits], ["height", "hits"]))
                x, y, zmin, zmax = lmap.columns()
                sel = zmin <= lmap.ground_at(x, y) + 0.5          # columns whose lowest return is the ground
                if sel.any():
                    rv.ground_pub.publish(rv.cloud(np.stack([x[sel], y[sel], zmin[sel]], axis=1), [], []))
                if pose is not None:
                    gz_ = lmap.ground_at(np.array([pose[0][0]]), np.array([pose[0][1]]))[0]
                    gz_ = 0.0 if not np.isfinite(gz_) else gz_
                    rv.vehicle(pose[0], pose[1], camera_footprint(cfg, pose[0], pose[1], gz_))
                rv.rclpy.spin_once(rv.node, timeout_sec=0)
        if now - last_print >= 5 and not state.get("told"):      # quiet once the replay has finished
            last_print = now
            print(f"t {state['t'] or 0:8.1f} s  points {lmap.points:,}  voxels {lmap.n_voxels():,}  "
                  f"columns {lmap.n_columns():,}", flush=True)
        if state.get("done"):
            if rv is None:
                break
            if not state.get("told"):
                print("replay finished; the map stays in RViz until Ctrl+C", flush=True)
                state["told"] = True
                info = lmap.save(out, args.ply)
                print("lidar_map: " + json.dumps(info), flush=True)
    if not state.get("told"):
        info = lmap.save(out, args.ply)
        print("lidar_map: " + json.dumps(info), flush=True)
    if rv is not None:
        rv.node.destroy_node()
        rv.rclpy.try_shutdown()
    os._exit(0)


if __name__ == "__main__":
    main()
