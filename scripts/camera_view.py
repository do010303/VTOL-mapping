#!/usr/bin/env python3
"""Camera view: the nadir camera's preview (same mount and field of view as the mapping camera, 792 x 528,
10 Hz) written to a video, with sim time, altitude, ground speed and a scale bar. One video frame per preview
frame, so the video runs at sim speed (10 fps = real time).

The LiDAR's scan line is drawn where it falls in the image (the ray lean puts it ~1 px behind the centre row),
with its swath as a band: the part of the view the LiDAR covers at this moment.

  scripts/camera_view.py --world forest --model standard_vtol_mapping_0 --out out/<run>/camera_view.mp4
  (run_sitl.sh and static_run.sh start it with the other nodes; VIEW=0 to leave it out)

  scripts/camera_window.sh       # live, in a window of its own (q or Esc closes it), next to a running sim
"""

import argparse
import json
import math
import os
import signal
import sys
import threading
import time

from sensor_common import PoseFeed, load_config, stamp_s   # first: protobuf backend

import cv2
import numpy as np
from gz.msgs10.image_pb2 import Image
from gz.msgs10.camera_info_pb2 import CameraInfo
from gz.msgs10.laserscan_pb2 import LaserScan
from gz.msgs10.world_stats_pb2 import WorldStatistics
from gz.transport13 import Node

from sensor_common import quat_to_matrix
from static_report import lidar_dirs
from perf_monitor import PerfMonitor

PROFILE_H = 190          # px, the cross-section panel under the image
LIDAR_KEEP_S = 1.0       # s of LiDAR lines drawn on the image


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--world", required=True)
    ap.add_argument("--model", required=True)
    ap.add_argument("--out", help="video file (.mp4)")
    ap.add_argument("--window", action="store_true", help="show it live in a window (q / Esc closes)")
    ap.add_argument("--scale", type=float, default=1.5, help="window size relative to the 792 x 528 preview")
    ap.add_argument("--config", help="sensors.json (default config/sensors.json)")
    ap.add_argument("--topic", default="/vtol/camera/preview")
    ap.add_argument("--min-alt", type=float, default=-1e9, help="only record above this altitude, m")
    ap.add_argument("--duration", type=float, default=0, help="stop after this much sim time, s")
    ap.add_argument("--no-lidar", action="store_true", help="without the LiDAR points and the cross-section panel")
    ap.add_argument("--no-perf", action="store_true", help="without the performance panel (rates, latency, CPU/GPU)")
    ap.add_argument("--perf-log", help="CSV of the performance checks, once a second (default: next to the video, "
                                       "or run/perf_<time>.csv for the window)")
    args = ap.parse_args()

    cfg = load_config(args.config)
    c, l = cfg["camera"], cfg["lidar"]
    node = Node()
    poses = PoseFeed(node, args.world, args.model)
    lock = threading.Lock()
    state = {"writer": None, "frames": 0, "t0": None, "last": None, "prev": None}
    if not args.out and not args.window:
        sys.exit("--out and/or --window")
    if args.out:
        os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    state["show"] = None
    perf = None
    if not args.no_perf:
        log = args.perf_log
        if log is None:
            log = (os.path.splitext(args.out)[0] + "_perf.csv") if args.out else \
                os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "run",
                             time.strftime("perf_%Y%m%d_%H%M%S.csv"))
        perf = PerfMonitor(l["lines_per_s"], l["rays_per_line"], 10.0, log)

    def newest_pose_t():
        cur = poses.latest()
        return cur[0] if cur else None

    # LiDAR: every 3rd ray of each line, in world coordinates (pose at the line's time), for the last second.
    d_l, beta = lidar_dirs(cfg)
    d_l = d_l[::3]
    t_bl = np.array(l["t_BL"])
    lines = []                                   # (t, world points (n, 3), across-track (n,), height (n,))

    def on_scan(msg):
        t = stamp_s(msg.header)
        if perf:
            perf.lidar(t, newest_pose_t())
        cur = poses.at(t)
        if cur is None:
            return
        p, q = cur
        r = np.asarray(msg.ranges, dtype=np.float64)[::3]
        ok = np.isfinite(r) & (r >= l["range_min_m"]) & (r <= l["range_max_m"])
        rot = quat_to_matrix(*q)
        pts_b = t_bl + r[ok, None] * d_l[ok]
        pts_w = pts_b @ rot.T + p
        with lock:
            lines.append((t, pts_w, pts_b[:, 1], pts_w[:, 2]))
            while lines and lines[0][0] < t - LIDAR_KEEP_S:
                lines.pop(0)

    def colour(z):
        return cv2.applyColorMap(np.clip(z / 30 * 255, 0, 255).astype(np.uint8).reshape(-1, 1),
                                 cv2.COLORMAP_VIRIDIS)[:, 0]

    def on_image(msg):
        t_proc = time.monotonic()
        t = stamp_s(msg.header)
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)[:, :, ::-1].copy()
        cur = poses.at(t) or (poses.latest()[1:] if poses.latest() else None)
        if cur is None:
            return
        p, q = cur
        if p[2] < args.min_alt:
            return
        h, w = img.shape[:2]
        scale = w / c["width"]
        fx = c["fx"] * scale
        cam_h = p[2] + c["t_BC"][2]                         # above the ground (flat terrain)
        # LiDAR swath in the image: the scan line is across the image (body y), at the centre row.
        half = math.tan(math.radians(l["scan_max_deg"]))
        u0, u1 = int(w / 2 - fx * half), int(w / 2 + fx * half)
        row = int(round(h / 2 - 0.5 + fx * 0.977e-3))      # ~1 mrad behind (calibrated ray lean)
        rot = quat_to_matrix(*q)
        profile = None
        if args.no_lidar:
            over = img.copy()
            cv2.line(over, (max(u0, 0), row), (min(u1, w - 1), row), (0, 0, 255), 2)
            img = cv2.addWeighted(over, 0.7, img, 0.3, 0)
        else:
            with lock:
                recent = [ln for ln in lines if ln[0] <= t + 0.01]
            if recent:
                # The last second of returns, projected into this image, coloured by height (viridis: 0-30 m).
                pw = np.concatenate([ln[1] for ln in recent])
                pc = ((pw - p) @ rot - np.array(c["t_BC"])) @ np.array(c["R_BC"])
                vis = pc[:, 2] > 1
                uu = (fx * pc[vis, 0] / pc[vis, 2] + c["cx"] * scale).astype(int)
                vv = (fx * pc[vis, 1] / pc[vis, 2] + c["cy"] * scale).astype(int)
                col = colour(pw[vis, 2])
                ins = (uu >= 1) & (uu < w - 1) & (vv >= 1) & (vv < h - 1)
                for du in (0, 1):
                    for dv in (0, 1):
                        img[vv[ins] + dv, uu[ins] + du] = col[ins]
                # Cross-section of the newest line: across track (left of the nose to the left) vs height.
                _, _, across, zz = recent[-1]
                profile = np.full((PROFILE_H, w, 3), 30, np.uint8)
                x_m = 2 * (cam_h + 30) * half                         # full swath width at the ground, m
                px_x = (w / 2 - across / x_m * (w - 40)).astype(int)
                z_lo, z_hi = -3.0, 35.0
                px_y = (PROFILE_H - 22 - (zz - z_lo) / (z_hi - z_lo) * (PROFILE_H - 40)).astype(int)
                for zl, txt in ((0, "0 m"), (10, "10"), (20, "20"), (30, "30")):
                    yl = int(PROFILE_H - 22 - (zl - z_lo) / (z_hi - z_lo) * (PROFILE_H - 40))
                    cv2.line(profile, (34, yl), (w - 6, yl), (70, 70, 70) if zl else (110, 110, 110), 1)
                    cv2.putText(profile, txt, (4, yl + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (170, 170, 170), 1, cv2.LINE_AA)
                cp = colour(zz)
                ok = (px_x >= 0) & (px_x < w) & (px_y >= 0) & (px_y < PROFILE_H)
                for du in (0, 1):
                    for dv in (0, 1):
                        profile[np.clip(px_y[ok] + dv, 0, PROFILE_H - 1), np.clip(px_x[ok] + du, 0, w - 1)] = cp[ok]
                g = float(np.mean(zz < 0.5)) if len(zz) else 0.0
                cv2.putText(profile, f"LiDAR cross-section (newest line, {len(zz)} of {len(beta)} rays shown; "
                                     f"ground {100 * g:.0f} %)   left <-  -> right",
                            (40, PROFILE_H - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (220, 220, 220), 1, cv2.LINE_AA)
        # Scale bar: 50 m on the ground.
        gsd = cam_h / fx if cam_h > 1 else float("nan")
        if np.isfinite(gsd):
            px = int(50 / gsd)
            cv2.rectangle(img, (14, h - 30), (14 + px, h - 24), (255, 255, 255), -1)
            cv2.putText(img, "50 m", (14, h - 36), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        with lock:
            if state["prev"] is not None and t > state["prev"][0]:
                v = np.hypot(*(p[:2] - state["prev"][1][:2])) / (t - state["prev"][0])
            else:
                v = 0.0
            state["prev"] = (t, p.copy())
            text = f"t {t:7.1f} s   alt {p[2]:6.1f} m   {v:4.1f} m/s   GSD(full res) {100 * cam_h / c['fx']:.1f} cm"
            cv2.rectangle(img, (0, 0), (w, 24), (0, 0, 0), -1)
            cv2.putText(img, text, (8, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            # The image's top is the nose: its compass heading.
            hdg = (90 - math.degrees(math.atan2(rot[1, 0], rot[0, 0]))) % 360
            cv2.putText(img, f"up = {hdg:03.0f} deg", (w - 120, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
                        cv2.LINE_AA)
            if not args.no_lidar:
                cv2.putText(img, "LiDAR points: last 1 s, colour = height 0-30 m", (8, 44),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
                img = np.vstack([img, profile if profile is not None else np.full((PROFILE_H, w, 3), 30, np.uint8)])
                h = img.shape[0]
            if perf:
                perf.preview(t, newest_pose_t(), time.monotonic() - t_proc)
                img = np.hstack([img, perf.panel(img.shape[0])])
            if state["t0"] is None:
                state["t0"] = t
            if args.out:
                if state["writer"] is None:
                    state["writer"] = cv2.VideoWriter(args.out, cv2.VideoWriter_fourcc(*"mp4v"), 10,
                                                      (img.shape[1], img.shape[0]))
                state["writer"].write(img)
            state["show"] = img
            state["frames"] += 1
            state["last"] = t

    if not node.subscribe(Image, args.topic, on_image):
        sys.exit(f"failed to subscribe to {args.topic}")
    if not args.no_lidar and not node.subscribe(LaserScan, l["topic"], on_scan):
        sys.exit(f"failed to subscribe to {l['topic']}")
    if perf:
        if args.no_lidar:
            node.subscribe(LaserScan, l["topic"], lambda m: perf.lidar(stamp_s(m.header), newest_pose_t()))
        node.subscribe(CameraInfo, c["info_topic"], lambda m: perf.caminfo(stamp_s(m.header)))
        node.subscribe(WorldStatistics, f"/world/{args.world}/stats", perf.stats)
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    print(f"camera_view: {args.topic} -> {args.out or 'window'}", flush=True)
    if args.window:
        name = "Camera view (nadir) - VTOL-mapping"
        cv2.namedWindow(name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(name, int((792 + (0 if args.no_perf else 360)) * args.scale * 0.85),
                         int((528 + (0 if args.no_lidar else PROFILE_H)) * args.scale * 0.85))
        blank = np.zeros((528, 792, 3), np.uint8)
        cv2.putText(blank, "waiting for /vtol/camera/preview ...", (180, 270), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (255, 255, 255), 1, cv2.LINE_AA)
        shown = False           # GTK reports the window invisible until it is mapped: only then can it be "closed"
        while not stop.is_set():
            with lock:
                img = state["show"]
            cv2.imshow(name, img if img is not None else blank)
            key = cv2.waitKey(50) & 0xFF
            visible = cv2.getWindowProperty(name, cv2.WND_PROP_VISIBLE) >= 1
            shown = shown or visible
            if key in (ord("q"), 27) or (shown and not visible):
                break
        cv2.destroyAllWindows()
    while not args.window and not stop.wait(0.5):
        with lock:
            if args.duration and state["t0"] is not None and state["last"] - state["t0"] >= args.duration:
                break
    with lock:
        if state["writer"] is not None:
            state["writer"].release()
        print("camera_view: " + json.dumps({"frames": state["frames"], "file": args.out}), flush=True)
    os._exit(0)


if __name__ == "__main__":
    main()
