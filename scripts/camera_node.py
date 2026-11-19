#!/usr/bin/env python3
"""Camera node: saves every image of the nadir camera with its sim time and pose.

Writes to <out>/camera/:
  img_<id>.png      RGB image (or .jpg with --format jpg: JPEG, --quality, spec 8 allows PNG or JPEG);
                    --scale 2 writes it at half the rendered size (test runs; K in camera_info.json scaled)
  images.csv        image_id, t_s, file, width, height, x_m, y_m, z_m, qx, qy, qz, qw (vehicle pose at t_s),
                    trigger_t_s (when the trigger for it was sent, flight only)
  camera_info.json  K, size, distortion as Gazebo publishes them
  stats.json        frames, rate, decode/write timing

With --trigger-every D (flight model, camera on a trigger topic) the node fires the camera each
time the vehicle has moved D metres horizontally (spec 3: every 60 m), while above --min-alt. The check runs on
every pose message (500 Hz sim time), so the spacing does not depend on how fast the sim runs. With --on-line
only while the vehicle flies a survey line: course north or south within 10 deg, roll under 8 deg (as
scripts/strips.py finds the lines in the reports); the 60 m count restarts at each line.

  scripts/camera_node.py --world forest_static --model vtol_rig --out out/static_500 --duration 10
"""

import argparse
import json
import os
import queue
import signal
import sys
import threading
import time

from sensor_common import PoseFeed, load_config, stamp_s   # first: protobuf backend

import cv2
import numpy as np
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.camera_info_pb2 import CameraInfo
from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node


class CameraRecorder:
    def __init__(self, cfg, out, poses, png_level=1, fmt="png", quality=85, scale=1):
        self.cfg = cfg["camera"]
        self.dir = os.path.join(out, "camera")
        os.makedirs(self.dir, exist_ok=True)
        self.poses = poses
        self.png_level = png_level
        self.fmt = fmt
        self.quality = quality
        self.scale = scale
        self.lock = threading.Lock()
        self.n = 0
        self.first_t = self.last_t = None
        self.first_wall = None
        self.intervals = []
        self.write_s = []
        self.info = None
        self.pending_triggers = []
        self.csv = open(os.path.join(self.dir, "images.csv"), "w", buffering=1)
        self.csv.write("image_id,t_s,file,width,height,x_m,y_m,z_m,qx,qy,qz,qw,trigger_t_s\n")
        self.q = queue.Queue(maxsize=8)
        self.writer = threading.Thread(target=self._write_loop, daemon=True)
        self.writer.start()

    def on_info(self, msg):
        if self.info is None:
            k = list(msg.intrinsics.k)
            s = self.scale
            if s > 1:                     # the written images are s times smaller: pixel centres (x + 0.5) / s - 0.5
                k = [k[0] / s, 0.0, (k[2] + 0.5) / s - 0.5, 0.0, k[4] / s, (k[5] + 0.5) / s - 0.5, 0.0, 0.0, 1.0]
            self.info = {"width": msg.width // s, "height": msg.height // s, "K": [k[0:3], k[3:6], k[6:9]],
                         "distortion": list(msg.distortion.k), "rendered_size": [msg.width, msg.height],
                         "scale": s}
            with open(os.path.join(self.dir, "camera_info.json"), "w") as f:
                json.dump(self.info, f, indent=2)

    def on_image(self, msg):
        t = stamp_s(msg.header)
        with self.lock:
            if self.first_t is None:
                self.first_t, self.first_wall = t, time.monotonic()
            else:
                self.intervals.append(t - self.last_t)
            self.last_t = t
            image_id = self.n
            self.n += 1
            trig = self.pending_triggers.pop(0) if self.pending_triggers else None
        pose = self.poses.at(t) if self.poses else None
        try:
            self.q.put_nowait((image_id, t, msg.width, msg.height, msg.data, pose, trig))
        except queue.Full:
            print(f"camera_node: writer behind, image {image_id} not saved", flush=True)

    def _write_loop(self):
        while True:
            item = self.q.get()
            if item is None:
                return
            image_id, t, w, h, data, pose, trig = item
            t0 = time.monotonic()
            for _ in range(20):          # the first pose message may come after the first image
                if pose is not None or self.poses is None:
                    break
                time.sleep(0.05)
                pose = self.poses.at(t)
            img = np.frombuffer(data, dtype=np.uint8).reshape(h, w, 3)
            if self.scale > 1:
                img = cv2.resize(img, (w // self.scale, h // self.scale), interpolation=cv2.INTER_AREA)
                h, w = img.shape[:2]
            name = f"img_{image_id:05d}.{self.fmt}"
            opts = [cv2.IMWRITE_JPEG_QUALITY, self.quality] if self.fmt == "jpg" else \
                [cv2.IMWRITE_PNG_COMPRESSION, self.png_level]
            cv2.imwrite(os.path.join(self.dir, name), img[:, :, ::-1], opts)
            p, q = pose if pose is not None else ((np.nan,) * 3, (np.nan,) * 4)
            self.csv.write(f"{image_id},{t:.4f},{name},{w},{h},{p[0]:.4f},{p[1]:.4f},{p[2]:.4f},"
                           f"{q[0]:.6f},{q[1]:.6f},{q[2]:.6f},{q[3]:.6f},{'' if trig is None else f'{trig:.4f}'}\n")
            self.write_s.append(time.monotonic() - t0)

    def stats(self):
        with self.lock:
            sim = (self.last_t - self.first_t) if self.n > 1 else 0.0
            iv = np.array(self.intervals) if self.intervals else np.array([np.nan])
            return {"frames": self.n, "sim_s": sim,
                    "interval_mean_s": float(np.nanmean(iv)), "interval_max_s": float(np.nanmax(iv)),
                    "write_mean_s": float(np.mean(self.write_s)) if self.write_s else None,
                    "camera_info": self.info}

    def close(self):
        self.q.put(None)
        self.writer.join(timeout=60)
        self.csv.close()
        s = self.stats()
        with open(os.path.join(self.dir, "stats.json"), "w") as f:
            json.dump(s, f, indent=2)
        return s


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--world", required=True)
    ap.add_argument("--model", required=True, help="vehicle model (its pose goes with each image)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--config", help="sensors.json (default config/sensors.json)")
    ap.add_argument("--duration", type=float, default=0, help="stop after this much sim time, s (0: on SIGINT)")
    ap.add_argument("--max-frames", type=int, default=0, help="stop after this many images")
    ap.add_argument("--trigger-every", type=float, default=0, help="fire the camera every D m (triggered camera)")
    ap.add_argument("--min-alt", type=float, default=300, help="only trigger above this altitude, m")
    ap.add_argument("--trigger-box", type=float, nargs=4, metavar=("XMIN", "XMAX", "YMIN", "YMAX"),
                    help="only trigger while the vehicle is inside this area (world x, y), m")
    ap.add_argument("--format", choices=("png", "jpg"), default="png", help="image files")
    ap.add_argument("--quality", type=int, default=85, help="JPEG quality")
    ap.add_argument("--scale", type=int, default=1, help="write images this many times smaller (test runs)")
    ap.add_argument("--on-line", action="store_true", help="trigger only on survey lines (N/S course, level)")
    ap.add_argument("--line-x", type=float, nargs="*", help="with --on-line: x of the planned lines (within 20 m)")
    args = ap.parse_args()

    cfg = load_config(args.config)
    node = Node()
    trig = {"last_xy": None, "off_since": None, "pub": None}
    sin_course, sin_roll = np.sin(np.radians(10.0)), np.sin(np.radians(8.0))

    def on_pose(t, p, q):
        """Camera trigger, on every pose (sim time). Phase of the D m spacing kept through short gate drops."""
        if trig["pub"] is None:
            return
        box = args.trigger_box
        ok = p[2] >= args.min_alt and (box is None or (box[0] <= p[0] <= box[1] and box[2] <= p[1] <= box[3]))
        if ok and args.on_line:
            back = poses.at(t - 1.0)
            if back is None:
                ok = False
            else:
                v = p[:2] - back[0][:2]
                sp = np.hypot(*v)
                x, y, z, w = q
                roll_s = 2 * (y * z + w * x)                       # R[2, 1]: sin(roll) for small pitch
                ok = sp > 5 and abs(v[0]) < sin_course * sp and abs(roll_s) < sin_roll
                if ok and args.line_x:
                    # Within 20 m of a planned line (scripts/strips.py max_offset): the straight leg back to the
                    # landing also runs north-south over the area.
                    ok = min(abs(p[0] - lx) for lx in args.line_x) <= 20.0
        if not ok:
            if trig["off_since"] is None:
                trig["off_since"] = t
            elif t - trig["off_since"] > 2.0:
                trig["last_xy"] = None                             # next line: the count starts again
            return
        trig["off_since"] = None
        last = trig["last_xy"]
        d = np.hypot(*(p[:2] - last)) if last is not None else None
        if last is None or d >= args.trigger_every:
            # The spacing point, not where this pose happened to be (at 500 Hz and 25 m/s: 5 cm past it).
            trig["last_xy"] = p[:2].copy() if last is None else last + (p[:2] - last) * args.trigger_every / d
            with rec.lock:
                rec.pending_triggers.append(t)
            trig["pub"].publish(Boolean(data=True))

    poses = PoseFeed(node, args.world, args.model, on_pose=on_pose)
    rec = CameraRecorder(cfg, args.out, poses, fmt=args.format, quality=args.quality, scale=args.scale)
    c = cfg["camera"]
    if not node.subscribe(Image, c["topic"], rec.on_image):
        sys.exit(f"failed to subscribe to {c['topic']}")
    node.subscribe(CameraInfo, c["info_topic"], rec.on_info)
    trig["pub"] = node.advertise(c["trigger_topic"], Boolean) if args.trigger_every else None

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    print(f"camera_node: recording {c['topic']} -> {rec.dir}", flush=True)
    first_sim = None
    last_print = time.monotonic()
    while not stop.wait(0.05):
        cur = poses.latest()
        if cur is not None:
            first_sim = cur[0] if first_sim is None else first_sim
            if args.duration and cur[0] - first_sim >= args.duration:
                break
        if args.max_frames and rec.n >= args.max_frames:
            time.sleep(0.5)
            break
        if time.monotonic() - last_print >= 2 and rec.n:
            last_print = time.monotonic()
            s = rec.stats()
            print(f"camera frames {s['frames']:4d}  interval {s['interval_mean_s']:.3f} s (sim)  "
                  f"write {s['write_mean_s'] or 0:.2f} s", flush=True)
    s = rec.close()
    print("camera_node: " + json.dumps(s), flush=True)
    os._exit(0)          # gz-transport's callback threads abort the interpreter's teardown


if __name__ == "__main__":
    main()
