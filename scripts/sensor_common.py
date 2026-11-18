"""Shared by lidar_node.py and camera_node.py: gz bindings, config, the vehicle's pose feed.

Import this before any gz.msgs module: it picks the protobuf backend.
"""

import bisect
import json
import os
import threading

# The distro's gz.msgs bindings are generated for protobuf 3: run with `python3 -s` (system protobuf 3.12,
# C++ backend, fast) as the launch scripts do; with a user-site protobuf 4+ only the pure-Python backend works.
try:
    import google.protobuf
    if int(google.protobuf.__version__.split(".")[0]) >= 4:
        os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")
    else:
        os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "cpp")
except ImportError:
    pass

import numpy as np
from gz.msgs10.pose_v_pb2 import Pose_V

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)


def load_config(path=None):
    with open(path or os.path.join(ROOT, "config", "sensors.json")) as f:
        return json.load(f)


def stamp_s(header):
    return header.stamp.sec + header.stamp.nsec * 1e-9


def quat_to_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


class PoseFeed:
    """World pose of one model over sim time, from the world's pose topics (T_WB of the spec)."""

    def __init__(self, node, world, model, keep_s=30.0, log=None):
        self.model = model
        self.keep_s = keep_s
        self.t, self.p, self.q = [], [], []
        self.lock = threading.Lock()
        self.log = log                     # file: every new pose as t,x,y,z,qx,qy,qz,qw
        for topic in (f"/world/{world}/dynamic_pose/info", f"/world/{world}/pose/info"):
            if not node.subscribe(Pose_V, topic, self._on_poses):
                raise RuntimeError(f"failed to subscribe to {topic}")

    def _on_poses(self, msg):
        t = stamp_s(msg.header)
        for pose in msg.pose:
            if pose.name != self.model:
                continue
            p, q = pose.position, pose.orientation
            with self.lock:
                if self.t and t <= self.t[-1]:
                    return
                self.t.append(t)
                self.p.append((p.x, p.y, p.z))
                self.q.append((q.x, q.y, q.z, q.w))
                if self.log:
                    self.log.write(f"{t:.4f},{p.x:.4f},{p.y:.4f},{p.z:.4f},{q.x:.6f},{q.y:.6f},{q.z:.6f},{q.w:.6f}\n")
                while self.t and self.t[0] < t - self.keep_s:
                    self.t.pop(0), self.p.pop(0), self.q.pop(0)
            return

    def latest(self):
        with self.lock:
            if not self.t:
                return None
            return self.t[-1], np.array(self.p[-1]), np.array(self.q[-1])

    def at(self, t):
        """Interpolated (position, quaternion xyzw) at sim time t, or None if t is outside the buffer.

        A static model is published once (no later poses): its single pose holds for any t."""
        with self.lock:
            if not self.t:
                return None
            if len(self.t) == 1:
                return np.array(self.p[0]), np.array(self.q[0])
            i = bisect.bisect_left(self.t, t)
            if i == 0 or i == len(self.t):
                if abs(t - self.t[min(i, len(self.t) - 1)]) < 0.02:
                    j = min(i, len(self.t) - 1)
                    return np.array(self.p[j]), np.array(self.q[j])
                return None
            t0, t1 = self.t[i - 1], self.t[i]
            a = (t - t0) / (t1 - t0)
            p = (1 - a) * np.array(self.p[i - 1]) + a * np.array(self.p[i])
            q0, q1 = np.array(self.q[i - 1]), np.array(self.q[i])
            if np.dot(q0, q1) < 0:
                q1 = -q1
            q = (1 - a) * q0 + a * q1           # nlerp: the poses are 1 ms apart
            return p, q / np.linalg.norm(q)
