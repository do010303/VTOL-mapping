"""Can the nodes keep up on this machine? Rates, latency and resources, for the camera view's side panel and a log.

Sim-side rates (from the topics the view already receives, so it costs nothing extra):
  LiDAR lines per sim second / per wall second and gaps (spec 2: 200 lines/s, 1333 rays/line),
  preview frames per sim / wall second (10 Hz), mapping-camera frames (camera_info, one per image) and their
  spacing, real-time factor (/stats).
Latency (wall ms): how far behind the newest pose (500 Hz stream) a LiDAR line / preview frame arrives, scaled by
  the RTF, plus the view's own processing time per frame.
Resources (psutil, nvidia-smi, once a second): CPU and memory of each process of this example (Gazebo server and
  GUI, PX4, the nodes) and of the machine, GPU use, memory and temperature.

Thresholds come from the spec's numbers; where it has none (CPU, GPU, latency) they are working margins.
"""

import csv
import os
import subprocess
import threading
import time

import cv2
import numpy as np
import psutil

ROLES = [  # (label, substring of the command line)
    ("gz server", "vtol_mapping_gazebo --verbose"),
    ("gz GUI", "vtol_mapping_gazebo -g"),
    ("PX4", "/bin/px4 "),
    ("lidar_node", "lidar_node.py"),
    ("camera_node", "camera_node.py"),
    ("view/video", "camera_view.py"),
]

GREEN, YELLOW, RED, GREY = (90, 200, 90), (60, 200, 230), (70, 70, 230), (190, 190, 190)


class PerfMonitor:
    def __init__(self, lines_per_s=200.0, rays=1333, preview_hz=10.0, log_path=None):
        self.lines_per_s, self.rays, self.preview_hz = lines_per_s, rays, preview_hz
        self.lock = threading.Lock()
        self.events = {"lidar": [], "preview": [], "caminfo": []}     # (sim t, wall t)
        self.gaps = 0
        self.lag = {"lidar": [], "preview": []}                         # sim s behind the newest pose
        self.proc_ms = []
        self.rtf = None
        self.res = {}
        self.cam_intervals = []
        self.log_path = log_path
        self.log = None
        if log_path:
            os.makedirs(os.path.dirname(os.path.abspath(log_path)), exist_ok=True)
            self.log = open(log_path, "w", newline="")
            self.writer = csv.writer(self.log)
            self.writer.writerow(["wall_s", "rtf", "lidar_lines_per_sim_s", "lidar_lines_per_wall_s", "lidar_gaps",
                                  "preview_fps_sim", "preview_fps_wall", "lidar_lag_ms", "preview_lag_ms",
                                  "view_ms", "cpu_total_pct", "ram_avail_mb", "gpu_util_pct", "gpu_mem_mb",
                                  "gpu_temp_c"] + [f"cpu_{r[0]}" for r in ROLES] + [f"rss_mb_{r[0]}" for r in ROLES])
        self.t_start = time.monotonic()
        self.procs = {}
        self.stop = threading.Event()
        threading.Thread(target=self._sample_loop, daemon=True).start()

    # --- events from the view's subscriptions -------------------------------------------------------
    def _add(self, key, t_sim, newest_pose_t):
        now = time.monotonic()
        with self.lock:
            ev = self.events[key]
            if key == "lidar" and ev and t_sim - ev[-1][0] > 1.5 / self.lines_per_s:
                self.gaps += 1
            if key == "caminfo" and ev:
                self.cam_intervals.append(t_sim - ev[-1][0])
                self.cam_intervals = self.cam_intervals[-20:]
            ev.append((t_sim, now))
            while ev and ev[0][1] < now - 5:
                ev.pop(0)
            if key in self.lag and newest_pose_t is not None:
                self.lag[key].append(max(0.0, newest_pose_t - t_sim))
                self.lag[key] = self.lag[key][-200:]

    def lidar(self, t_sim, newest_pose_t):
        self._add("lidar", t_sim, newest_pose_t)

    def preview(self, t_sim, newest_pose_t, proc_s):
        self._add("preview", t_sim, newest_pose_t)
        with self.lock:
            self.proc_ms.append(proc_s * 1000)
            self.proc_ms = self.proc_ms[-50:]

    def caminfo(self, t_sim):
        self._add("caminfo", t_sim, None)

    def stats(self, msg):
        self.rtf = msg.real_time_factor

    # --- resources ------------------------------------------------------------------------------------
    def _find_procs(self):
        found = {}
        for p in psutil.process_iter(["pid", "cmdline"]):
            cmd = " ".join(p.info["cmdline"] or [])
            for label, key in ROLES:
                if key in cmd and "pgrep" not in cmd:
                    found.setdefault(label, []).append(p)
        return found

    def _sample_loop(self):
        psutil.cpu_percent(None)
        last_find = 0
        while not self.stop.wait(1.0):
            if time.monotonic() - last_find > 10:
                self.procs = self._find_procs()
                for ps in self.procs.values():
                    for p in ps:
                        try:
                            p.cpu_percent(None)
                        except psutil.Error:
                            pass
                last_find = time.monotonic()
                continue
            res = {"cpu_total": psutil.cpu_percent(None), "ram_avail_mb": psutil.virtual_memory().available / 2**20,
                   "ncpu": psutil.cpu_count(), "proc": {}}
            for label, ps in self.procs.items():
                cpu = rss = 0.0
                for p in ps:
                    try:
                        cpu += p.cpu_percent(None)
                        rss += p.memory_info().rss / 2**20
                    except psutil.Error:
                        pass
                res["proc"][label] = (cpu, rss)
            try:
                o = subprocess.run(["nvidia-smi", "--query-gpu=utilization.gpu,memory.used,memory.total,temperature.gpu",
                                    "--format=csv,noheader,nounits"], capture_output=True, text=True, timeout=3).stdout
                u, mu, mt, tc = [float(x) for x in o.strip().split("\n")[0].split(",")]
                res["gpu"] = (u, mu, mt, tc)
            except Exception:
                res["gpu"] = None
            with self.lock:
                self.res = res
            self._log()

    # --- summary ----------------------------------------------------------------------------------------
    def summary(self):
        with self.lock:
            out = {"rtf": self.rtf, "gaps": self.gaps, "res": dict(self.res)}
            for key in ("lidar", "preview", "caminfo"):
                ev = self.events[key]
                if len(ev) > 1:
                    out[key + "_sim_rate"] = (len(ev) - 1) / max(ev[-1][0] - ev[0][0], 1e-9)
                    out[key + "_wall_rate"] = (len(ev) - 1) / max(ev[-1][1] - ev[0][1], 1e-9)
                else:
                    out[key + "_sim_rate"] = out[key + "_wall_rate"] = None
            rtf = self.rtf or 1.0
            for key in ("lidar", "preview"):
                lag = self.lag[key]
                out[key + "_lag_ms"] = float(np.median(lag)) * 1000 / max(rtf, 1e-3) if lag else None
            out["view_ms"] = float(np.mean(self.proc_ms)) if self.proc_ms else None
            out["cam_interval"] = float(np.median(self.cam_intervals)) if self.cam_intervals else None
            out["cam_frames"] = len(self.events["caminfo"])
        return out

    def _log(self):
        if not self.log:
            return
        s = self.summary()
        r = s["res"]
        g = r.get("gpu") or (None,) * 4
        procs = r.get("proc", {})
        row = [round(time.monotonic() - self.t_start, 1), s["rtf"], s["lidar_sim_rate"], s["lidar_wall_rate"], s["gaps"],
               s["preview_sim_rate"], s["preview_wall_rate"], s["lidar_lag_ms"], s["preview_lag_ms"], s["view_ms"],
               r.get("cpu_total"), r.get("ram_avail_mb"), g[0], g[1], g[3]]
        row += [procs.get(lbl, (None, None))[0] for lbl, _ in ROLES] + [procs.get(lbl, (None, None))[1] for lbl, _ in ROLES]
        self.writer.writerow([("" if v is None else (round(v, 2) if isinstance(v, float) else v)) for v in row])
        self.log.flush()

    def panel(self, height, width=360):
        """The side panel: one line per check, green / yellow / red against its threshold."""
        s = self.summary()
        img = np.full((height, width, 3), 24, np.uint8)
        y = [22]

        def line(text, colour=GREY, size=0.45, bold=1):
            cv2.putText(img, text, (10, y[0]), cv2.FONT_HERSHEY_SIMPLEX, size, colour, bold, cv2.LINE_AA)
            y[0] += 19

        def grade(v, good, ok, higher_better=True):
            if v is None:
                return GREY
            if higher_better:
                return GREEN if v >= good else YELLOW if v >= ok else RED
            return GREEN if v <= good else YELLOW if v <= ok else RED

        def fmt(v, f="{:.1f}"):
            return "-" if v is None else f.format(v)

        line("NODE PERFORMANCE", (255, 255, 255), 0.55, 1)
        y[0] += 4
        line(f"real-time factor  {fmt(s['rtf'], '{:.2f}')}", grade(s["rtf"], 0.8, 0.5))
        line("LiDAR (spec: 200 lines/s x 1333)", (230, 230, 230))
        ls = s["lidar_sim_rate"]
        line(f"  lines/sim-s {fmt(ls)}   gaps {s['gaps']}",
             GREEN if ls and abs(ls - self.lines_per_s) < 2 and s["gaps"] == 0 else (RED if s["gaps"] else YELLOW))
        line(f"  lines/wall-s {fmt(s['lidar_wall_rate'])}   rays/sim-s {fmt(ls * self.rays if ls else None, '{:,.0f}')}")
        line(f"  latency {fmt(s['lidar_lag_ms'], '{:.0f}')} ms", grade(s["lidar_lag_ms"], 50, 200, False))
        line("Camera view (preview, 10 Hz)", (230, 230, 230))
        ps = s["preview_sim_rate"]
        line(f"  fps sim {fmt(ps)}   fps wall {fmt(s['preview_wall_rate'])}",
             GREEN if ps and abs(ps - self.preview_hz) < 0.6 else YELLOW)
        line(f"  latency {fmt(s['preview_lag_ms'], '{:.0f}')} ms   draw {fmt(s['view_ms'], '{:.0f}')} ms",
             grade((s["preview_lag_ms"] or 0) + (s["view_ms"] or 0), 150, 400, False))
        line("Mapping camera (15 MP, trigger)", (230, 230, 230))
        line(f"  frames (last 5 s) {s['cam_frames']}   interval {fmt(s['cam_interval'], '{:.2f}')} s")
        r = s["res"]
        y[0] += 4
        line("MACHINE", (255, 255, 255), 0.55, 1)
        line(f"CPU total {fmt(r.get('cpu_total'), '{:.0f}')} %  ({r.get('ncpu', '-')} threads)",
             grade(r.get("cpu_total"), 70, 90, False))
        line(f"RAM free {fmt(r.get('ram_avail_mb'), '{:,.0f}')} MB", grade(r.get("ram_avail_mb"), 2000, 1000))
        g = r.get("gpu")
        if g:
            line(f"GPU {g[0]:.0f} %   {g[1]:,.0f}/{g[2]:,.0f} MB   {g[3]:.0f} C",
                 GREEN if g[0] < 85 and g[1] / g[2] < 0.9 and g[3] < 85 else YELLOW)
        else:
            line("GPU -")
        y[0] += 4
        line("process        CPU %    RAM MB", (230, 230, 230))
        for label, _ in ROLES:
            cpu, rss = r.get("proc", {}).get(label, (None, None))
            if cpu is None:
                continue
            line(f"{label:<13}{cpu:6.0f}  {rss:8,.0f}", grade(cpu, 90, 100, False))
        y[0] += 6
        line("CPU % is per core (100 = one core)", (140, 140, 140), 0.4)
        if self.log_path:
            line(f"log: {os.path.basename(self.log_path)}", (140, 140, 140), 0.4)
        return img
