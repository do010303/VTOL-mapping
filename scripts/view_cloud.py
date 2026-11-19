#!/usr/bin/env python3
"""The LiDAR map of a flight in a 3D viewer (Open3D, no ROS / RViz), with the flight around it.

Reads <run>/lidar_map.npz (scripts/lidar_map.py: 1 m voxels, height above the estimated ground per voxel) and
<run>/trajectory.csv, <run>/camera/images.csv, <run>/sensors.json. Layers (tick them on/off in the panel):

  ground          voxels within 0.5 m of the ground, grey by elevation (relief)
  vegetation      0.5 m .. 5 m above the ground (trunks, undergrowth), colour by height (turbo)
  canopy          above 5 m, same colour scale (blue low .. red top of the crowns)
  flight path     the whole flight, grey; the survey lines found in it (scripts/strips.py) bright, labelled
  camera          a yellow dot per photo, its footprint on the ground (every --footprint-every th photo)
  grid            100 m on the ground, with x / y labels every 200 m; E (x), N (y), Up axes at the origin

By default only the survey area (the lines' swaths) is shown: the map also holds the scans of the turns and of
the climb, sparse and far out; --whole shows them too. A label above the map gives the numbers: area, voxels,
how much of the area the LiDAR reached the ground in, canopy height, and the flight report's (swath, points/m^2,
coverage, DSM accuracy, photo overlaps) when scripts/flight_report.py has run.

  python3 scripts/lidar_map.py out/<run>                  # once: builds lidar_map.npz
  python3 scripts/view_cloud.py out/<run>                 # the viewer
  python3 scripts/view_cloud.py out/<run> --crop -100 100 -100 100 --max-points 0   # one block, every voxel
  python3 scripts/view_cloud.py out/<run> --colour hits   # vegetation/canopy by returns per voxel instead
  python3 scripts/view_cloud.py out/<run> --png map.png   # a picture, no window

Mouse: left drag turns, wheel zooms, shift + left drag (or middle) pans. The panel (top right, "Settings" ...)
ticks layers on and off and sets point size and lighting.
"""
import argparse
import json
import os
import sys

import numpy as np
from matplotlib import colormaps

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from flight_report import interp_poses  # noqa: E402
from strips import segment  # noqa: E402

GROUND_BAND = 0.5          # m above the ground estimate: ground layer
UNDER_TOP = 5.0            # m: vegetation layer below, canopy above
THEMES = {      # background, text, grid, flight path, survey lines, photos
    "light": dict(bg=(0.97, 0.97, 0.96), text=(0.10, 0.11, 0.13), grid=(0.72, 0.73, 0.75), path=(0.45, 0.47, 0.52),
                  lines=(0.05, 0.40, 0.85), photo=(0.90, 0.45, 0.00)),
    "dark": dict(bg=(0.07, 0.08, 0.10), text=(0.92, 0.93, 0.95), grid=(0.30, 0.31, 0.35), path=(0.55, 0.57, 0.62),
                 lines=(0.20, 0.85, 1.00), photo=(1.00, 0.85, 0.20)),
}
FONT = "/usr/share/fonts/truetype/noto/NotoSans-Regular.ttf"


def turbo(v, lo, hi):
    return colormaps["turbo"](np.clip((v - lo) / max(hi - lo, 1e-6), 0, 1))[:, :3]


def relief(z):
    lo, hi = np.percentile(z, [1, 99]) if len(z) else (0, 1)
    t = np.clip((z - lo) / max(hi - lo, 1e-6), 0, 1)[:, None]
    return (1 - t) * np.array([0.30, 0.27, 0.24]) + t * np.array([0.62, 0.58, 0.52])


def lines(o3d, pts, segs, colour):
    ls = o3d.geometry.LineSet(o3d.utility.Vector3dVector(np.asarray(pts, float)),
                              o3d.utility.Vector2iVector(np.asarray(segs, int)))
    ls.paint_uniform_color(colour)
    return ls


def polyline(o3d, p, colour):
    return lines(o3d, p, np.column_stack((np.arange(len(p) - 1), np.arange(1, len(p)))), colour)


def cloud(o3d, xyz, rgb):
    pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.asarray(xyz, float)))
    pc.colors = o3d.utility.Vector3dVector(rgb)
    return pc


def flight_report(run):
    """The flight report's numbers (scripts/flight_report.py), if there is one: flight_report_exact.json first."""
    names = sorted((f for f in os.listdir(run) if f.startswith("flight_report") and f.endswith(".json")),
                   key=lambda f: (f != "flight_report_exact.json", "exact" not in f, f != "flight_report.json", f))
    if not names:
        return {}
    r = json.load(open(os.path.join(run, names[0])))
    r["_file"] = names[0]
    return r


def build(run, a):
    import open3d as o3d

    th = THEMES[a.theme]
    report = flight_report(run)
    strips, pp, rot = [], None, None
    tf = os.path.join(run, "trajectory.csv")
    if os.path.exists(tf):
        traj = np.loadtxt(tf, delimiter=",", skiprows=1)
        traj = traj[np.concatenate([[True], np.diff(traj[:, 0]) > 0])]
        ts = np.arange(traj[0, 0], traj[-1, 0], 0.2)
        pp, rot = interp_poses(traj, ts)
        _, strips = segment(ts, pp, rot, a.y_range, min_frac=0.5)
    if not a.crop and not a.whole and strips:
        # The survey area: what the lines' swaths cover (the map also holds the turns' and the climb's scans,
        # sparse, far out, with a poor ground estimate).
        half = report.get("swath_m", 450.0) / 2
        xs = [np.median(pp[st["first"]:st["last"] + 1, 0]) for st in strips]
        a.crop = (min(xs) - half, max(xs) + half, a.y_range[0] - 50, a.y_range[1] + 50)

    d = np.load(os.path.join(run, "lidar_map.npz"))
    c, h, hits = d["centres"], d["height"], d["hits"]
    vox = float(d["voxel_m"])
    cx, cy = d["col_x"], d["col_y"]
    if a.crop:
        x0, x1, y0, y1 = a.crop
        m = (c[:, 0] >= x0) & (c[:, 0] <= x1) & (c[:, 1] >= y0) & (c[:, 1] <= y1)
        c, h, hits = c[m], h[m], hits[m]
        mc = (cx >= x0) & (cx <= x1) & (cy >= y0) & (cy <= y1)
        cx, cy = cx[mc], cy[mc]

    # Numbers, on every voxel (before any subsampling).
    area = len(cx) * 1.0                                       # 1 m columns with a return
    g = h < GROUND_BAND
    gi = np.unique((np.floor(c[g, 0]).astype(np.int64) << 32) + np.floor(c[g, 1]).astype(np.int64))
    canopy = h[h >= UNDER_TOP]
    returns = int(hits.sum())
    stats = {"area_km2": area / 1e6, "voxels": len(c), "voxel_m": vox, "returns": returns,
             "returns_per_m2": returns / max(area, 1), "ground_reached": len(gi) / max(area, 1),
             "canopy_p95_m": float(np.percentile(canopy, 95)) if len(canopy) else 0.0,
             "canopy_mean_m": float(canopy.mean()) if len(canopy) else 0.0}

    n = len(c)
    if a.max_points and n > a.max_points:
        k = np.random.default_rng(0).choice(n, a.max_points, replace=False)
        c, h, hits = c[k], h[k], hits[k]
    p = c.astype(np.float64)
    p[:, 2] *= a.zscale
    top = np.percentile(h, 99) if len(h) else 1
    geoms, labels = [], []
    g = h < GROUND_BAND
    u = (h >= GROUND_BAND) & (h < UNDER_TOP)
    t = h >= UNDER_TOP
    # Colour scale from the bottom of the canopy to its top (most voxels seen from above are crowns: a 0..top scale
    # leaves them all orange); the vegetation layer below sits at the blue end.
    can = h[h >= UNDER_TOP]
    lo = max(UNDER_TOP, float(np.percentile(can, 5))) if len(can) else UNDER_TOP
    a.scale = (lo, top)
    vcol = turbo(np.log1p(hits), 0, np.log1p(np.percentile(hits, 99))) if a.colour == "hits" else turbo(h, lo, top)
    geoms.append(("ground", cloud(o3d, p[g], relief(c[g, 2]))))
    tu = np.clip((h[u] - GROUND_BAND) / (UNDER_TOP - GROUND_BAND), 0, 1)[:, None]
    ucol = vcol[u] if a.colour == "hits" else (1 - tu) * np.array([0.20, 0.45, 0.25]) + tu * np.array([0.45, 0.75, 0.35])
    geoms.append((f"undergrowth {GROUND_BAND:g}-{UNDER_TOP:g} m", cloud(o3d, p[u], ucol)))
    geoms.append((f"canopy > {UNDER_TOP:g} m", cloud(o3d, p[t], vcol[t])))
    zg = float(np.median(c[g, 2])) if g.any() else 0.0
    bb = (p[:, 0].min(), p[:, 0].max(), p[:, 1].min(), p[:, 1].max()) if len(p) else (-100, 100, -100, 100)

    # Flight path and survey lines.
    if pp is not None:
        q = pp.copy()
        q[:, 2] *= a.zscale
        geoms.append(("flight path", polyline(o3d, q[::5], th["path"])))
        sp, ss = [], []
        for st in strips:
            s = q[st["first"]:st["last"] + 1:5]
            ss += [[len(sp) + i, len(sp) + i + 1] for i in range(len(s) - 1)]
            sp += list(s)
            # Label where the line starts, on the ground (600 m up it is out of the picture); lines alternate
            # direction, so neighbours' labels sit at opposite ends.
            end = s[0]
            labels.append(((end[0] - 30, end[1] + (40 if st["dir"] == "S" else -40), zg * a.zscale + 40), f"line {st['id']}  x {np.median(s[:, 0]):.1f} m  "
                                              f"{'N' if st['dir'] == 'N' else 'S'}-bound  {abs(s[-1, 1] - s[0, 1]):.0f} m"))
        if sp:
            geoms.append(("survey lines", lines(o3d, sp, ss, th["lines"])))
        labels.append((q[0] + [0, 0, 15], "take-off / landing"))
        stats["strips"] = len(strips)
        stats["altitude_m"] = float(np.median(np.concatenate([pp[st["first"]:st["last"] + 1, 2] for st in strips])) - zg) \
            if strips else None

    # Photos: positions and footprints.
    imf = os.path.join(run, "camera", "images.csv")
    cfg_f = os.path.join(run, "sensors.json")
    if os.path.exists(imf) and os.path.exists(cfg_f):
        from lidar_map import camera_footprint
        cfg = json.load(open(cfg_f))
        im = np.genfromtxt(imf, delimiter=",", names=True, dtype=None, encoding=None)
        im = np.atleast_1d(im)
        ip = np.column_stack((im["x_m"], im["y_m"], im["z_m"] * a.zscale))
        geoms.append((f"camera positions ({len(im)} photos)", cloud(o3d, ip, np.tile(th["photo"], (len(ip), 1)))))
        from static_report import quat_to_matrix
        fp, fs = [], []
        inside = np.ones(len(im), bool) if not a.crop else \
            (im["x_m"] >= a.crop[0]) & (im["x_m"] <= a.crop[1]) & (im["y_m"] >= a.crop[2]) & (im["y_m"] <= a.crop[3])
        for k in np.nonzero(inside)[0][::max(a.footprint_every, 1)]:
            r = quat_to_matrix(im["qx"][k], im["qy"][k], im["qz"][k], im["qw"][k])
            corners = camera_footprint(cfg, np.array([im["x_m"][k], im["y_m"][k], im["z_m"][k]]), r, zg)
            if corners is None:
                continue
            b = len(fp)
            fp += [[x, y, (z + 0.5) * a.zscale] for x, y, z in corners[:4]]
            fs += [[b, b + 1], [b + 1, b + 2], [b + 2, b + 3], [b + 3, b]]
        if fp:
            geoms.append(("photo footprints", lines(o3d, fp, fs, th["photo"])))
        stats["photos"] = len(im)

    # Ground grid (100 m) and axes.
    x0, x1 = np.floor(bb[0] / 100) * 100, np.ceil(bb[1] / 100) * 100
    y0, y1 = np.floor(bb[2] / 100) * 100, np.ceil(bb[3] / 100) * 100
    zz = (zg - 0.3) * a.zscale
    gp, gs = [], []
    for x in np.arange(x0, x1 + 1, 100):
        gs.append([len(gp), len(gp) + 1]); gp += [(x, y0, zz), (x, y1, zz)]
        if x % 200 == 0:
            labels.append(((x, y0 - 25, zz), f"x {x:.0f}"))
    for y in np.arange(y0, y1 + 1, 100):
        gs.append([len(gp), len(gp) + 1]); gp += [(x0, y, zz), (x1, y, zz)]
        if y % 200 == 0:
            labels.append(((x0 - 60, y, zz), f"y {y:.0f}"))
    geoms.append(("grid 100 m", lines(o3d, gp, gs, th["grid"])))
    L = 50.0
    o = np.array([x0, y0, zz])                    # the axes at the grid's south-west corner
    ax = lines(o3d, [o, o + [L, 0, 0], o + [0, L, 0], o + [0, 0, L]], [[0, 1], [0, 2], [0, 3]], (1, 1, 1))
    ax.colors = o3d.utility.Vector3dVector([(0.85, 0.15, 0.15), (0.10, 0.60, 0.15), (0.15, 0.35, 0.90)])
    geoms.append(("axes E / N / Up", ax))
    labels += [(o + [L + 10, 0, 0], "E (x)"), (o + [0, L + 10, 0], "N (y)"), (o + [0, 0, L + 10], "Up")]

    s = stats
    where = "whole map" if not a.crop else f"x {a.crop[0]:.0f}..{a.crop[1]:.0f}, y {a.crop[2]:.0f}..{a.crop[3]:.0f} m"
    text = (f"{os.path.basename(run.rstrip('/'))}:  {s.get('strips', '?')} survey line(s)"
            + (f" at {s['altitude_m']:.0f} m AGL" if s.get("altitude_m") else "")
            + (f", {s['photos']} photos" if "photos" in s else "")
            + f"\nmap ({where}): {s['area_km2']:.2f} km2, {s['voxels']:,} voxels of {vox:g} m, "
            + f"{s['returns']:,} returns in the map"
            + f"\nground reached in {100 * s['ground_reached']:.0f} % of 1 m cells;  canopy mean "
            + f"{s['canopy_mean_m']:.1f} m, p95 {s['canopy_p95_m']:.1f} m above ground"
            + (f"\ncolour: blue {a.scale[0]:.0f} m .. red {a.scale[1]:.0f} m above ground (ground grey)"
               if a.colour == "height" else "\ncolour: LiDAR returns per voxel, blue few .. red many")
            + (f"\nshowing {len(c):,} of {n:,} voxels (--max-points 0: all)" if len(c) < n else ""))
    if report:
        r, im = report, report.get("images", {})
        text += (f"\nreport ({r['_file']}): swath {r.get('swath_m', 0):.0f} m, "
                 f"{r.get('density_pts_m2', 0):.0f} pts/m2, 1 m coverage {100 * r.get('coverage_1m', 0):.0f} %, "
                 f"DSM within 1 m {100 * r.get('dsm_1m', {}).get('within_1m', 0):.0f} %"
                 + (f"\nphoto overlap forward {100 * im['forward_overlap_at_30m_canopy']:.0f} %"
                    if im.get("forward_overlap_at_30m_canopy") is not None else "")
                 + (f", side {100 * im['side_overlap_at_30m_canopy']:.0f} %"
                    if im.get("side_overlap_at_30m_canopy") is not None else ""))
    info = [("Flight", [f"{s.get('strips', '?')} survey line(s)" + (f" at {s['altitude_m']:.0f} m AGL" if s.get("altitude_m") else ""),
                        f"{s['photos']} photos" if "photos" in s else ""]),
            ("LiDAR map", [where, f"{s['area_km2']:.2f} km\u00b2, {s['voxels']:,} voxels of {vox:g} m",
                           f"ground reached in {100 * s['ground_reached']:.0f} % of 1 m cells",
                           f"canopy height: mean {s['canopy_mean_m']:.1f} m, p95 {s['canopy_p95_m']:.1f} m",
                           f"showing {len(c):,} of {n:,} voxels" if len(c) < n else ""])]
    if report:
        r, im = report, report.get("images", {})
        rows = [f"swath {r.get('swath_m', 0):.0f} m,  {r.get('density_pts_m2', 0):.0f} points/m\u00b2",
                f"1 m cells covered {100 * r.get('coverage_1m', 0):.0f} %",
                f"DSM within 1 m of the truth: {100 * r.get('dsm_1m', {}).get('within_1m', 0):.0f} %"]
        if im.get("forward_overlap_at_30m_canopy") is not None:
            rows.append(f"photo overlap: forward {100 * im['forward_overlap_at_30m_canopy']:.0f} %, "
                        f"side {100 * im.get('side_overlap_at_30m_canopy', 0):.0f} %")
        info.append((f"Report ({r['_file'].replace('.json', '')})", rows))
    info = [(hd, [x for x in rows if x]) for hd, rows in info]
    return geoms, labels, stats, text, (bb, zz), info


def snapshot(path, geoms, box, point_size, close=False, theme="light"):
    """A picture from the south-west, above: Open3D's classic viewer in a hidden window (no labels)."""
    import open3d as o3d

    (x0, x1, y0, y1), zz = box
    vis = o3d.visualization.Visualizer()
    vis.create_window(width=1600, height=1000, visible=False)
    for name, g in geoms:
        if close and name.split()[0] in ("flight", "survey", "camera"):
            continue                       # a close-up: the flight 600 m up would set the zoom
        vis.add_geometry(g)
    opt = vis.get_render_option()
    opt.background_color = np.array(THEMES[theme]["bg"])
    opt.point_size = point_size
    ctl = vis.get_view_control()
    ctl.set_lookat([(x0 + x1) / 2, (y0 + y1) / 2, zz])
    ctl.set_front([-0.45, -0.65, 0.62])
    ctl.set_up([0.0, 0.0, 1.0])
    ctl.set_zoom(0.35)
    vis.poll_events()
    vis.update_renderer()
    vis.capture_screen_image(path, do_render=True)
    vis.destroy_window()
    print(f"wrote {path}")


class Viewer:
    """Our own window (Open3D GUI): the 3D view, and a panel on the right with the numbers, a tick box per layer,
    point size and the view buttons, in a readable font; labels in the 3D view in the theme's text colour."""

    def __init__(self, a, geoms, labels, info, box):
        import open3d as o3d
        from open3d.visualization import gui, rendering

        self.o3d, self.gui, self.rendering = o3d, gui, rendering
        self.a, self.geoms, self.box = a, geoms, box
        th = THEMES[a.theme]
        app = gui.Application.instance
        app.initialize()
        if os.path.exists(FONT):
            app.set_font(gui.Application.DEFAULT_FONT_ID, gui.FontDescription(FONT, point_size=a.font_size))
        bold = big = gui.Application.DEFAULT_FONT_ID            # fonts: added before the window (or it crashes)
        if os.path.exists(FONT.replace("Regular", "Bold")):
            bold = app.add_font(gui.FontDescription(FONT.replace("Regular", "Bold"), point_size=a.font_size))
            big = app.add_font(gui.FontDescription(FONT.replace("Regular", "Bold"), point_size=a.font_size + 4))
        self.win = w = app.create_window(f"LiDAR map - {os.path.basename(a.run.rstrip('/'))}", 1700, 1050)
        em = w.theme.font_size

        self.view = gui.SceneWidget()
        self.view.scene = rendering.Open3DScene(w.renderer)
        self.view.scene.set_background([*th["bg"], 1.0])
        self.view.scene.show_axes(False)
        self.view.scene.scene.set_sun_light([0.3, 0.3, -1.0], [1, 1, 1], 60000)
        for name, g in geoms:
            # Filament shows vertex colours as linear (washed out): decode sRGB first, on a copy (the panel's
            # swatches use the originals).
            g2 = type(g)(g)
            if g2.has_colors():
                g2.colors = o3d.utility.Vector3dVector(np.asarray(g.colors) ** 2.2)
            self.view.scene.add_geometry(name, g2, self.material(g2))
        for at, t in labels:
            lab = self.view.add_3d_label(np.asarray(at, np.float32), t)
            lab.color = gui.Color(*th["text"])
            lab.scale = 1.0
        w.add_child(self.view)

        dark, mid = gui.Color(*th["text"]), gui.Color(*(0.45 + 0.1 * np.array(th["text"])))

        def label(t, colour=dark, font=None):
            lb = gui.Label(t)
            lb.text_color = colour
            if font is not None:
                lb.font_id = font
            return lb

        def heading(t):
            self.panel.add_fixed(0.6 * em)
            self.panel.add_child(label(t, font=bold))

        def swatch(rgb, w=22, h=14):
            # A coloured box: an image (the font has no block glyph, a label's background is not drawn).
            img = np.zeros((h, w, 3), np.uint8)
            img[:] = (np.clip(np.asarray(rgb, float)[:3], 0, 1) * 255).astype(np.uint8)
            return gui.ImageWidget(o3d.geometry.Image(img))

        panel_bg = (0.92, 0.92, 0.91) if a.theme == "light" else (0.14, 0.15, 0.17)
        self.panel = gui.Vert(0.25 * em, gui.Margins(em, em, em, em))
        self.panel.background_color = gui.Color(*panel_bg)
        self.panel.add_child(label(os.path.basename(a.run.rstrip("/")), font=big))
        for hd, rows in info:
            heading(hd)
            for t in rows:
                self.panel.add_child(label(t))
        heading("Layers")
        for name, g in geoms:
            row = gui.Horiz(0.4 * em)
            cb = gui.Checkbox("")
            cb.checked = True
            cb.set_on_checked(lambda on, n=name: self.view.scene.show_geometry(n, on))
            row.add_child(cb)
            cols = np.asarray(g.colors)
            rgb = colormaps["turbo"](0.8)[:3] if name.startswith("canopy") and a.colour == "height" else \
                np.median(cols, axis=0) if len(cols) else th["text"]
            row.add_child(swatch(rgb))
            row.add_child(label(name))
            self.panel.add_child(row)
        if a.colour == "height":
            heading("Canopy colour (height above ground)")
            row = gui.Horiz(0)
            row.add_child(label(f"{a.scale[0]:.0f} m ", mid))
            bar = np.zeros((16, 240, 3), np.uint8)
            bar[:] = (colormaps["turbo"](np.linspace(0, 1, 240))[:, :3] * 255).astype(np.uint8)
            row.add_child(gui.ImageWidget(o3d.geometry.Image(bar)))
            row.add_child(label(f" {a.scale[1]:.0f} m", mid))
            self.panel.add_child(row)
        else:
            heading("Colour: LiDAR returns per voxel (blue few, red many)")
        heading("View")
        row = gui.Horiz(0.5 * em)
        row.add_child(label("Point size"))
        ps = gui.Slider(gui.Slider.INT)
        ps.set_limits(1, 8)
        ps.int_value = int(round(a.point_size))
        ps.set_on_value_changed(self.point_size)
        row.add_child(ps)
        self.panel.add_child(row)
        row = gui.Horiz(0.5 * em)
        for t, fn in (("Oblique", self.oblique), ("Top", self.top), ("Close-up", self.close)):
            b = gui.Button(t)
            b.background_color = gui.Color(0.16, 0.42, 0.78)
            b.set_on_clicked(fn)
            row.add_child(b)
        self.panel.add_child(row)
        save = gui.Button("Save picture (PNG)")
        save.background_color = gui.Color(0.16, 0.42, 0.78)
        save.set_on_clicked(self.save)
        self.panel.add_child(save)
        self.status = label("", mid)
        self.panel.add_child(self.status)
        self.panel.add_stretch()
        self.panel.add_child(label("Mouse: left drag turns, wheel zooms,", mid))
        self.panel.add_child(label("shift + left drag (or middle) pans", mid))
        w.add_child(self.panel)
        w.set_on_layout(self.layout)
        self.oblique()

    def material(self, g):
        m = self.rendering.MaterialRecord()
        if isinstance(g, self.o3d.geometry.LineSet):
            m.shader, m.line_width = "unlitLine", 2.0
        else:
            m.shader, m.point_size = "defaultUnlit", float(self.a.point_size)
        return m

    def layout(self, ctx):
        r = self.win.content_rect
        pw = min(30 * self.win.theme.font_size, r.width // 3)
        self.view.frame = self.gui.Rect(r.x, r.y, r.width - pw, r.height)
        self.panel.frame = self.gui.Rect(r.get_right() - pw, r.y, pw, r.height)

    def point_size(self, v):
        for name, g in self.geoms:
            if not isinstance(g, self.o3d.geometry.LineSet):
                m = self.material(g)
                m.point_size = float(v)
                self.view.scene.modify_geometry_material(name, m)

    def _look(self, front, dist):
        (x0, x1, y0, y1), zz = self.box
        c = np.array([(x0 + x1) / 2, (y0 + y1) / 2, zz])
        size = max(x1 - x0, y1 - y0)
        bb = self.o3d.geometry.AxisAlignedBoundingBox(c - [size / 2, size / 2, 10], c + [size / 2, size / 2, 60])
        self.view.setup_camera(45, bb, c)
        f = np.asarray(front, float)
        self.view.scene.camera.look_at(c, c + f / np.linalg.norm(f) * dist * size, [0, 0, 1] if f[2] < 0.99 else [0, 1, 0])

    def oblique(self):
        self._look([-0.45, -0.75, 0.6], 1.15)

    def top(self):
        self._look([0, 0, 1], 1.25)

    def close(self):
        self._look([-0.5, -0.8, 0.35], 0.25)

    def save(self):
        path = os.path.join(self.a.run, "view_cloud.png")

        def done(img):
            self.o3d.io.write_image(path, img)
            self.status.text = f"saved {path}"
        self.view.scene.scene.render_to_image(done)

    def run(self):
        self.gui.Application.instance.run()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run")
    ap.add_argument("--colour", choices=("height", "hits"), default="height",
                    help="vegetation and canopy by height above ground (default) or by LiDAR returns per voxel")
    ap.add_argument("--crop", type=float, nargs=4, metavar=("X0", "X1", "Y0", "Y1"),
                    help="only this area, m (default: the survey lines' swaths)")
    ap.add_argument("--whole", action="store_true", help="the whole map, turns and climb included")
    ap.add_argument("--max-points", type=int, default=3_000_000, help="random subsample above this (0: every voxel)")
    ap.add_argument("--zscale", type=float, default=1.0, help="vertical exaggeration")
    ap.add_argument("--point-size", type=float, default=2.0)
    ap.add_argument("--footprint-every", type=int, default=4, help="draw every n-th photo's footprint")
    ap.add_argument("--y-range", type=float, nargs=2, default=(-650, 650), help="where the survey lines are")
    ap.add_argument("--theme", choices=tuple(THEMES), default="light")
    ap.add_argument("--font-size", type=int, default=17, help="panel and label text, pt")
    ap.add_argument("--png", help="render a picture to this file instead of opening a window")
    a = ap.parse_args()
    a.user_crop = a.crop

    if not os.path.exists(os.path.join(a.run, "lidar_map.npz")):
        sys.exit(f"{a.run}/lidar_map.npz missing: run  python3 scripts/lidar_map.py {a.run}  first")
    geoms, labels, stats, text, box, info = build(a.run, a)
    print(text)
    if a.png:
        snapshot(a.png, geoms, box, a.point_size, close=bool(a.user_crop), theme=a.theme)
        return 0

    Viewer(a, geoms, labels, info, box).run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
