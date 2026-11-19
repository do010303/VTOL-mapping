"""Ground truth of a generated forest scene (models/<scene>/: truth.npz, trees.csv), shared by the reports.

Which scene a run used is recorded in its sensors.json ("scene"), written by run_sitl.sh / static_run.sh.
Runs from before 19/11/2026 have no such key: they were all flown over the 1.4 km forest, now models/forest_1400.

World -> scene: forest, forest_static, flat_static -> forest; forest_slope, forest_slope_static -> forest_slope.
"""

import json
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SIDES = 7
LEGACY_SCENE = "forest_1400"


def scene_of_world(world):
    if world.startswith("forest_slope"):
        return "forest_slope"
    if world.startswith("forest_1400"):
        return "forest_1400"
    return "forest"


def scene_of_run(run):
    """Scene name of a run directory (its sensors.json), legacy runs -> forest_1400."""
    try:
        cfg = json.load(open(os.path.join(run, "sensors.json")))
    except OSError:
        return LEGACY_SCENE
    return cfg.get("scene", LEGACY_SCENE)


class Truth:
    """Rasters (DTM, DSM, CHM on a 1 m grid), the exact terrain and the tree list of a scene. z is absolute
    (world ENU); on the flat scenes the terrain is z = 0."""

    def __init__(self, scene="forest"):
        self.scene = scene
        d = os.path.join(ROOT, "models", scene)
        z = np.load(os.path.join(d, "truth.npz"))
        self.x, self.y, self.dsm = z["x"], z["y"], z["dsm"]
        self.dtm = z["dtm"] if "dtm" in z.files else np.zeros_like(self.dsm)
        self.cell = self.x[1] - self.x[0]
        if "tz" in z.files:
            self.tx, self.ty, self.tz = z["tx"], z["ty"], z["tz"]
        else:
            self.tx = self.ty = np.array([-1e5, 1e5])
            self.tz = np.zeros((2, 2))
        self.flat = bool(np.all(self.tz == 0))
        self.trees = np.genfromtxt(os.path.join(d, "trees.csv"), delimiter=",", names=True, dtype=None, encoding=None)
        names = self.trees.dtype.names
        self.tree_z0 = self.trees["ground_z_m"] if "ground_z_m" in names else np.zeros(len(self.trees))
        self.mesh_dir = os.path.join(d, "meshes")

    def dtm_at(self, x, y):
        """Exact terrain height (the triangulated mesh of make_forest.Terrain) at x, y."""
        x = np.asarray(x, dtype=np.float64)
        y = np.asarray(y, dtype=np.float64)
        if self.flat:
            return np.zeros(np.broadcast(x, y).shape)
        j = np.clip(np.searchsorted(self.tx, x, side="right") - 1, 0, len(self.tx) - 2)
        i = np.clip(np.searchsorted(self.ty, y, side="right") - 1, 0, len(self.ty) - 2)
        u = np.clip((x - self.tx[j]) / (self.tx[j + 1] - self.tx[j]), 0, 1)
        v = np.clip((y - self.ty[i]) / (self.ty[i + 1] - self.ty[i]), 0, 1)
        z00, z10 = self.tz[i, j], self.tz[i, j + 1]
        z01, z11 = self.tz[i + 1, j], self.tz[i + 1, j + 1]
        return np.where(u >= v, z00 + u * (z10 - z00) + v * (z11 - z10), z00 + v * (z01 - z00) + u * (z11 - z01))

    def surface_at(self, x, y):
        """Exact height of the highest surface (crown or terrain) above each (x, y), from the tree list."""
        from scipy.spatial import cKDTree
        tr = self.trees
        if not hasattr(self, "kd"):
            self.kd = cKDTree(np.stack([tr["x_m"], tr["y_m"]], axis=1))
        out = self.dtm_at(x, y)
        sector = 2 * np.pi / SIDES
        _, idx = self.kd.query(np.stack([x, y], axis=1), k=12, distance_upper_bound=5.0)
        for c in range(idx.shape[1]):
            i = idx[:, c]
            ok = i < len(tr)
            ii = i[ok]
            dx, dy = x[ok] - tr["x_m"][ii], y[ok] - tr["y_m"][ii]
            d = np.hypot(dx, dy)
            a = np.mod(np.arctan2(dy, dx) - tr["yaw_rad"][ii], sector) - sector / 2
            edge = tr["crown_radius_m"][ii] * np.cos(sector / 2) / np.cos(a)
            h, low = tr["height_m"][ii], tr["crown_low_m"][ii]
            z = np.where(d <= edge, self.tree_z0[ii] + h - (h - low) * d / edge, -np.inf)
            out[ok] = np.maximum(out[ok], z)
        return out

    def _raster_at(self, r, x, y):
        j = np.round((x - self.x[0]) / self.cell).astype(int)
        i = np.round((y - self.y[0]) / self.cell).astype(int)
        ok = (i >= 0) & (i < len(self.y)) & (j >= 0) & (j < len(self.x))
        out = np.full(np.shape(x), np.nan)
        out[ok] = r[i[ok], j[ok]]
        return out

    def dsm_at(self, x, y):
        return self._raster_at(self.dsm, x, y)

    def meshes(self):
        """OBJ files of the scene (terrain + tree tiles), for ray casting."""
        return sorted(os.path.join(self.mesh_dir, f) for f in os.listdir(self.mesh_dir) if f.endswith(".obj"))
