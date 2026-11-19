#!/usr/bin/env python3
"""Procedural forest for the VTOL LiDAR + camera simulation (spec section 6).

Writes a static Gazebo model `models/<name>/` made of mesh tiles (low-poly trees: trunk prism + conical or bicone
crown, gaps between crowns so thin rays can reach the ground, clearings and dense patches), a textured terrain mesh,
and the ground truth taken directly from the scene:

  models/<name>/trees.csv        tree_id, x_m, y_m, height_m, crown_base_m, crown_radius_m, kind, crown_low_m, yaw_rad,
                                 ground_z_m (heights are above the ground at the trunk; ground_z_m is absolute)
  models/<name>/truth.npz        on a 1 m grid: DTM (terrain at the cell centre), DSM (highest surface in the cell),
                                 CHM = DSM - DTM, dsm_centre; plus the terrain mesh grid (tx, ty, tz) itself
  models/<name>/truth_chm.png    preview of the CHM
  models/<name>/forest.json      parameters (seed, size, terrain, counts)

Terrain (spec 6: "a flat test scene and a gently sloping forest"):
  flat   z = 0 everywhere
  slope  a 2.5 deg grade rising to the east plus smooth hills (+-8 m, ~300 m wide), inside the forest and 150 m
         around it; it blends to z = 0 within the next 450 m, so the take-off area (y = -1900) stays flat at z = 0.
         Mean height over the 800 x 800 m evaluation area is about 0, so a survey at 600 m above home is ~600 m AGL
         there (556-644 m across the 2.5 deg grade).

Textures (spec 6: varied, structured texture for SfM tie points): each of the 8 crown materials has its own
leaf-clump texture (4 m per repeat, a random offset per tree, so neighbouring crowns differ); the ground has a
2048 px soil/grass texture per 100 m.

World frame: ENU, metres; the forest is centred on the origin.

  python3 scripts/make_forest.py                            # models/forest: 2000 x 2000 m, flat, seed 2026
  python3 scripts/make_forest.py --terrain slope --name forest_slope
"""

import argparse
import json
import os

import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

SIDES = 7          # polygon sides of trunks and crowns
TRUNK_SIDES = 5
CROWN_UV_M = 4.0   # crown texture repeat, m
GROUND_UV_M = 100.0

# Crown greens (texture base colours), varied so the images have structure for SfM.
GREENS = [
    (0.13, 0.30, 0.10), (0.18, 0.38, 0.12), (0.10, 0.24, 0.09), (0.24, 0.42, 0.16),
    (0.16, 0.33, 0.20), (0.28, 0.40, 0.12), (0.09, 0.20, 0.12), (0.21, 0.30, 0.09),
]
TRUNK = (0.30, 0.22, 0.14)

SLOPE_DEG = 2.5
HILL_AMP_M = 8.0


def smooth_noise(rng, n_cells, shape):
    """Bilinear upsampling of a coarse random grid: values in [0, 1]."""
    coarse = rng.random((n_cells + 1, n_cells + 1))
    ys = np.linspace(0, n_cells, shape[0])
    xs = np.linspace(0, n_cells, shape[1])
    y0 = np.minimum(ys.astype(int), n_cells - 1)
    x0 = np.minimum(xs.astype(int), n_cells - 1)
    fy = (ys - y0)[:, None]
    fx = (xs - x0)[None, :]
    a = coarse[y0][:, x0]
    b = coarse[y0][:, x0 + 1]
    c = coarse[y0 + 1][:, x0]
    d = coarse[y0 + 1][:, x0 + 1]
    return (a * (1 - fx) + b * fx) * (1 - fy) + (c * (1 - fx) + d * fx) * fy


def density_at(field, size, x, y):
    n = field.shape[0]
    i = np.clip(((y + size / 2) / size * (n - 1)).astype(int), 0, n - 1)
    j = np.clip(((x + size / 2) / size * (n - 1)).astype(int), 0, n - 1)
    return field[i, j]


# ---------------------------------------------------------------------------------------------------------- terrain

class Terrain:
    """Height field on a tensor grid (tx, ty), triangulated per cell as (00, 10, 11) and (00, 11, 01): the mesh
    written to ground.obj and the function used for the truth are the same piecewise-linear surface."""

    def __init__(self, tx, ty, tz):
        self.tx, self.ty, self.tz = tx, ty, tz            # tz[i, j] at (tx[j], ty[i])

    @staticmethod
    def make(kind, size, rng):
        if kind == "flat":
            g = 2500.0
            ax = np.array([-g, g])
            return Terrain(ax, ax.copy(), np.zeros((2, 2)))
        from scipy.ndimage import gaussian_filter
        inner, blend, outer = size / 2 + 150, 450.0, 2500.0
        edge = inner + blend + 100
        fine = np.arange(-inner - 50, inner + 50 + 1e-6, 5.0)
        coarse_lo = np.arange(-edge, -inner - 50, 25.0)
        ax = np.concatenate([[-outer], coarse_lo, fine, -coarse_lo[::-1], [outer]])
        gx, gy = np.meshgrid(ax, ax)
        # Smooth hills: white noise on a 5 m lattice, Gaussian-filtered (sigma 60 m), scaled to +-HILL_AMP_M.
        n = int(np.ceil(2 * edge / 5.0)) + 1
        noise = gaussian_filter(rng.standard_normal((n, n)), sigma=12, mode="wrap")
        noise /= np.abs(noise).max()
        ii = np.clip(((gy + edge) / 5.0).round().astype(int), 0, n - 1)
        jj = np.clip(((gx + edge) / 5.0).round().astype(int), 0, n - 1)
        hills = HILL_AMP_M * noise[ii, jj]
        z = np.tan(np.radians(SLOPE_DEG)) * gx + hills
        # Blend to 0 outside the forest: smoothstep on the Chebyshev distance beyond `inner`.
        d = np.maximum(np.abs(gx), np.abs(gy))
        s = np.clip((d - inner) / blend, 0, 1)
        w = 1 - s * s * (3 - 2 * s)
        return Terrain(ax, ax.copy(), z * w)

    def height(self, x, y):
        x = np.asarray(x, dtype=np.float64)
        y = np.asarray(y, dtype=np.float64)
        j = np.clip(np.searchsorted(self.tx, x, side="right") - 1, 0, len(self.tx) - 2)
        i = np.clip(np.searchsorted(self.ty, y, side="right") - 1, 0, len(self.ty) - 2)
        u = np.clip((x - self.tx[j]) / (self.tx[j + 1] - self.tx[j]), 0, 1)
        v = np.clip((y - self.ty[i]) / (self.ty[i + 1] - self.ty[i]), 0, 1)
        z00, z10 = self.tz[i, j], self.tz[i, j + 1]
        z01, z11 = self.tz[i + 1, j], self.tz[i + 1, j + 1]
        lower = z00 + u * (z10 - z00) + v * (z11 - z10)          # triangle (00, 10, 11): u >= v
        upper = z00 + v * (z01 - z00) + u * (z11 - z01)          # triangle (00, 11, 01): u < v
        return np.where(u >= v, lower, upper)

    def triangles(self):
        X, Y = np.meshgrid(self.tx, self.ty)
        P = np.stack([X, Y, self.tz], axis=-1)
        p00, p10 = P[:-1, :-1], P[:-1, 1:]
        p01, p11 = P[1:, :-1], P[1:, 1:]
        t1 = np.stack([p00, p10, p11], axis=2).reshape(-1, 3, 3)
        t2 = np.stack([p00, p11, p01], axis=2).reshape(-1, 3, 3)
        return np.concatenate([t1, t2])


# ------------------------------------------------------------------------------------------------------------ trees

def make_trees(rng, size, spacing, terrain):
    # Two octaves: patches of a few hundred metres, and finer structure.
    field = 0.65 * smooth_noise(rng, 8, (512, 512)) + 0.35 * smooth_noise(rng, 32, (512, 512))
    g = np.arange(-size / 2 + spacing / 2, size / 2, spacing)
    gx, gy = np.meshgrid(g, g)
    x = gx.ravel() + rng.uniform(-0.45, 0.45, gx.size) * spacing
    y = gy.ravel() + rng.uniform(-0.45, 0.45, gx.size) * spacing
    d = density_at(field, size, x, y)
    # Clearings below 0.25, then denser with the field; 0.92 at most so there are always gaps.
    p = np.clip((d - 0.25) / 0.2, 0, 1) * 0.92
    keep = rng.random(x.size) < p
    x, y, d = x[keep], y[keep], d[keep]
    n = x.size
    # Taller where denser; 10-30 m.
    h = np.clip(10 + 20 * (0.5 * rng.beta(2.2, 2.0, n) + 0.5 * np.clip((d - 0.3) / 0.5, 0, 1)), 10, 30)
    conifer = rng.random(n) < 0.6
    r = 0.11 * h + rng.uniform(0.3, 1.2, n)                       # crown radius 1.4-4.5 m
    r[conifer] *= 0.8
    base = np.where(conifer, rng.uniform(0.30, 0.45, n), rng.uniform(0.45, 0.6, n)) * h
    green = rng.integers(0, len(GREENS), n)
    trunk_r = 0.012 * h + 0.08
    yaw = rng.uniform(0, 2 * np.pi, n)
    uv_off = rng.uniform(0, CROWN_UV_M, (n, 2))
    z0 = terrain.height(x, y)
    low = np.where(conifer, base, base + 0.45 * (h - base))
    return dict(x=x, y=y, h=h, r=r, base=base, conifer=conifer, green=green, trunk_r=trunk_r, yaw=yaw,
                uv_off=uv_off, z0=z0, low=low)


class Obj:
    """Accumulates triangles per material and writes a Wavefront OBJ with flat normals. Materials listed in
    uv_scale get planar texture coordinates (x + offset, y + offset) / scale, with a per-chunk offset."""

    def __init__(self, uv_scale=None):
        self.tris = {}
        self.uv_scale = uv_scale or {}

    def add(self, material, tris, uv_offset=(0.0, 0.0)):
        t = np.asarray(tris, dtype=np.float64).reshape(-1, 3, 3)
        off = np.broadcast_to(np.asarray(uv_offset, dtype=np.float64), (len(t), 2))
        self.tris.setdefault(material, []).append((t, off))

    def write(self, path, mtl_name):
        """One vertex per triangle corner and one normal per triangle (flat shading)."""
        lines = [f"mtllib {mtl_name}"]
        v_next = 1          # 1-based OBJ index of the next vertex
        n_next = 1          # ... and of the next normal
        t_next = 1          # ... and of the next texture coordinate (only textured materials write them)
        total = 0
        for material, chunks in self.tris.items():
            t = np.concatenate([c[0] for c in chunks])
            off = np.concatenate([c[1] for c in chunks])
            n = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
            n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
            verts = t.reshape(-1, 3)
            lines.append("\n".join(f"v {a:.3f} {b:.3f} {c:.3f}" for a, b, c in verts))
            lines.append("\n".join(f"vn {a:.4f} {b:.4f} {c:.4f}" for a, b, c in n))
            sc = self.uv_scale.get(material)
            if sc:
                uv = (t[:, :, :2] + off[:, None, :]).reshape(-1, 2) / sc
                lines.append("\n".join(f"vt {a:.4f} {b:.4f}" for a, b in uv))
            lines.append(f"usemtl {material}")
            k = np.arange(len(t))
            a = v_next + 3 * k
            nn = n_next + k
            if sc:
                tt = t_next + 3 * k
                faces = [f"f {p}/{u}/{q} {p + 1}/{u + 1}/{q} {p + 2}/{u + 2}/{q}" for p, u, q in zip(a, tt, nn)]
                t_next += 3 * len(t)
            else:
                faces = [f"f {p}//{q} {p + 1}//{q} {p + 2}//{q}" for p, q in zip(a, nn)]
            lines.append("\n".join(faces))
            v_next += 3 * len(t)
            n_next += len(t)
            total += len(t)
        with open(path, "w") as f:
            f.write("\n".join(lines) + "\n")
        return total


def ring(cx, cy, z, radius, sides, yaw):
    a = yaw + np.arange(sides) * 2 * np.pi / sides
    return np.stack([cx + radius * np.cos(a), cy + radius * np.sin(a), np.full(sides, z)], axis=1)


def tree_tris(t, i):
    """(material, triangles) of tree i: trunk prism (from 1 m below the ground, for slopes), then the crown."""
    x, y, h, r, base, yaw, z0 = t["x"][i], t["y"][i], t["h"][i], t["r"][i], t["base"][i], t["yaw"][i], t["z0"][i]
    out = []
    lo = ring(x, y, z0 - 1.0, t["trunk_r"][i], TRUNK_SIDES, yaw)
    hi = ring(x, y, z0 + base + 0.5, t["trunk_r"][i] * 0.7, TRUNK_SIDES, yaw)
    trunk = []
    for k in range(TRUNK_SIDES):
        k1 = (k + 1) % TRUNK_SIDES
        trunk += [lo[k], lo[k1], hi[k1], lo[k], hi[k1], hi[k]]
    out.append(("trunk", trunk))
    top = np.array([x, y, z0 + h])
    green = f"green{t['green'][i]}"
    if t["conifer"][i]:
        rb = ring(x, y, z0 + base, r, SIDES, yaw)
        centre = np.array([x, y, z0 + base])
        crown = []
        for k in range(SIDES):
            k1 = (k + 1) % SIDES
            crown += [rb[k], rb[k1], top, rb[k1], rb[k], centre]
    else:
        zm = z0 + base + 0.45 * (h - base)
        rm = ring(x, y, zm, r, SIDES, yaw)
        bottom = np.array([x, y, z0 + base])
        crown = []
        for k in range(SIDES):
            k1 = (k + 1) % SIDES
            crown += [rm[k], rm[k1], top, rm[k1], rm[k], bottom]
    out.append((green, crown))
    return out


# --------------------------------------------------------------------------------------------------------- textures

def ground_texture(rng, path, px=2048):
    """Soil/grass texture: several octaves of value noise."""
    img = np.zeros((px, px))
    for cells, w in ((8, 0.35), (32, 0.25), (128, 0.2), (512, 0.12), (1024, 0.08)):
        img += w * smooth_noise(rng, cells, (px, px))
    img = (img - img.min()) / (img.max() - img.min())
    img = np.clip((img - 0.5) * 1.8 + 0.5, 0, 1)                 # more contrast: patches of soil and grass
    soil = np.array([0.42, 0.34, 0.22])
    grass = np.array([0.30, 0.42, 0.18])
    rgb = soil[None, None] * (1 - img[..., None]) + grass[None, None] * img[..., None]
    # Litter and stones: patches of 20-50 cm (2048 px per 100 m), no per-pixel noise (it aliases away).
    rgb *= 0.7 + 0.6 * (0.6 * smooth_noise(rng, 512, (px, px)) + 0.4 * smooth_noise(rng, 256, (px, px)))[..., None]
    Image.fromarray((np.clip(rgb, 0, 1) * 255).astype(np.uint8)).save(path)


def crown_texture(rng, base, path, px=512):
    """Leaf clumps: blobs of ~0.2-0.6 m (at 4 m per repeat), darker gaps between them, fine leaf speckle, and a
    slight hue shift between clumps. Tileable (the noise wraps), so the repeat has no seams."""
    def wrap_noise(cells):
        c = rng.random((cells, cells))
        c = np.vstack([c, c[:1]])
        c = np.hstack([c, c[:, :1]])
        ys = np.linspace(0, cells, px, endpoint=False)
        y0 = ys.astype(int)
        f = (ys - y0)
        f = f * f * (3 - 2 * f)
        a = c[y0][:, y0]
        b = c[y0][:, y0 + 1]
        cc = c[y0 + 1][:, y0]
        d = c[y0 + 1][:, y0 + 1]
        fx, fy = f[None, :], f[:, None]
        return (a * (1 - fx) + b * fx) * (1 - fy) + (cc * (1 - fx) + d * fx) * fy

    clumps = 0.6 * wrap_noise(6) + 0.4 * wrap_noise(12)               # clumps of ~0.3-0.7 m
    clumps = (clumps - clumps.min()) / (clumps.max() - clumps.min())
    shade = 0.45 + 0.85 * np.clip((clumps - 0.25) / 0.6, 0, 1)          # dark gaps between clumps
    leaf = 0.8 + 0.4 * wrap_noise(128)                                  # leaf-scale speckle
    hue = wrap_noise(6)[..., None]
    col = np.asarray(base)[None, None] * (0.9 + 0.2 * np.concatenate([hue, 1 - hue, hue * 0.5], axis=-1))
    rgb = col * (shade * leaf)[..., None] * 1.15
    # Gazebo (ogre2) decodes textures as sRGB: encode, so a crown is as bright as the same diffuse colour
    # without a texture (phase 1/2 forest), not near-black.
    rgb = np.clip(rgb, 0, 1) ** (1 / 2.2)
    Image.fromarray((np.clip(rgb, 0, 1) * 255).astype(np.uint8)).save(path)


# ------------------------------------------------------------------------------------------------------------ truth

def crown_height(dx, dy, h, r, z_low, yaw):
    """Top surface of a crown (the mesh: a pyramid over a regular SIDES-gon of circumradius r) at offsets
    dx, dy from the trunk, above the ground at the trunk; -inf outside it."""
    d = np.hypot(dx, dy)
    sector = 2 * np.pi / SIDES
    a = np.mod(np.arctan2(dy, dx) - yaw, sector) - sector / 2
    edge = r * np.cos(sector / 2) / np.cos(a)            # distance to the polygon's edge in that direction
    return np.where(d <= edge, h - (h - z_low) * d / edge, -np.inf)


def truth_rasters(t, size, terrain, cell=1.0, sub=4):
    """DTM = terrain at the cell centres; DSM = the highest surface in each cell (terrain and crowns, sampled every
    cell/sub), CHM = DSM - DTM. Also the surface at the cell centres only (dsm_centre). All z absolute."""
    fine = cell / sub
    n = int(round(size / fine))
    axis = -size / 2 + fine / 2 + np.arange(n) * fine
    dsm = np.empty((n, n), dtype=np.float32)
    for r0 in range(0, n, 1000):                                        # terrain, in row blocks
        gx, gy = np.meshgrid(axis, axis[r0:r0 + 1000])
        dsm[r0:r0 + 1000] = terrain.height(gx, gy)
    for i in range(len(t["x"])):
        x, y, h, r = t["x"][i], t["y"][i], t["h"][i], t["r"][i]
        j0 = max(int((x - r + size / 2) / fine), 0)
        j1 = min(int((x + r + size / 2) / fine) + 1, n)
        i0 = max(int((y - r + size / 2) / fine), 0)
        i1 = min(int((y + r + size / 2) / fine) + 1, n)
        if j0 >= j1 or i0 >= i1:
            continue
        gx, gy = np.meshgrid(axis[j0:j1], axis[i0:i1])
        z = t["z0"][i] + crown_height(gx - x, gy - y, h, r, t["low"][i], t["yaw"][i])
        dsm[i0:i1, j0:j1] = np.maximum(dsm[i0:i1, j0:j1], z)
    m = n // sub
    blocks = dsm.reshape(m, sub, m, sub)
    dsm_max = blocks.max(axis=(1, 3))
    centre = 0.25 * (blocks[:, sub // 2 - 1, :, sub // 2 - 1] + blocks[:, sub // 2, :, sub // 2] +
                     blocks[:, sub // 2 - 1, :, sub // 2] + blocks[:, sub // 2, :, sub // 2 - 1])
    axis1 = -size / 2 + cell / 2 + np.arange(m) * cell
    gx, gy = np.meshgrid(axis1, axis1)
    dtm = terrain.height(gx, gy).astype(np.float32)
    return axis1, dtm, dsm_max, centre


# ------------------------------------------------------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--name", default="forest", help="model name = folder under models/ (default forest)")
    ap.add_argument("--terrain", choices=("flat", "slope"), default="flat")
    ap.add_argument("--size", type=float, default=2000, help="forest side, m (default 2000, spec 6)")
    ap.add_argument("--tile", type=float, default=200, help="mesh tile side, m")
    ap.add_argument("--spacing", type=float, default=5.0, help="candidate tree grid, m")
    ap.add_argument("--seed", type=int, default=2026)
    args = ap.parse_args()
    out_dir = os.path.join(ROOT, "models", args.name)

    rng = np.random.default_rng(args.seed)
    mesh_dir = os.path.join(out_dir, "meshes")
    os.makedirs(mesh_dir, exist_ok=True)
    for f in os.listdir(mesh_dir):
        os.remove(os.path.join(mesh_dir, f))

    terrain = Terrain.make(args.terrain, args.size, np.random.default_rng(args.seed + 1))
    t = make_trees(rng, args.size, args.spacing, terrain)
    n = len(t["x"])

    with open(os.path.join(mesh_dir, "forest.mtl"), "w") as f:
        f.write(f"newmtl trunk\nKa {TRUNK[0]} {TRUNK[1]} {TRUNK[2]}\nKd {TRUNK[0]} {TRUNK[1]} {TRUNK[2]}\nKs 0 0 0\n\n")
        for k in range(len(GREENS)):
            f.write(f"newmtl green{k}\nKa 1 1 1\nKd 1 1 1\nKs 0.02 0.02 0.02\nmap_Kd crown{k}.png\n\n")
        f.write("newmtl ground\nKa 1 1 1\nKd 1 1 1\nKs 0 0 0\nmap_Kd ground.png\n")
    ground_texture(rng, os.path.join(mesh_dir, "ground.png"))
    for k, base in enumerate(GREENS):
        crown_texture(rng, base, os.path.join(mesh_dir, f"crown{k}.png"))
    uv = {f"green{k}": CROWN_UV_M for k in range(len(GREENS))}

    # Tiles, so the renderer can cull what is outside a sensor's view.
    n_tiles = int(np.ceil(args.size / args.tile))
    tiles = []
    total_tris = 0
    ti = np.clip(((t["x"] + args.size / 2) // args.tile).astype(int), 0, n_tiles - 1)
    tj = np.clip(((t["y"] + args.size / 2) // args.tile).astype(int), 0, n_tiles - 1)
    for a in range(n_tiles):
        for b in range(n_tiles):
            idx = np.nonzero((ti == a) & (tj == b))[0]
            if idx.size == 0:
                continue
            obj = Obj(uv)
            for i in idx:
                for mat, tris in tree_tris(t, i):
                    obj.add(mat, tris, t["uv_off"][i])
            name = f"tile_{a}_{b}.obj"
            total_tris += obj.write(os.path.join(mesh_dir, name), "forest.mtl")
            tiles.append(name)

    ground = Obj({"ground": GROUND_UV_M})
    gtris = terrain.triangles()
    ground.add("ground", gtris)
    ground.write(os.path.join(mesh_dir, "ground.obj"), "forest.mtl")

    visuals = "\n".join(
        f"""      <visual name="{name[:-4]}">
        <cast_shadows>false</cast_shadows>
        <geometry><mesh><uri>model://{args.name}/meshes/{name}</uri></mesh></geometry>
      </visual>""" for name in tiles)
    sdf = f"""<?xml version="1.0"?>
<!-- Generated by scripts/make_forest.py (seed {args.seed}, terrain {args.terrain}); do not edit. -->
<sdf version="1.9">
  <model name="{args.name}">
    <static>true</static>
    <link name="link">
      <visual name="ground">
        <cast_shadows>false</cast_shadows>
        <geometry><mesh><uri>model://{args.name}/meshes/ground.obj</uri></mesh></geometry>
      </visual>
{visuals}
    </link>
  </model>
</sdf>
"""
    with open(os.path.join(out_dir, "model.sdf"), "w") as f:
        f.write(sdf)
    with open(os.path.join(out_dir, "model.config"), "w") as f:
        f.write(f'<?xml version="1.0"?>\n<model><name>{args.name}</name><version>1.0</version>'
                '<sdf version="1.9">model.sdf</sdf><description>Procedural forest (make_forest.py)</description></model>\n')

    kinds = np.where(t["conifer"], "conifer", "broadleaf")
    with open(os.path.join(out_dir, "trees.csv"), "w") as f:
        f.write("tree_id,x_m,y_m,height_m,crown_base_m,crown_radius_m,kind,crown_low_m,yaw_rad,ground_z_m\n")
        for i in range(n):
            f.write(f"{i},{t['x'][i]:.3f},{t['y'][i]:.3f},{t['h'][i]:.3f},{t['base'][i]:.3f},{t['r'][i]:.3f},{kinds[i]},"
                    f"{t['low'][i]:.3f},{t['yaw'][i]:.5f},{t['z0'][i]:.4f}\n")

    axis, dtm, dsm, dsm_centre = truth_rasters(t, args.size, terrain)
    chm = dsm - dtm
    ev = np.abs(axis) <= 400
    dtm_ev = dtm[np.ix_(ev, ev)]
    params = dict(name=args.name, size_m=args.size, tile_m=args.tile, spacing_m=args.spacing, terrain=args.terrain,
                  slope_deg=SLOPE_DEG if args.terrain == "slope" else 0.0,
                  hill_amplitude_m=HILL_AMP_M if args.terrain == "slope" else 0.0,
                  seed=args.seed, trees=n, triangles=int(total_tris), terrain_triangles=int(len(gtris)), cell_m=1.0,
                  crown_texture_m=CROWN_UV_M, ground_texture_m=GROUND_UV_M)
    np.savez_compressed(os.path.join(out_dir, "truth.npz"), x=axis, y=axis, dtm=dtm, dsm=dsm, chm=chm,
                        dsm_centre=dsm_centre, tx=terrain.tx, ty=terrain.ty, tz=terrain.tz, params=json.dumps(params))
    prev = (np.clip(chm / 30.0, 0, 1) * 255).astype(np.uint8)[::-1]          # north up
    Image.fromarray(prev).resize((700, 700)).save(os.path.join(out_dir, "truth_chm.png"))
    stats = dict(canopy_cover=float((chm > 2).mean()), canopy_cover_eval=float((chm[np.ix_(ev, ev)] > 2).mean()),
                 height_mean=float(t["h"].mean()), dtm_eval_min=float(dtm_ev.min()), dtm_eval_max=float(dtm_ev.max()),
                 dtm_eval_mean=float(dtm_ev.mean()))
    with open(os.path.join(out_dir, "forest.json"), "w") as f:
        json.dump(params | stats, f, indent=2)
    print(f"{n} trees, {total_tris} triangles in {len(tiles)} tiles + {len(gtris)} terrain; canopy cover "
          f"{stats['canopy_cover']:.0%} (CHM > 2 m); heights {t['h'].min():.1f}-{t['h'].max():.1f} m; terrain in the "
          f"evaluation area {stats['dtm_eval_min']:.1f}..{stats['dtm_eval_max']:.1f} m -> {out_dir}")


if __name__ == "__main__":
    main()
