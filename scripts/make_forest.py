#!/usr/bin/env python3
"""Procedural forest for the VTOL LiDAR + camera simulation (spec section 6).

Writes a static Gazebo model `models/forest/` made of mesh tiles (low-poly
trees: trunk prism + conical or bicone crown, gaps between crowns so thin rays
can reach the ground, clearings and dense patches), a textured ground mesh, and
the ground truth taken directly from the scene:

  models/forest/trees.csv        tree_id, x_m, y_m, height_m, crown_base_m, crown_radius_m, kind, crown_low_m, yaw_rad
  models/forest/truth.npz        DTM, DSM (highest surface in each cell), CHM on a 1 m grid (+ x/y axes), parameters
  models/forest/truth_chm.png    preview of the CHM

World frame: ENU, metres; the forest is centred on the origin, ground at z = 0.

  python3 scripts/make_forest.py                   # 1400 x 1400 m forest, seed 2026
  python3 scripts/make_forest.py --size 600 --seed 1
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

# Crown greens (diffuse), varied so the images have structure for SfM.
GREENS = [
    (0.13, 0.30, 0.10), (0.18, 0.38, 0.12), (0.10, 0.24, 0.09), (0.24, 0.42, 0.16),
    (0.16, 0.33, 0.20), (0.28, 0.40, 0.12), (0.09, 0.20, 0.12), (0.21, 0.30, 0.09),
]
TRUNK = (0.30, 0.22, 0.14)


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


def make_trees(rng, size, spacing):
    # Two octaves: patches of a few hundred metres, and finer structure.
    field = 0.65 * smooth_noise(rng, 6, (512, 512)) + 0.35 * smooth_noise(rng, 24, (512, 512))
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
    return dict(x=x, y=y, h=h, r=r, base=base, conifer=conifer, green=green, trunk_r=trunk_r, yaw=yaw)


class Obj:
    """Accumulates triangles per material and writes a Wavefront OBJ with flat normals."""

    def __init__(self):
        self.tris = {}

    def add(self, material, tris):
        self.tris.setdefault(material, []).append(np.asarray(tris, dtype=np.float64).reshape(-1, 3, 3))

    def write(self, path, mtl_name, uv_scale=None):
        """One vertex per triangle corner and one normal per triangle (flat shading)."""
        lines = [f"mtllib {mtl_name}"]
        v_next = 1          # 1-based OBJ index of the next vertex (and texture coordinate)
        n_next = 1          # ... and of the next normal
        total = 0
        for material, chunks in self.tris.items():
            t = np.concatenate(chunks)
            n = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
            n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
            verts = t.reshape(-1, 3)
            lines.append("\n".join(f"v {a:.3f} {b:.3f} {c:.3f}" for a, b, c in verts))
            lines.append("\n".join(f"vn {a:.4f} {b:.4f} {c:.4f}" for a, b, c in n))
            if uv_scale:
                lines.append("\n".join(f"vt {a / uv_scale:.4f} {b / uv_scale:.4f}" for a, b, _ in verts))
            lines.append(f"usemtl {material}")
            faces = []
            for k in range(len(t)):
                a, nn = v_next + 3 * k, n_next + k
                if uv_scale:
                    faces.append(f"f {a}/{a}/{nn} {a + 1}/{a + 1}/{nn} {a + 2}/{a + 2}/{nn}")
                else:
                    faces.append(f"f {a}//{nn} {a + 1}//{nn} {a + 2}//{nn}")
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
    """(material, triangles) of tree i: trunk prism, then the crown."""
    x, y, h, r, base, yaw = t["x"][i], t["y"][i], t["h"][i], t["r"][i], t["base"][i], t["yaw"][i]
    out = []
    lo = ring(x, y, 0.0, t["trunk_r"][i], TRUNK_SIDES, yaw)
    hi = ring(x, y, base + 0.5, t["trunk_r"][i] * 0.7, TRUNK_SIDES, yaw)
    trunk = []
    for k in range(TRUNK_SIDES):
        k1 = (k + 1) % TRUNK_SIDES
        trunk += [lo[k], lo[k1], hi[k1], lo[k], hi[k1], hi[k]]
    out.append(("trunk", trunk))
    top = np.array([x, y, h])
    green = f"green{t['green'][i]}"
    if t["conifer"][i]:
        rb = ring(x, y, base, r, SIDES, yaw)
        centre = np.array([x, y, base])
        crown = []
        for k in range(SIDES):
            k1 = (k + 1) % SIDES
            crown += [rb[k], rb[k1], top, rb[k1], rb[k], centre]
    else:
        zm = base + 0.45 * (h - base)
        rm = ring(x, y, zm, r, SIDES, yaw)
        bottom = np.array([x, y, base])
        crown = []
        for k in range(SIDES):
            k1 = (k + 1) % SIDES
            crown += [rm[k], rm[k1], top, rm[k1], rm[k], bottom]
    out.append((green, crown))
    return out


def ground_texture(rng, path, px=1024):
    """Tileable-ish soil/grass texture: several octaves of value noise."""
    img = np.zeros((px, px))
    for cells, w in ((8, 0.4), (32, 0.3), (128, 0.2), (512, 0.1)):
        img += w * smooth_noise(rng, cells, (px, px))
    img = (img - img.min()) / (img.max() - img.min())
    soil = np.array([0.42, 0.34, 0.22])
    grass = np.array([0.30, 0.42, 0.18])
    rgb = soil[None, None] * (1 - img[..., None]) + grass[None, None] * img[..., None]
    rgb *= 0.85 + 0.3 * rng.random((px, px, 1))
    Image.fromarray((np.clip(rgb, 0, 1) * 255).astype(np.uint8)).save(path)


def crown_height(dx, dy, h, r, z_low, yaw):
    """Top surface of a crown (the mesh: a pyramid over a regular SIDES-gon of circumradius r) at offsets
    dx, dy from the trunk; 0 outside it."""
    d = np.hypot(dx, dy)
    sector = 2 * np.pi / SIDES
    a = np.mod(np.arctan2(dy, dx) - yaw, sector) - sector / 2
    edge = r * np.cos(sector / 2) / np.cos(a)            # distance to the polygon's edge in that direction
    return np.where(d <= edge, h - (h - z_low) * d / edge, 0.0)


def truth_rasters(t, size, cell=1.0, sub=4):
    """DTM flat 0; DSM = the highest surface in each cell (sampled every cell/sub), CHM = DSM - DTM.
    Also the surface at the cell centres only (dsm_centre)."""
    fine = cell / sub
    n = int(round(size / fine))
    axis = -size / 2 + fine / 2 + np.arange(n) * fine
    dsm = np.zeros((n, n), dtype=np.float32)
    for i in range(len(t["x"])):
        x, y, h, r, base = t["x"][i], t["y"][i], t["h"][i], t["r"][i], t["base"][i]
        j0 = max(int((x - r + size / 2) / fine), 0)
        j1 = min(int((x + r + size / 2) / fine) + 1, n)
        i0 = max(int((y - r + size / 2) / fine), 0)
        i1 = min(int((y + r + size / 2) / fine) + 1, n)
        if j0 >= j1 or i0 >= i1:
            continue
        gx, gy = np.meshgrid(axis[j0:j1], axis[i0:i1])
        z_low = base if t["conifer"][i] else base + 0.45 * (h - base)
        z = crown_height(gx - x, gy - y, h, r, z_low, t["yaw"][i])
        dsm[i0:i1, j0:j1] = np.maximum(dsm[i0:i1, j0:j1], z)
    m = n // sub
    blocks = dsm.reshape(m, sub, m, sub)
    dsm_max = blocks.max(axis=(1, 3))
    centre = 0.25 * (blocks[:, sub // 2 - 1, :, sub // 2 - 1] + blocks[:, sub // 2, :, sub // 2] +
                     blocks[:, sub // 2 - 1, :, sub // 2] + blocks[:, sub // 2, :, sub // 2 - 1])
    axis1 = -size / 2 + cell / 2 + np.arange(m) * cell
    return axis1, np.zeros_like(dsm_max), dsm_max, centre


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--size", type=float, default=1400, help="forest side, m (default 1400)")
    ap.add_argument("--tile", type=float, default=200, help="mesh tile side, m")
    ap.add_argument("--spacing", type=float, default=5.0, help="candidate tree grid, m")
    ap.add_argument("--ground", type=float, default=5000, help="ground side, m")
    ap.add_argument("--seed", type=int, default=2026)
    ap.add_argument("--out", default=os.path.join(ROOT, "models", "forest"))
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)
    mesh_dir = os.path.join(args.out, "meshes")
    os.makedirs(mesh_dir, exist_ok=True)
    for f in os.listdir(mesh_dir):
        os.remove(os.path.join(mesh_dir, f))

    t = make_trees(rng, args.size, args.spacing)
    n = len(t["x"])

    with open(os.path.join(mesh_dir, "forest.mtl"), "w") as f:
        f.write(f"newmtl trunk\nKa {TRUNK[0]} {TRUNK[1]} {TRUNK[2]}\nKd {TRUNK[0]} {TRUNK[1]} {TRUNK[2]}\nKs 0 0 0\n\n")
        for k, (r, g, b) in enumerate(GREENS):
            f.write(f"newmtl green{k}\nKa {r} {g} {b}\nKd {r} {g} {b}\nKs 0.02 0.02 0.02\n\n")
        f.write("newmtl ground\nKa 1 1 1\nKd 1 1 1\nKs 0 0 0\nmap_Kd ground.png\n")
    ground_texture(rng, os.path.join(mesh_dir, "ground.png"))

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
            obj = Obj()
            for i in idx:
                for mat, tris in tree_tris(t, i):
                    obj.add(mat, tris)
            name = f"tile_{a}_{b}.obj"
            total_tris += obj.write(os.path.join(mesh_dir, name), "forest.mtl")
            tiles.append(name)

    g = args.ground / 2
    ground = Obj()
    ground.add("ground", [[-g, -g, 0], [g, -g, 0], [g, g, 0], [-g, -g, 0], [g, g, 0], [-g, g, 0]])
    ground.write(os.path.join(mesh_dir, "ground.obj"), "forest.mtl", uv_scale=25.0)

    visuals = "\n".join(
        f"""      <visual name="{name[:-4]}">
        <cast_shadows>false</cast_shadows>
        <geometry><mesh><uri>model://forest/meshes/{name}</uri></mesh></geometry>
      </visual>""" for name in tiles)
    sdf = f"""<?xml version="1.0"?>
<!-- Generated by scripts/make_forest.py (seed {args.seed}); do not edit. -->
<sdf version="1.9">
  <model name="forest">
    <static>true</static>
    <link name="link">
      <visual name="ground">
        <cast_shadows>false</cast_shadows>
        <geometry><mesh><uri>model://forest/meshes/ground.obj</uri></mesh></geometry>
      </visual>
{visuals}
    </link>
  </model>
</sdf>
"""
    with open(os.path.join(args.out, "model.sdf"), "w") as f:
        f.write(sdf)
    with open(os.path.join(args.out, "model.config"), "w") as f:
        f.write('<?xml version="1.0"?>\n<model><name>forest</name><version>1.0</version>'
                '<sdf version="1.9">model.sdf</sdf><description>Procedural forest (make_forest.py)</description></model>\n')

    kinds = np.where(t["conifer"], "conifer", "broadleaf")
    with open(os.path.join(args.out, "trees.csv"), "w") as f:
        f.write("tree_id,x_m,y_m,height_m,crown_base_m,crown_radius_m,kind,crown_low_m,yaw_rad\n")
        for i in range(n):
            f.write(f"{i},{t['x'][i]:.3f},{t['y'][i]:.3f},{t['h'][i]:.3f},{t['base'][i]:.3f},{t['r'][i]:.3f},{kinds[i]},"
                    f"{(t['base'][i] if t['conifer'][i] else t['base'][i] + 0.45 * (t['h'][i] - t['base'][i])):.3f},"
                    f"{t['yaw'][i]:.5f}\n")

    axis, dtm, dsm, dsm_centre = truth_rasters(t, args.size)
    chm = dsm - dtm
    params = dict(size_m=args.size, tile_m=args.tile, spacing_m=args.spacing, ground_m=args.ground,
                  seed=args.seed, trees=n, triangles=int(total_tris), cell_m=1.0)
    np.savez_compressed(os.path.join(args.out, "truth.npz"), x=axis, y=axis, dtm=dtm, dsm=dsm, chm=chm, dsm_centre=dsm_centre,
                        params=json.dumps(params))
    prev = (np.clip(chm / 30.0, 0, 1) * 255).astype(np.uint8)[::-1]          # north up
    Image.fromarray(prev).resize((700, 700)).save(os.path.join(args.out, "truth_chm.png"))
    with open(os.path.join(args.out, "forest.json"), "w") as f:
        json.dump(params | dict(canopy_cover=float((chm > 2).mean()), height_mean=float(t["h"].mean())), f, indent=2)
    print(f"{n} trees, {total_tris} triangles in {len(tiles)} tiles; canopy cover {(chm > 2).mean():.0%}"
          f" (CHM > 2 m); heights {t['h'].min():.1f}-{t['h'].max():.1f} m -> {args.out}")


if __name__ == "__main__":
    main()
