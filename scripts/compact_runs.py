#!/usr/bin/env python3
"""Shrink the run folders under out/ without losing data.

  python3 scripts/compact_runs.py                 # every run under out/ (dry run: only prints what it would do)
  python3 scripts/compact_runs.py --apply         # do it
  python3 scripts/compact_runs.py out/r5_main --apply

Per run:
  lidar/, lidar_exact/ scans_*.npz            ranges as mm deltas, zip-compressed (scripts/scan_codec.py: within 0.5 mm)
  export/lidar/ *.npz                         rewritten zip-compressed (lossless; np.load reads both)
  lidar_map.ply                               removed: the same voxels are in lidar_map.npz
                                              (scripts/view_cloud.py, or lidar_map.py --ply to write it again)
  px4.ulg                                     -> px4.ulg.zst (zstd, lossless; `zstd -d px4.ulg.zst` to read it)
  export/las/*.las                            -> *.laz (LAZ, lossless, needs laspy + lazrs) unless --keep-las
Camera images are not touched (PNG -> JPEG would lose data: scripts/compact_runs.py --png-to-jpg QUALITY does it).
"""
import argparse
import glob
import os
import shutil
import subprocess
import sys
import zipfile

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import scan_codec  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)


def size(paths):
    return sum(os.path.getsize(p) for p in paths if os.path.exists(p))


def is_compressed(path):
    with zipfile.ZipFile(path) as z:
        return all(i.compress_type == zipfile.ZIP_DEFLATED for i in z.infolist())


def to_codec(path):
    """A scans_<k>.npz with float32 ranges_m -> mm deltas (scripts/scan_codec.py), checked to 0.5 mm."""
    d = np.load(path)
    data = {k: d[k] for k in d.files if k != "ranges_m"}
    r = d["ranges_m"]
    tmp = path + ".tmp.npz"
    scan_codec.save(tmp, r, **data)
    back = scan_codec.load(tmp)["ranges_m"]
    fin = np.isfinite(r)
    if not (np.array_equal(fin, np.isfinite(back)) and np.all(np.abs(back[fin] - r[fin]) <= 0.00051)):
        os.remove(tmp)
        raise RuntimeError(f"{path}: ranges differ after encoding")
    os.replace(tmp, path)


def needs_codec(path):
    return os.path.basename(path).startswith("scans_") and "ranges_m" in np.load(path).files


def recompress_npz(path):
    if needs_codec(path):
        return to_codec(path)
    d = np.load(path)
    data = {k: d[k] for k in d.files}
    tmp = path + ".tmp.npz"
    np.savez_compressed(tmp, **data)
    chk = np.load(tmp)
    for k, v in data.items():                   # lossless check before replacing
        if not np.array_equal(chk[k], v, equal_nan=True):
            os.remove(tmp)
            raise RuntimeError(f"{path}: {k} differs after compression")
    os.replace(tmp, path)


def las_to_laz(path):
    import laspy

    out = path[:-4] + ".laz"
    with laspy.open(path) as f:
        las = f.read()
    las.write(out, laz_backend=laspy.LazBackend.Lazrs)
    with laspy.open(out) as f:
        if f.header.point_count != las.header.point_count:
            os.remove(out)
            raise RuntimeError(f"{out}: point count differs")
    os.remove(path)
    return out


def png_to_jpg(run, quality):
    from PIL import Image

    csv = os.path.join(run, "camera", "images.csv")
    pngs = sorted(glob.glob(os.path.join(run, "camera", "*.png")))
    for p in pngs:
        Image.open(p).convert("RGB").save(p[:-4] + ".jpg", quality=quality)
        os.remove(p)
    if pngs and os.path.exists(csv):
        txt = open(csv).read().replace(".png,", ".jpg,")
        open(csv, "w").write(txt)
    return len(pngs)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="*")
    ap.add_argument("--apply", action="store_true", help="change the files (default: dry run)")
    ap.add_argument("--keep-las", action="store_true")
    ap.add_argument("--png-to-jpg", type=int, metavar="QUALITY", help="also convert camera PNGs (lossy)")
    a = ap.parse_args()
    runs = a.runs or sorted(d for d in glob.glob(os.path.join(ROOT, "out", "*")) if os.path.isdir(d))
    total_before = total_after = 0
    for run in runs:
        npz = [p for sub in ("lidar", "lidar_exact", os.path.join("export", "lidar"))
               for p in sorted(glob.glob(os.path.join(run, sub, "*.npz")))]
        npz = [p for p in npz if not is_compressed(p) or needs_codec(p)]
        ply = [p for p in [os.path.join(run, "lidar_map.ply")] if os.path.exists(p) and
               os.path.exists(os.path.join(run, "lidar_map.npz"))]
        ulg = [p for p in [os.path.join(run, "px4.ulg")] if os.path.exists(p)]
        las = [] if a.keep_las else sorted(glob.glob(os.path.join(run, "export", "las", "*.las")))
        pngs = sorted(glob.glob(os.path.join(run, "camera", "*.png"))) if a.png_to_jpg else []
        before = size(npz + ply + ulg + las + pngs)
        if not before:
            continue
        print(f"{os.path.relpath(run, ROOT)}: {len(npz)} npz, {len(ply)} ply, {len(ulg)} ulg, {len(las)} las, "
              f"{len(pngs)} png: {before / 1e9:.2f} GB", flush=True)
        total_before += before
        if not a.apply:
            continue
        for p in npz:
            recompress_npz(p)
        after = size(npz)
        for p in ply:
            os.remove(p)
        for p in ulg:
            subprocess.run(["zstd", "-q", "-T0", "-10", "--rm", p], check=True)
            after += os.path.getsize(p + ".zst")
        for p in las:
            after += os.path.getsize(las_to_laz(p))
        if pngs:
            png_to_jpg(run, a.png_to_jpg)
            after += size(glob.glob(os.path.join(run, "camera", "*.jpg")))
        total_after += after
        print(f"   -> {after / 1e9:.2f} GB", flush=True)
    print(f"total {total_before / 1e9:.2f} GB" + (f" -> {total_after / 1e9:.2f} GB" if a.apply else " (dry run)"))
    print(f"free: {shutil.disk_usage(ROOT).free / 1e9:.1f} GB")


if __name__ == "__main__":
    sys.exit(main())
