#!/usr/bin/env python3
"""TUM 轨迹 -> 单层 FREE 规划地图（不读激光、不裁墙）。"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np


def read_tum(path: Path) -> np.ndarray:
    rows = []
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        fields = line.split()
        if len(fields) < 4:
            raise SystemExit("{}: 每行至少 t x y z".format(path))
        values = [float(v) for v in fields[:4]]
        if all(math.isfinite(v) for v in values):
            rows.append(values)
    if len(rows) < 2:
        raise SystemExit("TUM 有效位姿少于 2: {}".format(path))
    return np.asarray(rows, dtype=np.float64)


def resample(traj: np.ndarray, stride: float) -> np.ndarray:
    if stride <= 0.0:
        return traj
    kept = [traj[0]]
    travelled = 0.0
    prev = traj[0, 1:4]
    for row in traj[1:]:
        point = row[1:4]
        travelled += float(np.linalg.norm(point - prev))
        prev = point
        if travelled >= stride:
            kept.append(row)
            travelled = 0.0
    if not np.allclose(kept[-1][1:4], traj[-1, 1:4]):
        kept.append(traj[-1])
    if len(kept) < 2:
        raise SystemExit("抽稀后位姿少于 2")
    return np.asarray(kept)


def inflate_single_layer(xyz: np.ndarray, resolution: float, half_width: float):
    keys = set()
    pad = half_width + 0.5 * resolution
    for p0, p1 in zip(xyz[:-1], xyz[1:]):
        delta = p1 - p0
        dxy = delta[:2]
        norm = float(np.linalg.norm(dxy))
        left = np.array([0.0, 1.0]) if norm < 1.0e-8 else np.array([-dxy[1], dxy[0]]) / norm
        mins = np.minimum(p0[:2], p1[:2]) - pad
        maxs = np.maximum(p0[:2], p1[:2]) + pad
        ix0 = int(math.floor(mins[0] / resolution))
        ix1 = int(math.floor((maxs[0] - 1.0e-9) / resolution))
        iy0 = int(math.floor(mins[1] / resolution))
        iy1 = int(math.floor((maxs[1] - 1.0e-9) / resolution))
        length_sq = float(np.dot(delta, delta))
        for ix in range(ix0, ix1 + 1):
            for iy in range(iy0, iy1 + 1):
                cx = (ix + 0.5) * resolution
                cy = (iy + 0.5) * resolution
                if length_sq <= 1.0e-12:
                    closest = p0
                else:
                    query = np.array([cx, cy, p0[2]])
                    t = min(1.0, max(0.0, float(np.dot(query - p0, delta) / length_sq)))
                    closest = p0 + t * delta
                if abs(float(np.dot(np.array([cx, cy]) - closest[:2], left))) <= half_width:
                    keys.add((ix, iy, int(math.floor(closest[2] / resolution))))
    collapsed = set()
    for ix, iy in {(k[0], k[1]) for k in keys}:
        cx, cy = (ix + 0.5) * resolution, (iy + 0.5) * resolution
        nearest = int(np.argmin((xyz[:, 0] - cx) ** 2 + (xyz[:, 1] - cy) ** 2))
        collapsed.add((ix, iy, int(math.floor(xyz[nearest, 2] / resolution))))
    if not collapsed:
        raise SystemExit("没有生成任何体素")
    return collapsed


def write_tum_xyz(path: Path, traj: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as stream:
        stream.write("# timestamp tx ty tz qx qy qz qw\n")
        for t, x, y, z in traj:
            stream.write("{:.6f} {:.6f} {:.6f} {:.6f} 0 0 0 1\n".format(t, x, y, z))


def write_pcd(path: Path, points: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    count = int(points.shape[0])
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        "WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary\n"
    ).format(n=count)
    with path.open("wb") as stream:
        stream.write(header.encode("ascii"))
        stream.write(np.asarray(points, dtype=np.float32).tobytes(order="C"))


def main() -> int:
    parser = argparse.ArgumentParser(description="由 TUM 生成单层规划地图")
    parser.add_argument("--traj", type=Path, required=True, help="TUM: t x y z [qx qy qz qw]")
    parser.add_argument("--output-prefix", type=Path, default=None)
    parser.add_argument("--resolution", type=float, default=0.20)
    parser.add_argument("--half-width", type=float, default=1.0)
    parser.add_argument("--pose-stride", type=float, default=0.10)
    args = parser.parse_args()
    if args.resolution <= 0.0 or args.half_width <= 0.0:
        raise SystemExit("resolution / half-width 必须为正")

    raw = read_tum(args.traj)
    traj = resample(raw, args.pose_stride)
    xyz = traj[:, 1:4]
    voxels = inflate_single_layer(xyz, args.resolution, args.half_width)
    points = (np.asarray(list(voxels), dtype=np.float64) + 0.5) * args.resolution

    prefix = args.output_prefix or args.traj.expanduser().resolve().with_suffix("")
    prefix = prefix.expanduser()
    if prefix.suffix:
        prefix = prefix.with_suffix("")
    free_pcd = Path(str(prefix) + "_free.pcd")
    path_tum = Path(str(prefix) + "_path.tum")
    meta = Path(str(prefix) + "_plan.json")

    write_pcd(free_pcd, points)
    write_tum_xyz(path_tum, traj)
    length = float(np.linalg.norm(np.diff(xyz, axis=0), axis=1).sum())
    meta.write_text(
        json.dumps(
            {
                "traj": str(args.traj.expanduser().resolve()),
                "free_pcd": str(free_pcd),
                "path_tum": str(path_tum),
                "resolution": args.resolution,
                "half_width": args.half_width,
                "pose_stride": args.pose_stride,
                "raw_then_resampled": [int(raw.shape[0]), int(traj.shape[0])],
                "path_length_m": length,
                "free_voxels": int(points.shape[0]),
                "z_min": float(xyz[:, 2].min()),
                "z_max": float(xyz[:, 2].max()),
            },
            indent=2,
            ensure_ascii=False,
        )
        + "\n",
        encoding="utf-8",
    )
    print(
        "plan map: {} voxels, {:.1f} m, z {:.2f}..{:.2f}\n  {}\n  {}".format(
            points.shape[0], length, xyz[:, 2].min(), xyz[:, 2].max(), free_pcd, meta
        )
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
