#!/usr/bin/env python3
"""TUM 轨迹 -> 连续 FREE 规划地图（不读激光、不裁墙）。"""

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


def inflate_continuous_layers(xyz: np.ndarray, resolution: float, half_width: float):
    """沿三维轨迹密集铺设单体素厚的连续表面。

    同一 XY 允许保留不同楼层的体素，避免楼梯往返或楼层投影重叠时，
    后选中的高度覆盖前一段轨迹并把可通行带切断。同一次经过在每个
    XY 只保留离轨迹最近的高度，避免斜坡/台阶叠成竖向厚块。
    """
    candidates = {}
    center_keys = set()
    radius = half_width + 0.5 * resolution
    cell_radius = int(math.ceil(radius / resolution))
    dense_step = 0.4 * resolution
    sequence = 0

    for p0, p1 in zip(xyz[:-1], xyz[1:]):
        delta = p1 - p0
        length = float(np.linalg.norm(delta))
        samples = max(1, int(math.ceil(length / dense_step)))
        for sample in range(samples):
            p = p0 + delta * (float(sample) / samples)
            base_ix = int(math.floor(p[0] / resolution))
            base_iy = int(math.floor(p[1] / resolution))
            center_keys.add((base_ix, base_iy, int(math.floor(p[2] / resolution))))
            for ix in range(base_ix - cell_radius, base_ix + cell_radius + 1):
                cx = (ix + 0.5) * resolution
                for iy in range(base_iy - cell_radius, base_iy + cell_radius + 1):
                    cy = (iy + 0.5) * resolution
                    distance_sq = (cx - p[0]) ** 2 + (cy - p[1]) ** 2
                    if distance_sq <= radius * radius:
                        candidates.setdefault((ix, iy), []).append(
                            (sequence, distance_sq, float(p[2]))
                        )
            sequence += 1

    # 保证终点也参与铺设。
    p = xyz[-1]
    base_ix = int(math.floor(p[0] / resolution))
    base_iy = int(math.floor(p[1] / resolution))
    center_keys.add((base_ix, base_iy, int(math.floor(p[2] / resolution))))
    for ix in range(base_ix - cell_radius, base_ix + cell_radius + 1):
        cx = (ix + 0.5) * resolution
        for iy in range(base_iy - cell_radius, base_iy + cell_radius + 1):
            cy = (iy + 0.5) * resolution
            distance_sq = (cx - p[0]) ** 2 + (cy - p[1]) ** 2
            if distance_sq <= radius * radius:
                candidates.setdefault((ix, iy), []).append(
                    (sequence, distance_sq, float(p[2]))
                )

    if not candidates:
        raise SystemExit("没有生成任何体素")

    keys = set()
    # 同一次经过会连续命中一个格子；离开后再次命中则是另一次经过，
    # 可对应不同楼层。序号间隔阈值略大于走廊直径覆盖的采样数。
    visit_gap = max(4, int(math.ceil(2.5 * radius / dense_step)))
    for (ix, iy), values in candidates.items():
        visit = [values[0]]
        visits = []
        for value in values[1:]:
            if value[0] - visit[-1][0] > visit_gap:
                visits.append(visit)
                visit = []
            visit.append(value)
        visits.append(visit)
        for samples in visits:
            _seq, _distance_sq, z = min(samples, key=lambda value: value[1])
            keys.add((ix, iy, int(math.floor(z / resolution))))

    keys.update(center_keys)
    return keys


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
    parser = argparse.ArgumentParser(description="由 TUM 生成连续规划地图")
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
    voxels = inflate_continuous_layers(xyz, args.resolution, args.half_width)
    points = (np.asarray(sorted(voxels), dtype=np.float64) + 0.5) * args.resolution

    prefix = args.output_prefix or args.traj.expanduser().resolve().with_suffix("")
    prefix = prefix.expanduser().resolve()
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
