#!/usr/bin/env python3
"""规划地图读写（不依赖 ROS）。"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np


def read_pcd_xyz(path: Path) -> np.ndarray:
    raw = path.read_bytes()
    offset = 0
    header = {}
    for line in raw.decode("ascii", errors="ignore").splitlines(True):
        offset += len(line.encode("ascii", errors="ignore"))
        parts = line.strip().split()
        if not parts:
            continue
        header[parts[0].upper()] = " ".join(parts[1:])
        if parts[0].upper() == "DATA":
            break
    count = int(float(header["POINTS"]))
    kind = header["DATA"].lower()
    payload = raw[offset:]
    if kind == "ascii":
        return np.loadtxt(payload.decode("ascii").splitlines(), dtype=np.float64).reshape(-1, 3)
    if kind == "binary":
        return np.frombuffer(payload, dtype="<f4", count=count * 3).reshape(count, 3).astype(np.float64)
    raise SystemExit("不支持的 PCD DATA: {}".format(kind))


def read_tum_xyz(path: Path) -> np.ndarray:
    rows = []
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        fields = line.split()
        rows.append((float(fields[1]), float(fields[2]), float(fields[3])))
    if not rows:
        raise SystemExit("TUM 没有有效位姿: {}".format(path))
    return np.asarray(rows, dtype=np.float64)


def load_plan_meta(path: Path) -> dict:
    meta = json.loads(path.read_text(encoding="utf-8"))
    if "free_pcd" not in meta:
        raise SystemExit("plan json 缺少 free_pcd: {}".format(path))
    return meta
