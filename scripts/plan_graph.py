#!/usr/bin/env python3
"""单层 FREE 体素上的快速 A*（偏好走廊中央）。"""

from __future__ import annotations

import heapq
import math
from collections import deque
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

Key = Tuple[int, int, int]
Neighbor = Tuple[int, float]

XY_DIRS = (
    (1, 0, 1.0),
    (-1, 0, 1.0),
    (0, 1, 1.0),
    (0, -1, 1.0),
    (1, 1, math.sqrt(2.0)),
    (1, -1, math.sqrt(2.0)),
    (-1, 1, math.sqrt(2.0)),
    (-1, -1, math.sqrt(2.0)),
)
CARDINALS = ((1, 0), (-1, 0), (0, 1), (0, -1))


class FreeGraph:
    def __init__(
        self,
        points: np.ndarray,
        resolution: float,
        max_step: float = 0.80,
        center_weight: float = 6.0,
        centerline: Optional[np.ndarray] = None,
    ) -> None:
        if resolution <= 0.0:
            raise ValueError("resolution 必须为正")
        self.points = np.asarray(points, dtype=np.float64)
        self.resolution = float(resolution)
        self.max_step = float(max_step)
        self.center_weight = max(0.0, float(center_weight))
        self.keys: List[Key] = []
        self.index: Dict[Key, int] = {}
        self.columns: Dict[Tuple[int, int], List[int]] = {}
        for i, (x, y, z) in enumerate(self.points):
            key = (
                int(math.floor(x / self.resolution)),
                int(math.floor(y / self.resolution)),
                int(math.floor(z / self.resolution)),
            )
            self.keys.append(key)
            self.index[key] = i
            self.columns.setdefault(key[:2], []).append(i)
        self.clearance = self._clearance_cells()
        self.center_pen = self._center_penalty(centerline)
        self.adj: List[List[Neighbor]] = [[] for _ in range(len(self.points))]
        self._build_edges()

    def _clearance_cells(self) -> np.ndarray:
        """到走廊边界的格子距离：0 在边，越大越居中。"""
        xy_keys = list(self.columns)
        xy_index = {key: i for i, key in enumerate(xy_keys)}
        dist = np.full(len(xy_keys), -1, dtype=np.int32)
        queue: deque = deque()
        for i, (ix, iy) in enumerate(xy_keys):
            if any((ix + dx, iy + dy) not in xy_index for dx, dy in CARDINALS):
                dist[i] = 0
                queue.append(i)
        while queue:
            i = queue.popleft()
            ix, iy = xy_keys[i]
            for dx, dy in CARDINALS:
                j = xy_index.get((ix + dx, iy + dy))
                if j is None or dist[j] >= 0:
                    continue
                dist[j] = dist[i] + 1
                queue.append(j)
        dist[dist < 0] = 0
        by_xy = {key: float(dist[i]) for i, key in enumerate(xy_keys)}
        return np.asarray([by_xy[key[:2]] for key in self.keys], dtype=np.float64)

    def _center_penalty(self, centerline: Optional[np.ndarray]) -> np.ndarray:
        """0=最居中，1=贴边。净空 + 到中线距离。"""
        cmax = float(self.clearance.max()) if len(self.clearance) else 0.0
        if cmax <= 0.0:
            edge = np.zeros(len(self.points), dtype=np.float64)
        else:
            edge = 1.0 - self.clearance / cmax

        if centerline is None or len(centerline) == 0:
            return edge

        center_xy = np.asarray(centerline, dtype=np.float64)[:, :2]
        lateral = np.empty(len(self.points), dtype=np.float64)
        batch = 1024
        for begin in range(0, len(self.points), batch):
            query = self.points[begin : begin + batch, :2]
            delta = query[:, None, :] - center_xy[None, :, :]
            lateral[begin : begin + batch] = np.sqrt(
                np.min(np.einsum("ijk,ijk->ij", delta, delta), axis=1)
            )
        lmax = max(float(lateral.max()), self.resolution)
        return np.clip(0.35 * edge + 0.65 * (lateral / lmax), 0.0, 1.0)

    def _build_edges(self) -> None:
        for i, (ix, iy, iz) in enumerate(self.keys):
            z0 = self.points[i, 2]
            for dx, dy, horiz in XY_DIRS:
                candidates = self.columns.get((ix + dx, iy + dy), ())
                if not candidates:
                    continue
                j = min(candidates, key=lambda candidate: abs(float(self.points[candidate, 2] - z0)))
                dz = abs(float(self.points[j, 2] - z0))
                if dz > self.max_step:
                    continue
                geom = math.hypot(horiz * self.resolution, dz)
                mid = 0.5 * (self.center_pen[i] + self.center_pen[j])
                cost = geom * (1.0 + self.center_weight * mid * mid)
                self.adj[i].append((j, cost))
            for dz_key in (-1, 1):
                j = self.index.get((ix, iy, iz + dz_key))
                if j is not None:
                    self.adj[i].append((j, self.resolution))

    def nearest(self, xyz: Sequence[float], max_dist: float) -> Optional[int]:
        query = np.asarray(xyz, dtype=np.float64)
        dxy2 = (self.points[:, 0] - query[0]) ** 2 + (self.points[:, 1] - query[1]) ** 2
        nearby = np.flatnonzero(dxy2 <= max_dist * max_dist)
        if nearby.size == 0:
            return None
        # 点击附近选更居中的格子，避免从走廊边缘起步
        dz2 = (self.points[nearby, 2] - query[2]) ** 2
        score = (
            dxy2[nearby] / (max_dist * max_dist + 1.0e-9)
            + dz2 / (self.max_step * self.max_step + 1.0e-9)
            + 2.5 * self.center_pen[nearby]
        )
        return int(nearby[int(np.argmin(score))])

    def astar(self, start: int, goal: int) -> Optional[np.ndarray]:
        if start == goal:
            return self.points[[start]]
        goal_xyz = self.points[goal]
        open_heap = [(0.0, 0.0, start)]
        came: Dict[int, int] = {}
        best = {start: 0.0}
        closed = set()
        while open_heap:
            _f, g, node = heapq.heappop(open_heap)
            if node in closed:
                continue
            if node == goal:
                return self._rebuild(came, node)
            closed.add(node)
            for nxt, cost in self.adj[node]:
                if nxt in closed:
                    continue
                tentative = g + cost
                if tentative >= best.get(nxt, 1.0e300):
                    continue
                came[nxt] = node
                best[nxt] = tentative
                remain = float(np.linalg.norm(self.points[nxt] - goal_xyz))
                heapq.heappush(open_heap, (tentative + remain, tentative, nxt))
        return None

    def _rebuild(self, came: Dict[int, int], node: int) -> np.ndarray:
        chain = [node]
        while node in came:
            node = came[node]
            chain.append(node)
        chain.reverse()
        return self.points[np.asarray(chain, dtype=np.int64)]
