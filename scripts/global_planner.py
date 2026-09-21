#!/usr/bin/env python3
"""在单层 FREE 体素上做快速全局规划。

起点：/initialpose（3D Pose Estimate 或 2D Pose Estimate）
终点：/move_base_simple/goal（3D Nav Goal 或 2D Nav Goal）
结果：/plan_map/global_path
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

_SCRIPT_DIR = Path(__file__).resolve().parent
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path as PathMsg
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

from plan_graph import FreeGraph
from plan_io import load_plan_meta, read_pcd_xyz, read_tum_xyz


def _xyz_from_pose(pose) -> np.ndarray:
    return np.array([pose.position.x, pose.position.y, pose.position.z], dtype=np.float64)


def _yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _yaws_along(points: np.ndarray, start_yaw: float, goal_yaw: float) -> np.ndarray:
    yaws = np.zeros(len(points), dtype=np.float64)
    for i in range(len(points)):
        if i + 1 < len(points):
            dx = points[i + 1, 0] - points[i, 0]
            dy = points[i + 1, 1] - points[i, 1]
            if dx * dx + dy * dy > 1.0e-8:
                yaws[i] = math.atan2(dy, dx)
                continue
        yaws[i] = yaws[i - 1] if i else start_yaw
    if len(points):
        yaws[0] = start_yaw
        yaws[-1] = goal_yaw
    return yaws


def to_path(points: np.ndarray, frame_id: str, stamp, yaws=None) -> PathMsg:
    path = PathMsg()
    path.header = Header(stamp=stamp, frame_id=frame_id)
    for i, (x, y, z) in enumerate(points):
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        if yaws is None:
            pose.pose.orientation.w = 1.0
        else:
            yaw = float(yaws[i])
            pose.pose.orientation.z = math.sin(yaw * 0.5)
            pose.pose.orientation.w = math.cos(yaw * 0.5)
        path.poses.append(pose)
    return path


def ball(marker_id, xyz, rgba, frame_id, stamp, scale=0.7) -> Marker:
    marker = Marker()
    marker.header = Header(stamp=stamp, frame_id=frame_id)
    marker.ns = "plan_query"
    marker.id = marker_id
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, xyz)
    marker.pose.orientation.w = 1.0
    marker.scale.x = marker.scale.y = marker.scale.z = scale
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba
    return marker


class GlobalPlanner:
    def __init__(self, graph: FreeGraph, frame_id: str, snap_radius: float) -> None:
        self.graph = graph
        self.frame_id = frame_id
        self.snap_radius = snap_radius
        self.start_xyz = None
        self.goal_xyz = None
        self.start_yaw = 0.0
        self.goal_yaw = 0.0
        self.path_xyz = None
        self.path_yaws = None

        self.path_pub = rospy.Publisher("/plan_map/global_path", PathMsg, queue_size=1, latch=True)
        self.start_pub = rospy.Publisher("/plan_map/query_start", Marker, queue_size=1, latch=True)
        self.goal_pub = rospy.Publisher("/plan_map/query_goal", Marker, queue_size=1, latch=True)
        rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self._on_start, queue_size=1)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self._on_goal, queue_size=1)
        rospy.Subscriber("/plan_map/start_pose", PoseStamped, self._on_start_pose, queue_size=1)
        rospy.Subscriber("/plan_map/goal_pose", PoseStamped, self._on_goal, queue_size=1)

    def _on_start(self, msg: PoseWithCovarianceStamped) -> None:
        self._set_start(_xyz_from_pose(msg.pose.pose), _yaw_from_quat(msg.pose.pose.orientation))

    def _on_start_pose(self, msg: PoseStamped) -> None:
        self._set_start(_xyz_from_pose(msg.pose), _yaw_from_quat(msg.pose.orientation))

    def _on_goal(self, msg: PoseStamped) -> None:
        self._set_goal(_xyz_from_pose(msg.pose), _yaw_from_quat(msg.pose.orientation))

    def _set_start(self, xyz: np.ndarray, yaw: float) -> None:
        idx = self.graph.nearest(xyz, self.snap_radius)
        if idx is None:
            rospy.logwarn("起点离可通行地图太远: (%.2f, %.2f, %.2f)", *xyz)
            return
        self.start_xyz = self.graph.points[idx]
        self.start_yaw = yaw
        rospy.loginfo("起点吸附 (%.2f, %.2f, %.2f)", *self.start_xyz)
        self._publish_queries(rospy.Time.now())
        self._try_plan()

    def _set_goal(self, xyz: np.ndarray, yaw: float) -> None:
        idx = self.graph.nearest(xyz, self.snap_radius)
        if idx is None:
            rospy.logwarn("终点离可通行地图太远: (%.2f, %.2f, %.2f)", *xyz)
            return
        self.goal_xyz = self.graph.points[idx]
        self.goal_yaw = yaw
        rospy.loginfo("终点吸附 (%.2f, %.2f, %.2f)", *self.goal_xyz)
        self._publish_queries(rospy.Time.now())
        self._try_plan()

    def _try_plan(self) -> None:
        if self.start_xyz is None or self.goal_xyz is None:
            return
        start = self.graph.nearest(self.start_xyz, self.snap_radius)
        goal = self.graph.nearest(self.goal_xyz, self.snap_radius)
        if start is None or goal is None:
            rospy.logwarn("起终点无法吸附到 FREE 体素")
            return
        t0 = time.perf_counter()
        path = self.graph.astar(start, goal)
        dt_ms = (time.perf_counter() - t0) * 1000.0
        if path is None:
            self.path_xyz = None
            self.path_yaws = None
            rospy.logwarn("规划失败: 无可通行路径 (%.1f ms)", dt_ms)
            self._publish_path(rospy.Time.now())
            return
        length = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum()) if len(path) > 1 else 0.0
        self.path_xyz = path
        self.path_yaws = _yaws_along(path, self.start_yaw, self.goal_yaw)
        rospy.loginfo("规划成功: %d 点, %.1f m, %.1f ms", len(path), length, dt_ms)
        self._publish_path(rospy.Time.now())

    def _publish_queries(self, stamp) -> None:
        if self.start_xyz is not None:
            self.start_pub.publish(ball(0, self.start_xyz, (0.15, 0.85, 1.0, 1.0), self.frame_id, stamp))
        if self.goal_xyz is not None:
            self.goal_pub.publish(ball(1, self.goal_xyz, (1.0, 0.45, 0.85, 1.0), self.frame_id, stamp))

    def _publish_path(self, stamp) -> None:
        if self.path_xyz is None:
            empty = PathMsg()
            empty.header = Header(stamp=stamp, frame_id=self.frame_id)
            self.path_pub.publish(empty)
            return
        self.path_pub.publish(to_path(self.path_xyz, self.frame_id, stamp, self.path_yaws))
        self._publish_queries(stamp)

    def spin(self) -> None:
        rate = rospy.Rate(2.0)
        while not rospy.is_shutdown():
            self._publish_path(rospy.Time.now())
            rate.sleep()


def _load_graph(meta: dict, max_step: float, center_weight: float) -> FreeGraph:
    points = read_pcd_xyz(Path(meta["free_pcd"]))
    centerline = None
    path_tum = meta.get("path_tum")
    if path_tum:
        tum_path = Path(path_tum)
        if tum_path.is_file():
            centerline = read_tum_xyz(tum_path)
    return FreeGraph(
        points,
        float(meta.get("resolution", 0.2)),
        max_step=max_step,
        center_weight=center_weight,
        centerline=centerline,
    )


def _path_center_score(graph: FreeGraph, path: np.ndarray) -> float:
    keys = [
        graph.index.get(
            (
                int(math.floor(x / graph.resolution)),
                int(math.floor(y / graph.resolution)),
                int(math.floor(z / graph.resolution)),
            )
        )
        for x, y, z in path
    ]
    vals = [graph.center_pen[i] for i in keys if i is not None]
    return float(np.mean(vals)) if vals else 1.0


def self_test(meta_path: Path, snap_radius: float, max_step: float, center_weight: float) -> int:
    meta = load_plan_meta(meta_path)
    graph = _load_graph(meta, max_step, center_weight)
    start = graph.nearest(graph.points[0], snap_radius)
    goal = graph.nearest(graph.points[len(graph.points) // 2], snap_radius)
    if start is None or goal is None:
        print("self-test: 无法吸附起终点")
        return 1
    t0 = time.perf_counter()
    path = graph.astar(start, goal)
    dt_ms = (time.perf_counter() - t0) * 1000.0
    if path is None:
        print("self-test: 规划失败 {:.1f} ms, nodes={}".format(dt_ms, len(graph.points)))
        return 1
    length = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
    print(
        "self-test: {} -> {} points, {:.1f} m, {:.1f} ms, center_pen={:.3f}, free={}".format(
            start, goal, length, dt_ms, _path_center_score(graph, path), len(graph.points)
        )
    )
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="FREE 体素全局规划")
    parser.add_argument("--meta", type=Path, required=True)
    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--snap-radius", type=float, default=2.0)
    parser.add_argument("--max-step", type=float, default=0.80, help="相邻体素允许的最大高度差，米")
    parser.add_argument("--center-weight", type=float, default=6.0, help="越大越贴走廊中央")
    parser.add_argument("--self-test", action="store_true")
    argv = sys.argv[1:] if "--self-test" in sys.argv else rospy.myargv(argv=sys.argv)[1:]
    args = parser.parse_args(argv)

    if args.self_test:
        return self_test(args.meta, args.snap_radius, args.max_step, args.center_weight)

    meta = load_plan_meta(args.meta)
    graph = _load_graph(meta, args.max_step, args.center_weight)
    rospy.init_node("global_planner")
    frame_id = rospy.get_param("~frame_id", args.frame_id)
    snap_radius = float(rospy.get_param("~snap_radius", args.snap_radius))
    rospy.loginfo(
        "FREE=%d res=%.2f snap=%.2f max_step=%.2f center_weight=%.2f",
        len(graph.points),
        graph.resolution,
        snap_radius,
        graph.max_step,
        graph.center_weight,
    )
    GlobalPlanner(graph, frame_id, snap_radius).spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
