#!/usr/bin/env python3
"""在 RViz 里发布规划地图（FREE 体素 + 轨迹）。"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMsg
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from visualization_msgs.msg import Marker


def read_tum_xyz(path: Path) -> np.ndarray:
    rows = []
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        f = line.split()
        rows.append((float(f[1]), float(f[2]), float(f[3])))
    return np.asarray(rows, dtype=np.float64)


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


def resolve_meta_file(meta_path: Path, value: str) -> Path:
    candidate = Path(value).expanduser()
    if candidate.is_absolute():
        return candidate
    choices = (
        meta_path.parent / candidate,
        meta_path.parent.parent / candidate,
        Path.cwd() / candidate,
    )
    return next((item.resolve() for item in choices if item.exists()), choices[0].resolve())


def to_cloud(points: np.ndarray, frame_id: str, stamp) -> PointCloud2:
    msg = PointCloud2()
    msg.header = Header(stamp=stamp, frame_id=frame_id)
    msg.height, msg.width = 1, int(points.shape[0])
    msg.fields = [
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField("y", 4, PointField.FLOAT32, 1),
        PointField("z", 8, PointField.FLOAT32, 1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * msg.width
    msg.is_dense = True
    msg.data = np.asarray(points, dtype=np.float32).reshape(-1).tobytes()
    return msg


def to_path(points: np.ndarray, frame_id: str, stamp) -> PathMsg:
    path = PathMsg()
    path.header = Header(stamp=stamp, frame_id=frame_id)
    for x, y, z in points:
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = float(x), float(y), float(z)
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    return path


def ball(marker_id, xyz, rgba, frame_id, stamp) -> Marker:
    marker = Marker()
    marker.header = Header(stamp=stamp, frame_id=frame_id)
    marker.ns = "plan_ends"
    marker.id = marker_id
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, xyz)
    marker.pose.orientation.w = 1.0
    marker.scale.x = marker.scale.y = marker.scale.z = 0.8
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba
    return marker


def main() -> int:
    parser = argparse.ArgumentParser(description="发布规划地图到 RViz")
    parser.add_argument("--meta", type=Path, required=True, help="tum_to_plan_map.py 写出的 *_plan.json")
    parser.add_argument("--frame-id", default="map")
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    meta_path = args.meta.expanduser().resolve()
    meta = json.loads(meta_path.read_text(encoding="utf-8"))
    free_pts = read_pcd_xyz(resolve_meta_file(meta_path, meta["free_pcd"]))
    path_pts = read_tum_xyz(resolve_meta_file(meta_path, meta["path_tum"]))

    rospy.init_node("visualize_plan_map")
    frame_id = rospy.get_param("~frame_id", args.frame_id)
    free_pub = rospy.Publisher("/plan_map/free", PointCloud2, queue_size=1, latch=True)
    path_pub = rospy.Publisher("/plan_map/path", PathMsg, queue_size=1, latch=True)
    start_pub = rospy.Publisher("/plan_map/start", Marker, queue_size=1, latch=True)
    end_pub = rospy.Publisher("/plan_map/end", Marker, queue_size=1, latch=True)
    rospy.loginfo("FREE=%d path=%d res=%.2f", len(free_pts), len(path_pts), float(meta.get("resolution", 0.2)))

    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        stamp = rospy.Time.now()
        free_pub.publish(to_cloud(free_pts, frame_id, stamp))
        path_pub.publish(to_path(path_pts, frame_id, stamp))
        start_pub.publish(ball(0, path_pts[0], (0.15, 0.85, 1.0, 1.0), frame_id, stamp))
        end_pub.publish(ball(1, path_pts[-1], (1.0, 0.75, 0.15, 1.0), frame_id, stamp))
        rate.sleep()
    return 0


if __name__ == "__main__":
    sys.exit(main())
