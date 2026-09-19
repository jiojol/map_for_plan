#!/usr/bin/env bash
# 输入一份 TUM，生成规划地图并打开 RViz。
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "用法: $0 /path/to/traj.tum [output_prefix]" >&2
  exit 2
fi

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
export ROS_PACKAGE_PATH="${ROOT}${ROS_PACKAGE_PATH:+:${ROS_PACKAGE_PATH}}"

TRAJ="$1"
PREFIX="${2:-${ROOT}/output/$(basename "${TRAJ%.*}")}"

python3 "${ROOT}/scripts/tum_to_plan_map.py" --traj "${TRAJ}" --output-prefix "${PREFIX}"
exec roslaunch map_for_plan plan.launch meta:="${PREFIX}_plan.json"
