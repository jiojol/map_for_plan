#!/usr/bin/env bash
# 编译 RViz 3D 起终点工具。规划节点本身不需要编译。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${ROOT}/.catkin_ws"

source /opt/ros/noetic/setup.bash
mkdir -p "${WS}/src"
ln -sfn "${ROOT}" "${WS}/src/map_for_plan"
cd "${WS}"
catkin_make -DCMAKE_BUILD_TYPE=Release

echo
echo "编译完成。请重新 source："
echo "  source ${ROOT}/setup.bash"
