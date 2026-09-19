# source /opt/ros/noetic/setup.bash
# source /media/xzq/数据/代码合集/map_for_plan/setup.bash
_MAP_FOR_PLAN_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROS_PACKAGE_PATH="${_MAP_FOR_PLAN_ROOT}${ROS_PACKAGE_PATH:+:${ROS_PACKAGE_PATH}}"
unset _MAP_FOR_PLAN_ROOT
