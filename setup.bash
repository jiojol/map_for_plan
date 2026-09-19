# source /opt/ros/noetic/setup.bash
# source /media/xzq/数据/代码合集/map_for_plan/setup.bash
_MAP_FOR_PLAN_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f "${_MAP_FOR_PLAN_ROOT}/.catkin_ws/devel/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "${_MAP_FOR_PLAN_ROOT}/.catkin_ws/devel/setup.bash"
else
  export ROS_PACKAGE_PATH="${_MAP_FOR_PLAN_ROOT}${ROS_PACKAGE_PATH:+:${ROS_PACKAGE_PATH}}"
fi
unset _MAP_FOR_PLAN_ROOT
