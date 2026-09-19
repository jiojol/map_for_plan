cmake_minimum_required(VERSION 3.0.2)
project(map_for_plan)

find_package(catkin REQUIRED COMPONENTS
  rospy
  nav_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
)
catkin_package()

catkin_install_python(PROGRAMS
  scripts/tum_to_plan_map.py
  scripts/visualize_plan_map.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
