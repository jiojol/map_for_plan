cmake_minimum_required(VERSION 3.0.2)
project(map_for_plan)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  rviz
  pluginlib
  tf2
)

find_package(Qt5 REQUIRED COMPONENTS Widgets)

catkin_package(
  CATKIN_DEPENDS rospy roscpp nav_msgs sensor_msgs geometry_msgs visualization_msgs rviz pluginlib tf2
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(map_for_plan_rviz
  src/pose3d_tool.cpp
  src/goal3d_tool.cpp
  src/init3d_tool.cpp
  include/plan_map_rviz/goal3d_tool.h
  include/plan_map_rviz/init3d_tool.h
)

target_link_libraries(map_for_plan_rviz
  ${catkin_LIBRARIES}
  Qt5::Widgets
)

catkin_install_python(PROGRAMS
  scripts/tum_to_plan_map.py
  scripts/visualize_plan_map.py
  scripts/global_planner.py
  scripts/plan_graph.py
  scripts/plan_io.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS map_for_plan_rviz
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY launch rviz icons
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
