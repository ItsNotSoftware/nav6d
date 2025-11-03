cmake_minimum_required(VERSION 3.10)
project(nav6d)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_executable(n6d_planner src/n6d_planner_node.cpp)
ament_target_dependencies(
  n6d_planner
  rclcpp
  geometry_msgs
  nav_msgs
  octomap
  octomap_msgs
  visualization_msgs
  tf2
)

install(
  TARGETS n6d_planner
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}/resource
)

ament_package()
