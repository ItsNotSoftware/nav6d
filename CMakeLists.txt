cmake_minimum_required(VERSION 3.10)
project(nav6d)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/PathQuality.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

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

add_executable(n6d_force_controller src/n6d_force_controller_node.cpp)
ament_target_dependencies(
  n6d_force_controller
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
)

install(
  TARGETS n6d_force_controller
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(n6d_velocity_controller src/n6d_velocity_controller_node.cpp)
ament_target_dependencies(
  n6d_velocity_controller
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
)

install(
  TARGETS n6d_velocity_controller
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(n6d_path_evaluator src/n6d_path_evaluator_node.cpp)
ament_target_dependencies(
  n6d_path_evaluator
  rclcpp
  nav_msgs
  octomap
  octomap_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(n6d_path_evaluator ${cpp_typesupport_target})

install(
  TARGETS n6d_path_evaluator
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

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
