cmake_minimum_required(VERSION 3.5)
project(convoy_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

set(
  DEPENDENCIES
  "rclcpp"
  "ackermann_msgs"
  "sensor_msgs"
  "std_msgs"
  "geometry_msgs"
  "visualization_msgs"
  "nav_msgs"
)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(ackermann_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


set(msg_files
  "msg/LeaderGapFollow.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

ament_export_dependencies(rosidl_default_runtime)

include_directories(include)

# add executables

add_executable(safety_node 
  src/safety.cpp
  src/safety_node.cpp
)
ament_target_dependencies(
  safety_node 
  rclcpp 
  ackermann_msgs 
  nav_msgs
  sensor_msgs 
  std_msgs
)

add_executable(wall_follow_node
  src/wall_follow.cpp
  src/wall_follow_node.cpp
)
ament_target_dependencies(
  wall_follow_node
  rclcpp
  ackermann_msgs
  sensor_msgs
  std_msgs
)

add_executable(gap_follow_node
  src/gap_follow.cpp
  src/gap_follow_node.cpp
)
ament_target_dependencies(
  gap_follow_node
  rclcpp
  ackermann_msgs
  sensor_msgs
  std_msgs
)

add_executable(pure_pursuit_node
  src/pure_pursuit.cpp
  src/pure_pursuit_node.cpp
)
ament_target_dependencies(
  pure_pursuit_node
  rclcpp
  ackermann_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  nav_msgs
)

add_executable(leader_gf_node
  src/leader_gf.cpp
  src/leader_gf_node.cpp
)
ament_target_dependencies(
  leader_gf_node
  rclcpp
  ackermann_msgs
  sensor_msgs
  nav_msgs
)

add_executable(follower_pp_node
  src/follower_pp.cpp
  src/follower_pp_node.cpp
)
ament_target_dependencies(
  follower_pp_node
  rclcpp
  ackermann_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  nav_msgs
)

# Install launch, config, rviz files
install(
  DIRECTORY
  config
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

# Install nodes/targets
install(
  TARGETS
  safety_node
  wall_follow_node
  gap_follow_node
  pure_pursuit_node
  leader_gf_node
  follower_pp_node
  DESTINATION lib/${PROJECT_NAME}/
)

rosidl_target_interfaces(
  leader_gf_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
rosidl_target_interfaces(
  follower_pp_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

install(
  DIRECTORY include/
  DESTINATION include
)

# install python modules
ament_python_install_package(${PROJECT_NAME})

# install python executables
install(PROGRAMS
  scripts/safety_node.py
	DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)

ament_export_dependencies(${DEPENDENCIES})

ament_package()
