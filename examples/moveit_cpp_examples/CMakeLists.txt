cmake_minimum_required(VERSION 3.22)
project(x1_coffee_moveit_cpp_examples)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(coffee_moveit_cpp_demo src/coffee_moveit_cpp_demo.cpp)
ament_target_dependencies(coffee_moveit_cpp_demo geometry_msgs moveit_ros_planning_interface rclcpp)

add_executable(coffee_bimanual_cartesian_demo src/coffee_bimanual_cartesian_demo.cpp)
ament_target_dependencies(
  coffee_bimanual_cartesian_demo
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
)

install(
  TARGETS coffee_moveit_cpp_demo coffee_bimanual_cartesian_demo
  DESTINATION lib/${PROJECT_NAME}
)
install(
  PROGRAMS run_single_arm.sh run_bimanual.sh run_bimanual_cartesian.sh
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
