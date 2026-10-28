cmake_minimum_required(VERSION 3.8)
project(x1_coffee_device_sdk_example)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(x1_coffee_device_sdk REQUIRED)

add_executable(device_sdk_example main.cpp)
ament_target_dependencies(device_sdk_example rclcpp x1_coffee_device_sdk)
install(TARGETS device_sdk_example DESTINATION lib/${PROJECT_NAME})

ament_package()
