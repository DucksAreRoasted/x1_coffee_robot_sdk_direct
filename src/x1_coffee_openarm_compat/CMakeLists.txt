cmake_minimum_required(VERSION 3.8)
project(x1_coffee_openarm_compat)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(x1_coffee_interfaces REQUIRED)
find_package(x1_coffee_hardware_core REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/openarm_position_controller.cpp
  src/openarm_joint_state_broadcaster.cpp
  src/motor_state_broadcaster.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  controller_interface hardware_interface pluginlib rclcpp rclcpp_lifecycle realtime_tools
  sensor_msgs std_msgs x1_coffee_interfaces x1_coffee_hardware_core)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)

pluginlib_export_plugin_description_file(controller_interface x1_coffee_openarm_compat.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES x1_coffee_openarm_compat.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(
  controller_interface hardware_interface pluginlib rclcpp rclcpp_lifecycle realtime_tools
  sensor_msgs std_msgs x1_coffee_interfaces x1_coffee_hardware_core)
ament_package()
