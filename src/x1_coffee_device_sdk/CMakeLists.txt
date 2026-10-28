cmake_minimum_required(VERSION 3.8)
project(x1_coffee_device_sdk)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(x1_coffee_interfaces REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/axis_clients.cpp
  src/gripper_client.cpp
  src/motor_state_reader.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs x1_coffee_interfaces)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

add_executable(gripper_action_server src/gripper_action_server.cpp)
ament_target_dependencies(
  gripper_action_server control_msgs rclcpp rclcpp_action std_msgs)
target_compile_options(gripper_action_server PRIVATE -Wall -Wextra -Wpedantic)

add_executable(safety_service_server src/safety_service_server.cpp)
ament_target_dependencies(
  safety_service_server rclcpp std_msgs std_srvs x1_coffee_interfaces)
target_compile_options(safety_service_server PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS gripper_action_server safety_service_server
  DESTINATION lib/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  control_msgs rclcpp rclcpp_action std_msgs std_srvs x1_coffee_interfaces)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(device_sdk_contract_test test/device_sdk_contract_test.cpp)
  target_link_libraries(device_sdk_contract_test ${PROJECT_NAME})
  ament_target_dependencies(
    device_sdk_contract_test rclcpp std_msgs std_srvs x1_coffee_interfaces)
  add_dependencies(device_sdk_contract_test safety_service_server)
endif()

ament_package()
