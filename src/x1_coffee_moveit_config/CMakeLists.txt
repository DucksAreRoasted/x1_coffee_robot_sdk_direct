cmake_minimum_required(VERSION 3.22)
project(x1_coffee_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch test DESTINATION share/${PROJECT_NAME})

ament_package()
