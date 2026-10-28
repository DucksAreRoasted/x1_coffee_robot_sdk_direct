cmake_minimum_required(VERSION 3.22)
project(x1_coffee_moveit_py_examples)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/coffee_moveit_py_demo.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
