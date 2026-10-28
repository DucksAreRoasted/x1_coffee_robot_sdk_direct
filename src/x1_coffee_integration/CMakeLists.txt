cmake_minimum_required(VERSION 3.8)
project(x1_coffee_integration)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch urdf vendor DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/switch_control_mode.sh
  scripts/robot_startup.py
  scripts/limit_zero_calibration.py
  scripts/spawn_controllers.py
  scripts/keyboard_tcp_teleop.py
  scripts/align_joint_direction.py
  scripts/align_urdf_zero.py
  scripts/joint_angle_check.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
