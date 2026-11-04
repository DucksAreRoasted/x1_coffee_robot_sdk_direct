cmake_minimum_required(VERSION 3.8)
project(x1_coffee_integration)

find_package(ament_cmake REQUIRED)

# 不安装 Python 缓存。root 跑过的 pyc 会让 symlink install 报 Permission denied。
install(DIRECTORY config launch urdf vendor
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)
install(PROGRAMS
  scripts/switch_control_mode.sh
  scripts/robot_startup.py
  scripts/limit_zero_calibration.py
  scripts/spawn_controllers.py
  scripts/keyboard_tcp_teleop.py
  scripts/pvt_trajectory_executor.py
  scripts/align_joint_direction.py
  scripts/align_urdf_zero.py
  scripts/joint_angle_check.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  ament_add_test(test_startup_motion
    COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/test/test_startup_motion.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  ament_add_test(test_pvt_trajectory_executor
    COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/test/test_pvt_trajectory_executor.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
endif()

ament_package()
