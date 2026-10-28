cmake_minimum_required(VERSION 3.8)
project(arm_describle)

find_package(ament_cmake REQUIRED)

# SDK 只携带当前 HRB3Q-LC 模型实际使用的资源，避免复制 ros2_ws
# 中无关的多个历史模型和网格目录。
install(DIRECTORY meshes0408 DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})

ament_package()
