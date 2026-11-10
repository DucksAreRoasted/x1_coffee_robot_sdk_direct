cmake_minimum_required(VERSION 3.8)
project(x1_coffee_hardware_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

# SOEM 2.0.0，GPLv3 / 商业双许可。嵌进本包时不安装、不编示例。
set(SOEM_BUILD_SAMPLES OFF CACHE BOOL "" FORCE)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_subdirectory(third_party/SOEM)

add_library(${PROJECT_NAME} SHARED
  src/coffee_axis_model.cpp
  src/command_guard.cpp
  src/erob_error_names.cpp
  src/ecat_coe_codec.cpp
  src/memory_ethercat_transport.cpp
  src/direct_ecat_init.cpp
  src/zeroerr_drive.cpp
  src/pending_gtn_transport.cpp
  src/soem_ethercat_transport.cpp
  src/urdf_gravity.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(${PROJECT_NAME} PRIVATE soem ${CMAKE_DL_LIBS})

add_executable(ecat_one_axis_jog src/ecat_one_axis_jog.c)
target_link_libraries(ecat_one_axis_jog PRIVATE soem)
target_compile_options(ecat_one_axis_jog PRIVATE -Wall -Wextra)
add_executable(ecat_one_axis_torque src/ecat_one_axis_torque.c)
target_link_libraries(ecat_one_axis_torque PRIVATE soem)
target_compile_options(ecat_one_axis_torque PRIVATE -Wall -Wextra)
add_executable(ecat_safeop_diag src/ecat_safeop_diag.c)
target_link_libraries(ecat_safeop_diag PRIVATE soem)
target_compile_options(ecat_safeop_diag PRIVATE -Wall -Wextra)
install(TARGETS ecat_one_axis_jog ecat_one_axis_torque ecat_safeop_diag
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(coffee_axis_model_test test/coffee_axis_model_test.cpp)
  target_link_libraries(coffee_axis_model_test ${PROJECT_NAME})
  ament_add_gtest(command_guard_test test/command_guard_test.cpp)
  target_link_libraries(command_guard_test ${PROJECT_NAME})
  ament_add_gtest(soem_ethercat_transport_test test/soem_ethercat_transport_test.cpp)
  target_link_libraries(soem_ethercat_transport_test ${PROJECT_NAME})
  target_compile_definitions(soem_ethercat_transport_test PRIVATE
    X1_GRAVITY_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../arm_describle/urdf/HRB3Q-LC.urdf"
    X1_CURRENT_BUNDLE_FIXTURE="${CMAKE_CURRENT_SOURCE_DIR}/test/data/current_bundle.txt")
  ament_add_gtest(erob_error_names_test test/erob_error_names_test.cpp)
  target_link_libraries(erob_error_names_test ${PROJECT_NAME})
  ament_add_gtest(ecat_coe_codec_test test/ecat_coe_codec_test.cpp)
  target_link_libraries(ecat_coe_codec_test ${PROJECT_NAME})
  ament_add_gtest(memory_ethercat_transport_test test/memory_ethercat_transport_test.cpp)
  target_link_libraries(memory_ethercat_transport_test ${PROJECT_NAME})
  ament_add_gtest(direct_ecat_init_test test/direct_ecat_init_test.cpp)
  target_link_libraries(direct_ecat_init_test ${PROJECT_NAME})
  ament_add_gtest(zeroerr_drive_test test/zeroerr_drive_test.cpp)
  target_link_libraries(zeroerr_drive_test ${PROJECT_NAME})
  ament_add_gtest(current_gravity_test test/current_gravity_test.cpp)
  target_link_libraries(current_gravity_test ${PROJECT_NAME})
  target_compile_definitions(current_gravity_test PRIVATE
    X1_CURRENT_BUNDLE_FIXTURE="${CMAKE_CURRENT_SOURCE_DIR}/test/data/current_bundle.txt"
    X1_GRAVITY_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../arm_describle/urdf/HRB3Q-LC.urdf")
  ament_add_gtest(urdf_gravity_test test/urdf_gravity_test.cpp)
  target_link_libraries(urdf_gravity_test ${PROJECT_NAME})
  target_compile_definitions(urdf_gravity_test PRIVATE
    X1_CURRENT_BASIS_FIXTURE="${CMAKE_CURRENT_SOURCE_DIR}/test/data/current_basis.txt"
    X1_GRAVITY_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../arm_describle/urdf/HRB3Q-LC.urdf")
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
