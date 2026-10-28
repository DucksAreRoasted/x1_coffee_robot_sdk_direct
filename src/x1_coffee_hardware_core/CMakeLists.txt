cmake_minimum_required(VERSION 3.8)
project(x1_coffee_hardware_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/coffee_axis_model.cpp
  src/command_guard.cpp
  src/erob_error_names.cpp
  src/gtn_native_api.cpp
  src/pending_gtn_transport.cpp
  src/production_gtn_transport.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(${PROJECT_NAME} PRIVATE ${CMAKE_DL_LIBS})

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(coffee_axis_model_test test/coffee_axis_model_test.cpp)
  target_link_libraries(coffee_axis_model_test ${PROJECT_NAME})
  ament_add_gtest(command_guard_test test/command_guard_test.cpp)
  target_link_libraries(command_guard_test ${PROJECT_NAME})
  ament_add_gtest(production_gtn_transport_test test/production_gtn_transport_test.cpp)
  target_link_libraries(production_gtn_transport_test ${PROJECT_NAME})
  ament_add_gtest(erob_error_names_test test/erob_error_names_test.cpp)
  target_link_libraries(erob_error_names_test ${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
