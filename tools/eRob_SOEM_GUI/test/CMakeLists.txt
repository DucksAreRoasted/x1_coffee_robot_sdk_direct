# 此测试不链接真实 SOEM 库，所有总线函数由测试内替身提供。
add_executable(single_axis_test
  single_axis_test.cpp
  ../src/ethercat/pdo_manager.cpp
  ../src/ethercat/sdo_manager.cpp
  ../src/ethercat/ethercat_manager.cpp
  ../src/ethercat/dc_manager.cpp
)
target_include_directories(single_axis_test PRIVATE
  ../include ../third_party/SOEM/soem ../third_party/SOEM/osal
  ../third_party/SOEM/osal/linux ../third_party/SOEM/oshw/linux
)
set_target_properties(single_axis_test PROPERTIES AUTOMOC OFF AUTOUIC OFF AUTORCC OFF)
add_test(NAME single_axis_routing COMMAND single_axis_test)

# 复用实际窗口源码，只排除应用 main；不确认网口，因此不访问硬件。
find_package(Qt5 COMPONENTS Widgets PrintSupport Network Concurrent REQUIRED)
get_target_property(GUI_SOURCES ethercat_monitor SOURCES)
list(TRANSFORM GUI_SOURCES PREPEND "${PROJECT_SOURCE_DIR}/src/")
add_executable(single_axis_ui_test single_axis_ui_test.cpp ${GUI_SOURCES})
target_compile_definitions(single_axis_ui_test PRIVATE X1_GUI_NO_MAIN)
get_target_property(GUI_INCLUDES ethercat_monitor INCLUDE_DIRECTORIES)
target_include_directories(single_axis_ui_test PRIVATE ${GUI_INCLUDES})
target_link_libraries(single_axis_ui_test PRIVATE ethercat_core Qt5::Widgets
  Qt5::PrintSupport Qt5::Network Qt5::Concurrent qcustomplot)
add_test(NAME single_axis_ui COMMAND single_axis_ui_test)
set_tests_properties(single_axis_ui PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen" TIMEOUT 15)
