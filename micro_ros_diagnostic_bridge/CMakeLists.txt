cmake_minimum_required(VERSION 3.5)
project(micro_ros_diagnostic_bridge)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(micro_ros_diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)

# bridge executable
include_directories(include)
add_executable(diagnostic_bridge
  src/diagnostic_bridge_node.cpp
  src/micro_ros_diagnostic_bridge/micro_ros_diagnostic_bridge.cpp)
target_include_directories(diagnostic_bridge
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
)
ament_target_dependencies(diagnostic_bridge
  rclcpp
  diagnostic_msgs
  micro_ros_diagnostic_msgs
)
install(TARGETS diagnostic_bridge
  DESTINATION lib/${PROJECT_NAME})

# launch
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# install
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(osrf_testing_tools_cpp REQUIRED)
  find_package(ros_environment REQUIRED)

  # the following line skips the linter which checks for copyrights
  # remove the line when a copyright and license is present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# export dependencies
# specific order: dependents before dependencies
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(ament_cmake)
ament_export_dependencies(micro_ros_diagnostic_msgs)
ament_export_dependencies(rosidl_generator_c)
ament_export_dependencies(rcl)
ament_export_dependencies(rcutils)
ament_package()
