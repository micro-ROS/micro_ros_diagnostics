cmake_minimum_required(VERSION 3.5)
project(micro_ros_diagnostic_bridge)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# build options
option(MICRO_ROS_DIAGNOSTIC_BRIDGE_EXAMPLES "Build example updaters for the micro-ROS diagnostic bridge package." FALSE)

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

if(MICRO_ROS_DIAGNOSTIC_BRIDGE_EXAMPLES)
  # launch
  install(DIRECTORY example/launch DESTINATION share/${PROJECT_NAME}/)
  # table
  install(FILES example/example_table.yaml DESTINATION share/${PROJECT_NAME}/)
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(osrf_testing_tools_cpp REQUIRED)

  # the following line skips the linter which checks for copyrights
  # remove the line when a copyright and license is present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  set(LOOKUP_TABLE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/test/test_lookup_table.yaml)
  configure_file(test/lookup_tables.h.in ${CMAKE_CURRENT_BINARY_DIR}/micro_ros_diagnostic_bridge/lookup_tables.h)

  ament_add_gtest(test_diagnostic_bridge
    test/test_diagnostic_bridge.cpp
    src/micro_ros_diagnostic_bridge/micro_ros_diagnostic_bridge.cpp)
  if(TARGET test_diagnostic_bridge)
    target_include_directories(test_diagnostic_bridge PUBLIC
      ${micro_ros_diagnostic_msgs_INCLUDE_DIRS}
      ${diagnostic_msgs_INCLUDE_DIRS}
      ${rclcpp_INCLUDE_DIRS}
      ${CMAKE_CURRENT_BINARY_DIR}
    )
    ament_target_dependencies(test_diagnostic_bridge
      "rclcpp"
      "diagnostic_msgs"
      "micro_ros_diagnostic_msgs")
  endif()
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
