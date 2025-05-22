cmake_minimum_required(VERSION 3.5)
project(micro_ros_diagnostic_updater)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(micro_ros_diagnostic_msgs REQUIRED)
find_package(rclc REQUIRED)


if("${rcl_VERSION}" VERSION_LESS "1.0.0")
  message(STATUS "Found rcl_lifecycle version ${rcl_lifecycle_VERSION}, which belongs to Eloquent or earlier")
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra)
  endif()
else()
  message(STATUS "Found rcl_lifecycle version ${rcl_lifecycle_VERSION}, which belongs to Foxy or later")
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
  endif()
endif()

# Build options
option(MICRO_ROS_DIAGNOSTIC_UPDATER_EXAMPLES "Build example updaters for the micro-ROS diagnostic updater package." FALSE)

# Configuration
set(MICRO_ROS_DIAGNOSTIC_UPDATER_MAX_TASKS_PER_UPDATER "5" CACHE STRING "Number of tasks preallocated per updater.")
set(MICRO_ROS_DIAGNOSTIC_UPDATER_DIAGNOSTICS_TOPIC_PREFIX "" CACHE STRING "Prefix for micro-ROS diagnostic topic (append slash '/')")
set(MICRO_ROS_DIAGNOSTIC_UPDATER_MAX_VALUES_PER_TASK "4" CACHE STRING "Number of pre allocated key value pairs.")

# Create library
add_library(${PROJECT_NAME}
  src/micro_ros_diagnostic_updater/micro_ros_diagnostic_updater.c
)
target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  rclc::rclc
  rcutils::rcutils
  ${micro_ros_diagnostic_msgs_TARGETS}
)

# Create source files with the define
configure_file(${PROJECT_SOURCE_DIR}/src/config.h.in
  ${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}/config.h)

include_directories(include)

# examples
if(MICRO_ROS_DIAGNOSTIC_UPDATER_EXAMPLES)
  message(STATUS "Building example updaters")

  set(processor_example example_processor_updater)
  add_executable(${processor_example} example/example_processor_updater.c)
  target_link_libraries(${processor_example}
    ${PROJECT_NAME}
    rclc::rclc
    rcutils::rcutils
    ${micro_ros_diagnostic_msgs_TARGETS}
  )

  install(TARGETS
    ${processor_example}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

  set(website_example example_website_checker)
  add_executable(${website_example} example/example_website_checker.c)
  target_link_libraries(${website_example}
    ${PROJECT_NAME}
    rclc::rclc
    rcutils::rcutils
    ${micro_ros_diagnostic_msgs_TARGETS}
  )

  install(TARGETS
    ${website_example}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endif()

# Install
install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME} DESTINATION lib)

# Install config.h file.
install(
  FILES
    ${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}/config.h
  DESTINATION
    include/${PROJECT_NAME}
)

# Export old-style CMake variables
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

# Export modern CMake targets
ament_export_targets(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(osrf_testing_tools_cpp REQUIRED)
  # the following line skips the linter which checks for copyrights
  # remove the line when a copyright and license is present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(${PROJECT_NAME}_test
    test/test_diagnostic_updater.cpp)

  target_include_directories(${PROJECT_NAME}_test PRIVATE include src)
  target_link_libraries(${PROJECT_NAME}_test
    ${PROJECT_NAME}
    rcl::rcl
    rclc::rclc
    ${micro_ros_diagnostic_msgs_TARGETS}
  )
endif()

ament_export_dependencies(
  micro_ros_diagnostic_msgs
  rosidl_generator_c
  rcl
  rclc
  rcutils
)
ament_package()
