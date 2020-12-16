cmake_minimum_required(VERSION 3.5)
project(micro_ros_common_diagnostics)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclc REQUIRED)
find_package(rcutils REQUIRED)
find_package(micro_ros_diagnostic_msgs REQUIRED)
find_package(micro_ros_diagnostic_updater REQUIRED)
set(ament_cmake_copyright_FOUND TRUE)

add_executable(hwmonitor src/hwmonitor.cpp)
target_include_directories(hwmonitor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(hwmonitor
  "rclc"
  "rcutils"
  "micro_ros_diagnostic_msgs"
  "micro_ros_diagnostic_updater"
)

install(TARGETS hwmonitor
  DESTINATION lib/${PROJECT_NAME})

ament_package()
