cmake_minimum_required(VERSION 3.5)
project(micro_ros_diagnostic_updater)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# find dependencies
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

# build options
option(MICRO_ROS_DIAGNOSTIC_UPDATER_EXAMPLES "Build example updaters for the micro-ROS diagnostic updater package." FALSE)

# create library
add_library(${PROJECT_NAME}
  src/micro_ros_diagnostic_updater/micro_ros_diagnostic_updater.c
)
target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
)
ament_target_dependencies(${PROJECT_NAME}
  rclc
  rcutils
  micro_ros_diagnostic_msgs
)

include_directories(include)

# examples
if(MICRO_ROS_DIAGNOSTIC_UPDATER_EXAMPLES)
  message(STATUS "Building example updaters")

  set(processor_example example_processor_updater)
  add_executable(${processor_example} example/example_processor_updater.c)
  target_link_libraries(${processor_example} ${PROJECT_NAME})
  ament_target_dependencies(${processor_example}
    rclc
    rcutils
    micro_ros_diagnostic_msgs)

  install(TARGETS
    ${processor_example}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

  set(website_example example_website_checker)
  add_executable(${website_example} example/example_website_checker.c)
  target_link_libraries(${website_example} ${PROJECT_NAME})
  ament_target_dependencies(${website_example}
    rclc
    rcutils
    micro_ros_diagnostic_msgs)

  install(TARGETS
    ${website_example}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endif()

# install
install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} DESTINATION lib)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

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
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
  ament_target_dependencies(${PROJECT_NAME}_test
    rcl
    rclc
    micro_ros_diagnostic_msgs
  )
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
