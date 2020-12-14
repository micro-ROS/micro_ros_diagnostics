// Copyright 2020 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// testing default transition sequence.
// This test requires that the transitions are set
// as depicted in design.ros2.org

#include <stdio.h>
#include <iostream>
#include <gtest/gtest.h>

extern "C"
{
#include <rclc/rclc.h> 
#include "micro_ros_diagnostic_updater/micro_ros_diagnostic_updater.h"
}

static int diagnostic_mockup_counter_0 = 0;
static int diagnostic_mockup_counter_1 = 0;

const char* update_function_mockup_0()
{
  ++diagnostic_mockup_counter_0;
  return "23 degrees";
}

const char* update_function_mockup_1()
{
  ++diagnostic_mockup_counter_1;
  return "42 degrees";
}

TEST(TestDiagnosticUpdater, create_diagnostic_task) {
  diagnostic_task_t task;
  rcl_ret_t res = rclc_diagnostic_task_init(
    &task,
    "mytemperatur",
    &update_function_mockup_0);
  EXPECT_EQ(RCL_RET_OK, res);
}

TEST(TestDiagnosticUpdater, create_updater) {
  rclc_support_t support;
  rcl_ret_t rc;

  // node
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rc = rclc_support_init(&support, 0, nullptr, &allocator);
  const char * my_name = "test_updater_node";
  const char * my_namespace = "";
  const char * topic_name = "diagnostic_test";
  rcl_node_t node = rcl_get_zero_initialized_node();
  rc = rclc_node_init_default(&node, my_name, my_namespace, &support);

 // updater
  diagnostic_updater_t updater;
  rcl_ret_t res = rclc_diagnostic_updater_init(
    &updater,
    &node,
    "hw",
    "mocked hardware monitoring",
    "42");
  EXPECT_EQ(RCL_RET_OK, res);
}

TEST(TestDiagnosticUpdater, updater_add_tasks) {
  rclc_support_t support;
  rcl_ret_t rc;

  // node
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rc = rclc_support_init(&support, 0, nullptr, &allocator);
  const char * my_name = "test_updater_node";
  const char * my_namespace = "";
  const char * topic_name = "diagnostic_test";
  rcl_node_t node = rcl_get_zero_initialized_node();
  rc = rclc_node_init_default(&node, my_name, my_namespace, &support);

 // updater
  diagnostic_updater_t updater;
  rcl_ret_t res = rclc_diagnostic_updater_init(
    &updater,
    &node,
    "hw",
    "mocked hardware monitoring",
    "42");
  EXPECT_EQ(RCL_RET_OK, res);

  diagnostic_task_t task;
  res = rclc_diagnostic_task_init(
    &task,
    "mytemperatur",
    &update_function_mockup_0);
  EXPECT_EQ(RCL_RET_OK, res);

  res = rclc_diagnostic_updater_add_task(
    &updater,
    &task);
  EXPECT_EQ(RCL_RET_OK, res);

  for (unsigned int i = 1; i < MICRO_ROS_UPDATER_MAX_NUMBER_OF_TASKS; ++i) {
    res = rclc_diagnostic_updater_add_task(
      &updater,
      &task);
    EXPECT_EQ(RCL_RET_OK, res);
  }

  // Should exceed maximum number of tasks per updater
  res = rclc_diagnostic_updater_add_task(
    &updater,
    &task);
  EXPECT_EQ(RCL_RET_ERROR, res);
}

TEST(TestDiagnosticUpdater, updater_update) {
  rclc_support_t support;
  rcl_ret_t rc;

  // node
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rc = rclc_support_init(&support, 0, nullptr, &allocator);
  const char * my_name = "test_updater_node";
  const char * my_namespace = "";
  const char * topic_name = "diagnostic_test";
  rcl_node_t node = rcl_get_zero_initialized_node();
  rc = rclc_node_init_default(&node, my_name, my_namespace, &support);

 // updater
  diagnostic_updater_t updater;
  rcl_ret_t res = rclc_diagnostic_updater_init(
    &updater,
    &node,
    "hw",
    "mocked hardware monitoring",
    "42");
  EXPECT_EQ(RCL_RET_OK, res);

  diagnostic_task_t task0, task1;
  res = rclc_diagnostic_task_init(
    &task0,
    "mytemperatur",
    &update_function_mockup_0);
  res = rclc_diagnostic_task_init(
    &task1,
    "myothertemperatur",
    &update_function_mockup_1);

  res = rclc_diagnostic_updater_add_task(&updater, &task0);
  res = rclc_diagnostic_updater_update(&updater);
  EXPECT_EQ(1, diagnostic_mockup_counter_0);

  res = rclc_diagnostic_updater_add_task(&updater, &task1);
  res = rclc_diagnostic_updater_update(&updater);
  res = rclc_diagnostic_updater_update(&updater);
  EXPECT_EQ(3, diagnostic_mockup_counter_0);
  EXPECT_EQ(2, diagnostic_mockup_counter_1);

  EXPECT_EQ(RCL_RET_OK, res);
}
