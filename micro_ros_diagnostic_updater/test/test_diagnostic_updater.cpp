// Copyright (c) 2020 - for information on the respective copyright owner
// see the NOTICE file and/or the repository
// https://github.com/micro-ROS/micro_ros_diagnostics.
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

rcl_ret_t
update_function_mockup_0(diagnostic_value_t * kv)
{
  ++diagnostic_mockup_counter_0;
  rclc_diagnostic_value_set_int(kv, 17);
  rclc_diagnostic_value_set_level(
    kv,
    micro_ros_diagnostic_msgs__msg__MicroROSDiagnosticStatus__OK);

  return RCL_RET_OK;
}

rcl_ret_t
update_function_mockup_1(diagnostic_value_t * kv)
{
  ++diagnostic_mockup_counter_1;
  rclc_diagnostic_value_set_int(kv, 42);
  rclc_diagnostic_value_set_level(
    kv,
    micro_ros_diagnostic_msgs__msg__MicroROSDiagnosticStatus__STALE);

  return RCL_RET_OK;
}

TEST(TestDiagnosticUpdater, create_diagnostic_task) {
  diagnostic_task_t task;
  rcl_ret_t rc = rclc_diagnostic_task_init(&task, 0, &update_function_mockup_0);
  EXPECT_EQ(RCL_RET_OK, rc);
}

TEST(TestDiagnosticUpdater, create_diagnostic_values) {
  diagnostic_value_t value;

  rclc_diagnostic_value_set_int(&value, 17);
  EXPECT_EQ(value.int_value, 17);
  EXPECT_EQ(
    value.value_type,
    micro_ros_diagnostic_msgs__msg__MicroROSDiagnosticStatus__VALUE_INT);

  rclc_diagnostic_value_lookup(&value, 17);
  EXPECT_EQ(value.value_id, 17);
  EXPECT_EQ(
    value.value_type,
    micro_ros_diagnostic_msgs__msg__MicroROSDiagnosticStatus__VALUE_LOOKUP);

  rclc_diagnostic_value_set_level(
    &value,
    micro_ros_diagnostic_msgs__msg__MicroROSDiagnosticStatus__OK);
  EXPECT_EQ(
    value.level,
    micro_ros_diagnostic_msgs__msg__MicroROSDiagnosticStatus__OK);
  rclc_diagnostic_value_set_level(
    &value,
    micro_ros_diagnostic_msgs__msg__MicroROSDiagnosticStatus__WARN);
  EXPECT_EQ(
    value.level,
    micro_ros_diagnostic_msgs__msg__MicroROSDiagnosticStatus__WARN);
}

TEST(TestDiagnosticUpdater, create_updater) {
  rclc_support_t support;
  rcl_ret_t rc;

  // node
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rc = rclc_support_init(&support, 0, nullptr, &allocator);
  const char * my_name = "test_updater_node";
  const char * my_namespace = "";
  rcl_node_t node = rcl_get_zero_initialized_node();
  rc = rclc_node_init_default(&node, my_name, my_namespace, &support);

  // updater
  diagnostic_updater_t updater;
  rc = rclc_diagnostic_updater_init(&updater, &node, 0, 0);
  EXPECT_EQ(RCL_RET_OK, rc);

  // updater
  rc = rclc_diagnostic_updater_fini(&updater, &node);
  EXPECT_EQ(RCL_RET_OK, rc);
}

TEST(TestDiagnosticUpdater, updater_add_tasks) {
  rclc_support_t support;
  rcl_ret_t rc;

  // node
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rc = rclc_support_init(&support, 0, nullptr, &allocator);
  const char * my_name = "test_updater_node";
  const char * my_namespace = "";
  rcl_node_t node = rcl_get_zero_initialized_node();
  rc = rclc_node_init_default(&node, my_name, my_namespace, &support);

  // updater
  diagnostic_updater_t updater;
  rc = rclc_diagnostic_updater_init(&updater, &node, 0, 0);
  EXPECT_EQ(RCL_RET_OK, rc);

  diagnostic_task_t task;
  rc = rclc_diagnostic_task_init(&task, 17, &update_function_mockup_0);
  EXPECT_EQ(RCL_RET_OK, rc);

  rc = rclc_diagnostic_updater_add_task(&updater, &task);
  EXPECT_EQ(RCL_RET_OK, rc);

  for (unsigned int i = 1; i < MICRO_ROS_UPDATER_MAX_NUMBER_OF_TASKS; ++i) {
    rc = rclc_diagnostic_updater_add_task(&updater, &task);
    EXPECT_EQ(RCL_RET_OK, rc);
  }

  // Should exceed maximum number of tasks per updater
  rc = rclc_diagnostic_updater_add_task(&updater, &task);
  EXPECT_EQ(RCL_RET_ERROR, rc);

  // updater
  rc = rclc_diagnostic_updater_fini(&updater, &node);
  EXPECT_EQ(RCL_RET_OK, rc);
}

TEST(TestDiagnosticUpdater, updater_update) {
  rclc_support_t support;
  rcl_ret_t rc;

  // node
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rc = rclc_support_init(&support, 0, nullptr, &allocator);
  const char * my_name = "test_updater_node";
  const char * my_namespace = "";
  rcl_node_t node = rcl_get_zero_initialized_node();
  rc = rclc_node_init_default(&node, my_name, my_namespace, &support);

  // updater
  diagnostic_updater_t updater;
  rc = rclc_diagnostic_updater_init(&updater, &node, 0, 0);
  EXPECT_EQ(RCL_RET_OK, rc);

  diagnostic_task_t task0, task1;
  rc = rclc_diagnostic_task_init(
    &task0,
    0,
    &update_function_mockup_0);
  rc = rclc_diagnostic_task_init(
    &task1,
    1,
    &update_function_mockup_1);

  rc = rclc_diagnostic_updater_add_task(&updater, &task0);
  rc = rclc_diagnostic_updater_update(&updater);
  EXPECT_EQ(1, diagnostic_mockup_counter_0);

  rc = rclc_diagnostic_updater_add_task(&updater, &task1);
  rc = rclc_diagnostic_updater_update(&updater);
  rc = rclc_diagnostic_updater_update(&updater);
  EXPECT_EQ(3, diagnostic_mockup_counter_0);
  EXPECT_EQ(2, diagnostic_mockup_counter_1);
  EXPECT_EQ(RCL_RET_OK, rc);

  // updater
  rc = rclc_diagnostic_updater_fini(&updater, &node);
  EXPECT_EQ(RCL_RET_OK, rc);
}
