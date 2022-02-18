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
#ifndef MICRO_ROS_DIAGNOSTIC_UPDATER__MICRO_ROS_DIAGNOSTIC_UPDATER_H_
#define MICRO_ROS_DIAGNOSTIC_UPDATER__MICRO_ROS_DIAGNOSTIC_UPDATER_H_

#include <rcl/types.h>
#include <rclc/publisher.h>
#include <micro_ros_diagnostic_msgs/msg/micro_ros_diagnostic_status.h>
#include "micro_ros_diagnostic_updater/config.h"

typedef struct diagnostic_value_t
{
  int8_t value_type;
  uint16_t key;

  bool bool_value;
  int32_t int_value;
  float double_value;
  int16_t value_id;

  int8_t level;
} diagnostic_value_t;

typedef struct diagnostic_task_t
{
  int16_t id;
  uint8_t number_of_key_values;
  diagnostic_value_t key_values[MICRO_ROS_DIAGNOSTIC_UPDATER_MAX_KEY_VALUES_PER_TASK];
  int16_t hardware_id;
  int16_t updater_id;
  rcl_ret_t (* function)(diagnostic_value_t[MICRO_ROS_DIAGNOSTIC_UPDATER_MAX_KEY_VALUES_PER_TASK]);
} diagnostic_task_t;

typedef struct diagnostic_updater_t
{
  int16_t id;
  uint8_t num_tasks;
  diagnostic_task_t * tasks[MICRO_ROS_DIAGNOSTIC_UPDATER_MAX_TASKS_PER_UPDATER];
  rcl_publisher_t diag_pub;
  micro_ros_diagnostic_msgs__msg__MicroROSDiagnosticStatus diag_status;
} diagnostic_updater_t;

void
rclc_diagnostic_value_set_int(
  diagnostic_value_t * kv,
  int32_t value);

void
rclc_diagnostic_value_lookup(
  diagnostic_value_t * kv,
  int16_t value_id);

void
rclc_diagnostic_value_set_level(
  diagnostic_value_t * kv,
  int8_t level);

rcl_ret_t
rclc_diagnostic_task_init(
  diagnostic_task_t * task,
  int16_t hardware_id,
  int16_t updater_id,
  int16_t id,
  rcl_ret_t (* function)(diagnostic_value_t *));

rcl_ret_t
rclc_diagnostic_updater_init(
  diagnostic_updater_t * updater,
  const rcl_node_t * node);

rcl_ret_t
rclc_diagnostic_updater_fini(
  diagnostic_updater_t * updater,
  rcl_node_t * node);

rcl_ret_t
rclc_diagnostic_updater_add_task(
  diagnostic_updater_t * updater,
  diagnostic_task_t * task);

rcl_ret_t
rclc_diagnostic_call_task(
  diagnostic_task_t * task);

rcl_ret_t
rclc_diagnostic_updater_update(
  diagnostic_updater_t * updater);

#endif  // MICRO_ROS_DIAGNOSTIC_UPDATER__MICRO_ROS_DIAGNOSTIC_UPDATER_H_
