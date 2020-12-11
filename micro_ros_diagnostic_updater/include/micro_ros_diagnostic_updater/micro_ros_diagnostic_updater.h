// Copyright (c) 2020 - for information on the respective copyright owner
// see the NOTICE file and/or the repository https://github.com/ros2/rclc.
// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#define MICRO_ROS_UPDATER_MAX_NUMBER_OF_TASKS 5

typedef struct diagnostic_key_value_t
{
  const char * key;
  const char * value;
} diagnostic_key_value_t;

typedef struct diagnostic_task_t
{
  const char * name;
  const char* (* function)(void);
} diagnostic_task_t;

typedef struct diagnostic_updater_t
{
  const char * name;
  const char * message;
  const char * hardware_id;
  int num_tasks;
  diagnostic_task_t * tasks[MICRO_ROS_UPDATER_MAX_NUMBER_OF_TASKS];
} diagnostic_updater_t;

rcl_ret_t
rclc_diagnostic_task_init(
  diagnostic_task_t * task,
  const char * name,
  const char* (* function)(void));

rcl_ret_t
rclc_diagnostic_updater_init(
  diagnostic_updater_t * updater,
  const char * name,
  const char * message,
  const char * hardware_id);

rcl_ret_t
rclc_diagnostic_updater_add_task(
  diagnostic_updater_t * updater,
  diagnostic_task_t * task);

const char*
rclc_diagnostic_call_task(
  diagnostic_task_t * task);

rcl_ret_t
rclc_diagnostic_updater_update(
  diagnostic_updater_t * updater);

#endif  // MICRO_ROS_DIAGNOSTIC_UPDATER__MICRO_ROS_DIAGNOSTIC_UPDATER_H_
