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

#include <rcl/error_handling.h>
#include <rcutils/logging_macros.h>

#include <micro_ros_diagnostic_updater/micro_ros_diagnostic_updater.h>

rcl_ret_t
rclc_diagnostic_task_init(
  diagnostic_task_t * task,
  const char * name,
  const char* (* function)(void))
{
  RCL_CHECK_FOR_NULL_WITH_MSG(
    task, "task is a null pointer", return RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_FOR_NULL_WITH_MSG(
    name, "name is a null pointer", return RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_FOR_NULL_WITH_MSG(
    function, "function is a null pointer", return RCL_RET_INVALID_ARGUMENT);

  task->name = name;
  task->function = function;

  return RCL_RET_OK;
}

rcl_ret_t
rclc_diagnostic_updater_init(
  diagnostic_updater_t * updater,
  const char * name,
  const char * message,
  const char * hardware_id)
{
  RCL_CHECK_FOR_NULL_WITH_MSG(
    updater, "updater is a null pointer", return RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_FOR_NULL_WITH_MSG(
    name, "name is a null pointer", return RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_FOR_NULL_WITH_MSG(
    message, "message is a null pointer", return RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_FOR_NULL_WITH_MSG(
    hardware_id, "hardware_id is a null pointer", return RCL_RET_INVALID_ARGUMENT);

  updater->name = name;
  updater->message = message;
  updater->hardware_id = hardware_id;
  updater->num_tasks = 0;

  return RCL_RET_OK;
}

rcl_ret_t
rclc_diagnostic_updater_add_task(
  diagnostic_updater_t * updater,
  diagnostic_task_t * task)
{
  RCL_CHECK_FOR_NULL_WITH_MSG(
    updater, "updater is a null pointer", return RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_FOR_NULL_WITH_MSG(
    task, "task is a null pointer", return RCL_RET_INVALID_ARGUMENT);

  if (updater->num_tasks >= MICRO_ROS_UPDATER_MAX_NUMBER_OF_TASKS) {
    RCUTILS_LOG_ERROR(
      "Updater '%s' could not add task '%s', already %d tasks added.",
      updater->name,
      task->name,
      MICRO_ROS_UPDATER_MAX_NUMBER_OF_TASKS);
    return RCL_RET_ERROR;
  }
  updater->tasks[updater->num_tasks] = task;
  ++(updater->num_tasks);

  return RCL_RET_OK;
}

const char*
rclc_diagnostic_call_task(
  diagnostic_task_t * task)
{
  return (task->function)();
}

rcl_ret_t
rclc_diagnostic_updater_update(
  diagnostic_updater_t * updater)
{
  RCL_CHECK_FOR_NULL_WITH_MSG(
    updater, "updater is a null pointer", return RCL_RET_INVALID_ARGUMENT);

  for (unsigned int i = 0; i < updater->num_tasks; ++i) {
    const char* value = rclc_diagnostic_call_task(updater->tasks[i]);
    // TODO(anordman): publish diagnostic status
  }

  return RCL_RET_OK;
}
