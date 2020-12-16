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
#include <rcl/error_handling.h>
#include <rcutils/logging_macros.h>

#include <micro_ros_diagnostic_updater/micro_ros_diagnostic_updater.h>

rcl_ret_t
rclc_diagnostic_task_init(
  diagnostic_task_t * task,
  const char * name,
  const char * (*function)(void))
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
  const rcl_node_t * node,
  const char * name,
  const char * message,
  const char * hardware_id)
{
  RCL_CHECK_FOR_NULL_WITH_MSG(
    updater, "updater is a null pointer", return RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_FOR_NULL_WITH_MSG(
    node, "node is a null pointer", return RCL_RET_INVALID_ARGUMENT);
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

  // publisher
  updater->diag_pub = rcl_get_zero_initialized_publisher();
  const char * topic_name = "diagnostics_uc";
  const rosidl_message_type_support_t * diag_type_support =
    ROSIDL_GET_MSG_TYPE_SUPPORT(
    micro_ros_diagnostic_msgs,
    msg,
    MicroROSDiagnosticStatus);
  rcl_publisher_options_t pub_options = rcl_publisher_get_default_options();
  rcl_ret_t rc = rcl_publisher_init(
    &updater->diag_pub,
    node,
    diag_type_support,
    topic_name,
    &pub_options);
  if (RCL_RET_OK != rc) {
    RCUTILS_LOG_ERROR(
      "Updater '%s' could not create publisher /%s.",
      updater->name,
      topic_name);
    return RCL_RET_ERROR;
  }

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

const char *
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

  micro_ros_diagnostic_msgs__msg__MicroROSDiagnosticStatus diag_msg;
  micro_ros_diagnostic_msgs__msg__MicroROSDiagnosticStatus__init(&diag_msg);

  const int STRSIZE = 99;

  // name
  diag_msg.name.data = malloc(STRSIZE);
  diag_msg.name.capacity = STRSIZE;
  diag_msg.name.size = snprintf(
    diag_msg.name.data,
    diag_msg.name.capacity,
    "%s",
    updater->name);

  // message
  diag_msg.message.data = malloc(STRSIZE);
  diag_msg.message.capacity = STRSIZE;
  diag_msg.message.size = snprintf(
    diag_msg.message.data,
    diag_msg.message.capacity,
    "%s",
    updater->message);

  // hardware id
  diag_msg.hardware_id.data = malloc(STRSIZE);
  diag_msg.hardware_id.capacity = STRSIZE;
  diag_msg.hardware_id.size = snprintf(
    diag_msg.hardware_id.data,
    diag_msg.hardware_id.capacity,
    "%s",
    updater->hardware_id);

  for (int i = 0; i < updater->num_tasks; ++i) {
    diag_msg.level = 0;

    diag_msg.key.data = malloc(STRSIZE);
    diag_msg.key.capacity = STRSIZE;
    diag_msg.key.size = snprintf(
      diag_msg.key.data,
      diag_msg.key.capacity,
      "%s",
      updater->tasks[i]->name);

    diag_msg.value.data = malloc(STRSIZE);
    diag_msg.value.capacity = STRSIZE;
    diag_msg.value.size = snprintf(
      diag_msg.value.data,
      diag_msg.value.capacity,
      "%s",
      rclc_diagnostic_call_task(updater->tasks[i]));

    rcl_ret_t rc = rcl_publish(&updater->diag_pub, &diag_msg, NULL);

    if (rc == RCL_RET_OK) {
      RCUTILS_LOG_DEBUG(
        "Updater '%s' published '%s':'%s'.",
        updater->name,
        diag_msg.key.data,
        diag_msg.value.data);
    }
  }

  micro_ros_diagnostic_msgs__msg__MicroROSDiagnosticStatus__fini(&diag_msg);

  return RCL_RET_OK;
}
