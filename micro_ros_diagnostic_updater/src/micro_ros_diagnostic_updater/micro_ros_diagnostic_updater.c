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
#include <micro_ros_diagnostic_msgs/msg/micro_ros_diagnostic_status.h>


void
rclc_diagnostic_value_set_int(
  diagnostic_value_t * kv,
  int64_t value)
{
  kv->value_type = micro_ros_diagnostic_msgs__msg__MicroROSDiagnosticStatus__VALUE_INT;
  kv->int_value = value;
}

void
rclc_diagnostic_value_lookup(
  diagnostic_value_t * kv,
  int16_t value_id)
{
  kv->value_type = micro_ros_diagnostic_msgs__msg__MicroROSDiagnosticStatus__VALUE_LOOKUP;
  kv->value_id = value_id;
}

void
rclc_diagnostic_value_set_level(
  diagnostic_value_t * kv,
  int8_t level)
{
  kv->level = level;
}

rcl_ret_t
rclc_diagnostic_task_init(
  diagnostic_task_t * task,
  int16_t key,
  rcl_ret_t (* function)(diagnostic_value_t *))
{
  RCL_CHECK_FOR_NULL_WITH_MSG(
    task, "task is a null pointer", return RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_FOR_NULL_WITH_MSG(
    function, "function is a null pointer", return RCL_RET_INVALID_ARGUMENT);

  task->id = key;
  task->function = function;

  return RCL_RET_OK;
}

rcl_ret_t
rclc_diagnostic_updater_init(
  diagnostic_updater_t * updater,
  const rcl_node_t * node,
  int16_t id,
  int16_t hardware_id)
{
  RCL_CHECK_FOR_NULL_WITH_MSG(
    updater, "updater is a null pointer", return RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_FOR_NULL_WITH_MSG(
    node, "node is a null pointer", return RCL_RET_INVALID_ARGUMENT);

  updater->id = id;
  updater->hardware_id = hardware_id;
  updater->num_tasks = 0;

  // publisher
  updater->diag_pub = rcl_get_zero_initialized_publisher();
  const char * topic_name = "/diagnostics_uros";
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
      "Updater '%d' could not create publisher /%s.",
      updater->id,
      topic_name);
    return RCL_RET_ERROR;
  }

  return RCL_RET_OK;
}

rcl_ret_t
rclc_diagnostic_updater_fini(
  diagnostic_updater_t * updater,
  rcl_node_t * node)
{
  RCL_CHECK_FOR_NULL_WITH_MSG(
    updater, "updater is a null pointer", return RCL_RET_INVALID_ARGUMENT);

  rcl_ret_t rc = rcl_publisher_fini(&updater->diag_pub, node);
  if (RCL_RET_OK != rc) {
    RCUTILS_LOG_ERROR(
      "Error when cleaning updater %d. Could not delete publisher.",
      updater->id);
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
      "Updater %d could not add task %d, already %d tasks added.",
      updater->id,
      task->id,
      MICRO_ROS_UPDATER_MAX_NUMBER_OF_TASKS);
    return RCL_RET_ERROR;
  }
  updater->tasks[updater->num_tasks] = task;
  ++(updater->num_tasks);

  return RCL_RET_OK;
}

rcl_ret_t
rclc_diagnostic_call_task(
  diagnostic_task_t * task)
{
  return (task->function)(&task->value);
}

rcl_ret_t
rclc_diagnostic_updater_update(
  diagnostic_updater_t * updater)
{
  RCL_CHECK_FOR_NULL_WITH_MSG(
    updater, "updater is a null pointer", return RCL_RET_INVALID_ARGUMENT);

  micro_ros_diagnostic_msgs__msg__MicroROSDiagnosticStatus diag_msg;
  micro_ros_diagnostic_msgs__msg__MicroROSDiagnosticStatus__init(&diag_msg);

  // name
  diag_msg.updater_id = updater->id;
  diag_msg.hardware_id = updater->hardware_id;

  for (unsigned int i = 0; i < updater->num_tasks; ++i) {
    rcl_ret_t task_ok = rclc_diagnostic_call_task(updater->tasks[i]);

    if (task_ok == RCL_RET_OK) {
      diag_msg.key = updater->tasks[i]->id;
      diag_msg.value_type = updater->tasks[i]->value.value_type;
      diag_msg.bool_value = updater->tasks[i]->value.bool_value;
      diag_msg.int_value = updater->tasks[i]->value.int_value;
      diag_msg.double_value = updater->tasks[i]->value.double_value;
      diag_msg.value_id = updater->tasks[i]->value.value_id;
      diag_msg.level = updater->tasks[i]->value.level;

      rcl_ret_t rc = rcl_publish(&updater->diag_pub, &diag_msg, NULL);
      if (rc == RCL_RET_OK) {
        RCUTILS_LOG_DEBUG(
          "Updater '%d' published value for '%d'.",
          updater->id,
          updater->tasks[i]->id);
      }
    } else {
      RCUTILS_LOG_ERROR(
        "Updater '%d' could not update diagnostic task '%d'.",
        updater->id,
        updater->tasks[i]->id);
    }
  }

  micro_ros_diagnostic_msgs__msg__MicroROSDiagnosticStatus__fini(&diag_msg);

  return RCL_RET_OK;
}
