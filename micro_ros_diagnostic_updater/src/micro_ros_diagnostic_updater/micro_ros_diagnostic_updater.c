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
  int32_t value)
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
  int16_t hardware_id,
  int16_t updater_id,
  int16_t id,
  rcl_ret_t (* function)(diagnostic_value_t[MICRO_ROS_DIAGNOSTIC_UPDATER_MAX_KEY_VALUES_PER_TASK], uint8_t * number_of_key_values))
{
  RCL_CHECK_FOR_NULL_WITH_MSG(
    task, "task is a null pointer", return RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_FOR_NULL_WITH_MSG(
    function, "function is a null pointer", return RCL_RET_INVALID_ARGUMENT);

  task->id = id;
  task->function = function;
  task->updater_id = updater_id;
  task->hardware_id = hardware_id;

  return RCL_RET_OK;
}

rcl_ret_t
rclc_diagnostic_updater_init(
  diagnostic_updater_t * updater,
  const rcl_node_t * node)
{
  RCL_CHECK_FOR_NULL_WITH_MSG(
    updater, "updater is a null pointer", return RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_FOR_NULL_WITH_MSG(
    node, "node is a null pointer", return RCL_RET_INVALID_ARGUMENT);

  updater->num_tasks = 0;

  // publisher
  updater->diag_pub = rcl_get_zero_initialized_publisher();
  const rosidl_message_type_support_t * diag_type_support =
    ROSIDL_GET_MSG_TYPE_SUPPORT(
    micro_ros_diagnostic_msgs,
    msg,
    MicroROSDiagnosticStatus);
  rcl_ret_t rc = rclc_publisher_init_default(
    &updater->diag_pub,
    node,
    diag_type_support,
    UROS_DIAGNOSTIC_UPDATER_TOPIC);
  if (RCL_RET_OK != rc) {
    RCUTILS_LOG_ERROR(
      "Updater '%d' could not create publisher /%s.",
      updater->id,
      UROS_DIAGNOSTIC_UPDATER_TOPIC);
    return RCL_RET_ERROR;
  }

  // message
  micro_ros_diagnostic_msgs__msg__MicroROSDiagnosticStatus__init(&updater->diag_status);

  return RCL_RET_OK;
}

rcl_ret_t
rclc_diagnostic_updater_fini(
  diagnostic_updater_t * updater,
  rcl_node_t * node)
{
  RCL_CHECK_FOR_NULL_WITH_MSG(
    updater, "updater is a null pointer", return RCL_RET_INVALID_ARGUMENT);

  micro_ros_diagnostic_msgs__msg__MicroROSDiagnosticStatus__fini(&updater->diag_status);

  rcl_ret_t rc = rcl_publisher_fini(&updater->diag_pub, node);
  if (RCL_RET_OK != rc) {
    RCUTILS_LOG_ERROR(
      "Error when cleaning updater. Could not delete publisher.");
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

  if (updater->num_tasks >= MICRO_ROS_DIAGNOSTIC_UPDATER_MAX_TASKS_PER_UPDATER) {
    RCUTILS_LOG_ERROR(
      "Updater could not add task %d, already %d tasks added.",
      task->id,
      MICRO_ROS_DIAGNOSTIC_UPDATER_MAX_TASKS_PER_UPDATER);
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
  return (task->function)(task->key_values, &task->number_of_key_values);
}

rcl_ret_t
rclc_diagnostic_updater_update(
  diagnostic_updater_t * updater)
{
  RCL_CHECK_FOR_NULL_WITH_MSG(
    updater, "updater is a null pointer", return RCL_RET_INVALID_ARGUMENT);

  for (unsigned int i = 0; i < updater->num_tasks; ++i) {
    rcl_ret_t task_ok = rclc_diagnostic_call_task(updater->tasks[i]);

    if ( (task_ok == RCL_RET_OK) &&
      (updater->tasks[i]->number_of_key_values <=
      MICRO_ROS_DIAGNOSTIC_UPDATER_MAX_KEY_VALUES_PER_TASK) )
    {
      // name
      updater->diag_status.updater_id = updater->tasks[i]->updater_id;
      updater->diag_status.hardware_id = updater->tasks[i]->hardware_id;
      updater->diag_status.number_of_key_values = updater->tasks[i]->number_of_key_values;

      for (uint8_t value_index = 0u; value_index < updater->tasks[i]->number_of_key_values;
        value_index++)
      {
        updater->diag_status.key_values[value_index].key =
          updater->tasks[i]->key_values[value_index].key;
        updater->diag_status.key_values[value_index].value_type =
          updater->tasks[i]->key_values[value_index].value_type;
        updater->diag_status.key_values[value_index].bool_value =
          updater->tasks[i]->key_values[value_index].bool_value;
        updater->diag_status.key_values[value_index].int_value =
          updater->tasks[i]->key_values[value_index].int_value;
        updater->diag_status.key_values[value_index].double_value =
          updater->tasks[i]->key_values[value_index].double_value;
        updater->diag_status.key_values[value_index].value_id =
          updater->tasks[i]->key_values[value_index].value_id;
        updater->diag_status.key_values[value_index].level =
          updater->tasks[i]->key_values[value_index].level;
      }

      rcl_ret_t rc = rcl_publish(&updater->diag_pub, &updater->diag_status, NULL);
      if (rc == RCL_RET_OK) {
        RCUTILS_LOG_DEBUG(
          "Updater published value for '%d'.",
          updater->tasks[i]->id);
      }
    } else {
      RCUTILS_LOG_ERROR(
        "Updater could not update diagnostic task '%d'.",
        updater->tasks[i]->id);
    }
  }

  return RCL_RET_OK;
}
