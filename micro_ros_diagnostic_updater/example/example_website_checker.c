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
#include <unistd.h>

#include <rclc/executor.h>

#include <micro_ros_diagnostic_updater/micro_ros_diagnostic_updater.h>
#include <micro_ros_diagnostic_msgs/msg/micro_ros_diagnostic_status.h>

static int my_diagnostic_status = 0;
static int my_website_status = 0;
// Hardware ID
static const int16_t WEBSITE_SERIAL = 998;
// Updater ID
static const uint16_t WEBSITE_ID = 0;
// Task ID
static const uint16_t WEBSITE_STATUS_TASK_ID = 42;


rcl_ret_t
my_diagnostic_website_check(
  diagnostic_value_t values[MICRO_ROS_DIAGNOSTIC_UPDATER_MAX_VALUES_PER_TASK],
  uint8_t * number_of_values)
{
  // Cast to avoid warnings
  (void)number_of_values;
  *number_of_values = 1;

  ++my_diagnostic_status;

  values[0].key = WEBSITE_STATUS_TASK_ID;
  if (my_diagnostic_status > 99) {
    my_diagnostic_status = 0;
  }
  if (my_diagnostic_status % 13 == 0) {
    my_website_status = 404;
    rclc_diagnostic_value_set_level(
      &values[0],
      micro_ros_diagnostic_msgs__msg__MicroROSDiagnosticStatus__WARN);
  } else if (my_diagnostic_status % 17 == 0) {
    my_website_status = 500;
    rclc_diagnostic_value_set_level(
      &values[0],
      micro_ros_diagnostic_msgs__msg__MicroROSDiagnosticStatus__ERROR);
  } else {
    my_website_status = 200;
    rclc_diagnostic_value_set_level(
      &values[0],
      micro_ros_diagnostic_msgs__msg__MicroROSDiagnosticStatus__OK);
  }
  rclc_diagnostic_value_lookup(&values[0], my_website_status);

  return RCL_RET_OK;
}

int main(int argc, const char * argv[])
{
  rcl_context_t context = rcl_get_zero_initialized_context();
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t rc;

  // create init_options
  rc = rcl_init_options_init(&init_options, allocator);
  if (rc != RCL_RET_OK) {
    printf("Error rcl_init_options_init.\n");
    return -1;
  }

  // create context
  rc = rcl_init(argc, argv, &init_options, &context);
  if (rc != RCL_RET_OK) {
    printf("Error in rcl_init.\n");
    return -1;
  }

  // create rcl_node
  rcl_node_t my_node = rcl_get_zero_initialized_node();
  rcl_node_options_t node_ops = rcl_node_get_default_options();
  rc = rcl_node_init(
    &my_node,
    "node_0",
    "",
    &context,
    &node_ops);
  if (rc != RCL_RET_OK) {
    printf("Error in rcl_node_init\n");
    return -1;
  }

  // executor
  rclc_executor_t executor;
  executor = rclc_executor_get_zero_initialized_executor();
  unsigned int num_handles = 1;
  rclc_executor_init(&executor, &context, num_handles, &allocator);

  // updater
  diagnostic_updater_t updater;
  rc = rclc_diagnostic_updater_init(&updater, &my_node, &executor);
  if (rc != RCL_RET_OK) {
    printf("Error in creating diagnostic updater\n");
    return -1;
  }
  diagnostic_task_t task;
  rc = rclc_diagnostic_task_init(
    &task, WEBSITE_SERIAL, WEBSITE_ID,
    &my_diagnostic_website_check);
  if (rc != RCL_RET_OK) {
    printf("Error in creating diagnostic task\n");
    return -1;
  }

  // adding tasks
  rc = rclc_diagnostic_updater_add_task(
    &updater,
    &task);
  if (rc != RCL_RET_OK) {
    printf("Error in adding diagnostic temp task\n");
    return -1;
  }

  for (unsigned int i = 0; i < 100; ++i) {
    rc = rclc_diagnostic_updater_update(&updater);
    if (rc != RCL_RET_OK) {
      printf("Error in publishing website diagnostics\n");
      return -1;
    }
    sleep(1);
  }

  rclc_diagnostic_updater_fini(&updater, &my_node, &executor);
  if (rc != RCL_RET_OK) {
    printf("Error while cleaning up!\n");
    return -1;
  }

  return 0;
}
