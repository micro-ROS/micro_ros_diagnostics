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
#include <unistd.h>

#include <rclc/executor.h>

#include "micro_ros_diagnostic_updater/micro_ros_diagnostic_updater.h"

rcl_ret_t
my_diagnostic_task(diagnostic_value_t * kv)
{
  // actual diagnostic task to be implemented
  rclc_diagnostic_value_set_level(
    kv,
    micro_ros_diagnostic_msgs__msg__MicroROSDiagnosticStatus__STALE);

  return RCL_RET_OK;
}

int main(int argc, const char * argv[])
{
  uint16_t hardware_id = 0;
  uint16_t updater_id = 0;
  uint16_t task_id = 0;
  if (argc < 2) {
    printf("Need at least one argument: hardware ID. Optional: updater ID, task ID.\n");
    exit(1);
  } else {
    hardware_id = atoi(argv[1]);
  }
  if (argc > 2) {
    updater_id = atoi(argv[2]);
  }
  if (argc > 3) {
    task_id = atoi(argv[3]);
  }
  printf("hwmonitor, hardware ID: %d, updater ID: %d.\n", hardware_id, updater_id);

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
  rcl_node_t updater_node = rcl_get_zero_initialized_node();
  rcl_node_options_t node_ops = rcl_node_get_default_options();
  rc = rcl_node_init(
    &updater_node,
    "hwmonitor",
    "",
    &context,
    &node_ops);
  if (rc != RCL_RET_OK) {
    printf("Error in rcl_node_init\n");
    return -1;
  }

  // updater
  diagnostic_updater_t updater;
  rc = rclc_diagnostic_updater_init(&updater, &updater_node, hardware_id, updater_id);
  if (rc != RCL_RET_OK) {
    printf("Error in creating diagnostic updater\n");
    return -1;
  }
  diagnostic_task_t task;
  rc = rclc_diagnostic_task_init(&task, task_id, &my_diagnostic_task);
  if (rc != RCL_RET_OK) {
    printf("Error in creating diagnostic task\n");
    return -1;
  }

  // adding tasks
  rc = rclc_diagnostic_updater_add_task(
    &updater,
    &task);
  if (rc != RCL_RET_OK) {
    printf("Error in adding diagnostic diagnostic task\n");
    return -1;
  }

  for (unsigned int i = 0; i < 100; ++i) {
    printf("Publishing processor diagnostics\n");
    rc = rclc_diagnostic_updater_update(&updater);
    if (rc != RCL_RET_OK) {
      printf("Error in publishing processor diagnostics\n");
      return -1;
    }
    sleep(1);
  }

  rclc_diagnostic_updater_fini(&updater, &updater_node);
  if (rc != RCL_RET_OK) {
    printf("Error while cleaning up!\n");
    return -1;
  }

  return 0;
}
