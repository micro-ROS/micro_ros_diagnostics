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

#include <rcl/rcl.h>
#include <micro_ros_diagnostic_updater/micro_ros_diagnostic_updater.h>

static const char * UPDATER_NAME = "Name of the updater";
static const char * UPDATER_DESC = "Description of the updater";
static const char * UPDATER_HW_ID = "Updater Hardware ID";
static const char * TASK_NAME = "Name of the diagnostic task";

const char * my_diagnostic_function()
{
  static char dvalue[20] = "to be implemented";

  // Diagnostics goes here, e.g., sensor readings

  return dvalue;
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
    UPDATER_NAME,
    "",
    &context,
    &node_ops);
  if (rc != RCL_RET_OK) {
    printf("Error in rcl_node_init\n");
    return -1;
  }

  // updater
  diagnostic_updater_t updater;
  rc = rclc_diagnostic_updater_init(
    &updater,
    &my_node,
    UPDATER_NAME,
    UPDATER_DESC,
    UPDATER_HW_ID);
  if (rc != RCL_RET_OK) {
    printf("Error in creating diagnostic updater\n");
    return -1;
  }

  diagnostic_task_t task;
  rc = rclc_diagnostic_task_init(
    &task,
    TASK_NAME,
    &my_diagnostic_function);
  if (rc != RCL_RET_OK) {
    printf("Error in creating diagnostic task\n");
    return -1;
  }

  rc = rclc_diagnostic_updater_add_task(
    &updater,
    &task);
  if (rc != RCL_RET_OK) {
    printf("Error in adding diagnostic task\n");
    return -1;
  }

  while (RCL_RET_OK) {
    rc = rclc_diagnostic_updater_update(&updater);
    sleep(1);
  }

  if (rc != RCL_RET_OK) {
    printf("Error while cleaning up!\n");
    return -1;
  }
  return RCL_RET_OK;
}
