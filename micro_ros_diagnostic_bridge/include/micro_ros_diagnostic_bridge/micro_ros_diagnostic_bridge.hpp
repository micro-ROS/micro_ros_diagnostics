// Copyright (c) 2021 - for information on the respective copyright owner
// see the NOTICE file and/or the repository https://github.com/microros/system_modes
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
#pragma once

#include <climits>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "micro_ros_diagnostic_msgs/msg/micro_ros_diagnostic_status.hpp"


static const char UROS_DIAGNOSTICS_BRIDGE_TOPIC_IN[] = "/diagnostics_uros";
static const char UROS_DIAGNOSTICS_BRIDGE_TOPIC_OUT[] = "/diagnostics";

namespace uros_diagnostic_msg = micro_ros_diagnostic_msgs::msg;
namespace diagnostic_msg = diagnostic_msgs::msg;

struct MicroROSDiagnosticUpdater
{
  std::string name;
  std::string description;
};

struct MicroROSDiagnosticKey
{
  int updater_id;
  int key_id;

  bool operator<(const MicroROSDiagnosticKey & rhs) const
  {
    return (updater_id * UINT_MAX + key_id) < (rhs.updater_id * UINT_MAX + rhs.key_id);
  }
};

struct MicroROSDiagnosticValue
{
  MicroROSDiagnosticKey task;
  int value_id;

  bool operator<(const MicroROSDiagnosticValue & rhs) const
  {
    return (task.updater_id * UINT_MAX * UINT_MAX + task.key_id * UINT_MAX + value_id) <
           (rhs.task.updater_id * UINT_MAX * UINT_MAX + rhs.task.key_id * UINT_MAX + rhs.value_id);
  }
};

typedef std::map<int, std::string> HardwareMap;
typedef std::map<int, MicroROSDiagnosticUpdater> UpdaterMap;
typedef std::map<MicroROSDiagnosticKey, std::string> KeyMap;
typedef std::map<MicroROSDiagnosticValue, std::string> ValueMap;

class MicroROSDiagnosticBridge : public rclcpp::Node
{
public:
  explicit MicroROSDiagnosticBridge(const std::string & path = "");

  virtual const std::string lookup_hardware(
    int hardware_id);
  virtual const MicroROSDiagnosticUpdater lookup_updater(
    int updater_id);
  virtual const std::string lookup_key(
    int updater_id,
    int key);
  virtual const std::string lookup_value(
    int updater_id,
    int key,
    int value_id);

protected:
  virtual void read_lookup_table(const std::string & path);

  rclcpp::Logger logger_;
  rclcpp::Subscription<uros_diagnostic_msg::MicroROSDiagnosticStatus>::SharedPtr uros_sub_;
  rclcpp::Publisher<diagnostic_msg::DiagnosticStatus>::SharedPtr ros2_pub_;
  std::unique_ptr<diagnostic_msg::DiagnosticStatus> msg_out_;

  HardwareMap hardware_map_;
  UpdaterMap updater_map_;
  KeyMap key_map_;
  ValueMap value_map_;
};
