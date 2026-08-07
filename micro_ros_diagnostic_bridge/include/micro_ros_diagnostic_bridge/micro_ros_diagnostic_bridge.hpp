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

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "micro_ros_diagnostic_msgs/msg/micro_ros_diagnostic_status.hpp"


static const char UROS_DIAGNOSTICS_BRIDGE_TOPIC_IN[] = "diagnostics_uros";
static const char UROS_DIAGNOSTICS_BRIDGE_TOPIC_OUT[] = "diagnostics";

namespace uros_diagnostic_msg = micro_ros_diagnostic_msgs::msg;
namespace diagnostic_msg = diagnostic_msgs::msg;

struct MicroROSDiagnosticUpdater
{
  std::string name;
  std::string description;
};

struct MicroROSDiagnosticKey
{
  uint16_t updater_id;
  uint16_t key_id;

  bool operator<(const MicroROSDiagnosticKey & rhs) const
  {
    return std::tie(updater_id, key_id) < std::tie(rhs.updater_id, rhs.key_id);
  }
};

struct MicroROSDiagnosticValue
{
  MicroROSDiagnosticKey task;
  uint16_t value_id;

  bool operator<(const MicroROSDiagnosticValue & rhs) const
  {
    return std::tie(task.updater_id, task.key_id, value_id) <
           std::tie(rhs.task.updater_id, rhs.task.key_id, rhs.value_id);
  }
};

typedef std::map<uint16_t, std::string> HardwareMap;
typedef std::map<uint16_t, MicroROSDiagnosticUpdater> UpdaterMap;
typedef std::map<MicroROSDiagnosticKey, std::string> KeyMap;
typedef std::map<MicroROSDiagnosticValue, std::string> ValueMap;

class MicroROSDiagnosticBridge : public rclcpp::Node
{
public:
  explicit MicroROSDiagnosticBridge(const std::string & path = "");

  std::string lookup_hardware(
    uint16_t hardware_id);
  const MicroROSDiagnosticUpdater lookup_updater(
    uint16_t updater_id);
  std::string lookup_key(
    uint16_t updater_id,
    uint16_t key);
  std::string lookup_value(
    uint16_t updater_id,
    uint16_t key,
    uint16_t value_id);

private:
  void read_lookup_table(const std::string & path);

  rclcpp::Logger logger_;
  rclcpp::Subscription<uros_diagnostic_msg::MicroROSDiagnosticStatus>::SharedPtr
    uros_diagnostics_sub_;
  rclcpp::Publisher<diagnostic_msg::DiagnosticArray>::SharedPtr ros2_diagnostics_pub_;

  HardwareMap hardware_map_;
  UpdaterMap updater_map_;
  KeyMap key_map_;
  ValueMap value_map_;
};
