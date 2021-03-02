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

#include <utility>
#include <tuple>
#include <string>
#include <map>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "micro_ros_diagnostic_msgs/msg/micro_ros_diagnostic_status.hpp"


static const char UROS_DIAGNOSTICS_BRIDGE_TOPIC_IN[] = "/diagnostics_uros";
static const char UROS_DIAGNOSTICS_BRIDGE_TOPIC_OUT[] = "/diagnostics";

namespace uros_diagnostic_msg = micro_ros_diagnostic_msgs::msg;
namespace diagnostic_msg = diagnostic_msgs::msg;

typedef std::pair<unsigned int, unsigned int> DiagnosticKey;
typedef std::tuple<unsigned int, unsigned int, unsigned int> ValueLookup;

typedef std::map<unsigned int, std::string> HardwareMap;
typedef std::map<unsigned int, std::pair<std::string, std::string>> UpdaterMap;
typedef std::map<DiagnosticKey, std::string> KeyMap;
typedef std::map<ValueLookup, std::string> ValueMap;

class MicroROSDiagnosticBridge : public rclcpp::Node
{
public:
  explicit MicroROSDiagnosticBridge(const std::string & path = "");

  virtual const std::string lookup_hardware(
    unsigned int hardware_id);
  virtual const std::pair<std::string, std::string> lookup_updater(
    unsigned int updater_id);
  virtual const std::string lookup_key(
    unsigned int updater_id,
    unsigned int key);
  virtual const std::string lookup_value(
    unsigned int updater_id,
    unsigned int key,
    unsigned int value_id);

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
