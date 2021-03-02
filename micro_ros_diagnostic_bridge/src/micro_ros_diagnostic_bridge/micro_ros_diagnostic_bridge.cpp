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
#include "micro_ros_diagnostic_bridge/micro_ros_diagnostic_bridge.hpp"

#include <string>
#include <memory>
#include <utility>

#include <rcl/error_handling.h>

#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_map.hpp>
#include <rcl_yaml_param_parser/parser.h>

using micro_ros_diagnostic_msgs::msg::MicroROSDiagnosticStatus;
using diagnostic_msgs::msg::DiagnosticStatus;

MicroROSDiagnosticBridge::MicroROSDiagnosticBridge(const std::string & path)
: Node("micro_ros_diagnostic_bridge"),
  logger_(rclcpp::get_logger("MicroROSDiagnosticBridge"))
{
  // Read lookup table
  declare_parameter("lookup_table", rclcpp::ParameterValue(path));
  std::string lookup_table_path = get_parameter("lookup_table").as_string();
  if (lookup_table_path.empty()) {
    throw std::invalid_argument("Need path to lookup table.");
  }
  read_lookup_table(lookup_table_path);

  auto callback =
    [this](const MicroROSDiagnosticStatus::SharedPtr msg_in) -> void
    {
      RCLCPP_DEBUG(
        get_logger(),
        "Bridging message from hardware %d, updater %d",
        msg_in->hardware_id, msg_in->updater_id);
      msg_out_ = std::make_unique<diagnostic_msgs::msg::DiagnosticStatus>();

      auto updater = lookup_updater(msg_in->updater_id);
      auto hardware = lookup_hardware(msg_in->hardware_id);
      msg_out_->hardware_id = hardware;
      msg_out_->name = updater.first;
      msg_out_->message = updater.second;
      msg_out_->level = msg_in->level;

      diagnostic_msgs::msg::KeyValue keyvalue;
      keyvalue.key = lookup_key(msg_in->updater_id, msg_in->key);
      switch (msg_in->value_type) {
        case micro_ros_diagnostic_msgs::msg::MicroROSDiagnosticStatus::VALUE_BOOL:
          keyvalue.value = std::to_string(msg_in->bool_value);
          break;
        case micro_ros_diagnostic_msgs::msg::MicroROSDiagnosticStatus::VALUE_INT:
          keyvalue.value = std::to_string(msg_in->int_value);
          break;
        case micro_ros_diagnostic_msgs::msg::MicroROSDiagnosticStatus::VALUE_DOUBLE:
          keyvalue.value = std::to_string(msg_in->double_value);
          break;
        case micro_ros_diagnostic_msgs::msg::MicroROSDiagnosticStatus::VALUE_LOOKUP:
          keyvalue.value = lookup_value(msg_in->updater_id, msg_in->key, msg_in->value_id);
          break;
      }
      msg_out_->values.push_back(keyvalue);

      ros2_pub_->publish(std::move(msg_out_));
    };

  uros_sub_ = create_subscription<MicroROSDiagnosticStatus>(
    UROS_DIAGNOSTICS_BRIDGE_TOPIC_IN,
    rclcpp::SystemDefaultsQoS(),
    callback);
  ros2_pub_ = create_publisher<DiagnosticStatus>(
    UROS_DIAGNOSTICS_BRIDGE_TOPIC_OUT,
    rclcpp::SystemDefaultsQoS());
}

const std::string
MicroROSDiagnosticBridge::lookup_key(
  unsigned int updater_id,
  unsigned int key)
{
  try {
    return key_map_.at(std::make_pair(updater_id, key));
  } catch (std::out_of_range & e) {
    RCLCPP_ERROR(
      get_logger(),
      "Updater %d and key %d, not found in lookup table.",
      updater_id, key);
    return "NOTFOUND";
  }
}

const std::string
MicroROSDiagnosticBridge::lookup_value(
  unsigned int updater_id,
  unsigned int key,
  unsigned int value_id)
{
  try {
    return value_map_.at(std::make_tuple(updater_id, key, value_id));
  } catch (std::out_of_range & e) {
    RCLCPP_ERROR(
      get_logger(),
      "Updater %d, key %d, and value id %d, not found in lookup table.",
      updater_id, key, value_id);
    return "NOTFOUND";
  }
}

const std::string
MicroROSDiagnosticBridge::lookup_hardware(unsigned int hardware_id)
{
  try {
    return hardware_map_.at(hardware_id);
  } catch (std::out_of_range & e) {
    RCLCPP_ERROR(
      get_logger(),
      "Hardware_id %d, not found in lookup table.",
      hardware_id);
    return "NOTFOUND";
  }
}

const std::pair<std::string, std::string>
MicroROSDiagnosticBridge::lookup_updater(unsigned int updater_id)
{
  try {
    return updater_map_.at(updater_id);
  } catch (std::out_of_range & e) {
    RCLCPP_ERROR(
      get_logger(),
      "Updater_id %d, not found in lookup table.",
      updater_id);
    return std::make_pair("NOTFOUND", "NOTFOUND");
  }
}

void
MicroROSDiagnosticBridge::read_lookup_table(const std::string & path)
{
  rcl_params_t * yaml_params = rcl_yaml_node_struct_init(rcl_get_default_allocator());
  if (!rcl_parse_yaml_file(path.c_str(), yaml_params)) {
    throw std::runtime_error(
            "Failed to parse parameters " + path + ". " + rcl_get_error_string().str);
  }

  rclcpp::ParameterMap param_map = rclcpp::parameter_map_from(yaml_params);
  rcl_yaml_node_struct_fini(yaml_params);

  rclcpp::ParameterMap::iterator it;
  for (it = param_map.begin(); it != param_map.end(); it++) {
    if (it->first.compare("/hardware_ids") == 0) {
      for (auto & p : it->second) {
        hardware_map_[std::stoi(p.get_name())] = p.value_to_string();
      }
    }

    if (it->first.compare("/updaters") == 0) {
      std::string updater_key, updater_name, updater_descr, key, key_name;
      for (auto & p : it->second) {
        auto pos = p.get_name().find('.');
        if (pos != std::string::npos) {
          updater_key = p.get_name().substr(0, pos);
        }

        // Updater
        if (p.get_name().compare(updater_key + ".name") == 0) {
          updater_name = p.value_to_string();
          updater_map_[std::stoi(updater_key)] = std::make_pair(updater_name, updater_descr);
        }
        if (p.get_name().compare(updater_key + ".description") == 0) {
          updater_descr = p.value_to_string();
          updater_map_[std::stoi(updater_key)] = std::make_pair(updater_name, updater_descr);
        }

        // Keys
        if (p.get_name().rfind(updater_key + ".keys", 0) == 0) {
          auto start = updater_key.length() + 6;
          pos = p.get_name().find('.', start);
          key = p.get_name().substr(start, pos - start);
        }
        if (p.get_name().compare(updater_key + ".keys." + key + ".name") == 0) {
          key_name = p.value_to_string();
          key_map_[std::make_pair(std::stoi(updater_key), std::stoi(key))] = key_name;
        }

        // Values lookup
        if (p.get_name().rfind(updater_key + ".keys." + key + ".values") == 0) {
          auto start = updater_key.length() + key.length() + 14;
          pos = p.get_name().find('.', start);
          auto value = std::stoi(p.get_name().substr(start, pos - start));
          value_map_[std::make_tuple(std::stoi(updater_key), std::stoi(key), value)] =
            p.value_to_string();
        }
      }
    }
  }

  // Debug
  RCLCPP_INFO(get_logger(), "Lookup table:");
  RCLCPP_INFO(get_logger(), " Found %lu hardware keys.", hardware_map_.size());
  RCLCPP_INFO(get_logger(), " Found %lu updaters.", updater_map_.size());
  for (auto const & k : updater_map_) {
    RCLCPP_DEBUG(
      get_logger(), "  Updater %d : %s(%s)",
      k.first, k.second.first.c_str(), k.second.second.c_str());
  }
  RCLCPP_INFO(
    get_logger(), " Found %lu diagnostic keys with %lu diagnostic values.",
    key_map_.size(), value_map_.size());
  for (auto const & k : value_map_) {
    RCLCPP_DEBUG(
      get_logger(), "  Value %d,%d,%d : %s",
      std::get<0>(k.first), std::get<1>(k.first), std::get<2>(k.first),
      k.second.c_str());
  }
}
