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

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include <rcl/error_handling.h>

#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_map.hpp>
#include <rcl_yaml_param_parser/parser.h>

using micro_ros_diagnostic_msgs::msg::MicroROSDiagnosticStatus;
using diagnostic_msgs::msg::DiagnosticArray;

static inline std::string VALUE_NOT_FOUND = "NOTFOUND";

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

  rclcpp::QoS qos{rclcpp::KeepLast{10}};
  qos.reliable();

  ros2_diagnostics_pub_ = create_publisher<DiagnosticArray>(
    UROS_DIAGNOSTICS_BRIDGE_TOPIC_OUT,
    qos);

  auto callback =
    [this](const MicroROSDiagnosticStatus::SharedPtr msg_in) -> void
    {
      RCLCPP_DEBUG(
        get_logger(),
        "Bridging message from hardware %d, updater %d", msg_in->hardware_id,
        msg_in->updater_id);
      auto msg_out = std::make_unique<DiagnosticArray>();
      diagnostic_msgs::msg::DiagnosticStatus status_msg{};

      auto updater = lookup_updater(msg_in->updater_id);
      auto hardware = lookup_hardware(msg_in->hardware_id);
      status_msg.hardware_id = hardware;
      status_msg.name = updater.name;
      status_msg.message = updater.description;
      status_msg.level = msg_in->level;
      RCLCPP_DEBUG(get_logger(), "Updater %s HW %s", updater.name.c_str(), hardware.c_str());
      for (size_t value_index = 0; value_index < msg_in->number_of_values; value_index++) {
        diagnostic_msgs::msg::KeyValue keyvalue;
        RCLCPP_DEBUG(
          get_logger(), "Bridging updater %d, key %d", msg_in->updater_id,
          msg_in->values[value_index].key);
        keyvalue.key = lookup_key(msg_in->updater_id, msg_in->values[value_index].key);
        switch (msg_in->values[value_index].value_type) {
          case MicroROSDiagnosticStatus::VALUE_BOOL:
            keyvalue.value = std::to_string(msg_in->values[value_index].bool_value);
            break;
          case MicroROSDiagnosticStatus::VALUE_INT:
            keyvalue.value = std::to_string(msg_in->values[value_index].int_value);
            break;
          case MicroROSDiagnosticStatus::VALUE_DOUBLE:
            keyvalue.value = std::to_string(msg_in->values[value_index].double_value);
            break;
          case MicroROSDiagnosticStatus::VALUE_LOOKUP:
            keyvalue.value = lookup_value(
              msg_in->updater_id, msg_in->values[value_index].key,
              msg_in->values[value_index].value_id);
            break;
        }
        status_msg.values.push_back(keyvalue);
        RCLCPP_DEBUG(get_logger(), "key %s value %s", keyvalue.key.c_str(), keyvalue.value.c_str());
      }
      msg_out->status.push_back(status_msg);
      ros2_diagnostics_pub_->publish(std::move(msg_out));
    };

  uros_diagnostics_sub_ = create_subscription<MicroROSDiagnosticStatus>(
    UROS_DIAGNOSTICS_BRIDGE_TOPIC_IN,
    qos,
    callback);
}

std::string
MicroROSDiagnosticBridge::lookup_key(
  int updater_id,
  int key)
{
  try {
    return key_map_.at({updater_id, key});
  } catch (std::out_of_range & e) {
    RCLCPP_ERROR(
      get_logger(),
      "Updater %d and key %d, not found in lookup table.",
      updater_id, key);
    return VALUE_NOT_FOUND;
  }
}

std::string
MicroROSDiagnosticBridge::lookup_value(
  int updater_id,
  int key,
  int value_id)
{
  try {
    return value_map_.at({{updater_id, key}, value_id});
  } catch (std::out_of_range & e) {
    RCLCPP_ERROR(
      get_logger(),
      "Updater %d, key %d, and value id %d, not found in lookup table.",
      updater_id, key, value_id);
    return VALUE_NOT_FOUND;
  }
}

std::string
MicroROSDiagnosticBridge::lookup_hardware(int hardware_id)
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

const MicroROSDiagnosticUpdater
MicroROSDiagnosticBridge::lookup_updater(int updater_id)
{
  try {
    return updater_map_.at(updater_id);
  } catch (std::out_of_range & e) {
    RCLCPP_ERROR(
      get_logger(),
      "Updater_id %d, not found in lookup table.",
      updater_id);
    return {"NOTFOUND", "NOTFOUND"};
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
        try {
          hardware_map_[std::stoi(p.get_name())] = p.value_to_string();
          RCLCPP_DEBUG(
            get_logger(), "FOUND Parameter: %s HW_ID %s",
            p.get_name().c_str(), p.value_to_string().c_str());
        } catch (const std::invalid_argument &) {
          throw std::runtime_error("Failed to parse hardware_id from lookup_table.");
        }
      }
    }

    if (it->first.compare("/updaters") == 0) {
      std::string updater_key, updater_name, updater_descr, key, key_name;
      for (auto & p : it->second) {
        auto pos = p.get_name().find('.');
        if (pos != std::string::npos) {
          updater_key = p.get_name().substr(0, pos);
          RCLCPP_DEBUG(get_logger(), "Updater key: %s", updater_key.c_str());
        } else {
          throw std::runtime_error("Failed to load updater key.");
        }

        // Updater
        if (p.get_name().compare(updater_key + ".name") == 0) {
          updater_name = p.value_to_string();
          RCLCPP_DEBUG(
            get_logger(), "Updater Name: %s, Description: %s",
            updater_name.c_str(), updater_descr.c_str());
          updater_map_[std::stoi(updater_key)] = {updater_name, updater_descr};
        }
        if (p.get_name().compare(updater_key + ".description") == 0) {
          updater_descr = p.value_to_string();
          updater_map_[std::stoi(updater_key)] = {updater_name, updater_descr};
          RCLCPP_DEBUG(
            get_logger(), "Updater Name: %s, Description: %s",
            updater_name.c_str(), updater_descr.c_str());
        }

        // Keys
        if (p.get_name().rfind(updater_key + ".keys", 0) == 0) {
          auto start = updater_key.length() + 6;
          pos = p.get_name().find('.', start);
          key = p.get_name().substr(start, pos - start);
          RCLCPP_DEBUG(get_logger(), "Updater key position: %ld, key: %s", pos, key.c_str());
        }
        if (p.get_name().compare(updater_key + ".keys." + key + ".name") == 0) {
          key_name = p.value_to_string();
          RCLCPP_DEBUG(get_logger(), "Key name: %s", key_name.c_str());
          key_map_[{std::stoi(updater_key), std::stoi(key)}] = key_name;
        }

        // Values lookup
        if (p.get_name().rfind(updater_key + ".keys." + key + ".values") == 0) {
          auto start = updater_key.length() + key.length() + 14;
          pos = p.get_name().find('.', start);
          auto value_id = std::stoi(p.get_name().substr(start, pos - start));
          value_map_[{{std::stoi(updater_key), std::stoi(key)}, value_id}] = p.value_to_string();
          RCLCPP_DEBUG(get_logger(), "Value ID %d Value %s", value_id, p.value_to_string().c_str());
        }
      }
    }
  }

  // Debug
  RCLCPP_INFO(get_logger(), "Lookup table:");
  RCLCPP_INFO(get_logger(), " Found %lu hardware keys.", hardware_map_.size());
  RCLCPP_INFO(get_logger(), " Found %lu updaters.", updater_map_.size());
  for (auto const & updater : updater_map_) {
    RCLCPP_DEBUG(
      get_logger(), "  Updater %d : %s(%s)",
      updater.first, updater.second.name.c_str(), updater.second.description.c_str());
  }
  RCLCPP_INFO(
    get_logger(), " Found %lu diagnostic keys with %lu diagnostic values.",
    key_map_.size(), value_map_.size());
  for (auto const & value : value_map_) {
    RCLCPP_DEBUG(
      get_logger(), "  Value %d,%d,%d : %s",
      value.first.task.updater_id,
      value.first.task.key_id,
      value.first.value_id,
      value.second.c_str());
  }
}
