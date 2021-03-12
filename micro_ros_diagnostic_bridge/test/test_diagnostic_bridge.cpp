// Copyright (c) 2021 - for information on the respective copyright owner
// see the NOTICE file and/or the repository https://github.com/microros/system_modes
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "micro_ros_diagnostic_bridge/micro_ros_diagnostic_bridge.hpp"
#include "micro_ros_diagnostic_bridge/lookup_tables.h"

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <string>


using std::string;


class TestDiagnosticBridge : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }
};

/*
   Testing parsing of lookup table file
 */
TEST_F(TestDiagnosticBridge, parsing) {
  MicroROSDiagnosticBridge * bridge;

  EXPECT_THROW(
    bridge = new MicroROSDiagnosticBridge(),
    std::invalid_argument);

  EXPECT_NO_THROW(bridge = new MicroROSDiagnosticBridge(LOOKUP_TABLE_PATH));

  (void) bridge;
}

/*
   Testing proper translation of IDs into strings
 */
TEST_F(TestDiagnosticBridge, translating) {
  MicroROSDiagnosticBridge * bridge = new MicroROSDiagnosticBridge(LOOKUP_TABLE_PATH);

  // Hardware
  EXPECT_EQ("esp32_01", bridge->lookup_hardware(0));
  EXPECT_EQ("esp32_foo", bridge->lookup_hardware(17));
  EXPECT_EQ("esp32_bar", bridge->lookup_hardware(42));

  EXPECT_NO_THROW(bridge->lookup_hardware(23)) << "should be rclcpp error log";
  EXPECT_EQ("NOTFOUND", bridge->lookup_hardware(23));

  // Updater
  EXPECT_EQ("google.com checker", bridge->lookup_updater(0).name);
  EXPECT_EQ(
    "Periodically checks the website google.com for availability.",
    bridge->lookup_updater(0).description);
  EXPECT_EQ("Processor info", bridge->lookup_updater(17).name);
  EXPECT_EQ(
    "Measuring processor temperature and load.",
    bridge->lookup_updater(17).description);

  EXPECT_NO_THROW(bridge->lookup_updater(23)) << "should be rclcpp error log";
  EXPECT_EQ("NOTFOUND", bridge->lookup_updater(23).name);
  EXPECT_EQ("NOTFOUND", bridge->lookup_updater(23).description);

  // Keys
  EXPECT_EQ("return code", bridge->lookup_key(0, 23));
  EXPECT_EQ("temp", bridge->lookup_key(17, 0));
  EXPECT_EQ("load", bridge->lookup_key(17, 1));

  EXPECT_NO_THROW(bridge->lookup_key(17, 23)) << "should be rclcpp error log";
  EXPECT_EQ("NOTFOUND", bridge->lookup_key(17, 23));
  EXPECT_EQ("NOTFOUND", bridge->lookup_key(0, 0));

  // Values
  EXPECT_EQ("ok", bridge->lookup_value(0, 23, 200));

  EXPECT_NO_THROW(bridge->lookup_value(0, 0, 0)) << "should be rclcpp error log";
  EXPECT_EQ("NOTFOUND", bridge->lookup_value(0, 0, 0));
}
