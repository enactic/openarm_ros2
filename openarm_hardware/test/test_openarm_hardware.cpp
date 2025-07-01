// Copyright 2025 Reazon Holdings, Inc.
// Copyright 2025 Stogl Robotics Consulting UG (haftungsbeschränkt)
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

#include <gmock/gmock.h>

#include <string>

#include "hardware_interface/resource_manager.hpp"
#include "ros2_control_test_assets/components_urdfs.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

class TestOpenArmHW : public ::testing::Test {
 protected:
  void SetUp() override {
    openarm_hardware_7dof_ =
        R"(
        <ros2_control name="OpenArmHW7DOF" type="system">
          <hardware>
            <!-- By default, set up controllers for simulation. This won't work on real hardware -->
            <plugin>mock_components/GenericSystem</plugin>
            <plugin>openarm_hardware/OpenArmHW</plugin>
          </hardware>
          <joint name="rev1">
            <command_interface name="position"/>
            <command_interface name="velocity"/>
            <command_interface name="effort"/>
            <state_interface name="position">
              <param name="initial_value">0</param>
            </state_interface>
            <state_interface name="velocity"/>
            <state_interface name="effort"/>
          </joint>
          <joint name="rev2">
            <command_interface name="position"/>
            <command_interface name="velocity"/>
            <command_interface name="effort"/>
            <state_interface name="position">
              <param name="initial_value">0</param>
            </state_interface>
            <state_interface name="velocity"/>
            <state_interface name="effort"/>
          </joint>
          <joint name="rev3">
            <command_interface name="position"/>
            <command_interface name="velocity"/>
            <command_interface name="effort"/>
            <state_interface name="position">
              <param name="initial_value">0</param>
            </state_interface>
            <state_interface name="velocity"/>
            <state_interface name="effort"/>
          </joint>
          <joint name="rev4">
            <command_interface name="position"/>
            <command_interface name="velocity"/>
            <command_interface name="effort"/>
            <state_interface name="position">
              <param name="initial_value">0</param>
            </state_interface>
            <state_interface name="velocity"/>
            <state_interface name="effort"/>
          </joint>
          <joint name="rev5">
            <command_interface name="position"/>
            <command_interface name="velocity"/>
            <command_interface name="effort"/>
            <state_interface name="position">
              <param name="initial_value">0</param>
            </state_interface>
            <state_interface name="velocity"/>
            <state_interface name="effort"/>
          </joint>
          <joint name="rev6">
            <command_interface name="position"/>
            <command_interface name="velocity"/>
            <command_interface name="effort"/>
            <state_interface name="position">
              <param name="initial_value">0</param>
            </state_interface>
            <state_interface name="velocity"/>
            <state_interface name="effort"/>
          </joint>
          <joint name="rev7">
            <command_interface name="position"/>
            <command_interface name="velocity"/>
            <command_interface name="effort"/>
            <state_interface name="position">
              <param name="initial_value">0</param>
            </state_interface>
            <state_interface name="velocity"/>
            <state_interface name="effort"/>
          </joint>
          <joint name="left_pris1">
            <command_interface name="position"/>
            <state_interface name="position">
              <param name="initial_value">0</param>
            </state_interface>
            <state_interface name="velocity"/>
          </joint>
          <joint name="right_pris2">
            <state_interface name="position">
              <param name="initial_value">0</param>
            </state_interface>
          </joint>
        </ros2_control>
    )";
  }

  std::string openarm_hardware_7dof_;
};

TEST_F(TestOpenArmHW, load_openarm_hardware_7dof) {
  auto urdf = ros2_control_test_assets::urdf_head + openarm_hardware_7dof_ +
              ros2_control_test_assets::urdf_tail;
  ASSERT_NO_THROW(hardware_interface::ResourceManager rm(urdf));
}
