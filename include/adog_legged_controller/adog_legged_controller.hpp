// Copyright (c) 2024, Dyyt587
// Copyright (c) 2024, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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

//
// Source of this file are templates in
// [RosTeamWorkspace](https://github.com/StoglRobotics/ros_team_workspace) repository.
//

#ifndef ADOG_LEGGED_CONTROLLER__ADOG_LEGGED_CONTROLLER_HPP_
#define ADOG_LEGGED_CONTROLLER__ADOG_LEGGED_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "adog_legged_controller/visibility_control.h"
#include "adog_legged_controller_parameters.hpp"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_srvs/srv/set_bool.hpp"

// TODO(anyone): Replace with controller specific messages
#include "adog_legged_interfaces/msg/joint_mit.hpp"
#include "adog_legged_interfaces/msg/multi_joint_mit.hpp"
#include "control_msgs/msg/joint_controller_state.hpp"
#include "control_msgs/msg/joint_jog.hpp"

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include "rcl_interfaces/msg/integer_range.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

namespace adog_legged_controller
{
// name constants for state interfaces
static constexpr size_t STATE_MY_ITFS = 0;

// name constants for command interfaces
static constexpr size_t CMD_MY_ITFS = 0;

// TODO(anyone: example setup for control mode (usually you will use some enums defined in messages)
enum class control_mode_type : std::uint8_t
{
  FAST = 0,
  SLOW = 1,
};

class AdogLeggedController : public controller_interface::ControllerInterface
{
public:
  ADOG_LEGGED_CONTROLLER__VISIBILITY_PUBLIC
  AdogLeggedController();

  ADOG_LEGGED_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  ADOG_LEGGED_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  ADOG_LEGGED_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  ADOG_LEGGED_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  ADOG_LEGGED_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  ADOG_LEGGED_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  ADOG_LEGGED_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // TODO(anyone): replace the state and command message types
  using ControllerReferenceMsg = adog_legged_interfaces::msg::MultiJointMit;
  // using ControllerReferenceMsg = control_msgs::msg::JointJog;
  using ControllerModeSrvType = std_srvs::srv::SetBool;
  // using ControllerStateMsg = control_msgs::msg::JointControllerState;

protected:
  std::shared_ptr<adog_legged_controller::ParamListener> param_listener_;
  adog_legged_controller::Params params_;

  std::vector<std::string> state_joints_;

  // Command subscribers and Controller State publisher
  rclcpp::Subscription<ControllerReferenceMsg>::SharedPtr ref_subscriber_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerReferenceMsg>> input_ref_;

  rclcpp::Service<ControllerModeSrvType>::SharedPtr set_slow_control_mode_service_;
  realtime_tools::RealtimeBuffer<control_mode_type> control_mode_;

  // using ControllerStatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;

  // rclcpp::Publisher<ControllerStateMsg>::SharedPtr s_publisher_;
  // std::unique_ptr<ControllerStatePublisher> state_publisher_;

private:
  bool is_simulate = true;
  // callback for topic interface
  ADOG_LEGGED_CONTROLLER__VISIBILITY_LOCAL
  void reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg);
};

}  // namespace adog_legged_controller

#endif  // ADOG_LEGGED_CONTROLLER__ADOG_LEGGED_CONTROLLER_HPP_
