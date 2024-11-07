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

#include "adog_legged_controller/adog_legged_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{  // utility

// TODO(destogl): remove this when merged upstream
// Changed services history QoS to keep all so we don't lose any client service calls
static constexpr rmw_qos_profile_t rmw_qos_profile_services_hist_keep_all = {
  RMW_QOS_POLICY_HISTORY_KEEP_ALL,
  1,  // message queue depth
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false};

using ControllerReferenceMsg = adog_legged_controller::AdogLeggedController::ControllerReferenceMsg;

// called from RT control loop
void reset_controller_reference_msg(
  std::shared_ptr<ControllerReferenceMsg> & msg, const std::vector<std::string> & joint_names)
{
  // msg->joint_names = joint_names;
  // msg[0]->position=0;
  (void)joint_names;
  for (auto & it : msg->multi_jointmit_array)
  {
    it.effort = 0;
    it.velocity = 0;
    it.position = 0;
  }
  // msg->displacements.resize(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
  // msg->velocities.resize(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
  // msg->duration = std::numeric_limits<double>::quiet_NaN();
}

}  // namespace

namespace adog_legged_controller
{
AdogLeggedController::AdogLeggedController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn AdogLeggedController::on_init()
{
  control_mode_.initRT(control_mode_type::FAST);

  RCLCPP_ERROR(get_node()->get_logger(), "1");

  try
  {
    param_listener_ = std::make_shared<adog_legged_controller::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Exception thrown during controller's init with message: %s \n",
      e.what());
    // return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AdogLeggedController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_ERROR(get_node()->get_logger(), "2");

  params_ = param_listener_->get_params();

  if (!params_.state_joints.empty())
  {
    state_joints_ = params_.state_joints;
  }
  else
  {
    state_joints_ = params_.joints;
  }

  if (params_.joints.size() != state_joints_.size())
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Size of 'joints' (%zu) and 'state_joints' (%zu) parameters has to be the same!",
      params_.joints.size(), state_joints_.size());
    // return CallbackReturn::FAILURE;
  }
  if (params_.interface_name.size() == 1)
  {
    is_simulate = true;
    RCLCPP_WARN(get_node()->get_logger(), "use simulate interface interl pid mode");
    // 加载并创建pid模块
  }
  else
  {
    is_simulate = false;
    RCLCPP_WARN(get_node()->get_logger(), "use hardware interface ");
  }
  // topics QoS
  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  // Reference Subscriber
  ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
    "~/reference", subscribers_qos,
    std::bind(&AdogLeggedController::reference_callback, this, std::placeholders::_1));

  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
  reset_controller_reference_msg(msg, params_.joints);
  input_ref_.writeFromNonRT(msg);

  auto set_slow_mode_service_callback =
    [&](
      const std::shared_ptr<ControllerModeSrvType::Request> request,
      std::shared_ptr<ControllerModeSrvType::Response> response)
  {
    if (request->data)
    {
      control_mode_.writeFromNonRT(control_mode_type::SLOW);
    }
    else
    {
      control_mode_.writeFromNonRT(control_mode_type::FAST);
    }
    response->success = true;
  };

  set_slow_control_mode_service_ = get_node()->create_service<ControllerModeSrvType>(
    "~/set_slow_control_mode", set_slow_mode_service_callback,
    rmw_qos_profile_services_hist_keep_all);

  // try
  // {
  //   // State publisher
  //   s_publisher_ =
  //     get_node()->create_publisher<ControllerStateMsg>("~/state", rclcpp::SystemDefaultsQoS());
  //   state_publisher_ = std::make_unique<ControllerStatePublisher>(s_publisher_);
  // }
  // catch (const std::exception & e)
  // {
  //   fprintf(
  //     stderr, "Exception thrown during publisher creation at configure stage with message : %s
  //     \n", e.what());
  //   return controller_interface::CallbackReturn::ERROR;
  // }

  // TODO(anyone): Reserve memory in state publisher depending on the message type
  // state_publisher_->lock();
  // state_publisher_->msg_.header.frame_id = params_.joints[0];
  // state_publisher_->unlock();

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

void AdogLeggedController::reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg)
{
  // RCLCPP_ERROR(get_node()->get_logger(), "3");
  if (msg->multi_jointmit_array.size() == params_.joints.size())
  {
    input_ref_.writeFromNonRT(msg);
  }
  else
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Received %zu , but expected %zu joints in command. Ignoring message.",
      msg->multi_jointmit_array.size(), params_.joints.size());
  }
}

controller_interface::InterfaceConfiguration AdogLeggedController::command_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.reserve(params_.joints.size());
  if (is_simulate)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "4_1");

    for (const auto & joint : params_.joints)
    {
      // RCLCPP_ERROR(get_node()->get_logger(), "4");

      // command_interfaces_config.names.push_back(joint + "/" + params_.interface_name);
      // command_interfaces_config.names.push_back(joint + "/" + "position");
      // command_interfaces_config.names.push_back(joint + "/" + "velocity");
      command_interfaces_config.names.push_back(joint + "/" + "effort");
      // command_interfaces_config.names.push_back(joint + "/" + "kp");
      // command_interfaces_config.names.push_back(joint + "/" + "kd");
    }
  }
  else
  {
    RCLCPP_ERROR(get_node()->get_logger(), "4");
    for (const auto & joint : params_.joints)
    {
      command_interfaces_config.names.push_back(joint + "/" + "position");
      command_interfaces_config.names.push_back(joint + "/" + "velocity");
      command_interfaces_config.names.push_back(joint + "/" + "effort");
      command_interfaces_config.names.push_back(joint + "/" + "kp");
      command_interfaces_config.names.push_back(joint + "/" + "kd");
    }
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration AdogLeggedController::state_interface_configuration()
  const
{
  RCLCPP_ERROR(get_node()->get_logger(), "5");

  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.reserve(state_joints_.size());
  for (const auto & joint : state_joints_)
  {
    state_interfaces_config.names.push_back(joint + "/" + "position");
    state_interfaces_config.names.push_back(joint + "/" + "velocity");
    state_interfaces_config.names.push_back(joint + "/" + "effort");
    // state_interfaces_config.names.push_back(joint + "/" + params_.interface_name);
  }

  return state_interfaces_config;
}

controller_interface::CallbackReturn AdogLeggedController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): if you have to manage multiple interfaces that need to be sorted check
  // `on_activate` method in `JointTrajectoryController` for exemplary use of
  // `controller_interface::get_ordered_interfaces` helper function
  RCLCPP_ERROR(get_node()->get_logger(), "6");

  // Set default value in command
  reset_controller_reference_msg(*(input_ref_.readFromRT)(), params_.joints);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AdogLeggedController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_ERROR(get_node()->get_logger(), "7");

  // TODO(anyone): depending on number of interfaces, use definitions, e.g., `CMD_MY_ITFS`,
  // instead of a loop
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    command_interfaces_[i].set_value(std::numeric_limits<double>::quiet_NaN());
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type AdogLeggedController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  auto current_ref = input_ref_.readFromRT();

  // TODO(anyone): depending on number of interfaces, use definitions, e.g., `CMD_MY_ITFS`,
  // instead of a loop

  if ((*current_ref)->multi_jointmit_array.size() > 0)
  {
    for (size_t i = 0, j = 0, k = 0;
         i < command_interfaces_.size();)  // 在hardware interface的commend接口上面的长度
    {
      try
      {
        // RCLCPP_ERROR(
        //   get_node()->get_logger(), "control joints %ld,total %ld %ld", j,
        //   (*current_ref)->multi_jointmit_array.size(), command_interfaces_.size());
        auto & jointmit = (*current_ref)->multi_jointmit_array.at(j);
        // jointmit.effort = 1.0;
        // jointmit.velocity = 2.0;
        // jointmit.position = 3.0;
        // command_interfaces_[i++].set_value(std::isnan(jointmit.effort) ? 0 : jointmit.effort);
        // command_interfaces_[i++].set_value((std::isnan(jointmit.velocity) ? 0 :
        // jointmit.velocity)); command_interfaces_[i++].set_value((std::isnan(jointmit.position) ?
        // 0 : jointmit.position)); command_interfaces_[i++].set_value(2.4);//kp
        // command_interfaces_[i++].set_value(2.5);//kd
        // jointmit.effort = std::numeric_limits<double>::quiet_NaN();
        // jointmit.velocity = std::numeric_limits<double>::quiet_NaN();
        // jointmit.position = std::numeric_limits<double>::quiet_NaN();

        // RCLCPP_ERROR_STREAM(get_node()->get_logger(), "type is" <<
        // command_interfaces_[i].get_name() << std::endl);
        if (is_simulate)
        {
          double current_position = state_interfaces_.at(k++).get_value();
          double current_velocity = state_interfaces_.at(k++).get_value();
          double current_effort = state_interfaces_.at(k++).get_value();
          command_interfaces_[i++].set_value(
            (jointmit.position - current_position) * jointmit.kp +
            (jointmit.velocity - current_velocity) * jointmit.kd + jointmit.effort);  // effort

          // command_interfaces_[i++].set_value(0);  // effort
        }
        else
        {
          RCLCPP_ERROR(get_node()->get_logger(), "9");

          command_interfaces_[i++].set_value(jointmit.position);  // position
          command_interfaces_[i++].set_value(jointmit.velocity);  // velocity
          command_interfaces_[i++].set_value(jointmit.effort);    // effort
          command_interfaces_[i++].set_value(jointmit.kp);        // kp
          command_interfaces_[i++].set_value(jointmit.kd);        // kd
        }
        j++;
      }
      catch (...)
      {
        RCLCPP_ERROR(get_node()->get_logger(), "out of range i j %ld %ld", i, j);
        return controller_interface::return_type::ERROR;
      }
    }
  }

  // RCLCPP_ERROR(
  //   get_node()->get_logger(), "multi jointmit array size:%ld",
  //   (*current_ref)->multi_jointmit_array.size());
  // RCLCPP_ERROR(
  //   get_node()->get_logger(), "multi jointmit :%f",
  //   (*current_ref)->multi_jointmit_array.at(0).position);
  // if (state_publisher_ && state_publisher_->trylock())
  // {
  //   state_publisher_->msg_.header.stamp = time;
  //   state_publisher_->msg_.set_point = command_interfaces_[CMD_MY_ITFS].get_value();
  //   state_publisher_->unlockAndPublish();
  // }

  return controller_interface::return_type::OK;
}

}  // namespace adog_legged_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  adog_legged_controller::AdogLeggedController, controller_interface::ControllerInterface)
