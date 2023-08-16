// Copyright (c) 2021 Franka Emika GmbH
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

#include <franka_example_controllers/runtime_position_controller.hpp>

#include <Eigen/Eigen>
#include <cassert>
#include <cmath>
#include <controller_interface/controller_interface.hpp>
#include <exception>
#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
RuntimePositionController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
RuntimePositionController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::return_type RuntimePositionController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  updateJointStates();

  if (new_goal_is_received_) {
    RCLCPP_INFO(get_node()->get_logger(), "received new goal! start executing now.");
    RCLCPP_INFO(get_node()->get_logger(), "q_goal is: '%f'", q_goal_[0]);
    RCLCPP_INFO(get_node()->get_logger(), "q_ is: '%f'", q_[0]);

    motion_generator_ = std::make_unique<MotionGenerator>(0.1, q_, q_goal_);

    start_time_ = this->get_node()->now();
    new_goal_is_received_ = false;  // follow the current goal until a different goal is received
  }
  q_current_goal_ = q_goal_;
  auto trajectory_time = this->get_node()->now() - start_time_;
  RCLCPP_INFO(get_node()->get_logger(), "trajectory_time_ is: %f", trajectory_time.seconds());

  auto motion_generator_output = motion_generator_->getDesiredJointPositions(trajectory_time);
  Vector7d q_desired = motion_generator_output.first;
  RCLCPP_INFO(get_node()->get_logger(), "current desired position of joint '%d' is: '%f'", 1,
              q_desired[0]);

  bool finished = motion_generator_output.second;
  if (not finished) {
    const double kAlpha = 0.99;
    dq_filtered_ = (1 - kAlpha) * dq_filtered_ + kAlpha * dq_;
    Vector7d tau_d_calculated =
        k_gains_.cwiseProduct(q_desired - q_) + d_gains_.cwiseProduct(-dq_filtered_);
    for (int i = 0; i < 7; ++i) {
      command_interfaces_[i].set_value(tau_d_calculated(i));
    }
  } else {
    // for (auto& command_interface : command_interfaces_) {
    //   command_interface.set_value(0);
    // }
    const double kAlpha = 0.99;
    dq_filtered_ = (1 - kAlpha) * dq_filtered_ + kAlpha * dq_;
    Vector7d tau_d_calculated =
        k_gains_.cwiseProduct(q_goal_ - q_) + d_gains_.cwiseProduct(-dq_filtered_);
    for (int i = 0; i < 7; ++i) {
      command_interfaces_[i].set_value(tau_d_calculated(i));
    }
  }
  return controller_interface::return_type::OK;
}

CallbackReturn RuntimePositionController::on_init() {
  q_goal_ << 0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4;
  try {
    auto_declare<std::string>("arm_id", "panda");
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn RuntimePositionController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
  auto d_gains = get_node()->get_parameter("d_gains").as_double_array();
  if (k_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains parameter not set");
    return CallbackReturn::FAILURE;
  }
  if (k_gains.size() != static_cast<uint>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains should be of size %d but is of size %ld",
                 num_joints, k_gains.size());
    return CallbackReturn::FAILURE;
  }
  if (d_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains parameter not set");
    return CallbackReturn::FAILURE;
  }
  if (d_gains.size() != static_cast<uint>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains should be of size %d but is of size %ld",
                 num_joints, d_gains.size());
    return CallbackReturn::FAILURE;
  }
  for (int i = 0; i < num_joints; ++i) {
    d_gains_(i) = d_gains.at(i);
    k_gains_(i) = k_gains.at(i);
  }
  dq_filtered_.setZero();
  goal_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      "/runtime_control/position_goal", rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<sensor_msgs::msg::JointState> msg) -> void {
        // if (!subscriber_is_active_) {
        // RCLCPP_WARN(get_node()->get_logger(),
        //             "Can't accept new commands. subscriber is inactive");
        // return;
        // }
        auto joint_goal = std::shared_ptr<sensor_msgs::msg::JointState>();
        RCLCPP_INFO(get_node()->get_logger(), "goal received ");
        joint_goal = msg;
        RCLCPP_INFO(get_node()->get_logger(), "Received joint1 position: %f", msg->position[0]);
        q_goal_ << joint_goal->position[0], joint_goal->position[1], joint_goal->position[2],
            joint_goal->position[3], joint_goal->position[4], joint_goal->position[5],
            joint_goal->position[6];
        // RCLCPP_INFO(get_node()->get_logger(), "Received joint1 position: %f",
        // joint_goal->position[0]);
        if (q_goal_ != q_current_goal_) {
          new_goal_is_received_ = true;
          RCLCPP_INFO(get_node()->get_logger(), "goal changed from the previous one");
        }
      });

  return CallbackReturn::SUCCESS;
}

CallbackReturn RuntimePositionController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  updateJointStates();
  motion_generator_ = std::make_unique<MotionGenerator>(0.1, q_, q_goal_);
  start_time_ = this->get_node()->now();
  return CallbackReturn::SUCCESS;
}

void RuntimePositionController::updateJointStates() {
  for (auto i = 0; i < num_joints; ++i) {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");
    // RCLCPP_INFO(get_node()->get_logger(), "Current position of joint %d is %f", i,
    //             position_interface.get_value());
    q_(i) = position_interface.get_value();
    dq_(i) = velocity_interface.get_value();
  }
}
}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::RuntimePositionController,
                       controller_interface::ControllerInterface)