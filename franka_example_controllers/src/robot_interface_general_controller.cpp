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

#include <franka_example_controllers/robot_interface_general_controller.hpp>

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

const int DYNAMIC_JOINT_POSITION_CONTROL = 0;
const int DYNAMIC_JOINT_VELOCITY_CONTROL = 1;
const int DYNAMIC_JOINT_IMPEDANCE_POSITION_CONTROL = 2;
const int JOINT_POSITION_CONTROL = 3;

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
RobotInterfaceGeneralController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
RobotInterfaceGeneralController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::return_type RobotInterfaceGeneralController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  updateJointStates();

  if (new_goal_is_received_) {
    start_execution_ = true;

    // if (not finished_) {
    //   RCLCPP_INFO(get_node()->get_logger(),
    //               "new goal will be executed after current goal reached.");
    //   // if (control_mode_ = 0 || control_mode_ == 2)
    //   //   q_goal_stack_.push(q_goal_);
    //   // else
    //   //   q_vel_stack_.push(q_vel_);
    // } else {
      RCLCPP_INFO(get_node()->get_logger(), "received new goal! start executing now.");
      // control_mode_ =  motion_mode_stack_.top();
      //
      switch (control_mode_) {
        case DYNAMIC_JOINT_POSITION_CONTROL:
          // q_goal_ = q_goal_stack_.top();
          RCLCPP_INFO(get_node()->get_logger(), "q_goal is: '%f'", q_goal_[0]);
          motion_generator_ = std::make_unique<MotionGenerator>(0.1, q_, q_goal_);
          for (int i = 0; i < num_joints; ++i) {
            d_gains_(i) = get_node()->get_parameter("d_gains").as_double_array().at(i);
            k_gains_(i) = get_node()->get_parameter("k_gains").as_double_array().at(i);
          }
          break;
        case DYNAMIC_JOINT_VELOCITY_CONTROL:
          // q_vel_ = q_vel_stack_.top();
          RCLCPP_INFO(get_node()->get_logger(), "q_vel is: '%f'", q_vel_[0]);
          speed_generator_ = std::make_unique<SpeedGenerator>(0.1, q_, q_vel_);
          for (int i = 0; i < num_joints; ++i) {
            d_gains_(i) = get_node()->get_parameter("d_gains").as_double_array().at(i);
            k_gains_(i) = get_node()->get_parameter("k_gains").as_double_array().at(i);
          }
          break;
        case DYNAMIC_JOINT_IMPEDANCE_POSITION_CONTROL:
          // q_goal_ = q_goal_stack_.top();
          RCLCPP_INFO(get_node()->get_logger(), "q_goal is: '%f'", q_goal_[0]);
          motion_generator_ = std::make_unique<MotionGenerator>(0.1, q_, q_goal_);
          for (int i = 0; i < num_joints; ++i) {
            d_gains_(i) = get_node()->get_parameter("impedance_d_gains").as_double_array().at(i);
            k_gains_(i) = get_node()->get_parameter("impedance_k_gains").as_double_array().at(i);
          }
          break;
        default:
          break;
      // }
      // start_time_ = this->get_node()->now();
      // new_goal_is_received_ = false;
    }
  }

  if (start_execution_) {
    // start executing after the command is received
    auto trajectory_time = this->get_node()->now() - start_time_;
    if (control_mode_ != DYNAMIC_JOINT_VELOCITY_CONTROL) {
      generator_output_ = motion_generator_->getDesiredJointPositions(trajectory_time);
    } else {
      generator_output_ = speed_generator_->getDesiredJointPositions(trajectory_time);
    }
    Vector7d q_desired = generator_output_.first;
    finished_ = generator_output_.second;

    // send computed torques to joints
    if (not finished_) {
      const double kAlpha = 0.99;
      dq_filtered_ = (1 - kAlpha) * dq_filtered_ + kAlpha * dq_;
      Vector7d tau_d_calculated =
          k_gains_.cwiseProduct(q_desired - q_) + d_gains_.cwiseProduct(-dq_filtered_);
      for (int i = 0; i < 7; ++i) {
        command_interfaces_[i].set_value(tau_d_calculated(i));
      }
    } else if (control_mode_ != DYNAMIC_JOINT_VELOCITY_CONTROL) {
      const double kAlpha = 0.99;
      dq_filtered_ = (1 - kAlpha) * dq_filtered_ + kAlpha * dq_;
      Vector7d tau_d_calculated =
          k_gains_.cwiseProduct(q_goal_ - q_) + d_gains_.cwiseProduct(-dq_filtered_);
      for (int i = 0; i < 7; ++i) {
        command_interfaces_[i].set_value(tau_d_calculated(i));
      }
    } else {
      for (auto& command_interface : command_interfaces_) {
        command_interface.set_value(0);
      }
    }
  } else {
    const double kAlpha = 0.99;
    dq_filtered_ = (1 - kAlpha) * dq_filtered_ + kAlpha * dq_;
    Vector7d tau_d_calculated =
        k_gains_.cwiseProduct(q_start_ - q_) + d_gains_.cwiseProduct(-dq_filtered_);
    for (int i = 0; i < 7; ++i) {
      command_interfaces_[i].set_value(tau_d_calculated(i));
    }
  }
  return controller_interface::return_type::OK;
}

CallbackReturn RobotInterfaceGeneralController::on_init() {
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

CallbackReturn RobotInterfaceGeneralController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
  auto d_gains = get_node()->get_parameter("d_gains").as_double_array();
  // check K/d_gains
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

  // Initiate different kinds of control
  goal_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      "/runtime_control/joint_position_goal", rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<sensor_msgs::msg::JointState> msg) -> void {
        auto joint_goal = std::shared_ptr<sensor_msgs::msg::JointState>();
        joint_goal = msg;
        q_goal_ << joint_goal->position[0], joint_goal->position[1], joint_goal->position[2],
            joint_goal->position[3], joint_goal->position[4], joint_goal->position[5],
            joint_goal->position[6];
        new_goal_is_received_ = true;
        control_mode_ = DYNAMIC_JOINT_POSITION_CONTROL;
      });
  velocity_goal_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      "/runtime_control/joint_velocity_goal", rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<sensor_msgs::msg::JointState> msg) -> void {
        auto joint_goal = std::shared_ptr<sensor_msgs::msg::JointState>();
        joint_goal = msg;
        q_vel_ << joint_goal->velocity[0], joint_goal->velocity[1], joint_goal->velocity[2],
            joint_goal->velocity[3], joint_goal->velocity[4], joint_goal->velocity[5],
            joint_goal->velocity[6];
        new_goal_is_received_ = true;
        control_mode_ = DYNAMIC_JOINT_VELOCITY_CONTROL;
      });
  impedance_goal_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      "/runtime_control/joint_impedance_position_goal", rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<sensor_msgs::msg::JointState> msg) -> void {
        auto joint_goal = std::shared_ptr<sensor_msgs::msg::JointState>();
        joint_goal = msg;
        q_goal_ << joint_goal->position[0], joint_goal->position[1], joint_goal->position[2],
            joint_goal->position[3], joint_goal->position[4], joint_goal->position[5],
            joint_goal->position[6];
        new_goal_is_received_ = true;
        control_mode_ = DYNAMIC_JOINT_IMPEDANCE_POSITION_CONTROL;
      });
  dynamic_control_subscriber_ = get_node()->create_subscription<std_msgs::msg::String>(
      "/runtime_control/dynamic_control_parameter", rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<std_msgs::msg::String> msg) -> void {
        auto dynamic_control_param = std::shared_ptr<std_msgs::msg::String>();
        dynamic_control_param = msg;
        RCLCPP_INFO(get_node()->get_logger(), "Publishing: '%s'",
                    dynamic_control_param->data.c_str());
        if (dynamic_control_param->data.c_str() == "false")
          dynamic_control_ = false;
      });

  return CallbackReturn::SUCCESS;
}

CallbackReturn RobotInterfaceGeneralController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  updateJointStates();
  q_start_ = q_;
  return CallbackReturn::SUCCESS;
}

void RobotInterfaceGeneralController::updateJointStates() {
  for (auto i = 0; i < num_joints; ++i) {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");
    q_(i) = position_interface.get_value();
    dq_(i) = velocity_interface.get_value();
  }
}
}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::RobotInterfaceGeneralController,
                       controller_interface::ControllerInterface)