#pragma once

#include <memory>
#include <string>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include "motion_generator.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "speed_generator.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

class RobotInterfaceGeneralController : public controller_interface::ControllerInterface {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  bool new_goal_is_received_ = false;
  bool start_execution_ = false;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr goal_subscriber_ = nullptr;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr velocity_goal_subscriber_ = nullptr;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr impedance_goal_subscriber_ =
      nullptr;
  std::string arm_id_;
  const int num_joints = 7;
  Vector7d q_;
  Vector7d q_goal_;
  Vector7d q_vel_;
  Vector7d q_current_goal_;
  Vector7d dq_;
  Vector7d dq_filtered_;
  Vector7d k_gains_;
  Vector7d d_gains_;

  int control_mode_ = 0;  // set default control mode to be position control
  rclcpp::Time start_time_;
  std::unique_ptr<MotionGenerator> motion_generator_;
  std::unique_ptr<SpeedGenerator> speed_generator_;
  std::pair<MotionGenerator::Vector7d, bool> generator_output_;

  void updateJointStates();
};
}  // namespace franka_example_controllers