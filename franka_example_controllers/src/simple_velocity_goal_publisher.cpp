#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class VelocityGoalPublisher : public rclcpp::Node {
 public:
  VelocityGoalPublisher() : Node("Velocity_Goal_publisher"), count_(0) {
    declare_parameter("goal_velocity", rclcpp::PARAMETER_DOUBLE_ARRAY);
    // read the parameters
    get_parameter("goal_velocity", goal_velocity_);
    publisher_ =
        this->create_publisher<sensor_msgs::msg::JointState>("/runtime_control/goal_velocity", 1);
    timer_ =
        this->create_wall_timer(500ms, std::bind(&VelocityGoalPublisher::timer_callback, this));
  }

 private:
  void timer_callback() {
    auto message = std::make_shared<sensor_msgs::msg::JointState>();
    message->velocity.resize(7);
    for (int i = 0; i < 7; i++) {
      message->velocity[i] = goal_velocity_[i];
      RCLCPP_INFO(this->get_logger(), "Publishing the goal, jnt'%d''s velocity goal is : '%f'",
                  i + 1, message->velocity[i]);
    }
    publisher_->publish(*message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<double> goal_velocity_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityGoalPublisher>());
  rclcpp::shutdown();
  return 0;
}