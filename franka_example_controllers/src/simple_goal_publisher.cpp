#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    publisher_ =
        this->create_publisher<sensor_msgs::msg::JointState>("/runtime_control/position_goal", 1);
    timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

 private:
  void timer_callback() {
    auto message = std::make_shared<sensor_msgs::msg::JointState>();
    message->position.resize(7);
    message->position[0] = 0.0;
    message->position[1] = -3.14 / 4;
    message->position[2] = 0.0;
    message->position[3] = -3 * 3.14 / 4;
    message->position[4] = 0.0;
    message->position[5] = 3.14 / 2;
    message->position[6] = 3.14 / 4;
    RCLCPP_INFO(this->get_logger(), "Publishing the goal, jnt1's position is : '%f'",
                message->position[0]);
    RCLCPP_INFO(this->get_logger(), "Publishing the goal, jnt2's position is : '%f'",
                message->position[1]);
    RCLCPP_INFO(this->get_logger(), "Publishing the goal, jnt3's position is : '%f'",
                message->position[2]);
    RCLCPP_INFO(this->get_logger(), "Publishing the goal, jnt7's position is : '%f'",
                message->position[6]);
    publisher_->publish(*message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}