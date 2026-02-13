#include "example_interfaces/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"


class RobotNewsStation : public rclcpp::Node {
public:
  RobotNewsStation() : Node("robot_news_station"), robot_name_("R2D2") {
    news_publisher_ = this->create_publisher<example_interfaces::msg::String>(
        "robot_news", 10);
    timer_ = this->create_wall_timer(
        std::chrono::seconds(2),
        std::bind(&RobotNewsStation::publish_news, this));
    RCLCPP_INFO(this->get_logger(), "Robot News Station '%s' started.",
                robot_name_.c_str());
  }

private:
  void publish_news() {
    auto message = example_interfaces::msg::String();
    message.data = "Breaking News # from " + robot_name_;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    news_publisher_->publish(message);
  }
  std::string robot_name_;
  rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr news_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  // Initialize the ROS 2 client library
  rclcpp::init(argc, argv);

  // Create a node named "robot_news_station"
  auto node = std::make_shared<RobotNewsStation>();

  // Keep the node alive until it is shut down
  rclcpp::spin(node);

  // Shutdown the ROS 2 client library
  rclcpp::shutdown();
  return 0;
}