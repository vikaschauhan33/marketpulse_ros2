#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node
{
public:
    MyNode() : Node("my_first_node"), count_(0)
    {
        RCLCPP_INFO(this->get_logger(), "Constructor Hello, ROS 2 from my_first_node!");
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MyNode::timer_callback, this));
    }
private:
    void timer_callback()
    {
        RCLCPP_INFO(this->get_logger(), "Timer callback triggered %d", count_++);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    int count_ = 0;
};
int main(int argc, char **argv)
{
    // Initialize the ROS 2 client library
    rclcpp::init(argc, argv);

    // Create a node named "my_first_node"
    auto node = std::make_shared<MyNode>();

    // Log an informational message
    RCLCPP_INFO(node->get_logger(), "Hello, ROS 33 from my_first_node!");

    // Keep the node alive until it is shut down
    rclcpp::spin(node);

    // Shutdown the ROS 2 client library
    rclcpp::shutdown();
    return 0;
}