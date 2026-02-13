#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"


class SmartphoneNode : public rclcpp::Node
{
public:
    SmartphoneNode() : Node("smartphone")
    {
        news_subscription_ = this->create_subscription<example_interfaces::msg::String>(
            "robot_news",
            10,
            std::bind(&SmartphoneNode::callbackRobotNews, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Smartphone node started and subscribed to 'robot_news' topic.");
    }

private:
    void callbackRobotNews(const example_interfaces::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Smartphone received news: '%s'", msg->data.c_str());
    }

    rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr news_subscription_;
};

int main(int argc, char **argv)
{
    // Initialize the ROS 2 client library
    rclcpp::init(argc, argv);

    // Create a node named "robot_news_station"
    auto node = std::make_shared<SmartphoneNode>();

    // Keep the node alive until it is shut down
    rclcpp::spin(node);

    // Shutdown the ROS 2 client library
    rclcpp::shutdown();
    return 0;
}