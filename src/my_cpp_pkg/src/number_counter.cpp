#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "example_interfaces/srv/set_bool.hpp"

#include <chrono>

using namespace std::chrono_literals;

/**
 * @class NumberCounterNode
 * @brief Subscribes to the "number" topic, aggregates the values,
 *        and publishes the total sum on the "count" topic every 5 seconds.
 */
class NumberCounterNode : public rclcpp::Node
{
public:
    NumberCounterNode() : Node("number_counter_node"), counter_(0)
    {
        // Subscriber to the "number" topic
        subscriber_ = this->create_subscription<example_interfaces::msg::Int64>(
            "number", 10, std::bind(&NumberCounterNode::callback_number, this, std::placeholders::_1));

        // Publisher to the "count" topic
        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("count", 10);

        // Timer to publish the current count every 5 seconds
        timer_ = this->create_wall_timer(
            5s, std::bind(&NumberCounterNode::publish_count, this));

        // Service server to reset the counter
        reset_service_ = this->create_service<example_interfaces::srv::SetBool>(
            "/reset_counter",
            std::bind(&NumberCounterNode::reset_counter_callback, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Number Counter Node has been started.");
    }

private:
    void callback_number(const example_interfaces::msg::Int64::SharedPtr msg)
    {
        counter_ += msg->data;
        RCLCPP_DEBUG(this->get_logger(), "Received number: %ld, current sum: %ld", msg->data, counter_);
    }

    void publish_count()
    {
        auto msg = example_interfaces::msg::Int64();
        msg.data = counter_;
        RCLCPP_INFO(this->get_logger(), "Publishing current sum to 'count': %ld", msg.data);
        publisher_->publish(msg);
    }

    void reset_counter_callback(
        const example_interfaces::srv::SetBool::Request::SharedPtr request,
        const example_interfaces::srv::SetBool::Response::SharedPtr response)
    {
        if (request->data)
        {
            counter_ = 0;
            response->success = true;
            response->message = "Counter has been successfully reset to 0";
            RCLCPP_INFO(this->get_logger(), "Counter reset to 0");
        }
        else
        {
            response->success = false;
            response->message = "Reset not performed. Request data was false.";
            RCLCPP_INFO(this->get_logger(), "Reset service called with false - counter not reset");
        }
    }

    int64_t counter_;
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr reset_service_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
