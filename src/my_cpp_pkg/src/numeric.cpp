#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

#include <chrono>
#include <cinttypes>

// Enable readable time literals like 1s, 100ms
using namespace std::chrono_literals;

/**
 * @class NumericNode
 * @brief A simple ROS 2 node that publishes increasing integer values
 *        on the "number" topic once per second.
 *
 * This example demonstrates:
 *  - A ROS 2 C++ publisher
 *  - Timers in rclcpp
 *  - Modern C++ (C++20) style and best practices
 */
class NumericNode final : public rclcpp::Node
{
public:
    // Type aliases to improve readability and reduce verbosity
    using Int64Msg  = example_interfaces::msg::Int64;
    using Publisher = rclcpp::Publisher<Int64Msg>;

    /**
     * @brief Construct the NumericNode
     *
     * - Initializes the ROS node with a name
     * - Creates a publisher on the "number" topic
     * - Starts a periodic timer to publish data every second
     */
    NumericNode()
        : Node{"numeric_node"}   // Name of the node in the ROS graph
    {
        // Create a publisher that publishes Int64 messages on topic "number"
        // The queue size (10) defines how many messages can be buffered
        publisher_ = create_publisher<Int64Msg>("number", 10);

        // Create a wall timer that triggers once per second
        // A lambda is preferred over std::bind for clarity
        timer_ = create_wall_timer(
            1s,
            [this] { publish_number(); }
        );

        // Informational log to confirm successful startup
        RCLCPP_INFO(
            get_logger(),
            "Numeric node started and publishing to 'number' topic."
        );
    }

private:
    /**
     * @brief Timer callback function
     *
     * This function is called once per second by the timer.
     * It creates a message, updates the counter, logs the value,
     * and publishes it to the "number" topic.
     */
    void publish_number()
    {
        // Create a new Int64 message
        Int64Msg msg{};

        // Assign the current counter value, then increment it
        msg.data = counter_++;

        // Log the published value (PRId64 ensures portability)
        RCLCPP_INFO(
            get_logger(),
            "Publishing: %" PRId64,
            msg.data
        );

        // Publish the message to the ROS 2 network
        publisher_->publish(msg);
    }

    // Internal counter that keeps track of the number being published
    int64_t counter_{0};

    // Timer responsible for triggering periodic publishing
    rclcpp::TimerBase::SharedPtr timer_;

    // Publisher responsible for sending Int64 messages on "number"
    Publisher::SharedPtr publisher_;
};

/**
 * @brief Main entry point of the program
 *
 * - Initializes the ROS 2 system
 * - Instantiates the NumericNode
 * - Keeps it running using rclcpp::spin()
 * - Cleans up before exiting
 */
int main(int argc, char* argv[])
{
    // Initialize ROS 2 (must be called before using rclcpp)
    rclcpp::init(argc, argv);

    // Create and run the NumericNode
    rclcpp::spin(std::make_shared<NumericNode>());

    // Shutdown ROS 2 cleanly
    rclcpp::shutdown();
    return 0;
}
