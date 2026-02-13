#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class AddTwoIntsServer : public rclcpp::Node
{
public:
  AddTwoIntsServer()
  : Node("add_two_ints_server")
  {
    // Create a service named "add_two_ints"
    // The callback function 'handle_add_two_ints' will be executed when a request is received.
    // std::bind is used to pass the 'this' pointer and placeholders for the request and response.
    server_ = this->create_service<example_interfaces::srv::AddTwoInts>(
      "add_two_ints",
      std::bind(&AddTwoIntsServer::handle_add_two_ints, this, _1, _2));
    RCLCPP_INFO(this->get_logger(), "Service server has been started.");
  }

private:
  // Callback function to handle the service request
  // It receives the request (containing 'a' and 'b') and the response object.
  void handle_add_two_ints(
    const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
    std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
  {
    // Perform the addition and assign it to the response
    response->sum = request->a + request->b;
    
    // Log the request and the calculated response
    RCLCPP_INFO_STREAM(this->get_logger(), "Incoming request\na: " << request->a << " b: " << request->b);
    RCLCPP_INFO_STREAM(this->get_logger(), "sending back response: [" << response->sum << "]");
  }

  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr server_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AddTwoIntsServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
