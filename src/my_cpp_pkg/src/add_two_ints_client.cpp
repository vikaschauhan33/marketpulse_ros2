#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class AddTwoIntsClient : public rclcpp::Node
{
public:
    AddTwoIntsClient() : Node("add_two_ints_client")
    {
        // We spawn a separate thread to handle the service call.
        // 
        // WHY USE A THREAD?
        // 1. rclcpp::spin(node) is a blocking call. It occupies the main thread to process callbacks 
        //    (like service responses). 
        // 2. If we called the service in the main function BEFORE spinning, the request would go out, 
        //    but the response would never be processed because the node isn't spinning yet.
        // 3. If we called the service inside a callback (e.g., a Timer) and used 'future.get()' 
        //    to wait for the result, we would cause a DEADLOCK. The 'get()' call blocks the thread 
        //    waiting for the response, but that same thread is needed by the executor to process 
        //    the response given that we are using a SingleThreadedExecutor by default.
        //
        // By using a separate thread, we can block THIS thread waiting for the response 
        // while the MAIN thread (running rclcpp::spin) is free to process the incoming response message.
        thread_ = std::thread(std::bind(&AddTwoIntsClient::callAddTwoIntsService, this, 6, 7));
    }

    ~AddTwoIntsClient()
    {
        if (thread_.joinable()) {
            thread_.join();
        }
    }

    void callAddTwoIntsService(int a, int b)
    {
        auto client = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
        while (!client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");
        }

        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = a;
        request->b = b;

        // Send the request asynchronously. The future will eventually contain the response.
        auto future = client->async_send_request(request);

        try {
            // Wait for the result.
            // Since we are in a separate thread, this blocking call is safe.
            // It will block ONLY this thread, leaving the main thread free to handle the response callback.
            auto response = future.get();
            RCLCPP_INFO_STREAM(this->get_logger(), "Success! " << a << " + " << b << " = " << response->sum);
        } catch (const std::exception &e) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Service call failed: " << e.what());
        }
    }

private:
    std::thread thread_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
