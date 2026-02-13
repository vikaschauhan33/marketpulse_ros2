#include "market_data_feed/market_data_feed_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<market_data_feed::MarketDataFeedNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
