#ifndef MARKET_DATA_FEED__MARKET_DATA_FEED_NODE_HPP_
#define MARKET_DATA_FEED__MARKET_DATA_FEED_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <random>
#include <string>
#include <vector>
#include "marketpulse_interfaces/msg/tick.hpp"
#include "marketpulse_interfaces/msg/pipeline_stats.hpp"
#include "marketpulse_interfaces/srv/set_bool_plus.hpp"
#include "marketpulse_interfaces/srv/set_rate.hpp"

namespace market_data_feed
{

class MarketDataFeedNode : public rclcpp::Node
{
public:
  explicit MarketDataFeedNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void publish_ticks();
  void publish_stats();
  
  void handle_set_enabled(
    const std::shared_ptr<marketpulse_interfaces::srv::SetBoolPlus::Request> request,
    std::shared_ptr<marketpulse_interfaces::srv::SetBoolPlus::Response> response);
  
  void handle_set_rate(
    const std::shared_ptr<marketpulse_interfaces::srv::SetRate::Request> request,
    std::shared_ptr<marketpulse_interfaces::srv::SetRate::Response> response);

  // Parameters
  std::vector<std::string> symbols_;
  double tick_rate_hz_;
  double price_start_;
  double price_volatility_;
  int rng_seed_;
  bool enabled_;

  // State
  std::mt19937 rng_;
  std::normal_distribution<double> price_dist_;
  std::unordered_map<std::string, double> current_prices_;
  uint64_t tick_seq_;
  uint64_t ticks_published_;
  
  // ROS 2 objects
  rclcpp::Publisher<marketpulse_interfaces::msg::Tick>::SharedPtr tick_publisher_;
  rclcpp::Publisher<marketpulse_interfaces::msg::PipelineStats>::SharedPtr stats_publisher_;
  rclcpp::Service<marketpulse_interfaces::srv::SetBoolPlus>::SharedPtr set_enabled_service_;
  rclcpp::Service<marketpulse_interfaces::srv::SetRate>::SharedPtr set_rate_service_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
};

}  // namespace market_data_feed

#endif  // MARKET_DATA_FEED__MARKET_DATA_FEED_NODE_HPP_
