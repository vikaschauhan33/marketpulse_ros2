#include "market_data_feed/market_data_feed_node.hpp"
#include <chrono>

using namespace std::chrono_literals;

namespace market_data_feed
{

MarketDataFeedNode::MarketDataFeedNode(const rclcpp::NodeOptions & options)
: Node("market_data_feed", options),
  tick_seq_(0),
  ticks_published_(0)
{
  // Declare and get parameters
  this->declare_parameter("symbols", std::vector<std::string>{"AAPL", "MSFT", "NVDA"});
  this->declare_parameter("tick_rate_hz", 50.0);
  this->declare_parameter("price_start", 100.0);
  this->declare_parameter("price_volatility", 0.5);
  this->declare_parameter("rng_seed", 42);
  this->declare_parameter("enabled", true);

  symbols_ = this->get_parameter("symbols").as_string_array();
  tick_rate_hz_ = this->get_parameter("tick_rate_hz").as_double();
  price_start_ = this->get_parameter("price_start").as_double();
  price_volatility_ = this->get_parameter("price_volatility").as_double();
  rng_seed_ = this->get_parameter("rng_seed").as_int();
  enabled_ = this->get_parameter("enabled").as_bool();

  // Initialize RNG
  rng_.seed(rng_seed_);
  price_dist_ = std::normal_distribution<double>(0.0, price_volatility_);

  // Initialize prices for all symbols
  for (const auto & symbol : symbols_) {
    current_prices_[symbol] = price_start_;
  }

  // Create publishers with explicit QoS
  auto tick_qos = rclcpp::QoS(50).best_effort();
  tick_publisher_ = this->create_publisher<marketpulse_interfaces::msg::Tick>(
    "/ticks", tick_qos);

  auto stats_qos = rclcpp::QoS(10).reliable();
  stats_publisher_ = this->create_publisher<marketpulse_interfaces::msg::PipelineStats>(
    "/feed_stats", stats_qos);

  // Create services
  set_enabled_service_ = this->create_service<marketpulse_interfaces::srv::SetBoolPlus>(
    "/feed/set_enabled",
    std::bind(&MarketDataFeedNode::handle_set_enabled, this,
              std::placeholders::_1, std::placeholders::_2));

  set_rate_service_ = this->create_service<marketpulse_interfaces::srv::SetRate>(
    "/feed/set_rate",
    std::bind(&MarketDataFeedNode::handle_set_rate, this,
              std::placeholders::_1, std::placeholders::_2));

  // Create timers
  auto tick_period = std::chrono::duration<double>(1.0 / tick_rate_hz_);
  tick_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(tick_period),
    std::bind(&MarketDataFeedNode::publish_ticks, this));

  stats_timer_ = this->create_wall_timer(
    1s, std::bind(&MarketDataFeedNode::publish_stats, this));

  RCLCPP_INFO(this->get_logger(), 
    "Market Data Feed started with %zu symbols at %.1f Hz (enabled: %s)",
    symbols_.size(), tick_rate_hz_, enabled_ ? "true" : "false");
}

void MarketDataFeedNode::publish_ticks()
{
  if (!enabled_) {
    return;
  }

  for (const auto & symbol : symbols_) {
    // Update price with Brownian motion
    double price_change = price_dist_(rng_);
    current_prices_[symbol] += price_change;
    
    // Ensure price stays positive
    if (current_prices_[symbol] < 1.0) {
      current_prices_[symbol] = 1.0;
    }

    // Create and publish tick
    auto tick = marketpulse_interfaces::msg::Tick();
    tick.stamp = this->now();
    tick.symbol = symbol;
    tick.price = current_prices_[symbol];
    tick.seq = tick_seq_++;

    tick_publisher_->publish(tick);
    ticks_published_++;
  }
}

void MarketDataFeedNode::publish_stats()
{
  auto stats = marketpulse_interfaces::msg::PipelineStats();
  stats.stamp = this->now();
  stats.node_name = this->get_name();
  stats.in_count = 0;  // Feed has no input
  stats.out_count = ticks_published_;
  stats.dropped = 0;
  stats.avg_latency_ms = 0.0;
  stats.avg_processing_us = 0.0;

  stats_publisher_->publish(stats);
}

void MarketDataFeedNode::handle_set_enabled(
  const std::shared_ptr<marketpulse_interfaces::srv::SetBoolPlus::Request> request,
  std::shared_ptr<marketpulse_interfaces::srv::SetBoolPlus::Response> response)
{
  enabled_ = request->enabled;
  response->ok = true;
  response->message = enabled_ ? "Feed enabled" : "Feed disabled";
  
  RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
}

void MarketDataFeedNode::handle_set_rate(
  const std::shared_ptr<marketpulse_interfaces::srv::SetRate::Request> request,
  std::shared_ptr<marketpulse_interfaces::srv::SetRate::Response> response)
{
  // Validate rate
  if (request->tick_rate_hz <= 0.0 || request->tick_rate_hz > 500.0) {
    response->ok = false;
    response->message = "Invalid rate. Must be > 0 and <= 500 Hz";
    RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
    return;
  }

  tick_rate_hz_ = request->tick_rate_hz;
  
  // Recreate timer with new period
  tick_timer_->cancel();
  auto tick_period = std::chrono::duration<double>(1.0 / tick_rate_hz_);
  tick_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(tick_period),
    std::bind(&MarketDataFeedNode::publish_ticks, this));

  response->ok = true;
  response->message = "Rate updated to " + std::to_string(tick_rate_hz_) + " Hz";
  
  RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
}

}  // namespace market_data_feed
