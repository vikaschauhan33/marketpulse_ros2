#include "risk_gate/risk_gate_node.hpp"
#include <chrono>

using namespace std::chrono_literals;

namespace risk_gate
{

RiskGateNode::RiskGateNode(const rclcpp::NodeOptions & options)
: Node("risk_gate", options),
  signals_received_(0),
  orders_sent_(0),
  orders_rejected_(0),
  order_seq_(0)
{
  // Declare and get parameters
  this->declare_parameter("max_position_per_symbol", 100);
  this->declare_parameter("max_orders_per_min", 120);
  this->declare_parameter("reject_on_stale_ms", 200);

  max_position_per_symbol_ = this->get_parameter("max_position_per_symbol").as_int();
  max_orders_per_min_ = this->get_parameter("max_orders_per_min").as_int();
  reject_on_stale_ms_ = this->get_parameter("reject_on_stale_ms").as_int();

  // Create subscription
  auto signal_qos = rclcpp::QoS(10).reliable();
  signal_subscriber_ = this->create_subscription<marketpulse_interfaces::msg::Signal>(
    "/signals", signal_qos,
    std::bind(&RiskGateNode::handle_signal, this, std::placeholders::_1));

  // Create publishers
  auto order_qos = rclcpp::QoS(10).reliable();
  order_publisher_ = this->create_publisher<marketpulse_interfaces::msg::Order>(
    "/orders", order_qos);

  auto stats_qos = rclcpp::QoS(10).reliable();
  stats_publisher_ = this->create_publisher<marketpulse_interfaces::msg::PipelineStats>(
    "/risk_stats", stats_qos);

  // Create service
  reset_service_ = this->create_service<marketpulse_interfaces::srv::Reset>(
    "/risk/reset",
    std::bind(&RiskGateNode::handle_reset, this,
              std::placeholders::_1, std::placeholders::_2));

  // Create stats timer
  stats_timer_ = this->create_wall_timer(
    1s, std::bind(&RiskGateNode::publish_stats, this));

  RCLCPP_INFO(this->get_logger(),
    "Risk Gate started (max_pos: %d, max_orders/min: %d, stale_ms: %d)",
    max_position_per_symbol_, max_orders_per_min_, reject_on_stale_ms_);
}

void RiskGateNode::handle_signal(
  const marketpulse_interfaces::msg::Signal::SharedPtr msg)
{
  signals_received_++;

  // Filter out HOLD signals (side == 0)
  if (msg->side == 0) {
    return;
  }

  // Check staleness
  if (!check_staleness(msg->stamp)) {
    orders_rejected_++;
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "Rejected stale signal for %s", msg->symbol.c_str());
    return;
  }

  // Check rate limit
  if (!check_rate_limit()) {
    orders_rejected_++;
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "Rate limit exceeded, rejecting order");
    return;
  }

  // Check position limit
  if (!check_position_limit(msg->symbol, msg->side)) {
    orders_rejected_++;
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "Position limit exceeded for %s", msg->symbol.c_str());
    return;
  }

  // All checks passed - create and publish order
  auto order = marketpulse_interfaces::msg::Order();
  order.stamp = this->now();
  order.symbol = msg->symbol;
  order.side = msg->side;
  order.qty = 10;  // Fixed quantity for simplicity
  order.ref_price = msg->price;
  order.reason = "Signal approved";
  order.seq = order_seq_++;

  order_publisher_->publish(order);
  orders_sent_++;

  // Update position
  positions_[msg->symbol].position += (msg->side * order.qty);

  // Record order time for rate limiting
  order_times_.push_back(std::chrono::steady_clock::now());
}

bool RiskGateNode::check_staleness(const builtin_interfaces::msg::Time & signal_time)
{
  rclcpp::Time now = this->now();
  rclcpp::Time signal_stamp(signal_time);
  
  // Use duration arithmetic to avoid overflow issues
  auto age = now - signal_stamp;
  int64_t age_ms = age.nanoseconds() / 1000000;
  
  return age_ms <= static_cast<int64_t>(reject_on_stale_ms_);
}

bool RiskGateNode::check_rate_limit()
{
  cleanup_old_orders();
  return static_cast<int>(order_times_.size()) < max_orders_per_min_;
}

bool RiskGateNode::check_position_limit(const std::string & symbol, int8_t side)
{
  int32_t current_pos = positions_[symbol].position;
  int32_t new_pos = current_pos + (side * 10);  // Assuming qty=10
  
  return std::abs(new_pos) <= max_position_per_symbol_;
}

void RiskGateNode::cleanup_old_orders()
{
  auto now = std::chrono::steady_clock::now();
  auto one_minute_ago = now - std::chrono::minutes(1);
  
  while (!order_times_.empty() && order_times_.front() < one_minute_ago) {
    order_times_.pop_front();
  }
}

void RiskGateNode::publish_stats()
{
  auto stats = marketpulse_interfaces::msg::PipelineStats();
  stats.stamp = this->now();
  stats.node_name = this->get_name();
  stats.in_count = signals_received_;
  stats.out_count = orders_sent_;
  stats.dropped = orders_rejected_;
  stats.avg_latency_ms = 0.0;
  stats.avg_processing_us = 0.0;

  stats_publisher_->publish(stats);
}

void RiskGateNode::handle_reset(
  const std::shared_ptr<marketpulse_interfaces::srv::Reset::Request> /* request */,
  std::shared_ptr<marketpulse_interfaces::srv::Reset::Response> response)
{
  positions_.clear();
  order_times_.clear();
  signals_received_ = 0;
  orders_sent_ = 0;
  orders_rejected_ = 0;
  order_seq_ = 0;

  response->ok = true;
  response->message = "Risk gate state reset successfully";
  
  RCLCPP_INFO(this->get_logger(), "Risk gate state reset");
}

}  // namespace risk_gate
