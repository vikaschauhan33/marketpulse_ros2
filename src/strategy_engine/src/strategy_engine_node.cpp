#include "strategy_engine/strategy_engine_node.hpp"
#include <chrono>

using namespace std::chrono_literals;

namespace strategy_engine
{

StrategyEngineNode::StrategyEngineNode(const rclcpp::NodeOptions & options)
: Node("strategy_engine", options),
  ticks_seen_(0),
  signals_sent_(0),
  signal_seq_(0),
  total_processing_us_(0.0)
{
  // Declare and get parameters
  this->declare_parameter("window_size", 20);
  this->declare_parameter("signal_threshold", 1.5);
  this->declare_parameter("max_symbols", 100);

  window_size_ = this->get_parameter("window_size").as_int();
  signal_threshold_ = this->get_parameter("signal_threshold").as_double();
  max_symbols_ = this->get_parameter("max_symbols").as_int();

  // Create callback groups
  reentrant_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::Reentrant);
  mutually_exclusive_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  // Create subscription with reentrant callback group
  auto sub_options = rclcpp::SubscriptionOptions();
  sub_options.callback_group = reentrant_group_;
  
  auto tick_qos = rclcpp::QoS(50).best_effort();
  tick_subscriber_ = this->create_subscription<marketpulse_interfaces::msg::Tick>(
    "/ticks", tick_qos,
    std::bind(&StrategyEngineNode::handle_tick, this, std::placeholders::_1),
    sub_options);

  // Create publishers
  auto signal_qos = rclcpp::QoS(10).reliable();
  signal_publisher_ = this->create_publisher<marketpulse_interfaces::msg::Signal>(
    "/signals", signal_qos);

  auto stats_qos = rclcpp::QoS(10).reliable();
  stats_publisher_ = this->create_publisher<marketpulse_interfaces::msg::PipelineStats>(
    "/strategy_stats", stats_qos);

  // Create service with mutually exclusive callback group
  metrics_service_ = this->create_service<marketpulse_interfaces::srv::GetMetrics>(
    "/strategy/get_metrics",
    std::bind(&StrategyEngineNode::handle_get_metrics, this,
              std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    mutually_exclusive_group_);

  // Create stats timer
  stats_timer_ = this->create_wall_timer(
    1s, std::bind(&StrategyEngineNode::publish_stats, this));

  RCLCPP_INFO(this->get_logger(),
    "Strategy Engine started (window: %d, threshold: %.2f)",
    window_size_, signal_threshold_);
}

void StrategyEngineNode::handle_tick(
  const marketpulse_interfaces::msg::Tick::SharedPtr msg)
{
  auto start_time = std::chrono::high_resolution_clock::now();

  // Check symbol limit
  if (symbol_states_.find(msg->symbol) == symbol_states_.end()) {
    if (symbol_states_.size() >= static_cast<size_t>(max_symbols_)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Max symbols reached (%d), dropping new symbol: %s",
        max_symbols_, msg->symbol.c_str());
      return;
    }
    symbol_states_[msg->symbol] = std::make_unique<SymbolState>(window_size_);
  }

  auto & state = symbol_states_[msg->symbol];
  state->window.add(msg->price);
  state->ticks_seen++;
  ticks_seen_++;

  // Only generate signals when window is full
  if (state->window.is_full()) {
    double mean = state->window.mean();
    double stddev = state->window.stddev();
    double eps = 1e-6;
    double score = (msg->price - mean) / std::max(eps, stddev);

    int8_t side = 0;  // HOLD
    if (score > signal_threshold_) {
      side = 1;  // BUY
    } else if (score < -signal_threshold_) {
      side = -1;  // SELL
    }

    // Publish signal only if side changed to dampen frequency
    if (side != state->last_side) {
      auto signal = marketpulse_interfaces::msg::Signal();
      signal.stamp = this->now();
      signal.symbol = msg->symbol;
      signal.price = msg->price;
      signal.mean = mean;
      signal.score = score;
      signal.side = side;
      signal.seq = signal_seq_++;

      signal_publisher_->publish(signal);
      signals_sent_++;
      state->last_side = side;
    }
  }

  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
    end_time - start_time);
  
  // Update average processing time
  std::lock_guard<std::mutex> lock(state_mutex_);
  total_processing_us_ += duration.count();
}

void StrategyEngineNode::publish_stats()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  
  auto stats = marketpulse_interfaces::msg::PipelineStats();
  stats.stamp = this->now();
  stats.node_name = this->get_name();
  stats.in_count = ticks_seen_;
  stats.out_count = signals_sent_;
  stats.dropped = 0;
  stats.avg_latency_ms = 0.0;
  stats.avg_processing_us = ticks_seen_ > 0 ? 
    total_processing_us_ / ticks_seen_ : 0.0;

  stats_publisher_->publish(stats);
}

void StrategyEngineNode::handle_get_metrics(
  const std::shared_ptr<marketpulse_interfaces::srv::GetMetrics::Request> /* request */,
  std::shared_ptr<marketpulse_interfaces::srv::GetMetrics::Response> response)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  
  response->ok = true;
  response->message = "Metrics retrieved successfully";
  response->ticks_seen = ticks_seen_;
  response->signals_sent = signals_sent_;
  response->avg_processing_us = ticks_seen_ > 0 ? 
    total_processing_us_ / ticks_seen_ : 0.0;

  RCLCPP_INFO(this->get_logger(),
    "Metrics: ticks=%lu, signals=%lu, avg_proc=%.2f us",
    response->ticks_seen, response->signals_sent, response->avg_processing_us);
}

}  // namespace strategy_engine
