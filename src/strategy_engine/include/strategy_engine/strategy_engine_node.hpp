#ifndef STRATEGY_ENGINE__STRATEGY_ENGINE_NODE_HPP_
#define STRATEGY_ENGINE__STRATEGY_ENGINE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <mutex>
#include <chrono>
#include "marketpulse_interfaces/msg/tick.hpp"
#include "marketpulse_interfaces/msg/signal.hpp"
#include "marketpulse_interfaces/msg/pipeline_stats.hpp"
#include "marketpulse_interfaces/srv/get_metrics.hpp"
#include "strategy_engine/rolling_window.hpp"

namespace strategy_engine
{

struct SymbolState
{
  RollingWindow window;
  uint64_t ticks_seen{0};
  int8_t last_side{0};
  
  explicit SymbolState(size_t window_size) : window(window_size) {}
};

class StrategyEngineNode : public rclcpp::Node
{
public:
  explicit StrategyEngineNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void handle_tick(const marketpulse_interfaces::msg::Tick::SharedPtr msg);
  void publish_stats();
  
  void handle_get_metrics(
    const std::shared_ptr<marketpulse_interfaces::srv::GetMetrics::Request> request,
    std::shared_ptr<marketpulse_interfaces::srv::GetMetrics::Response> response);

  // Parameters
  int window_size_;
  double signal_threshold_;
  int max_symbols_;

  // State (protected by mutex for service access)
  std::mutex state_mutex_;
  std::unordered_map<std::string, std::unique_ptr<SymbolState>> symbol_states_;
  uint64_t ticks_seen_;
  uint64_t signals_sent_;
  uint64_t signal_seq_;
  double total_processing_us_;

  // ROS 2 objects
  rclcpp::Subscription<marketpulse_interfaces::msg::Tick>::SharedPtr tick_subscriber_;
  rclcpp::Publisher<marketpulse_interfaces::msg::Signal>::SharedPtr signal_publisher_;
  rclcpp::Publisher<marketpulse_interfaces::msg::PipelineStats>::SharedPtr stats_publisher_;
  rclcpp::Service<marketpulse_interfaces::srv::GetMetrics>::SharedPtr metrics_service_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  
  rclcpp::CallbackGroup::SharedPtr reentrant_group_;
  rclcpp::CallbackGroup::SharedPtr mutually_exclusive_group_;
};

}  // namespace strategy_engine

#endif  // STRATEGY_ENGINE__STRATEGY_ENGINE_NODE_HPP_
