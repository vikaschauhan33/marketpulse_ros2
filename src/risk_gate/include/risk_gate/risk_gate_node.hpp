#ifndef RISK_GATE__RISK_GATE_NODE_HPP_
#define RISK_GATE__RISK_GATE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <deque>
#include <chrono>
#include "marketpulse_interfaces/msg/signal.hpp"
#include "marketpulse_interfaces/msg/order.hpp"
#include "marketpulse_interfaces/msg/pipeline_stats.hpp"
#include "marketpulse_interfaces/srv/reset.hpp"

namespace risk_gate
{

struct PositionState
{
  int32_t position{0};
};

class RiskGateNode : public rclcpp::Node
{
public:
  explicit RiskGateNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void handle_signal(const marketpulse_interfaces::msg::Signal::SharedPtr msg);
  void publish_stats();
  void cleanup_old_orders();
  
  void handle_reset(
    const std::shared_ptr<marketpulse_interfaces::srv::Reset::Request> request,
    std::shared_ptr<marketpulse_interfaces::srv::Reset::Response> response);

  bool check_staleness(const builtin_interfaces::msg::Time & signal_time);
  bool check_rate_limit();
  bool check_position_limit(const std::string & symbol, int8_t side);

  // Parameters
  int max_position_per_symbol_;
  int max_orders_per_min_;
  int reject_on_stale_ms_;

  // State
  std::unordered_map<std::string, PositionState> positions_;
  std::deque<std::chrono::steady_clock::time_point> order_times_;
  uint64_t signals_received_;
  uint64_t orders_sent_;
  uint64_t orders_rejected_;
  uint64_t order_seq_;

  // ROS 2 objects
  rclcpp::Subscription<marketpulse_interfaces::msg::Signal>::SharedPtr signal_subscriber_;
  rclcpp::Publisher<marketpulse_interfaces::msg::Order>::SharedPtr order_publisher_;
  rclcpp::Publisher<marketpulse_interfaces::msg::PipelineStats>::SharedPtr stats_publisher_;
  rclcpp::Service<marketpulse_interfaces::srv::Reset>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
};

}  // namespace risk_gate

#endif  // RISK_GATE__RISK_GATE_NODE_HPP_
