#include "risk_gate/risk_gate_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<risk_gate::RiskGateNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
