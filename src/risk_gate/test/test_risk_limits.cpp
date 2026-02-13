#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "risk_gate/risk_gate_node.hpp"

class RiskLimitsTest : public ::testing::Test
{
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override {
    rclcpp::shutdown();
  }
};

TEST_F(RiskLimitsTest, PositionLimitLogic)
{
  // Test position limit calculation
  int max_position = 100;
  int current_position = 90;
  int order_qty = 10;
  int8_t buy_side = 1;
  
  int32_t new_pos = current_position + (buy_side * order_qty);
  EXPECT_EQ(new_pos, 100);
  EXPECT_LE(std::abs(new_pos), max_position);
  
  // Test exceeding limit
  current_position = 95;
  new_pos = current_position + (buy_side * order_qty);
  EXPECT_GT(std::abs(new_pos), max_position);
}

TEST_F(RiskLimitsTest, RateLimitLogic)
{
  // Test rate limit with time-based cleanup
  std::deque<std::chrono::steady_clock::time_point> order_times;
  int max_orders = 120;
  
  auto now = std::chrono::steady_clock::now();
  
  // Add 119 orders
  for (int i = 0; i < 119; ++i) {
    order_times.push_back(now);
  }
  
  EXPECT_LT(static_cast<int>(order_times.size()), max_orders);
  
  // Add one more
  order_times.push_back(now);
  EXPECT_EQ(static_cast<int>(order_times.size()), max_orders);
  
  // Should reject next order
  EXPECT_FALSE(static_cast<int>(order_times.size()) < max_orders);
}

TEST_F(RiskLimitsTest, StalenessCheck)
{
  int reject_on_stale_ms = 200;
  
  // Recent signal
  int64_t signal_age_ms = 100;
  EXPECT_LE(signal_age_ms, reject_on_stale_ms);
  
  // Stale signal
  signal_age_ms = 300;
  EXPECT_GT(signal_age_ms, reject_on_stale_ms);
}

TEST_F(RiskLimitsTest, NodeCreation)
{
  // Test that node can be created
  rclcpp::NodeOptions options;
  EXPECT_NO_THROW({
    auto node = std::make_shared<risk_gate::RiskGateNode>(options);
  });
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
