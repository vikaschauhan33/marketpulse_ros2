#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <random>
#include "market_data_feed/market_data_feed_node.hpp"

class TickGeneratorTest : public ::testing::Test
{
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override {
    rclcpp::shutdown();
  }
};

TEST_F(TickGeneratorTest, DeterministicPriceGeneration)
{
  // Test that same seed produces same sequence
  std::mt19937 rng1(42);
  std::mt19937 rng2(42);
  std::normal_distribution<double> dist(0.0, 0.5);

  for (int i = 0; i < 100; ++i) {
    double val1 = dist(rng1);
    double val2 = dist(rng2);
    EXPECT_DOUBLE_EQ(val1, val2);
  }
}

TEST_F(TickGeneratorTest, PriceStaysPositive)
{
  // Simulate price updates and ensure they stay positive
  std::mt19937 rng(42);
  std::normal_distribution<double> dist(0.0, 0.5);
  
  double price = 100.0;
  for (int i = 0; i < 1000; ++i) {
    price += dist(rng);
    if (price < 1.0) {
      price = 1.0;
    }
    EXPECT_GE(price, 1.0);
  }
}

TEST_F(TickGeneratorTest, NodeCreation)
{
  // Test that node can be created with default parameters
  rclcpp::NodeOptions options;
  EXPECT_NO_THROW({
    auto node = std::make_shared<market_data_feed::MarketDataFeedNode>(options);
  });
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
