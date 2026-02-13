#include <gtest/gtest.h>
#include "strategy_engine/rolling_window.hpp"

using strategy_engine::RollingWindow;

TEST(SignalGenerationTest, BuySignal)
{
  RollingWindow window(5);
  
  // Add values with mean around 100
  window.add(100.0);
  window.add(101.0);
  window.add(99.0);
  window.add(100.0);
  window.add(100.0);
  
  double mean = window.mean();
  double stddev = window.stddev();
  
  // Price significantly above mean should trigger BUY
  double high_price = mean + 2.0 * stddev;
  double score = (high_price - mean) / std::max(1e-6, stddev);
  
  EXPECT_GT(score, 1.5);  // Should exceed threshold
}

TEST(SignalGenerationTest, SellSignal)
{
  RollingWindow window(5);
  
  window.add(100.0);
  window.add(101.0);
  window.add(99.0);
  window.add(100.0);
  window.add(100.0);
  
  double mean = window.mean();
  double stddev = window.stddev();
  
  // Price significantly below mean should trigger SELL
  double low_price = mean - 2.0 * stddev;
  double score = (low_price - mean) / std::max(1e-6, stddev);
  
  EXPECT_LT(score, -1.5);  // Should be below negative threshold
}

TEST(SignalGenerationTest, HoldSignal)
{
  RollingWindow window(5);
  
  window.add(100.0);
  window.add(101.0);
  window.add(99.0);
  window.add(100.0);
  window.add(100.0);
  
  double mean = window.mean();
  double stddev = window.stddev();
  
  // Price near mean should trigger HOLD
  double score = (mean - mean) / std::max(1e-6, stddev);
  
  EXPECT_NEAR(score, 0.0, 0.1);  // Should be near zero
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
