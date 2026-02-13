#include <gtest/gtest.h>
#include "strategy_engine/rolling_window.hpp"

using strategy_engine::RollingWindow;

TEST(RollingWindowTest, MeanCalculation)
{
  RollingWindow window(5);
  
  window.add(10.0);
  window.add(20.0);
  window.add(30.0);
  
  EXPECT_DOUBLE_EQ(window.mean(), 20.0);
}

TEST(RollingWindowTest, StdDevCalculation)
{
  RollingWindow window(5);
  
  window.add(10.0);
  window.add(20.0);
  window.add(30.0);
  
  double expected_stddev = std::sqrt(((10-20)*(10-20) + (20-20)*(20-20) + (30-20)*(30-20)) / 3.0);
  EXPECT_NEAR(window.stddev(), expected_stddev, 1e-6);
}

TEST(RollingWindowTest, WindowBounds)
{
  RollingWindow window(3);
  
  window.add(1.0);
  window.add(2.0);
  window.add(3.0);
  EXPECT_EQ(window.size(), 3);
  EXPECT_TRUE(window.is_full());
  
  window.add(4.0);
  EXPECT_EQ(window.size(), 3);  // Should not exceed window size
  EXPECT_DOUBLE_EQ(window.mean(), 3.0);  // (2 + 3 + 4) / 3
}

TEST(RollingWindowTest, Clear)
{
  RollingWindow window(5);
  window.add(1.0);
  window.add(2.0);
  
  window.clear();
  EXPECT_EQ(window.size(), 0);
  EXPECT_FALSE(window.is_full());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
