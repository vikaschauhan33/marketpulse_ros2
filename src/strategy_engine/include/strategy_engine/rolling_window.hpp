#ifndef STRATEGY_ENGINE__ROLLING_WINDOW_HPP_
#define STRATEGY_ENGINE__ROLLING_WINDOW_HPP_

#include <deque>
#include <cmath>
#include <algorithm>

namespace strategy_engine
{

class RollingWindow
{
public:
  explicit RollingWindow(size_t window_size)
  : window_size_(window_size) {}

  void add(double value)
  {
    values_.push_back(value);
    if (values_.size() > window_size_) {
      values_.pop_front();
    }
  }

  double mean() const
  {
    if (values_.empty()) {
      return 0.0;
    }
    double sum = 0.0;
    for (double val : values_) {
      sum += val;
    }
    return sum / values_.size();
  }

  double stddev() const
  {
    if (values_.size() < 2) {
      return 0.0;
    }
    double m = mean();
    double variance = 0.0;
    for (double val : values_) {
      double diff = val - m;
      variance += diff * diff;
    }
    return std::sqrt(variance / values_.size());
  }

  size_t size() const { return values_.size(); }
  bool is_full() const { return values_.size() >= window_size_; }
  void clear() { values_.clear(); }

private:
  size_t window_size_;
  std::deque<double> values_;
};

}  // namespace strategy_engine

#endif  // STRATEGY_ENGINE__ROLLING_WINDOW_HPP_
