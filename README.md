# MarketPulse ROS 2 - Market Data Pipeline Simulator

A production-ready ROS 2 (Humble) market data processing pipeline demonstrating distributed systems patterns, proper threading, QoS policies, and comprehensive testing.

## Architecture Overview

MarketPulse consists of 3 nodes forming a data processing pipeline:

```
┌─────────────────┐      ┌──────────────────┐      ┌─────────────┐
│ Market Data     │─────▶│ Strategy Engine  │─────▶│ Risk Gate   │
│ Feed            │      │                  │      │             │
└─────────────────┘      └──────────────────┘      └─────────────┘
   /ticks (BEST_EFFORT)    /signals (RELIABLE)      /orders (RELIABLE)
```

### Nodes

**1. market_data_feed_node**
- Generates synthetic market ticks using Brownian motion
- Publishes to `/ticks` (BEST_EFFORT QoS, depth 50)
- Publishes stats to `/feed_stats` (RELIABLE QoS)
- Services: `/feed/set_enabled`, `/feed/set_rate`
- Configurable: symbols, tick rate, volatility, RNG seed

**2. strategy_engine_node**
- Consumes ticks and computes rolling statistics
- Generates BUY/SELL/HOLD signals based on z-score
- **Signal Damping**: Only publishes to `/signals` when the recommended side changes
- Publishes to `/signals` (RELIABLE QoS)
- Uses MultiThreadedExecutor with callback groups
- Service: `/strategy/get_metrics`

**3. risk_gate_node**
- Applies risk limits to signals
- Checks: staleness, rate limits, position limits
- Publishes approved orders to `/orders` (RELIABLE QoS)
- Service: `/risk/reset`

## Build Instructions

### Prerequisites
- ROS 2 Humble on Ubuntu 22.04 / Pop!_OS
- C++20 compiler
- colcon build tool

### Build Steps

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Expected output: All packages build without warnings.

## Run Instructions

### Launch Full Pipeline

```bash
source install/setup.bash
ros2 launch marketpulse_bringup bringup.launch.py
```

### Run Individual Nodes

```bash
# Terminal 1: Market Data Feed
ros2 run market_data_feed market_data_feed_node

# Terminal 2: Strategy Engine
ros2 run strategy_engine strategy_engine_node

# Terminal 3: Risk Gate
ros2 run risk_gate risk_gate_node
```

### With Custom Parameters

```bash
ros2 run market_data_feed market_data_feed_node --ros-args \
  -p symbols:="['AAPL','MSFT','NVDA','GOOGL']" \
  -p tick_rate_hz:=100.0
```

## Example CLI Commands

### Monitor Topics

```bash
# List all topics
ros2 topic list

# Monitor tick rate
ros2 topic hz /ticks

# Echo ticks
ros2 topic echo /ticks

# Monitor bandwidth
ros2 topic bw /ticks

# Echo signals
ros2 topic echo /signals

# Echo orders
ros2 topic echo /orders
```

### Call Services

```bash
# Disable feed
ros2 service call /feed/set_enabled marketpulse_interfaces/srv/SetBoolPlus "{enabled: false}"

# Change tick rate to 100 Hz
ros2 service call /feed/set_rate marketpulse_interfaces/srv/SetRate "{tick_rate_hz: 100.0}"

# Get strategy metrics
ros2 service call /strategy/get_metrics marketpulse_interfaces/srv/GetMetrics "{dummy: true}"

# Reset risk gate
ros2 service call /risk/reset marketpulse_interfaces/srv/Reset "{dummy: true}"
```

### Record and Playback

```bash
# Record all topics
ros2 bag record -a

# Record specific topics
ros2 bag record /ticks /signals /orders

# Playback
ros2 bag play <bag_file>
```

## Testing

### Run All Tests

```bash
colcon test --packages-select marketpulse_interfaces market_data_feed strategy_engine risk_gate marketpulse_bringup
colcon test-result --verbose
```

### Run Specific Package Tests

```bash
# Unit tests only
colcon test --packages-select strategy_engine
colcon test-result --verbose

# Integration test
colcon test --packages-select marketpulse_bringup
```

### Expected Test Results
- All unit tests pass (deterministic RNG, rolling window, risk limits)
- Integration test verifies full pipeline message flow
- Services respond correctly

## Design Decisions

### QoS Policies

**BEST_EFFORT for /ticks:**
- High-frequency data where drops are acceptable
- Minimizes latency and prevents queue buildup
- Realistic market data behavior

**RELIABLE for /signals and /orders:**
- Critical data that must not be lost
- Lower frequency allows reliable delivery
- Ensures every signal is processed

### Threading

**MultiThreadedExecutor for strategy_engine:**
- Reentrant callback group for tick processing
- MutuallyExclusive callback group for services
- Prevents service calls from blocking tick processing

**Single-threaded for feed and risk_gate:**
- Sufficient for their workloads
- Simpler design, easier to reason about

### Memory Management

- Bounded rolling windows (fixed size)
- QoS depth limits prevent unbounded growth
- Per-symbol state isolation with max_symbols limit

## Performance Expectations

### Default Configuration (3 symbols, 50 Hz)
- Total tick rate: ~150 ticks/sec
- CPU usage: <10% on modern laptop
- Memory: <50 MB total
- No drops expected

### High Load (10 symbols, 50 Hz)
- Total tick rate: ~500 ticks/sec
- CPU usage: ~20-30%
- Expected: Some tick drops (BEST_EFFORT QoS)
- Signals/orders: No drops (RELIABLE QoS)

### Measuring Performance

```bash
# Tick rate
ros2 topic hz /ticks

# Bandwidth
ros2 topic bw /ticks

# Check stats
ros2 topic echo /feed_stats
ros2 topic echo /strategy_stats
ros2 topic echo /risk_stats
```

## Troubleshooting

### No ticks appearing

**Check if feed is enabled:**
```bash
ros2 service call /feed/set_enabled marketpulse_interfaces/srv/SetBoolPlus "{enabled: true}"
```

**Check node is running:**
```bash
ros2 node list | grep market_data_feed
```

### No signals appearing

**Check if ticks are flowing:**
```bash
ros2 topic hz /ticks
```

**Check strategy metrics:**
```bash
ros2 service call /strategy/get_metrics marketpulse_interfaces/srv/GetMetrics "{dummy: true}"
```

**Note:** Signals only appear after the rolling window fills (20 ticks) AND the recommendation changes from its previous state.

### No orders appearing

**Check if signals are BUY/SELL (not HOLD):**
```bash
ros2 topic echo /signals
```

**Check risk gate stats:**
```bash
ros2 topic echo /risk_stats
```

**Common causes:**
- All signals are HOLD (price not volatile enough)
- Rate limit exceeded
- Position limit exceeded
- Signals are stale

### Build errors

**Missing dependencies:**
```bash
rosdep install --from-paths src --ignore-src -r -y
```

**Clean build:**
```bash
rm -rf build install log
colcon build --symlink-install
```

## Configuration Files

Configuration files are located in each package's `config/` directory:

- `market_data_feed/config/market_data_feed.yaml`
- `strategy_engine/config/strategy_engine.yaml`
- `risk_gate/config/risk_gate.yaml`

Modify these files to change default parameters.

## Package Structure

```
ros2_ws/src/
├── marketpulse_interfaces/     # Custom messages and services
├── market_data_feed/           # Tick generator node
├── strategy_engine/            # Signal generation node
├── risk_gate/                  # Risk management node
└── marketpulse_bringup/        # Launch files and integration tests
```

## License

Apache-2.0

## Verification Checklist

After building, verify the following:

- [ ] All packages build without warnings
- [ ] All unit tests pass (`colcon test`)
- [ ] Integration test passes
- [ ] `ros2 topic list` shows: /ticks, /signals, /orders, /feed_stats, /strategy_stats, /risk_stats
- [ ] `ros2 service list` shows: /feed/set_enabled, /feed/set_rate, /strategy/get_metrics, /risk/reset
- [ ] `ros2 topic hz /ticks` shows ~150 Hz (3 symbols × 50 Hz)
- [ ] Service calls return expected responses
- [ ] Clean shutdown with Ctrl+C (no errors)

## Advanced Usage

### Multi-Pipeline Simulation

Run multiple pipelines with namespaces:

```bash
ros2 launch marketpulse_bringup bringup.launch.py namespace:=pipeline1
ros2 launch marketpulse_bringup bringup.launch.py namespace:=pipeline2
```

### Custom Symbols and Rates

```bash
ros2 run market_data_feed market_data_feed_node --ros-args \
  -p symbols:="['BTC','ETH','SOL','AVAX','MATIC']" \
  -p tick_rate_hz:=200.0 \
  -p price_volatility:=2.0
```

### Stress Testing

```bash
# High frequency test
ros2 service call /feed/set_rate marketpulse_interfaces/srv/SetRate "{tick_rate_hz: 500.0}"

# Monitor for drops
ros2 topic hz /ticks
ros2 topic echo /strategy_stats
```

## Contact

For issues or questions, please refer to the package maintainer information in each `package.xml`.
