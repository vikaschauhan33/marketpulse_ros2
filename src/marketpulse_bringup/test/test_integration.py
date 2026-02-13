#!/usr/bin/env python3

import unittest
import time
import rclpy
from rclpy.node import Node
from marketpulse_interfaces.msg import Tick, Signal, Order
from marketpulse_interfaces.srv import SetBoolPlus, SetRate, GetMetrics

import launch
import launch_ros
import launch_testing.actions
import pytest


@pytest.mark.launch_test
def generate_test_description():
    # Launch all nodes
    market_data_feed_node = launch_ros.actions.Node(
        package='market_data_feed',
        executable='market_data_feed_node',
        name='market_data_feed',
        output='screen'
    )

    strategy_engine_node = launch_ros.actions.Node(
        package='strategy_engine',
        executable='strategy_engine_node',
        name='strategy_engine',
        output='screen'
    )

    risk_gate_node = launch_ros.actions.Node(
        package='risk_gate',
        executable='risk_gate_node',
        name='risk_gate',
        output='screen'
    )

    return (
        launch.LaunchDescription([
            market_data_feed_node,
            strategy_engine_node,
            risk_gate_node,
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'market_data_feed': market_data_feed_node,
            'strategy_engine': strategy_engine_node,
            'risk_gate': risk_gate_node,
        }
    )


class TestMarketPulsePipeline(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = Node('test_node')
        self.ticks_received = []
        self.signals_received = []
        self.orders_received = []

    def tearDown(self):
        self.node.destroy_node()

    def test_ticks_published(self):
        """Test that ticks are being published"""
        def tick_callback(msg):
            self.ticks_received.append(msg)

        sub = self.node.create_subscription(
            Tick, '/ticks', tick_callback, 10)

        # Wait for messages
        end_time = time.time() + 3.0
        while time.time() < end_time and len(self.ticks_received) < 10:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertGreater(len(self.ticks_received), 0,
                          "No ticks received within 3 seconds")
        self.node.destroy_subscription(sub)

    def test_signals_published(self):
        """Test that signals are being published"""
        def signal_callback(msg):
            self.signals_received.append(msg)

        sub = self.node.create_subscription(
            Signal, '/signals', signal_callback, 10)

        # Wait for messages
        end_time = time.time() + 5.0
        while time.time() < end_time and len(self.signals_received) < 5:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertGreater(len(self.signals_received), 0,
                          "No signals received within 5 seconds")
        self.node.destroy_subscription(sub)

    def test_orders_published(self):
        """Test that orders are being published"""
        def order_callback(msg):
            self.orders_received.append(msg)

        sub = self.node.create_subscription(
            Order, '/orders', order_callback, 10)

        # Wait for messages
        end_time = time.time() + 7.0
        while time.time() < end_time and len(self.orders_received) < 3:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertGreater(len(self.orders_received), 0,
                          "No orders received within 7 seconds")
        self.node.destroy_subscription(sub)

    def test_set_enabled_service(self):
        """Test the set_enabled service"""
        client = self.node.create_client(SetBoolPlus, '/feed/set_enabled')
        
        self.assertTrue(client.wait_for_service(timeout_sec=5.0),
                       "Service /feed/set_enabled not available")

        request = SetBoolPlus.Request()
        request.enabled = False
        future = client.call_async(request)
        
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
        
        self.assertTrue(future.done(), "Service call timed out")
        response = future.result()
        self.assertTrue(response.ok, "Service call failed")
        
        self.node.destroy_client(client)

    def test_get_metrics_service(self):
        """Test the get_metrics service"""
        # Wait a bit for metrics to accumulate
        time.sleep(2.0)
        
        client = self.node.create_client(GetMetrics, '/strategy/get_metrics')
        
        self.assertTrue(client.wait_for_service(timeout_sec=5.0),
                       "Service /strategy/get_metrics not available")

        request = GetMetrics.Request()
        request.dummy = True
        future = client.call_async(request)
        
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
        
        self.assertTrue(future.done(), "Service call timed out")
        response = future.result()
        self.assertTrue(response.ok, "Service call failed")
        self.assertGreater(response.ticks_seen, 0,
                          "No ticks seen by strategy engine")
        
        self.node.destroy_client(client)
