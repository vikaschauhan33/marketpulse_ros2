#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for all nodes'
    )

    # Get config file paths
    feed_config = PathJoinSubstitution([
        FindPackageShare('market_data_feed'),
        'config',
        'market_data_feed.yaml'
    ])

    strategy_config = PathJoinSubstitution([
        FindPackageShare('strategy_engine'),
        'config',
        'strategy_engine.yaml'
    ])

    risk_config = PathJoinSubstitution([
        FindPackageShare('risk_gate'),
        'config',
        'risk_gate.yaml'
    ])

    namespace = LaunchConfiguration('namespace')

    # Market Data Feed Node
    market_data_feed_node = Node(
        package='market_data_feed',
        executable='market_data_feed_node',
        name='market_data_feed',
        namespace=namespace,
        parameters=[feed_config],
        output='screen'
    )

    # Strategy Engine Node
    strategy_engine_node = Node(
        package='strategy_engine',
        executable='strategy_engine_node',
        name='strategy_engine',
        namespace=namespace,
        parameters=[strategy_config],
        output='screen'
    )

    # Risk Gate Node
    risk_gate_node = Node(
        package='risk_gate',
        executable='risk_gate_node',
        name='risk_gate',
        namespace=namespace,
        parameters=[risk_config],
        output='screen'
    )

    return LaunchDescription([
        namespace_arg,
        market_data_feed_node,
        strategy_engine_node,
        risk_gate_node,
    ])
