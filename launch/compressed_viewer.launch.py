#!/usr/bin/env python3
"""
Launch file for compressed viewer node
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for compressed viewer"""
    
    # Get package directory
    pkg_dir = get_package_share_directory('compressed_viewer')
    config_file = os.path.join(pkg_dir, 'config', 'viewer_params.yaml')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'compressed_viewer.rviz')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to the configuration file'
    )
    
    rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=rviz_config,
        description='Path to RViz configuration file'
    )
    
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='compressed_pointcloud',
        description='Input topic for compressed point cloud'
    )
    
    # Create compressed viewer node
    compressed_viewer_node = Node(
        package='compressed_viewer',
        executable='compressed_viewer_node',
        name='compressed_viewer_node',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'topics.input_topic': LaunchConfiguration('input_topic')}
        ],
        output='screen',
        emulate_tty=True
    )
    
    # Create RViz node (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=LaunchConfiguration('launch_rviz'),
        output='screen'
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(use_sim_time_arg)
    ld.add_action(config_file_arg)
    ld.add_action(rviz_arg)
    ld.add_action(rviz_config_arg)
    ld.add_action(input_topic_arg)
    
    # Add nodes
    ld.add_action(compressed_viewer_node)
    
    # Conditionally add RViz
    from launch.conditions import IfCondition
    rviz_node_conditional = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
        output='screen'
    )
    ld.add_action(rviz_node_conditional)
    
    return ld