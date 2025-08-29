#!/usr/bin/env python3
"""
Demo launch file that starts both the pointcloud compressor and viewer
Simplified version: compress once and visualize
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """Generate launch description for demo"""
    
    # Get package directories
    compressor_pkg_dir = get_package_share_directory('pointcloud_compressor')
    viewer_pkg_dir = get_package_share_directory('compressed_viewer')
    
    # Configuration files
    demo_config = os.path.join(viewer_pkg_dir, 'config', 'demo_params.yaml')
    rviz_config = os.path.join(viewer_pkg_dir, 'rviz', 'compressed_viewer.rviz')
    
    # Declare launch arguments
    input_file_arg = DeclareLaunchArgument(
        'input_file',
        default_value='/tmp/sample.pcd',
        description='Path to the point cloud file to compress (PCD or PLY format)'
    )
    
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=demo_config,
        description='Path to configuration file (default: demo_params.yaml)'
    )
    
    # Pointcloud compressor node (compress once and publish)
    compressor_node = Node(
        package='pointcloud_compressor',
        executable='pointcloud_compressor_node',
        name='pointcloud_compressor_node',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'publish_once': True,
                'publish_patterns': True,
                'publish_occupied_voxel_markers': True,
                'output_topic': 'compressed_pointcloud',
                'pattern_topic': 'pattern_dictionary'
            }
        ],
        output='screen',
        emulate_tty=True
    )
    
    # Compressed viewer node (start with a small delay to ensure compressor is ready)
    viewer_node = TimerAction(
        period=1.0,  # 1 second delay
        actions=[
            Node(
                package='compressed_viewer',
                executable='compressed_viewer_node',
                name='compressed_viewer_node',
                parameters=[
                    LaunchConfiguration('config_file'),
                    {
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'input_topic': 'compressed_pointcloud',
                        'pattern_dictionary_topic': 'pattern_dictionary',
                        'frame_id': 'map'
                    }
                ],
                output='screen',
                emulate_tty=True
            )
        ]
    )
    
    # RViz node (optional, with delay)
    from launch.conditions import IfCondition
    rviz_node = TimerAction(
        period=2.0,  # 2 second delay
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                condition=IfCondition(LaunchConfiguration('launch_rviz')),
                output='screen'
            )
        ]
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(input_file_arg)
    ld.add_action(launch_rviz_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(config_file_arg)
    
    # Add nodes
    ld.add_action(compressor_node)
    ld.add_action(viewer_node)
    ld.add_action(rviz_node)
    
    return ld