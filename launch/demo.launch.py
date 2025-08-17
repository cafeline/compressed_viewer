#!/usr/bin/env python3
"""
Demo launch file that starts both the pointcloud compressor and viewer
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def get_input_file_value(context):
    """Helper function to determine the input file from parameters"""
    # Check if pcd_file is provided (for backward compatibility)
    pcd_file = LaunchConfiguration('pcd_file').perform(context)
    input_file = LaunchConfiguration('input_file').perform(context)
    
    # Use pcd_file if provided and input_file is default
    if pcd_file and input_file == '/tmp/sample.pcd':
        return pcd_file
    return input_file

def generate_launch_description():
    """Generate launch description for demo"""
    
    # Get package directories
    compressor_pkg_dir = get_package_share_directory('pointcloud_compressor')
    viewer_pkg_dir = get_package_share_directory('compressed_viewer')
    
    # Configuration files
    compressor_config = os.path.join(compressor_pkg_dir, 'config', 'pointcloud_compressor_params.yaml')
    viewer_config = os.path.join(viewer_pkg_dir, 'config', 'viewer_params.yaml')
    demo_config = os.path.join(viewer_pkg_dir, 'config', 'demo_params.yaml')
    rviz_config = os.path.join(viewer_pkg_dir, 'rviz', 'compressed_viewer.rviz')
    
    # Declare launch arguments
    input_file_arg = DeclareLaunchArgument(
        'input_file',
        default_value='/tmp/sample.pcd',
        description='Path to the point cloud file to compress (PCD or PLY format)'
    )
    
    # Keep backward compatibility
    pcd_file_arg = DeclareLaunchArgument(
        'pcd_file',
        default_value='',
        description='[DEPRECATED] Use input_file instead. Path to the PCD file to compress'
    )
    
    voxel_size_arg = DeclareLaunchArgument(
        'voxel_size',
        default_value='0.01',
        description='Voxel size for compression'
    )
    
    block_size_arg = DeclareLaunchArgument(
        'block_size',
        default_value='4',
        description='Block size for compression'
    )
    
    target_patterns_arg = DeclareLaunchArgument(
        'target_patterns',
        default_value='256',
        description='Target number of patterns in dictionary'
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
    
    # Pointcloud compressor node (using OpaqueFunction for conditional input_file)
    def create_compressor_node(context):
        # Determine input file (pcd_file takes precedence for backward compatibility)
        pcd_file = LaunchConfiguration('pcd_file').perform(context)
        input_file = LaunchConfiguration('input_file').perform(context)
        
        # Only override input_file if explicitly provided via command line
        override_params = {'use_sim_time': LaunchConfiguration('use_sim_time')}
        
        # Check if input_file was changed from default
        if pcd_file:
            override_params['input_file'] = pcd_file
        elif input_file != '/tmp/sample.pcd':  # Only override if not default
            override_params['input_file'] = input_file
            
        # Only override compression parameters if they were explicitly set
        voxel_size = LaunchConfiguration('voxel_size').perform(context)
        block_size = LaunchConfiguration('block_size').perform(context)
        target_patterns = LaunchConfiguration('target_patterns').perform(context)
        
        if voxel_size != '0.01':  # Default value
            override_params['voxel_size'] = LaunchConfiguration('voxel_size')
        if block_size != '4':  # Default value
            override_params['block_size'] = LaunchConfiguration('block_size')
        if target_patterns != '256':  # Default value
            override_params['target_patterns'] = LaunchConfiguration('target_patterns')
            
        # Always set these publishing parameters
        override_params.update({
            'publish_once': True,
            'publish_patterns': True,
            'output_topic': 'compressed_pointcloud',
            'pattern_topic': 'pattern_dictionary'
        })
        
        return [Node(
            package='pointcloud_compressor',
            executable='pointcloud_compressor_node',
            name='pointcloud_compressor_node',
            parameters=[
                LaunchConfiguration('config_file'),
                override_params
            ],
            output='screen',
            emulate_tty=True
        )]
    
    compressor_node = OpaqueFunction(function=create_compressor_node)
    
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
    
    # Static transform publisher for the map frame
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(input_file_arg)
    ld.add_action(pcd_file_arg)  # Keep for backward compatibility
    ld.add_action(voxel_size_arg)
    ld.add_action(block_size_arg)
    ld.add_action(target_patterns_arg)
    ld.add_action(launch_rviz_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(config_file_arg)
    
    # Add nodes
    ld.add_action(static_tf_node)
    ld.add_action(compressor_node)
    ld.add_action(viewer_node)
    ld.add_action(rviz_node)
    
    return ld