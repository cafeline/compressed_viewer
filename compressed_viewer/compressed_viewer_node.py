#!/usr/bin/env python3
"""
Main node for compressed point cloud viewer
Subscribes to compressed point cloud data and publishes visualization
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time
import numpy as np

# ROS messages
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Header

# Import compressed point cloud messages from pointcloud_compressor package
# These need to be built first with colcon build
try:
    from pointcloud_compressor.msg import CompressedPointCloud
except ImportError:
    # Fallback for development/testing
    print("Warning: pointcloud_compressor messages not found. Using mock message.")
    CompressedPointCloud = None

# Local modules
from .decompressor import Decompressor
from .visualizer import PointCloudVisualizer
from .statistics_display import StatisticsDisplay


class CompressedViewerNode(Node):
    """ROS2 node for viewing compressed point clouds"""
    
    def __init__(self):
        """Initialize the compressed viewer node"""
        super().__init__('compressed_viewer_node')
        
        # Declare parameters
        self.declare_parameter('input_topic', 'compressed_pointcloud')
        self.declare_parameter('pointcloud_topic', 'decompressed_pointcloud')
        self.declare_parameter('markers_topic', 'visualization_markers')
        self.declare_parameter('statistics_topic', 'statistics_markers')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('visualization_mode', 'points')  # points, voxels, both
        self.declare_parameter('show_statistics', True)
        self.declare_parameter('show_bounding_box', True)
        self.declare_parameter('point_size', 0.01)
        self.declare_parameter('point_color_r', 0.0)
        self.declare_parameter('point_color_g', 1.0)
        self.declare_parameter('point_color_b', 0.0)
        self.declare_parameter('point_color_a', 1.0)
        
        # Get parameters
        self.input_topic = self.get_parameter('input_topic').value
        self.pointcloud_topic = self.get_parameter('pointcloud_topic').value
        self.markers_topic = self.get_parameter('markers_topic').value
        self.statistics_topic = self.get_parameter('statistics_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.visualization_mode = self.get_parameter('visualization_mode').value
        self.show_statistics = self.get_parameter('show_statistics').value
        self.show_bounding_box = self.get_parameter('show_bounding_box').value
        self.point_size = self.get_parameter('point_size').value
        
        point_color = (
            self.get_parameter('point_color_r').value,
            self.get_parameter('point_color_g').value,
            self.get_parameter('point_color_b').value,
            self.get_parameter('point_color_a').value
        )
        
        # Initialize components
        self.decompressor = Decompressor()
        self.visualizer = PointCloudVisualizer(frame_id=self.frame_id)
        self.statistics_display = StatisticsDisplay()
        self.statistics_display.frame_id = self.frame_id
        
        # QoS settings
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create subscriber
        if CompressedPointCloud is not None:
            self.compressed_sub = self.create_subscription(
                CompressedPointCloud,
                self.input_topic,
                self.compressed_callback,
                qos
            )
        else:
            self.get_logger().warn("CompressedPointCloud message not available")
        
        # Create publishers
        self.pointcloud_pub = self.create_publisher(
            PointCloud2,
            self.pointcloud_topic,
            qos
        )
        
        self.markers_pub = self.create_publisher(
            MarkerArray,
            self.markers_topic,
            qos
        )
        
        self.statistics_pub = self.create_publisher(
            MarkerArray,
            self.statistics_topic,
            qos
        )
        
        # Statistics
        self.message_count = 0
        self.total_decompression_time = 0.0
        self.point_color = point_color
        
        self.get_logger().info(f"Compressed Viewer Node initialized")
        self.get_logger().info(f"Subscribing to: {self.input_topic}")
        self.get_logger().info(f"Publishing to: {self.pointcloud_topic}, {self.markers_topic}")
        self.get_logger().info(f"Visualization mode: {self.visualization_mode}")
        
    def compressed_callback(self, msg: 'CompressedPointCloud'):
        """
        Callback for compressed point cloud messages
        
        Args:
            msg: CompressedPointCloud message
        """
        self.message_count += 1
        self.get_logger().info(f"Received compressed message #{self.message_count}")
        
        try:
            # Start timing
            start_time = time.time()
            
            # Debug: Print message details
            self.get_logger().info(f"Block indices count: {len(msg.block_indices)}")
            self.get_logger().info(f"Pattern dictionary patterns: {msg.pattern_dictionary.num_patterns}")
            self.get_logger().info(f"Original points: {msg.original_point_count}")
            self.get_logger().info(f"Grid dimensions: {msg.voxel_grid_dimensions.x}x{msg.voxel_grid_dimensions.y}x{msg.voxel_grid_dimensions.z}")
            
            # Decompress the point cloud
            points = self.decompressor.decompress(msg)
            
            # Calculate decompression time
            decompression_time = time.time() - start_time
            self.total_decompression_time += decompression_time
            
            # Update statistics
            self.statistics_display.update_statistics(msg, decompression_time)
            self.statistics_display.statistics['reconstructed_points'] = len(points)
            
            self.get_logger().info(
                f"Decompressed {len(points)} points in {decompression_time:.3f}s"
            )
            
            # Create header
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = self.frame_id
            
            # Publish point cloud
            if len(points) > 0:
                self.get_logger().info(f"Creating PointCloud2 message with {len(points)} points")
                pointcloud_msg = self.visualizer.create_point_cloud2(points, header)
                self.get_logger().info(f"PointCloud2 message created: width={pointcloud_msg.width}, height={pointcloud_msg.height}, data_size={len(pointcloud_msg.data)}")
                self.pointcloud_pub.publish(pointcloud_msg)
                self.get_logger().info(f"Published PointCloud2 to {self.pointcloud_topic}")
                
                # Create and publish markers based on visualization mode
                marker_array = MarkerArray()
                
                if self.visualization_mode in ['points', 'both']:
                    points_markers = self.visualizer.create_marker_array(
                        points,
                        color=self.point_color,
                        size=self.point_size,
                        marker_type='cube'
                    )
                    marker_array.markers.extend(points_markers.markers)
                    
                if self.visualization_mode in ['voxels', 'both']:
                    # Also show voxel grid if requested
                    # This would require keeping the voxel grid from decompression
                    pass
                    
                if self.show_bounding_box and len(points) > 0:
                    # Calculate bounding box
                    min_bound = np.min(points, axis=0)
                    max_bound = np.max(points, axis=0)
                    
                    bbox_marker = self.visualizer.create_bounding_box(
                        tuple(min_bound),
                        tuple(max_bound),
                        color=(1.0, 1.0, 0.0, 0.5)
                    )
                    marker_array.markers.append(bbox_marker)
                    
                if len(marker_array.markers) > 0:
                    self.markers_pub.publish(marker_array)
                    
            # Publish statistics if enabled
            if self.show_statistics:
                stats_markers = MarkerArray()
                
                # Create info panel
                panel_markers = self.statistics_display.create_info_panel_markers(
                    position=(2.0, 0.0, 1.5)
                )
                stats_markers.markers.extend(panel_markers)
                
                self.statistics_pub.publish(stats_markers)
                
                # Print statistics to console
                self.statistics_display.print_statistics()
                
        except Exception as e:
            self.get_logger().error(f"Error processing compressed message: {str(e)}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            
    def clear_visualization(self):
        """Clear all visualization markers"""
        clear_markers = self.visualizer.clear_markers()
        self.markers_pub.publish(clear_markers)
        
        stats_clear = MarkerArray()
        stats_clear.markers = clear_markers.markers.copy()
        self.statistics_pub.publish(stats_clear)


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        node = CompressedViewerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()