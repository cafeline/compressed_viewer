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
import gc  # ガベージコレクション

# ROS messages
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Header

# Import compressed point cloud messages from pointcloud_compressor package
# These need to be built first with colcon build
try:
    from pointcloud_compressor.msg import CompressedPointCloud, PatternDictionary
except ImportError:
    # Fallback for development/testing
    print("Warning: pointcloud_compressor messages not found. Using mock message.")
    CompressedPointCloud = None
    PatternDictionary = None

# Local modules
from .decompressor import Decompressor
from .visualizer import PointCloudVisualizer
from .statistics_display import StatisticsDisplay
from .pattern_dictionary_decompressor import PatternDictionaryDecompressor
from .pattern_marker_visualizer import PatternMarkerVisualizer


class CompressedViewerNode(Node):
    """ROS2 node for viewing compressed point clouds"""
    
    def __init__(self):
        """Initialize the compressed viewer node"""
        super().__init__('compressed_viewer_node')
        
        # Publish tracking for memory leak detection
        self.last_pattern_publish_time = 0
        self.pattern_publish_count = 0
        self.cached_patterns = None  # パターンキャッシュ
        self.cached_pattern_hash = None
        
        # Declare parameters
        self.declare_parameter('input_topic', 'compressed_pointcloud')
        self.declare_parameter('pattern_dictionary_topic', 'pattern_dictionary')
        self.declare_parameter('pointcloud_topic', 'decompressed_pointcloud')
        self.declare_parameter('markers_topic', 'visualization_markers')
        self.declare_parameter('pattern_markers_topic', 'pattern_markers')
        self.declare_parameter('statistics_topic', 'statistics_markers')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('visualization_mode', 'points')  # points, voxels, both
        self.declare_parameter('show_statistics', True)
        self.declare_parameter('show_bounding_box', True)
        self.declare_parameter('show_pattern_visualization', True)
        self.declare_parameter('pattern_spacing', 3.0)
        self.declare_parameter('pattern_voxel_size', 0.1)
        self.declare_parameter('point_size', 0.01)
        self.declare_parameter('point_color_r', 0.0)
        self.declare_parameter('point_color_g', 1.0)
        self.declare_parameter('point_color_b', 0.0)
        self.declare_parameter('point_color_a', 1.0)
        self.declare_parameter('memory_cleanup_interval', 10)  # ガベージコレクション間隔
        
        # Get parameters
        self.input_topic = self.get_parameter('input_topic').value
        self.pattern_dictionary_topic = self.get_parameter('pattern_dictionary_topic').value
        self.pointcloud_topic = self.get_parameter('pointcloud_topic').value
        self.markers_topic = self.get_parameter('markers_topic').value
        self.pattern_markers_topic = self.get_parameter('pattern_markers_topic').value
        self.statistics_topic = self.get_parameter('statistics_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.visualization_mode = self.get_parameter('visualization_mode').value
        self.show_statistics = self.get_parameter('show_statistics').value
        self.show_bounding_box = self.get_parameter('show_bounding_box').value
        self.show_pattern_visualization = self.get_parameter('show_pattern_visualization').value
        self.pattern_spacing = self.get_parameter('pattern_spacing').value
        self.pattern_voxel_size = self.get_parameter('pattern_voxel_size').value
        self.point_size = self.get_parameter('point_size').value
        self.memory_cleanup_interval = self.get_parameter('memory_cleanup_interval').value
        
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
        self.pattern_decompressor = PatternDictionaryDecompressor()
        self.pattern_visualizer = PatternMarkerVisualizer(frame_id=self.frame_id)
        
        # QoS settings
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create subscribers
        if CompressedPointCloud is not None:
            self.compressed_sub = self.create_subscription(
                CompressedPointCloud,
                self.input_topic,
                self.compressed_callback,
                qos
            )
        else:
            self.get_logger().warn("CompressedPointCloud message not available")
            
        if PatternDictionary is not None:
            self.pattern_sub = self.create_subscription(
                PatternDictionary,
                self.pattern_dictionary_topic,
                self.pattern_dictionary_callback,
                qos
            )
        else:
            self.get_logger().warn("PatternDictionary message not available")
        
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
        
        self.pattern_markers_pub = self.create_publisher(
            MarkerArray,
            self.pattern_markers_topic,
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
        
        # Memory cleanup timer
        self.create_timer(self.memory_cleanup_interval, self.cleanup_memory)
        
        self.get_logger().info(f"Compressed Viewer Node initialized")
        self.get_logger().info(f"Subscribing to: {self.input_topic}, {self.pattern_dictionary_topic}")
        self.get_logger().info(f"Publishing to: {self.pointcloud_topic}, {self.markers_topic}, {self.pattern_markers_topic}")
        self.get_logger().info(f"Visualization mode: {self.visualization_mode}")
        self.get_logger().info(f"Pattern visualization: {self.show_pattern_visualization}")
        
    def cleanup_memory(self):
        """定期的にメモリをクリーンアップ"""
        gc.collect()
        self.get_logger().debug(f"Memory cleanup performed (message count: {self.message_count})")
        
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
                
            # Process PatternDictionary from the compressed message for pattern visualization
            if self.show_pattern_visualization and hasattr(msg, 'pattern_dictionary'):
                try:
                    self.get_logger().info("Processing PatternDictionary from compressed message")
                    # Use the original voxel size from compression settings for accurate visualization
                    original_voxel_size = msg.compression_settings.voxel_size
                    self.get_logger().info(f"Using original voxel size: {original_voxel_size}")
                    # Pass full compressed message for spatial positioning
                    self.process_pattern_dictionary(msg.pattern_dictionary, 
                                                   voxel_size=original_voxel_size,
                                                   compressed_msg=msg)
                except Exception as e:
                    self.get_logger().error(f"Error processing PatternDictionary: {str(e)}")
                
        except Exception as e:
            self.get_logger().error(f"Error processing compressed message: {str(e)}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            
        finally:
            # メモリ解放を促進
            if self.message_count % 10 == 0:
                gc.collect()
            
    def pattern_dictionary_callback(self, msg: 'PatternDictionary'):
        """
        Callback for PatternDictionary messages
        
        Args:
            msg: PatternDictionary message
        """
        # Use fallback voxel size for standalone PatternDictionary messages
        fallback_voxel_size = self.pattern_voxel_size
        self.process_pattern_dictionary(msg, voxel_size=fallback_voxel_size)
        
    def process_pattern_dictionary(self, pattern_dict, voxel_size=None, compressed_msg=None):
        """
        Process PatternDictionary data for visualization
        
        Args:
            pattern_dict: PatternDictionary message or data
            voxel_size: Voxel size to use for visualization (if None, uses parameter)
            compressed_msg: Full CompressedPointCloud message (for spatial positioning)
        """
        if not self.show_pattern_visualization:
            return
            
        # Use provided voxel_size or fallback to parameter
        actual_voxel_size = voxel_size if voxel_size is not None else self.pattern_voxel_size
        
        self.get_logger().info(f"Processing PatternDictionary with {pattern_dict.num_patterns} patterns")
        self.get_logger().info(f"Using voxel size: {actual_voxel_size} for pattern visualization")
        
        try:
            # Start timing
            start_time = time.time()
            
            # パターンのハッシュを計算してキャッシュを確認
            pattern_hash = hash(bytes(pattern_dict.dictionary_data))
            
            if self.cached_pattern_hash == pattern_hash and self.cached_patterns is not None:
                self.get_logger().info("Using cached patterns")
                patterns = self.cached_patterns
            else:
                # Decompress pattern dictionary into 3D boolean arrays
                patterns = self.pattern_decompressor.decompress_pattern_dictionary(
                    pattern_dict, 
                    validate_checksum=False  # Skip checksum for now as it may not be properly set
                )
                # キャッシュを更新
                self.cached_patterns = patterns
                self.cached_pattern_hash = pattern_hash
            
            # Calculate decompression time
            decompression_time = time.time() - start_time
            
            self.get_logger().info(
                f"Decompressed {len(patterns)} patterns in {decompression_time:.3f}s"
            )
            
            if len(patterns) > 0:
                # Check if we have spatial information from CompressedPointCloud
                use_spatial = False
                if compressed_msg and hasattr(compressed_msg, 'block_indices') and len(compressed_msg.block_indices) > 0:
                    # Check if block dimensions are valid
                    if compressed_msg.blocks_x > 0 and compressed_msg.blocks_y > 0 and compressed_msg.blocks_z > 0:
                        use_spatial = True
                        self.get_logger().info("Using spatial pattern visualization with actual block positions")
                        self.get_logger().info(f"Block indices count: {len(compressed_msg.block_indices)}")
                    else:
                        self.get_logger().warn(f"Invalid block dimensions: {compressed_msg.blocks_x}x{compressed_msg.blocks_y}x{compressed_msg.blocks_z}")
                
                if use_spatial:
                    # Get block dimensions and origin
                    blocks_dims = (compressed_msg.blocks_x, compressed_msg.blocks_y, compressed_msg.blocks_z)
                    grid_origin = (
                        compressed_msg.voxel_grid_origin.x,
                        compressed_msg.voxel_grid_origin.y,
                        compressed_msg.voxel_grid_origin.z
                    )
                    block_size = compressed_msg.compression_settings.block_size
                    
                    # Create spatial pattern markers
                    pattern_markers = self.pattern_visualizer.create_spatial_pattern_markers(
                        patterns,
                        compressed_msg.block_indices,
                        blocks_dims,
                        actual_voxel_size,
                        block_size,
                        grid_origin
                    )
                    
                    self.get_logger().info(f"Created {len(pattern_markers.markers)} spatial pattern markers")
                    
                    # Create info text
                    info_text = f"Spatial Pattern Visualization\n"
                    info_text += f"Total blocks: {len(compressed_msg.block_indices)}\n"
                    info_text += f"Block dimensions: {blocks_dims[0]}x{blocks_dims[1]}x{blocks_dims[2]}\n"
                    info_text += f"Unique patterns: {len(patterns)}\n"
                    info_text += f"Voxel size: {actual_voxel_size:.3f}m\n"
                    info_text += f"Block size: {block_size}x{block_size}x{block_size} voxels\n"
                    
                    # Check if we actually created markers
                    if len(pattern_markers.markers) == 0:
                        self.get_logger().warn("No spatial markers created, falling back to linear display")
                        use_spatial = False
                    
                if not use_spatial:
                    # Fallback to linear pattern display (for standalone PatternDictionary messages)
                    self.get_logger().info("Using linear pattern visualization")
                    
                    # Create pattern markers (limit to first 20 patterns for performance)
                    display_patterns = patterns[:20]
                    pattern_markers = self.pattern_visualizer.create_pattern_markers(
                        display_patterns,
                        pattern_spacing=self.pattern_spacing,
                        voxel_size=actual_voxel_size
                    )
                    
                    # Create info text
                    info_text = f"Pattern Dictionary: {len(patterns)} patterns (showing first {len(display_patterns)})\n"
                    info_text += f"Voxel size: {actual_voxel_size:.3f}m\n"
                    info_text += f"Block size: {display_patterns[0].shape[0] if display_patterns else 'N/A'}\n"
                    for i, pattern in enumerate(display_patterns):
                        voxel_count = int(np.sum(pattern))
                        total_voxels = pattern.size
                        density = (voxel_count / total_voxels) * 100 if total_voxels > 0 else 0
                        info_text += f"Pattern {i}: {voxel_count}/{total_voxels} voxels ({density:.1f}%)\n"
                
                # Create and publish info markers
                info_markers = self.pattern_visualizer.create_info_markers(
                    patterns[:20],  # Just for positioning
                    info_text,
                    position=(0.0, -2.0, 2.0)
                )
                
                # Combine pattern markers and info markers
                combined_markers = MarkerArray()
                if pattern_markers and len(pattern_markers.markers) > 0:
                    combined_markers.markers.extend(pattern_markers.markers)
                if info_markers and len(info_markers.markers) > 0:
                    combined_markers.markers.extend(info_markers.markers)
                
                if len(combined_markers.markers) > 0:
                    # 重複publish防止チェック
                    current_time = time.time()
                    time_since_last = current_time - self.last_pattern_publish_time
                    
                    # 最小間隔チェック（0.1秒未満の場合はスキップ）
                    if time_since_last < 0.1:
                        self.get_logger().warn(f"Skipping pattern publish (too frequent: {time_since_last:.3f}s)")
                        return
                    
                    self.pattern_markers_pub.publish(combined_markers)
                    self.last_pattern_publish_time = current_time
                    self.pattern_publish_count += 1
                    
                    # 日本語でpublish確認メッセージ
                    print("="*50)
                    print("🔵 pattern_markersをpublishしました")
                    print(f"  マーカー数: {len(pattern_markers.markers)}")
                    print(f"  タイプ: {'空間配置' if use_spatial else '線形配置'}")
                    print(f"  累計publish回数: {self.pattern_publish_count}")
                    print(f"  前回からの経過時間: {time_since_last:.2f}秒")
                    print("="*50)
                    
                    if use_spatial:
                        self.get_logger().info(f"Published {len(pattern_markers.markers)} spatial pattern markers for {len(compressed_msg.block_indices)} blocks")
                    else:
                        self.get_logger().info(f"Published {len(pattern_markers.markers)} linear pattern markers")
                else:
                    self.get_logger().warn("No markers to publish")
                
            else:
                self.get_logger().warn("No patterns found in PatternDictionary")
                
        except Exception as e:
            self.get_logger().error(f"Error processing PatternDictionary: {str(e)}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            
    def clear_visualization(self):
        """Clear all visualization markers"""
        clear_markers = self.visualizer.clear_markers()
        self.markers_pub.publish(clear_markers)
        
        # Clear pattern markers
        pattern_clear_markers = self.pattern_visualizer.clear_markers()
        self.pattern_markers_pub.publish(pattern_clear_markers)
        
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