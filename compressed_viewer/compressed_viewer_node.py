#!/usr/bin/env python3
"""
Refactored compressed viewer node - cleaner and more efficient
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time
import numpy as np

# ROS messages
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Header

# Import messages
try:
    from pointcloud_compressor.msg import PatternDictionary
except ImportError:
    print("Warning: pointcloud_compressor messages not found")
    PatternDictionary = None

# Local modules
from .pattern_dictionary_decompressor import PatternDictionaryDecompressor
from .pattern_marker_visualizer import PatternMarkerVisualizer


class CompressedViewerNode(Node):
    """Refactored compressed viewer node for pattern visualization"""
    
    def __init__(self):
        """Initialize the compressed viewer node"""
        super().__init__('compressed_viewer_node')
        
        # Declare parameters
        self._declare_parameters()
        
        # Get parameters
        self._get_parameters()
        
        # Initialize components
        self.pattern_decompressor = PatternDictionaryDecompressor()
        self.pattern_visualizer = PatternMarkerVisualizer(frame_id=self.frame_id)
        
        # State management
        self.cached_patterns = None
        self.cached_pattern_hash = None
        self.last_publish_time = 0
        self.min_publish_interval = 0.1  # Minimum 100ms between publishes
        
        # Setup ROS communication
        self._setup_ros_communication()
        
        self.get_logger().info("Compressed Viewer Node initialized")
        self.get_logger().info(f"Subscribing to: {self.pattern_dictionary_topic}")
        self.get_logger().info(f"Publishing to: {self.pattern_markers_topic}")
        
    def _declare_parameters(self):
        """Declare all ROS parameters"""
        self.declare_parameter('pattern_dictionary_topic', 'pattern_dictionary')
        self.declare_parameter('pattern_markers_topic', 'pattern_markers')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('pattern_voxel_size', 0.1)
        self.declare_parameter('show_pattern_visualization', True)
        
    def _get_parameters(self):
        """Get all parameter values"""
        self.pattern_dictionary_topic = self.get_parameter('pattern_dictionary_topic').value
        self.pattern_markers_topic = self.get_parameter('pattern_markers_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.pattern_voxel_size = self.get_parameter('pattern_voxel_size').value
        self.show_pattern_visualization = self.get_parameter('show_pattern_visualization').value
        
    def _setup_ros_communication(self):
        """Setup ROS subscribers and publishers"""
        # QoS settings
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create subscriber
        if PatternDictionary is not None:
            self.pattern_sub = self.create_subscription(
                PatternDictionary,
                self.pattern_dictionary_topic,
                self.pattern_dictionary_callback,
                qos
            )
        else:
            self.get_logger().error("PatternDictionary message not available")
            
        # Create publisher
        self.pattern_markers_pub = self.create_publisher(
            MarkerArray,
            self.pattern_markers_topic,
            qos
        )
        
    def pattern_dictionary_callback(self, msg: 'PatternDictionary'):
        """
        Process PatternDictionary messages
        
        Args:
            msg: PatternDictionary message
        """
        if not self.show_pattern_visualization:
            return
            
        try:
            # Rate limiting
            current_time = time.time()
            if current_time - self.last_publish_time < self.min_publish_interval:
                return
                
            # Process the pattern dictionary
            marker_array = self._process_pattern_dictionary(msg)
            
            # Publish if we have markers
            if marker_array and len(marker_array.markers) > 0:
                self.pattern_markers_pub.publish(marker_array)
                self.last_publish_time = current_time
                
                # Log marker information
                self._log_marker_info(marker_array)
                
        except Exception as e:
            self.get_logger().error(f"Error in pattern_dictionary_callback: {str(e)}")
            
    def _process_pattern_dictionary(self, msg: 'PatternDictionary') -> MarkerArray:
        """
        Process PatternDictionary and create markers
        
        Args:
            msg: PatternDictionary message
            
        Returns:
            MarkerArray with pattern markers
        """
        # Determine voxel size
        voxel_size = self._get_voxel_size(msg)
        
        self.get_logger().info(f"Processing {msg.num_patterns} patterns with voxel_size={voxel_size:.3f}")
        
        # Get patterns (from cache if available)
        patterns = self._get_patterns(msg)
        
        if not patterns:
            self.get_logger().warn("No patterns to process")
            return MarkerArray()
            
        # Create spatial markers if spatial information is available
        if self._has_spatial_info(msg):
            return self._create_spatial_markers(msg, patterns, voxel_size)
        else:
            self.get_logger().warn("No spatial information available")
            return MarkerArray()
            
    def _get_voxel_size(self, msg: 'PatternDictionary') -> float:
        """Get voxel size from message or use default"""
        if hasattr(msg, 'voxel_size') and msg.voxel_size > 0:
            return msg.voxel_size
        return self.pattern_voxel_size
        
    def _get_patterns(self, msg: 'PatternDictionary'):
        """Get patterns from cache or decompress"""
        pattern_hash = hash(bytes(msg.dictionary_data))
        
        if self.cached_pattern_hash == pattern_hash and self.cached_patterns is not None:
            self.get_logger().debug("Using cached patterns")
            return self.cached_patterns
            
        # Decompress patterns
        patterns = self.pattern_decompressor.decompress_pattern_dictionary(
            msg, 
            validate_checksum=False
        )
        
        # Update cache
        self.cached_patterns = patterns
        self.cached_pattern_hash = pattern_hash
        
        return patterns
        
    def _has_spatial_info(self, msg: 'PatternDictionary') -> bool:
        """Check if message has valid spatial information"""
        return (hasattr(msg, 'blocks_x') and 
                msg.blocks_x > 0 and 
                msg.blocks_y > 0 and 
                msg.blocks_z > 0 and
                len(msg.block_indices) > 0)
                
    def _create_spatial_markers(self, msg: 'PatternDictionary', patterns, voxel_size: float) -> MarkerArray:
        """Create spatial pattern markers"""
        blocks_dims = (msg.blocks_x, msg.blocks_y, msg.blocks_z)
        grid_origin = (
            msg.voxel_grid_origin.x,
            msg.voxel_grid_origin.y,
            msg.voxel_grid_origin.z
        )
        
        return self.pattern_visualizer.create_spatial_pattern_markers(
            patterns,
            msg.block_indices,
            blocks_dims,
            voxel_size,
            msg.block_size,
            grid_origin
        )
        
    def _log_marker_info(self, marker_array: MarkerArray):
        """Log information about published markers"""
        # Count marker types
        marker_types = {}
        for marker in marker_array.markers:
            mtype = marker.type
            marker_types[mtype] = marker_types.get(mtype, 0) + 1
            
        self.get_logger().info(f"Published {len(marker_array.markers)} markers")
        if marker_types:
            self.get_logger().debug(f"Marker types: {marker_types}")


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
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()