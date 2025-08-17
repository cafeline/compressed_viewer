#!/usr/bin/env python3
"""
Statistics display module for showing compression statistics
"""

import time
from typing import Dict, Tuple, Optional
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point


class StatisticsDisplay:
    """Handles display of compression statistics"""
    
    def __init__(self):
        """Initialize the statistics display"""
        self.statistics = {
            'original_points': 0,
            'reconstructed_points': 0,
            'compressed_size': 0,
            'original_size': 0,
            'compression_ratio': 0.0,
            'unique_patterns': 0,
            'compression_time': 0.0,
            'decompression_time': 0.0,
            'total_blocks': 0,
            'voxel_size': 0.0,
            'block_size': 0,
        }
        
        self.frame_id = 'map'
        self.marker_id_counter = 1000  # Start from high ID to avoid conflicts
        
    def update_statistics(self, compressed_msg, decompression_time: Optional[float] = None):
        """
        Update statistics from compressed message
        
        Args:
            compressed_msg: CompressedPointCloud message
            decompression_time: Optional decompression time in seconds
        """
        self.statistics['original_points'] = compressed_msg.original_point_count
        self.statistics['compressed_size'] = compressed_msg.compressed_data_size
        self.statistics['original_size'] = compressed_msg.original_data_size
        self.statistics['compression_ratio'] = compressed_msg.compression_ratio
        self.statistics['unique_patterns'] = compressed_msg.unique_patterns_count
        self.statistics['compression_time'] = compressed_msg.compression_time_seconds
        self.statistics['total_blocks'] = compressed_msg.total_blocks
        self.statistics['voxel_size'] = compressed_msg.compression_settings.voxel_size
        self.statistics['block_size'] = compressed_msg.compression_settings.block_size
        
        if decompression_time is not None:
            self.statistics['decompression_time'] = decompression_time
            
    def format_statistics(self) -> str:
        """
        Format statistics for display
        
        Returns:
            Formatted string with statistics
        """
        lines = [
            "=== Compression Statistics ===",
            f"Original Points: {self.statistics['original_points']:,}",
            f"Reconstructed Points: {self.statistics['reconstructed_points']:,}",
            f"",
            f"Original Size: {self._format_bytes(self.statistics['original_size'])}",
            f"Compressed Size: {self._format_bytes(self.statistics['compressed_size'])}",
            f"Compression Ratio: {self.statistics['compression_ratio']:.2%}",
            f"Space Saved: {(1 - self.statistics['compression_ratio']):.1%}",
            f"",
            f"Unique Patterns: {self.statistics['unique_patterns']}",
            f"Total Blocks: {self.statistics['total_blocks']}",
            f"Pattern Reuse: {self._calculate_reuse_ratio():.1%}",
            f"",
            f"Voxel Size: {self.statistics['voxel_size']:.3f} m",
            f"Block Size: {self.statistics['block_size']}³ voxels",
            f"",
            f"Compression Time: {self.statistics['compression_time']:.3f} s",
            f"Decompression Time: {self.statistics['decompression_time']:.3f} s",
            f"Total Processing: {self._total_processing_time():.3f} s",
            f"",
            f"Throughput: {self._calculate_throughput():.1f} points/s",
        ]
        
        return "\n".join(lines)
        
    def create_text_marker(self, 
                          text: Optional[str] = None,
                          position: Tuple[float, float, float] = (0, 0, 2),
                          scale: float = 0.1,
                          color: Tuple[float, float, float, float] = (1.0, 1.0, 1.0, 1.0)) -> Marker:
        """
        Create a text marker for RViz display
        
        Args:
            text: Text to display (uses formatted statistics if not provided)
            position: Position of the text marker
            scale: Text size
            color: RGBA color tuple
            
        Returns:
            Marker message for text display
        """
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.ns = "statistics"
        marker.id = self.marker_id_counter
        self.marker_id_counter += 1
        
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        # Set position
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.pose.orientation.w = 1.0
        
        # Set scale (text height)
        marker.scale.z = scale
        
        # Set color
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        
        # Set text
        if text is None:
            marker.text = self.format_statistics()
        else:
            marker.text = text
            
        marker.lifetime.sec = 0  # Persistent
        
        return marker
        
    def create_info_panel_markers(self, 
                                 position: Tuple[float, float, float] = (0, 0, 2)) -> list:
        """
        Create multiple markers for an information panel
        
        Args:
            position: Base position for the panel
            
        Returns:
            List of Marker messages
        """
        markers = []
        
        # Background panel
        panel_marker = Marker()
        panel_marker.header.frame_id = self.frame_id
        panel_marker.ns = "info_panel"
        panel_marker.id = self.marker_id_counter
        self.marker_id_counter += 1
        
        panel_marker.type = Marker.CUBE
        panel_marker.action = Marker.ADD
        
        panel_marker.pose.position.x = position[0]
        panel_marker.pose.position.y = position[1] - 0.5
        panel_marker.pose.position.z = position[2]
        panel_marker.pose.orientation.w = 1.0
        
        panel_marker.scale.x = 0.02
        panel_marker.scale.y = 2.0
        panel_marker.scale.z = 1.5
        
        panel_marker.color.r = 0.1
        panel_marker.color.g = 0.1
        panel_marker.color.b = 0.2
        panel_marker.color.a = 0.8
        
        markers.append(panel_marker)
        
        # Add text marker
        text_marker = self.create_text_marker(
            position=(position[0] + 0.05, position[1], position[2]),
            scale=0.08
        )
        markers.append(text_marker)
        
        # Add performance indicators
        if self.statistics['compression_ratio'] > 0:
            # Compression ratio bar
            bar_marker = self._create_bar_marker(
                value=self.statistics['compression_ratio'],
                position=(position[0] + 0.05, position[1] - 1.2, position[2] - 0.5),
                label="Compression",
                color=self._get_performance_color(self.statistics['compression_ratio'])
            )
            markers.append(bar_marker)
            
        return markers
        
    def _create_bar_marker(self,
                          value: float,
                          position: Tuple[float, float, float],
                          label: str,
                          color: Tuple[float, float, float, float],
                          max_value: float = 1.0) -> Marker:
        """Create a bar graph marker"""
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.ns = f"bar_{label}"
        marker.id = self.marker_id_counter
        self.marker_id_counter += 1
        
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # Scale bar based on value
        bar_width = min(value / max_value, 1.0) * 0.8
        
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1] + bar_width / 2
        marker.pose.position.z = position[2]
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = 0.05
        marker.scale.y = bar_width
        marker.scale.z = 0.1
        
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        
        marker.lifetime.sec = 0
        
        return marker
        
    def _format_bytes(self, bytes_value: int) -> str:
        """Format bytes to human-readable string"""
        for unit in ['B', 'KB', 'MB', 'GB']:
            if bytes_value < 1024.0:
                return f"{bytes_value:.1f} {unit}"
            bytes_value /= 1024.0
        return f"{bytes_value:.1f} TB"
        
    def _calculate_reuse_ratio(self) -> float:
        """Calculate pattern reuse ratio"""
        if self.statistics['total_blocks'] == 0:
            return 0.0
        return 1.0 - (self.statistics['unique_patterns'] / self.statistics['total_blocks'])
        
    def _total_processing_time(self) -> float:
        """Calculate total processing time"""
        return self.statistics['compression_time'] + self.statistics['decompression_time']
        
    def _calculate_throughput(self) -> float:
        """Calculate processing throughput"""
        total_time = self._total_processing_time()
        if total_time == 0:
            return 0.0
        return self.statistics['original_points'] / total_time
        
    def _get_performance_color(self, ratio: float) -> Tuple[float, float, float, float]:
        """Get color based on performance ratio"""
        if ratio < 0.3:  # Excellent compression
            return (0.0, 1.0, 0.0, 1.0)  # Green
        elif ratio < 0.5:  # Good compression
            return (0.5, 1.0, 0.0, 1.0)  # Yellow-green
        elif ratio < 0.7:  # Moderate compression
            return (1.0, 1.0, 0.0, 1.0)  # Yellow
        else:  # Poor compression
            return (1.0, 0.5, 0.0, 1.0)  # Orange
            
    def print_statistics(self):
        """Print statistics to console"""
        print(self.format_statistics())