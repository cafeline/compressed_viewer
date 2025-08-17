#!/usr/bin/env python3
"""
PatternMarkerVisualizer module for creating MarkerArray visualizations from pattern data
"""

import numpy as np
from typing import List, Tuple, Optional
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA, Header


class PatternMarkerVisualizer:
    """Creates MarkerArray visualizations for pattern dictionary data"""
    
    def __init__(self, frame_id: str = 'map'):
        """
        Initialize the PatternMarkerVisualizer
        
        Args:
            frame_id: Frame ID for the markers
        """
        self.frame_id = frame_id
        
    def create_pattern_markers(self, 
                             patterns: List[np.ndarray],
                             pattern_spacing: float = 3.0,
                             voxel_size: float = 0.1,
                             colors: Optional[List[Tuple[float, float, float, float]]] = None) -> MarkerArray:
        """
        Create MarkerArray from a list of pattern blocks
        
        Args:
            patterns: List of 3D boolean arrays representing patterns
            pattern_spacing: Spacing between patterns in meters
            voxel_size: Size of each voxel in meters
            colors: Optional list of (r, g, b, a) tuples for pattern colors
            
        Returns:
            MarkerArray message containing visualization markers
        """
        marker_array = MarkerArray()
        
        if not patterns:
            return marker_array
            
        for i, pattern in enumerate(patterns):
            if not self._validate_pattern(pattern):
                continue
                
            # Calculate pattern offset
            pattern_offset = (i * pattern_spacing, 0.0, 0.0)
            
            # Get voxel positions for this pattern (relative to pattern origin)
            voxel_positions = self._create_voxel_positions(pattern, voxel_size, (0.0, 0.0, 0.0))
            
            if not voxel_positions:
                continue
                
            # Create marker for this pattern
            marker = Marker()
            marker.header = Header()
            marker.header.frame_id = self.frame_id
            marker.id = i
            marker.type = Marker.CUBE_LIST
            marker.action = Marker.ADD
            
            # Set marker pose (position offset)
            marker.pose.position.x = pattern_offset[0]
            marker.pose.position.y = pattern_offset[1]
            marker.pose.position.z = pattern_offset[2]
            marker.pose.orientation.w = 1.0
            
            # Set marker scale (voxel size)
            marker.scale = Vector3()
            marker.scale.x = voxel_size
            marker.scale.y = voxel_size
            marker.scale.z = voxel_size
            
            # Set marker color
            if colors and i < len(colors):
                marker.color = ColorRGBA()
                marker.color.r = colors[i][0]
                marker.color.g = colors[i][1]
                marker.color.b = colors[i][2]
                marker.color.a = colors[i][3]
            else:
                default_color = self._get_default_color(i)
                marker.color = ColorRGBA()
                marker.color.r = default_color[0]
                marker.color.g = default_color[1]
                marker.color.b = default_color[2]
                marker.color.a = default_color[3]
            
            # Add voxel positions as points
            marker.points = []
            for pos in voxel_positions:
                point = Point()
                point.x = pos[0]
                point.y = pos[1]
                point.z = pos[2]
                marker.points.append(point)
                
            marker_array.markers.append(marker)
            
        return marker_array
        
    def create_info_markers(self, 
                          patterns: List[np.ndarray],
                          info_text: str,
                          position: Tuple[float, float, float] = (0.0, 0.0, 2.0)) -> MarkerArray:
        """
        Create information text markers for pattern display
        
        Args:
            patterns: List of patterns (used for positioning)
            info_text: Text to display
            position: Position for the text marker
            
        Returns:
            MarkerArray containing text marker
        """
        marker_array = MarkerArray()
        
        text_marker = Marker()
        text_marker.header = Header()
        text_marker.header.frame_id = self.frame_id
        text_marker.id = 999999  # High ID to avoid conflicts
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        
        text_marker.pose.position.x = position[0]
        text_marker.pose.position.y = position[1]
        text_marker.pose.position.z = position[2]
        text_marker.pose.orientation.w = 1.0
        
        text_marker.scale.z = 0.2  # Text height
        
        text_marker.color = ColorRGBA()
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        
        text_marker.text = info_text
        
        marker_array.markers.append(text_marker)
        
        return marker_array
        
    def clear_markers(self) -> MarkerArray:
        """
        Create MarkerArray to clear all markers
        
        Returns:
            MarkerArray with DELETE_ALL action
        """
        marker_array = MarkerArray()
        
        clear_marker = Marker()
        clear_marker.header = Header()
        clear_marker.header.frame_id = self.frame_id
        clear_marker.action = Marker.DELETEALL
        
        marker_array.markers.append(clear_marker)
        
        return marker_array
        
    def _create_voxel_positions(self, 
                              pattern: np.ndarray, 
                              voxel_size: float,
                              offset: Tuple[float, float, float]) -> List[Tuple[float, float, float]]:
        """
        Calculate world positions for occupied voxels in a pattern
        
        Args:
            pattern: 3D boolean array
            voxel_size: Size of each voxel
            offset: Offset for this pattern
            
        Returns:
            List of (x, y, z) positions for occupied voxels
        """
        positions = []
        
        # Find occupied voxels
        occupied = np.where(pattern)
        
        if len(occupied[0]) == 0:
            return positions
            
        # Calculate world positions (voxel centers)
        for i in range(len(occupied[0])):
            x = offset[0] + (occupied[0][i] + 0.5) * voxel_size
            y = offset[1] + (occupied[1][i] + 0.5) * voxel_size
            z = offset[2] + (occupied[2][i] + 0.5) * voxel_size
            positions.append((x, y, z))
            
        return positions
        
    def _get_default_color(self, pattern_index: int) -> Tuple[float, float, float, float]:
        """
        Generate default color for a pattern based on its index
        
        Args:
            pattern_index: Index of the pattern
            
        Returns:
            (r, g, b, a) color tuple
        """
        # Define a set of distinct colors
        default_colors = [
            (0.0, 1.0, 0.0, 1.0),  # Green
            (0.0, 0.0, 1.0, 1.0),  # Blue
            (1.0, 1.0, 0.0, 1.0),  # Yellow
            (1.0, 0.0, 1.0, 1.0),  # Magenta
            (0.0, 1.0, 1.0, 1.0),  # Cyan
            (1.0, 0.5, 0.0, 1.0),  # Orange
            (0.5, 0.0, 1.0, 1.0),  # Purple
            (0.0, 0.5, 0.0, 1.0),  # Dark Green
        ]
        
        return default_colors[pattern_index % len(default_colors)]
        
    def _validate_pattern(self, pattern: np.ndarray) -> bool:
        """
        Validate that a pattern is a 3D boolean array
        
        Args:
            pattern: Array to validate
            
        Returns:
            True if valid, False otherwise
        """
        if not isinstance(pattern, np.ndarray):
            return False
            
        if pattern.ndim != 3:
            return False
            
        if pattern.dtype != bool:
            return False
            
        return True