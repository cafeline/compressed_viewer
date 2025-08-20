#!/usr/bin/env python3
"""
PatternMarkerVisualizer module for creating MarkerArray visualizations from pattern data
"""

import numpy as np
from typing import List, Tuple, Optional, Dict
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
        
        marker_id = 0
        for i, pattern in enumerate(patterns):
            if not self._validate_pattern(pattern):
                continue
                
            # Calculate pattern offset
            pattern_offset = (i * pattern_spacing, 0.0, 0.0)
            
            # Get voxel positions for this pattern (relative to pattern origin)
            voxel_positions = self._create_voxel_positions(pattern, voxel_size, (0.0, 0.0, 0.0))
            
            if not voxel_positions:
                continue
                
            # Create individual CUBE markers for each voxel (matching occupied_voxel_markers format)
            for voxel_pos in voxel_positions:
                marker = Marker()
                marker.header = Header()
                marker.header.frame_id = self.frame_id
                marker.id = marker_id
                marker.type = Marker.CUBE  # Individual CUBE instead of CUBE_LIST
                marker.action = Marker.ADD
                
                # Set marker position (pattern offset + voxel position)
                marker.pose.position.x = pattern_offset[0] + voxel_pos[0]
                marker.pose.position.y = pattern_offset[1] + voxel_pos[1]
                marker.pose.position.z = pattern_offset[2] + voxel_pos[2]
                marker.pose.orientation.w = 1.0
                
                # Set marker scale (slightly smaller than voxel size for visibility, matching occupied_voxel_markers)
                marker.scale = Vector3()
                marker.scale.x = voxel_size * 0.9
                marker.scale.y = voxel_size * 0.9
                marker.scale.z = voxel_size * 0.9
                
                # Set marker color to blue (all markers same color)
                marker.color = ColorRGBA()
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
                marker.color.a = 0.5  # Semi-transparent
                
                marker_array.markers.append(marker)
                marker_id += 1
            
        return marker_array
        
    def create_spatial_pattern_markers(self,
                                      patterns: List[np.ndarray],
                                      block_indices: List[int],
                                      blocks_dims: Tuple[int, int, int],
                                      voxel_size: float,
                                      block_size: int,
                                      grid_origin: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                                      colors: Optional[List[Tuple[float, float, float, float]]] = None) -> MarkerArray:
        """
        Create MarkerArray with patterns positioned at their actual 3D locations
        
        Args:
            patterns: List of 3D boolean arrays representing patterns
            block_indices: List mapping block positions to pattern indices
            blocks_dims: Number of blocks in each dimension (x, y, z)
            voxel_size: Size of each voxel in meters
            block_size: Size of each block in voxels (e.g., 8 for 8x8x8)
            grid_origin: Origin of the voxel grid in world coordinates
            colors: Optional list of (r, g, b, a) tuples for pattern colors
            
        Returns:
            MarkerArray message containing spatially positioned visualization markers
        """
        marker_array = MarkerArray()
        
        if not patterns or not block_indices:
            print(f"WARNING: create_spatial_pattern_markers - patterns: {len(patterns) if patterns else 0}, block_indices: {len(block_indices) if block_indices else 0}")
            return marker_array
            
        blocks_x, blocks_y, blocks_z = blocks_dims
        block_size_meters = block_size * voxel_size
        
        # Create a marker for each block
        marker_id = 0
        skipped_blocks = 0
        empty_patterns = 0
        total_voxels = 0
        
        print(f"DEBUG: Creating spatial markers for {len(block_indices)} blocks with {len(patterns)} patterns")
        
        for block_idx, pattern_idx in enumerate(block_indices):
            if pattern_idx >= len(patterns):
                skipped_blocks += 1
                print(f"DEBUG: Block {block_idx} skipped - pattern_idx {pattern_idx} >= {len(patterns)}")
                continue
                
            pattern = patterns[pattern_idx]
            if not self._validate_pattern(pattern):
                continue
                
            # Calculate block position in 3D grid
            bz = block_idx // (blocks_x * blocks_y)
            by = (block_idx % (blocks_x * blocks_y)) // blocks_x
            bx = block_idx % blocks_x
            
            # Calculate world position for this block
            block_world_pos = (
                grid_origin[0] + bx * block_size_meters,
                grid_origin[1] + by * block_size_meters,
                grid_origin[2] + bz * block_size_meters
            )
            
            # Get voxel positions for this pattern (relative to block origin)
            voxel_positions = self._create_voxel_positions(pattern, voxel_size, (0.0, 0.0, 0.0))
            
            if not voxel_positions:
                empty_patterns += 1
                print(f"DEBUG: Block {block_idx} has empty pattern (pattern_idx={pattern_idx})")
                continue
                
            total_voxels += len(voxel_positions)
            
            # Create individual CUBE markers for each voxel (matching occupied_voxel_markers format)
            for voxel_pos in voxel_positions:
                marker = Marker()
                marker.header = Header()
                marker.header.frame_id = self.frame_id
                marker.id = marker_id
                marker.type = Marker.CUBE  # Individual CUBE instead of CUBE_LIST
                marker.action = Marker.ADD
                
                # Set marker position (block position + voxel position)
                marker.pose.position.x = block_world_pos[0] + voxel_pos[0]
                marker.pose.position.y = block_world_pos[1] + voxel_pos[1]
                marker.pose.position.z = block_world_pos[2] + voxel_pos[2]
                marker.pose.orientation.w = 1.0
                
                # Set marker scale (slightly smaller than voxel size for visibility, matching occupied_voxel_markers)
                marker.scale = Vector3()
                marker.scale.x = voxel_size * 0.9
                marker.scale.y = voxel_size * 0.9
                marker.scale.z = voxel_size * 0.9
                
                # Set marker color to blue (all markers same color)
                marker.color = ColorRGBA()
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
                marker.color.a = 0.5  # Semi-transparent
                
                marker_array.markers.append(marker)
                marker_id += 1
        
        print(f"DEBUG: Created {marker_id} markers with {total_voxels} total voxels")
        print(f"DEBUG: Skipped {skipped_blocks} blocks (invalid indices), {empty_patterns} blocks (empty patterns)")
        
        if skipped_blocks > 0:
            print(f"WARNING: Skipped {skipped_blocks} blocks due to invalid pattern indices")
            
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