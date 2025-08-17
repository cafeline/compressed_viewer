#!/usr/bin/env python3
"""
Visualization module for displaying decompressed point clouds in RViz2
"""

import numpy as np
from typing import Tuple, Optional, List
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import Header, ColorRGBA
import struct


class PointCloudVisualizer:
    """Handles visualization of point clouds in RViz2"""
    
    def __init__(self, frame_id: str = 'map'):
        """
        Initialize the visualizer
        
        Args:
            frame_id: TF frame ID for visualization
        """
        self.frame_id = frame_id
        self.marker_id_counter = 0
        
    def create_point_cloud2(self, points: np.ndarray, 
                           header: Optional[Header] = None,
                           rgb: Optional[np.ndarray] = None) -> PointCloud2:
        """
        Create a PointCloud2 message from numpy array
        
        Args:
            points: Nx3 numpy array of point coordinates
            header: Optional header (will create if not provided)
            rgb: Optional Nx3 array of RGB values (0-255)
            
        Returns:
            PointCloud2 message for publishing
        """
        msg = PointCloud2()
        
        # Set header
        if header:
            msg.header = header
        else:
            msg.header.frame_id = self.frame_id
            
        if len(points) == 0:
            msg.height = 1
            msg.width = 0
            return msg
            
        # Reshape points if necessary
        if points.ndim == 1:
            points = points.reshape(-1, 3)
            
        msg.height = 1
        msg.width = len(points)
        
        # Define fields
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        # Add RGB fields if color data provided
        if rgb is not None:
            msg.fields.append(
                PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1)
            )
            msg.point_step = 16
        else:
            msg.point_step = 12
            
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.is_bigendian = False
        
        # Pack point data
        buffer = []
        for i in range(len(points)):
            buffer.append(struct.pack('fff', 
                                     points[i, 0], 
                                     points[i, 1], 
                                     points[i, 2]))
            
            if rgb is not None:
                # Pack RGB as float (RViz2 format)
                r = int(rgb[i, 0]) if i < len(rgb) else 255
                g = int(rgb[i, 1]) if i < len(rgb) else 255
                b = int(rgb[i, 2]) if i < len(rgb) else 255
                rgb_packed = (r << 16) | (g << 8) | b
                buffer.append(struct.pack('f', float(rgb_packed)))
                
        msg.data = b''.join(buffer)
        
        return msg
        
    def create_marker_array(self, points: np.ndarray,
                           color: Tuple[float, float, float, float] = (0.0, 1.0, 0.0, 1.0),
                           size: float = 0.01,
                           marker_type: str = 'cube') -> MarkerArray:
        """
        Create a MarkerArray for visualizing points
        
        Args:
            points: Nx3 numpy array of point coordinates
            color: RGBA color tuple (values 0-1)
            size: Size of each marker
            marker_type: Type of marker ('cube', 'sphere', 'point')
            
        Returns:
            MarkerArray message
        """
        marker_array = MarkerArray()
        
        if len(points) == 0:
            return marker_array
            
        # Create a single marker with all points (more efficient)
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.ns = "point_cloud"
        marker.id = self.marker_id_counter
        self.marker_id_counter += 1
        
        # Set marker type
        if marker_type == 'cube':
            marker.type = Marker.CUBE_LIST
        elif marker_type == 'sphere':
            marker.type = Marker.SPHERE_LIST
        else:
            marker.type = Marker.POINTS
            
        marker.action = Marker.ADD
        
        # Set pose (identity since points are in world coordinates)
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # Set scale
        marker.scale.x = size
        marker.scale.y = size
        marker.scale.z = size
        
        # Set color
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        
        # Add points
        for point in points:
            p = Point()
            p.x = float(point[0])
            p.y = float(point[1])
            p.z = float(point[2])
            marker.points.append(p)
            
        marker.lifetime.sec = 0  # Persistent
        
        marker_array.markers.append(marker)
        
        return marker_array
        
    def create_bounding_box(self, 
                           min_bound: Tuple[float, float, float],
                           max_bound: Tuple[float, float, float],
                           color: Tuple[float, float, float, float] = (1.0, 1.0, 0.0, 1.0),
                           line_width: float = 0.01) -> Marker:
        """
        Create a bounding box marker
        
        Args:
            min_bound: Minimum coordinates (x, y, z)
            max_bound: Maximum coordinates (x, y, z)
            color: RGBA color tuple
            line_width: Width of the box lines
            
        Returns:
            Marker message for the bounding box
        """
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.ns = "bounding_box"
        marker.id = self.marker_id_counter
        self.marker_id_counter += 1
        
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        
        # Set pose
        marker.pose.orientation.w = 1.0
        
        # Set scale (line width)
        marker.scale.x = line_width
        
        # Set color
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        
        # Define the 8 corners of the box
        corners = [
            [min_bound[0], min_bound[1], min_bound[2]],
            [max_bound[0], min_bound[1], min_bound[2]],
            [max_bound[0], max_bound[1], min_bound[2]],
            [min_bound[0], max_bound[1], min_bound[2]],
            [min_bound[0], min_bound[1], max_bound[2]],
            [max_bound[0], min_bound[1], max_bound[2]],
            [max_bound[0], max_bound[1], max_bound[2]],
            [min_bound[0], max_bound[1], max_bound[2]]
        ]
        
        # Define the 12 edges of the box
        edges = [
            (0, 1), (1, 2), (2, 3), (3, 0),  # Bottom face
            (4, 5), (5, 6), (6, 7), (7, 4),  # Top face
            (0, 4), (1, 5), (2, 6), (3, 7)   # Vertical edges
        ]
        
        # Add edge points
        for edge in edges:
            for idx in edge:
                p = Point()
                p.x = corners[idx][0]
                p.y = corners[idx][1]
                p.z = corners[idx][2]
                marker.points.append(p)
                
        marker.lifetime.sec = 0  # Persistent
        
        return marker
        
    def create_voxel_grid_marker(self,
                                 voxel_grid: np.ndarray,
                                 voxel_size: float,
                                 origin: Tuple[float, float, float],
                                 color: Tuple[float, float, float, float] = (0.0, 0.5, 1.0, 0.5)) -> MarkerArray:
        """
        Create markers for visualizing a voxel grid
        
        Args:
            voxel_grid: 3D boolean array
            voxel_size: Size of each voxel
            origin: Origin of the grid
            color: RGBA color tuple
            
        Returns:
            MarkerArray with voxel visualization
        """
        marker_array = MarkerArray()
        
        # Find occupied voxels
        occupied = np.where(voxel_grid)
        
        if len(occupied[0]) == 0:
            return marker_array
            
        # Create cube list marker
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.ns = "voxel_grid"
        marker.id = self.marker_id_counter
        self.marker_id_counter += 1
        
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        
        # Set pose
        marker.pose.orientation.w = 1.0
        
        # Set scale to voxel size
        marker.scale.x = voxel_size * 0.9  # Slightly smaller for visibility
        marker.scale.y = voxel_size * 0.9
        marker.scale.z = voxel_size * 0.9
        
        # Set color
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        
        # Add voxel centers
        for i in range(len(occupied[0])):
            p = Point()
            p.x = origin[0] + (occupied[0][i] + 0.5) * voxel_size
            p.y = origin[1] + (occupied[1][i] + 0.5) * voxel_size
            p.z = origin[2] + (occupied[2][i] + 0.5) * voxel_size
            marker.points.append(p)
            
        marker.lifetime.sec = 0  # Persistent
        
        marker_array.markers.append(marker)
        
        return marker_array
        
    def clear_markers(self) -> MarkerArray:
        """
        Create a marker array that clears all previous markers
        
        Returns:
            MarkerArray with delete actions
        """
        marker_array = MarkerArray()
        
        # Create delete marker
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.ns = ""
        marker.id = 0
        marker.action = Marker.DELETEALL
        
        marker_array.markers.append(marker)
        
        return marker_array