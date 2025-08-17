#!/usr/bin/env python3
"""
Test suite for compressed_viewer package using Test-Driven Development (TDD)
"""

import unittest
import numpy as np
from unittest.mock import Mock, MagicMock, patch
import sys
import os

# Add the package to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from compressed_viewer.compressed_viewer_node import CompressedViewerNode
from compressed_viewer.decompressor import Decompressor
from compressed_viewer.visualizer import PointCloudVisualizer
from compressed_viewer.statistics_display import StatisticsDisplay


class TestDecompressor(unittest.TestCase):
    """Test cases for the Decompressor class"""
    
    def setUp(self):
        self.decompressor = Decompressor()
        
    def test_init(self):
        """Test Decompressor initialization"""
        self.assertIsNotNone(self.decompressor)
        self.assertEqual(self.decompressor.voxel_size, 0.01)
        self.assertEqual(self.decompressor.block_size, 8)
        
    def test_decompress_empty_data(self):
        """Test decompression with empty data"""
        mock_msg = Mock()
        mock_msg.block_indices = []
        mock_msg.pattern_dictionary.dictionary_data = []
        mock_msg.pattern_dictionary.num_patterns = 0
        
        result = self.decompressor.decompress(mock_msg)
        self.assertIsNotNone(result)
        self.assertEqual(len(result), 0)
        
    def test_decompress_simple_data(self):
        """Test decompression with simple test data"""
        mock_msg = Mock()
        mock_msg.block_indices = [0, 1, 0]  # 3 blocks, 2 unique patterns
        mock_msg.pattern_dictionary.num_patterns = 2
        mock_msg.pattern_dictionary.pattern_size_bytes = 64
        mock_msg.pattern_dictionary.dictionary_data = bytearray(128)  # 2 patterns * 64 bytes
        mock_msg.compression_settings.voxel_size = 0.01
        mock_msg.compression_settings.block_size = 8
        mock_msg.voxel_grid_dimensions.x = 24
        mock_msg.voxel_grid_dimensions.y = 8
        mock_msg.voxel_grid_dimensions.z = 8
        mock_msg.voxel_grid_origin.x = 0.0
        mock_msg.voxel_grid_origin.y = 0.0
        mock_msg.voxel_grid_origin.z = 0.0
        
        result = self.decompressor.decompress(mock_msg)
        self.assertIsNotNone(result)
        # Should return numpy array of points
        self.assertIsInstance(result, np.ndarray)
        
    def test_reconstruct_voxel_grid(self):
        """Test voxel grid reconstruction from blocks"""
        blocks = [
            np.zeros((8, 8, 8), dtype=bool),
            np.ones((8, 8, 8), dtype=bool)
        ]
        blocks[0][0, 0, 0] = True  # Set one voxel in first block
        
        indices = [0, 1]
        grid_shape = (16, 8, 8)
        
        voxel_grid = self.decompressor._reconstruct_voxel_grid(
            blocks, indices, grid_shape, block_size=8
        )
        
        self.assertEqual(voxel_grid.shape, grid_shape)
        self.assertTrue(voxel_grid[0, 0, 0])  # Check first voxel
        self.assertTrue(np.all(voxel_grid[8:16, :, :]))  # Check second block
        
    def test_voxel_to_points(self):
        """Test conversion from voxel grid to point cloud"""
        voxel_grid = np.zeros((10, 10, 10), dtype=bool)
        voxel_grid[0, 0, 0] = True
        voxel_grid[5, 5, 5] = True
        
        points = self.decompressor._voxel_to_points(
            voxel_grid, 
            voxel_size=0.1,
            origin=(0.0, 0.0, 0.0)
        )
        
        self.assertEqual(len(points), 2)
        # Check approximate positions (voxel centers)
        self.assertAlmostEqual(points[0][0], 0.05, places=2)
        self.assertAlmostEqual(points[1][0], 0.55, places=2)


class TestPointCloudVisualizer(unittest.TestCase):
    """Test cases for the PointCloudVisualizer class"""
    
    def setUp(self):
        self.visualizer = PointCloudVisualizer()
        
    def test_init(self):
        """Test PointCloudVisualizer initialization"""
        self.assertIsNotNone(self.visualizer)
        self.assertEqual(self.visualizer.frame_id, 'map')
        
    def test_create_point_cloud2_empty(self):
        """Test PointCloud2 message creation with empty points"""
        points = np.array([])
        msg = self.visualizer.create_point_cloud2(points)
        
        self.assertIsNotNone(msg)
        self.assertEqual(msg.height, 1)
        self.assertEqual(msg.width, 0)
        
    def test_create_point_cloud2_with_points(self):
        """Test PointCloud2 message creation with points"""
        points = np.array([
            [1.0, 2.0, 3.0],
            [4.0, 5.0, 6.0]
        ])
        
        msg = self.visualizer.create_point_cloud2(points)
        
        self.assertIsNotNone(msg)
        self.assertEqual(msg.height, 1)
        self.assertEqual(msg.width, 2)
        self.assertEqual(len(msg.fields), 3)  # x, y, z fields
        
    def test_create_marker_array_empty(self):
        """Test MarkerArray creation with empty points"""
        points = np.array([])
        markers = self.visualizer.create_marker_array(points)
        
        self.assertIsNotNone(markers)
        self.assertEqual(len(markers.markers), 0)
        
    def test_create_marker_array_with_points(self):
        """Test MarkerArray creation with points"""
        points = np.array([
            [1.0, 2.0, 3.0],
            [4.0, 5.0, 6.0]
        ])
        
        markers = self.visualizer.create_marker_array(
            points, 
            color=(1.0, 0.0, 0.0, 1.0),
            size=0.05
        )
        
        self.assertIsNotNone(markers)
        self.assertEqual(len(markers.markers), 1)  # One marker for all points
        marker = markers.markers[0]
        self.assertEqual(marker.type, 6)  # CUBE_LIST type
        self.assertEqual(len(marker.points), 2)
        
    def test_create_bounding_box(self):
        """Test bounding box marker creation"""
        min_bound = (0.0, 0.0, 0.0)
        max_bound = (1.0, 1.0, 1.0)
        
        marker = self.visualizer.create_bounding_box(min_bound, max_bound)
        
        self.assertIsNotNone(marker)
        self.assertEqual(marker.type, 5)  # LINE_LIST type
        self.assertEqual(len(marker.points), 24)  # 12 edges * 2 points


class TestStatisticsDisplay(unittest.TestCase):
    """Test cases for the StatisticsDisplay class"""
    
    def setUp(self):
        self.display = StatisticsDisplay()
        
    def test_init(self):
        """Test StatisticsDisplay initialization"""
        self.assertIsNotNone(self.display)
        self.assertIsNotNone(self.display.statistics)
        
    def test_update_statistics(self):
        """Test statistics update from compressed message"""
        mock_msg = Mock()
        mock_msg.original_point_count = 1000
        mock_msg.compressed_data_size = 500
        mock_msg.original_data_size = 2000
        mock_msg.compression_ratio = 0.25
        mock_msg.unique_patterns_count = 50
        mock_msg.compression_time_seconds = 1.5
        
        self.display.update_statistics(mock_msg)
        
        self.assertEqual(self.display.statistics['original_points'], 1000)
        self.assertEqual(self.display.statistics['compressed_size'], 500)
        self.assertEqual(self.display.statistics['compression_ratio'], 0.25)
        
    def test_format_statistics(self):
        """Test statistics formatting for display"""
        self.display.statistics = {
            'original_points': 1000,
            'compressed_size': 500,
            'original_size': 2000,
            'compression_ratio': 0.25,
            'unique_patterns': 50,
            'compression_time': 1.5,
            'decompression_time': 0.5
        }
        
        formatted = self.display.format_statistics()
        
        self.assertIsInstance(formatted, str)
        self.assertIn('1000', formatted)
        self.assertIn('0.25', formatted)
        self.assertIn('50', formatted)
        
    def test_create_text_marker(self):
        """Test text marker creation for RViz display"""
        text = "Test Statistics"
        marker = self.display.create_text_marker(text, position=(0, 0, 2))
        
        self.assertIsNotNone(marker)
        self.assertEqual(marker.type, 9)  # TEXT_VIEW_FACING type
        self.assertEqual(marker.text, text)
        self.assertEqual(marker.pose.position.z, 2)


class TestCompressedViewerNode(unittest.TestCase):
    """Test cases for the main CompressedViewerNode"""
    
    @patch('compressed_viewer.compressed_viewer_node.rclpy')
    @patch('compressed_viewer.compressed_viewer_node.Node')
    def setUp(self, mock_node, mock_rclpy):
        """Set up test fixtures with mocked ROS2 components"""
        self.mock_node = MagicMock()
        mock_node.return_value = self.mock_node
        
        # Note: We'll need to import after mocking
        # self.node = CompressedViewerNode()
        
    def test_node_initialization(self):
        """Test node initialization and component setup"""
        # This would test that all components are properly initialized
        pass
        
    def test_compressed_callback(self):
        """Test callback for compressed point cloud messages"""
        # This would test the main callback function
        pass
        
    def test_error_handling(self):
        """Test error handling in decompression and visualization"""
        # This would test various error conditions
        pass


if __name__ == '__main__':
    unittest.main()