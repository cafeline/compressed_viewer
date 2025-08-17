#!/usr/bin/env python3
"""
Test suite for PatternMarkerVisualizer using Test-Driven Development (TDD)
"""

import unittest
import numpy as np
from unittest.mock import Mock
import sys
import os

# Add the package to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from compressed_viewer.pattern_marker_visualizer import PatternMarkerVisualizer


class TestPatternMarkerVisualizer(unittest.TestCase):
    """Test cases for the PatternMarkerVisualizer class"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.visualizer = PatternMarkerVisualizer(frame_id='test_frame')
        
    def test_init(self):
        """Test PatternMarkerVisualizer initialization"""
        self.assertIsNotNone(self.visualizer)
        self.assertEqual(self.visualizer.frame_id, 'test_frame')
        
    def test_init_default_frame(self):
        """Test initialization with default frame_id"""
        visualizer = PatternMarkerVisualizer()
        self.assertEqual(visualizer.frame_id, 'map')
        
    def test_create_pattern_markers_empty(self):
        """Test marker creation with empty patterns"""
        patterns = []
        
        marker_array = self.visualizer.create_pattern_markers(patterns)
        
        self.assertIsNotNone(marker_array)
        self.assertEqual(len(marker_array.markers), 0)
        
    def test_create_pattern_markers_single_pattern(self):
        """Test marker creation with a single pattern"""
        # Create a simple 2x2x2 pattern with 2 voxels set
        pattern = np.zeros((2, 2, 2), dtype=bool)
        pattern[0, 0, 0] = True
        pattern[1, 1, 1] = True
        
        patterns = [pattern]
        
        marker_array = self.visualizer.create_pattern_markers(patterns)
        
        self.assertIsNotNone(marker_array)
        self.assertEqual(len(marker_array.markers), 1)
        
        marker = marker_array.markers[0]
        self.assertEqual(marker.header.frame_id, 'test_frame')
        self.assertEqual(marker.type, 6)  # CUBE_LIST
        self.assertEqual(len(marker.points), 2)  # 2 voxels
        
    def test_create_pattern_markers_multiple_patterns(self):
        """Test marker creation with multiple patterns"""
        # Create two patterns
        pattern1 = np.zeros((2, 2, 2), dtype=bool)
        pattern1[0, 0, 0] = True
        
        pattern2 = np.zeros((2, 2, 2), dtype=bool)
        pattern2[1, 1, 1] = True
        
        patterns = [pattern1, pattern2]
        
        marker_array = self.visualizer.create_pattern_markers(patterns)
        
        self.assertIsNotNone(marker_array)
        self.assertEqual(len(marker_array.markers), 2)
        
        # Check each pattern has its own marker
        for i, marker in enumerate(marker_array.markers):
            self.assertEqual(marker.id, i)
            self.assertEqual(marker.type, 6)  # CUBE_LIST
            self.assertEqual(len(marker.points), 1)  # 1 voxel per pattern
            
    def test_create_pattern_markers_with_spacing(self):
        """Test marker creation with custom pattern spacing"""
        pattern = np.zeros((2, 2, 2), dtype=bool)
        pattern[0, 0, 0] = True
        
        patterns = [pattern, pattern]
        spacing = 5.0
        
        marker_array = self.visualizer.create_pattern_markers(patterns, pattern_spacing=spacing)
        
        self.assertEqual(len(marker_array.markers), 2)
        
        # Check that patterns are spaced correctly
        marker1 = marker_array.markers[0]
        marker2 = marker_array.markers[1]
        
        # Pattern 2 should be offset by spacing in x direction
        expected_offset = spacing
        actual_offset = marker2.pose.position.x - marker1.pose.position.x
        self.assertAlmostEqual(actual_offset, expected_offset, places=2)
        
    def test_create_pattern_markers_with_custom_colors(self):
        """Test marker creation with custom colors"""
        pattern = np.zeros((2, 2, 2), dtype=bool)
        pattern[0, 0, 0] = True
        
        patterns = [pattern]
        colors = [(1.0, 0.0, 0.0, 1.0)]  # Red
        
        marker_array = self.visualizer.create_pattern_markers(patterns, colors=colors)
        
        marker = marker_array.markers[0]
        self.assertAlmostEqual(marker.color.r, 1.0)
        self.assertAlmostEqual(marker.color.g, 0.0)
        self.assertAlmostEqual(marker.color.b, 0.0)
        self.assertAlmostEqual(marker.color.a, 1.0)
        
    def test_create_pattern_markers_with_default_colors(self):
        """Test marker creation with default colors when fewer colors than patterns"""
        pattern = np.zeros((2, 2, 2), dtype=bool)
        pattern[0, 0, 0] = True
        
        patterns = [pattern, pattern, pattern]  # 3 patterns
        colors = [(1.0, 0.0, 0.0, 1.0)]  # Only 1 color
        
        marker_array = self.visualizer.create_pattern_markers(patterns, colors=colors)
        
        # First marker should have custom color
        self.assertAlmostEqual(marker_array.markers[0].color.r, 1.0)
        
        # Second and third markers should have default colors
        self.assertAlmostEqual(marker_array.markers[1].color.b, 1.0)  # Default blue (index 1)
        
    def test_create_voxel_positions(self):
        """Test voxel position calculation"""
        pattern = np.zeros((3, 3, 3), dtype=bool)
        pattern[0, 0, 0] = True
        pattern[2, 2, 2] = True
        
        voxel_size = 0.1
        offset = (1.0, 2.0, 3.0)
        
        positions = self.visualizer._create_voxel_positions(pattern, voxel_size, offset)
        
        self.assertEqual(len(positions), 2)
        
        # Check first position (0,0,0 voxel)
        expected_pos1 = (1.05, 2.05, 3.05)  # offset + voxel_center
        self.assertAlmostEqual(positions[0][0], expected_pos1[0], places=2)
        self.assertAlmostEqual(positions[0][1], expected_pos1[1], places=2)
        self.assertAlmostEqual(positions[0][2], expected_pos1[2], places=2)
        
        # Check second position (2,2,2 voxel)
        expected_pos2 = (1.25, 2.25, 3.25)  # offset + 2*voxel_size + voxel_center
        self.assertAlmostEqual(positions[1][0], expected_pos2[0], places=2)
        self.assertAlmostEqual(positions[1][1], expected_pos2[1], places=2)
        self.assertAlmostEqual(positions[1][2], expected_pos2[2], places=2)
        
    def test_create_info_markers(self):
        """Test creation of info text markers"""
        patterns = [np.ones((2, 2, 2), dtype=bool)]
        info_text = "Pattern 0: 8 voxels"
        
        marker_array = self.visualizer.create_info_markers(patterns, info_text)
        
        self.assertIsNotNone(marker_array)
        self.assertEqual(len(marker_array.markers), 1)
        
        marker = marker_array.markers[0]
        self.assertEqual(marker.type, 9)  # TEXT_VIEW_FACING
        self.assertEqual(marker.text, info_text)
        
    def test_clear_markers(self):
        """Test marker clearing functionality"""
        clear_array = self.visualizer.clear_markers()
        
        self.assertIsNotNone(clear_array)
        self.assertEqual(len(clear_array.markers), 1)
        
        marker = clear_array.markers[0]
        self.assertEqual(marker.action, 3)  # DELETE_ALL
        
    def test_get_default_color(self):
        """Test default color generation"""
        color1 = self.visualizer._get_default_color(0)
        color2 = self.visualizer._get_default_color(1)
        color3 = self.visualizer._get_default_color(2)
        
        # Colors should be different
        self.assertNotEqual(color1, color2)
        self.assertNotEqual(color2, color3)
        
        # All should have alpha = 1.0
        self.assertEqual(color1[3], 1.0)
        self.assertEqual(color2[3], 1.0)
        self.assertEqual(color3[3], 1.0)
        
    def test_validate_pattern_dimensions(self):
        """Test pattern dimension validation"""
        # Valid pattern
        valid_pattern = np.zeros((8, 8, 8), dtype=bool)
        self.assertTrue(self.visualizer._validate_pattern(valid_pattern))
        
        # Invalid patterns
        invalid_pattern_1d = np.zeros((8,), dtype=bool)
        self.assertFalse(self.visualizer._validate_pattern(invalid_pattern_1d))
        
        invalid_pattern_4d = np.zeros((8, 8, 8, 8), dtype=bool)
        self.assertFalse(self.visualizer._validate_pattern(invalid_pattern_4d))
        
        invalid_pattern_type = np.zeros((8, 8, 8), dtype=float)
        self.assertFalse(self.visualizer._validate_pattern(invalid_pattern_type))


if __name__ == '__main__':
    unittest.main()