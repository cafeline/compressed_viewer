#!/usr/bin/env python3
"""
Integration test for PatternDictionary functionality in compressed_viewer
"""

import unittest
import numpy as np
from unittest.mock import Mock, MagicMock, patch
import sys
import os

# Add the package to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from compressed_viewer.pattern_dictionary_decompressor import PatternDictionaryDecompressor
from compressed_viewer.pattern_marker_visualizer import PatternMarkerVisualizer


class TestPatternIntegration(unittest.TestCase):
    """Integration test cases for PatternDictionary workflow"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.decompressor = PatternDictionaryDecompressor()
        self.visualizer = PatternMarkerVisualizer(frame_id='test_frame')
        
    def test_end_to_end_pattern_workflow(self):
        """Test complete workflow from PatternDictionary to MarkerArray"""
        # Create mock PatternDictionary message
        mock_dict = Mock()
        mock_dict.num_patterns = 2
        mock_dict.pattern_size_bytes = 64  # 8x8x8 = 512 bits = 64 bytes
        
        # Create pattern data: first pattern empty, second pattern with some voxels
        pattern_data = bytearray(128)  # 2 patterns * 64 bytes
        
        # Second pattern: set some bits (pattern starts at offset 64)
        pattern_data[64] = 0xFF  # First 8 voxels
        pattern_data[65] = 0x0F  # Next 4 voxels
        
        mock_dict.dictionary_data = pattern_data
        
        # Step 1: Decompress patterns
        patterns = self.decompressor.decompress_pattern_dictionary(mock_dict, block_size=8)
        
        self.assertEqual(len(patterns), 2)
        self.assertEqual(patterns[0].shape, (8, 8, 8))
        self.assertEqual(patterns[1].shape, (8, 8, 8))
        
        # First pattern should be empty
        self.assertFalse(np.any(patterns[0]))
        
        # Second pattern should have 12 voxels set
        self.assertEqual(np.sum(patterns[1]), 12)
        
        # Step 2: Create visualization markers
        marker_array = self.visualizer.create_pattern_markers(
            patterns,
            pattern_spacing=5.0,
            voxel_size=0.1
        )
        
        self.assertIsNotNone(marker_array)
        self.assertEqual(len(marker_array.markers), 1)  # Only second pattern has voxels
        
        marker = marker_array.markers[0]
        self.assertEqual(marker.id, 1)  # Second pattern (index 1)
        self.assertEqual(len(marker.points), 12)  # 12 voxels
        self.assertEqual(marker.pose.position.x, 5.0)  # Pattern spacing
        
    def test_multiple_patterns_with_custom_colors(self):
        """Test visualization with multiple patterns and custom colors"""
        # Create three simple patterns
        pattern1 = np.zeros((2, 2, 2), dtype=bool)
        pattern1[0, 0, 0] = True
        
        pattern2 = np.zeros((2, 2, 2), dtype=bool)
        pattern2[1, 1, 1] = True
        
        pattern3 = np.zeros((2, 2, 2), dtype=bool)
        pattern3[0, 1, 0] = True
        pattern3[1, 0, 1] = True
        
        patterns = [pattern1, pattern2, pattern3]
        colors = [
            (1.0, 0.0, 0.0, 1.0),  # Red
            (0.0, 1.0, 0.0, 1.0),  # Green
            (0.0, 0.0, 1.0, 1.0)   # Blue
        ]
        
        marker_array = self.visualizer.create_pattern_markers(
            patterns,
            pattern_spacing=2.0,
            voxel_size=0.05,
            colors=colors
        )
        
        self.assertEqual(len(marker_array.markers), 3)
        
        # Check colors
        self.assertAlmostEqual(marker_array.markers[0].color.r, 1.0)  # Red
        self.assertAlmostEqual(marker_array.markers[1].color.g, 1.0)  # Green
        self.assertAlmostEqual(marker_array.markers[2].color.b, 1.0)  # Blue
        
        # Check positions
        self.assertAlmostEqual(marker_array.markers[0].pose.position.x, 0.0)
        self.assertAlmostEqual(marker_array.markers[1].pose.position.x, 2.0)
        self.assertAlmostEqual(marker_array.markers[2].pose.position.x, 4.0)
        
        # Check voxel counts
        self.assertEqual(len(marker_array.markers[0].points), 1)  # Pattern 1: 1 voxel
        self.assertEqual(len(marker_array.markers[1].points), 1)  # Pattern 2: 1 voxel
        self.assertEqual(len(marker_array.markers[2].points), 2)  # Pattern 3: 2 voxels
        
    def test_info_marker_creation(self):
        """Test creation of information markers"""
        patterns = [
            np.ones((2, 2, 2), dtype=bool),  # 8 voxels
            np.zeros((2, 2, 2), dtype=bool)  # 0 voxels
        ]
        patterns[1][0, 0, 0] = True  # 1 voxel
        
        info_text = "Test Pattern Info"
        
        info_markers = self.visualizer.create_info_markers(
            patterns,
            info_text,
            position=(1.0, 2.0, 3.0)
        )
        
        self.assertEqual(len(info_markers.markers), 1)
        
        marker = info_markers.markers[0]
        self.assertEqual(marker.type, 9)  # TEXT_VIEW_FACING
        self.assertEqual(marker.text, info_text)
        self.assertEqual(marker.pose.position.x, 1.0)
        self.assertEqual(marker.pose.position.y, 2.0)
        self.assertEqual(marker.pose.position.z, 3.0)
        
    def test_error_handling_invalid_pattern_data(self):
        """Test error handling with invalid pattern data"""
        # Test with insufficient data
        mock_dict = Mock()
        mock_dict.num_patterns = 2
        mock_dict.pattern_size_bytes = 64
        mock_dict.dictionary_data = bytearray(32)  # Only enough for 1 pattern
        
        with self.assertRaises(ValueError):
            self.decompressor.decompress_pattern_dictionary(mock_dict)
            
    def test_checksum_validation_integration(self):
        """Test checksum validation in integration workflow"""
        import zlib
        
        # Create valid pattern data
        pattern_data = bytearray([0xFF, 0x00, 0xAA, 0x55] * 16)  # 64 bytes
        expected_checksum = zlib.crc32(pattern_data) & 0xffffffff
        
        mock_dict = Mock()
        mock_dict.num_patterns = 1
        mock_dict.pattern_size_bytes = 64
        mock_dict.dictionary_data = pattern_data
        mock_dict.checksum = expected_checksum
        
        # Should succeed with correct checksum
        patterns = self.decompressor.decompress_pattern_dictionary(
            mock_dict, 
            validate_checksum=True
        )
        self.assertEqual(len(patterns), 1)
        
        # Should create markers successfully
        marker_array = self.visualizer.create_pattern_markers(patterns)
        self.assertIsNotNone(marker_array)
        
    def test_large_pattern_dictionary(self):
        """Test handling of larger pattern dictionaries"""
        # Create 10 patterns
        patterns = []
        for i in range(10):
            pattern = np.zeros((4, 4, 4), dtype=bool)
            # Create different patterns
            pattern[i % 4, (i * 2) % 4, (i * 3) % 4] = True
            patterns.append(pattern)
        
        marker_array = self.visualizer.create_pattern_markers(
            patterns,
            pattern_spacing=1.5,
            voxel_size=0.02
        )
        
        self.assertEqual(len(marker_array.markers), 10)
        
        # Check that each pattern has correct spacing
        for i, marker in enumerate(marker_array.markers):
            expected_x = i * 1.5
            self.assertAlmostEqual(marker.pose.position.x, expected_x, places=2)
            self.assertEqual(marker.id, i)
            
    def test_empty_pattern_dictionary(self):
        """Test handling of empty pattern dictionary"""
        mock_dict = Mock()
        mock_dict.num_patterns = 0
        mock_dict.pattern_size_bytes = 0
        mock_dict.dictionary_data = []
        
        patterns = self.decompressor.decompress_pattern_dictionary(mock_dict)
        self.assertEqual(len(patterns), 0)
        
        marker_array = self.visualizer.create_pattern_markers(patterns)
        self.assertEqual(len(marker_array.markers), 0)


if __name__ == '__main__':
    unittest.main()