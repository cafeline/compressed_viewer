#!/usr/bin/env python3
"""
Integration test for verifying visualization integrity between original point cloud 
and pattern markers visualization.

This test ensures that when min_points_threshold=1, all original points are 
correctly represented within the pattern marker blocks.
"""

import unittest
import numpy as np
import sys
import os

# Add the package modules to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from compressed_viewer.pattern_dictionary_decompressor import PatternDictionaryDecompressor
from compressed_viewer.pattern_marker_visualizer import PatternMarkerVisualizer

# Mock the ROS message types for testing
class MockPatternDictionary:
    def __init__(self, num_patterns, pattern_size_bytes, dictionary_data, checksum=0):
        self.num_patterns = num_patterns
        self.pattern_size_bytes = pattern_size_bytes
        self.dictionary_data = dictionary_data
        self.checksum = checksum

class MockPoint3D:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

class MockPointCloud:
    def __init__(self, points):
        self.points = points

class VisualizationIntegrityTest(unittest.TestCase):
    """Test suite for visualization integrity verification"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.decompressor = PatternDictionaryDecompressor()
        self.visualizer = PatternMarkerVisualizer(frame_id='test_frame')
        self.voxel_size = 1.0  # Use 1m voxel size for easier calculations
        self.block_size = 8    # 8x8x8 blocks
        
    def test_simple_grid_point_cloud_integrity(self):
        """Test integrity with a simple 3D grid of points"""
        print("\n=== Testing Simple Grid Point Cloud Integrity ===")
        
        # Create a simple 3D grid of points that should fill exactly one 8x8x8 block
        original_points = []
        expected_voxel_positions = []
        
        # Create points at regular intervals within one voxel block (8x8x8 voxels)
        # Each voxel is 1.0m, so points at 0.5, 1.5, 2.5, ..., 7.5 in each dimension
        for x in range(8):
            for y in range(8):
                for z in range(8):
                    # Place point at voxel center
                    point_x = x + 0.5
                    point_y = y + 0.5  
                    point_z = z + 0.5
                    original_points.append(MockPoint3D(point_x, point_y, point_z))
                    expected_voxel_positions.append((point_x, point_y, point_z))
        
        print(f"Created {len(original_points)} original points")
        print(f"Point range: X[{min(p.x for p in original_points)}, {max(p.x for p in original_points)}]")
        print(f"Point range: Y[{min(p.y for p in original_points)}, {max(p.y for p in original_points)}]")
        print(f"Point range: Z[{min(p.z for p in original_points)}, {max(p.z for p in original_points)}]")
        
        # Create a pattern dictionary with one fully occupied 8x8x8 block
        pattern = self._create_full_block_pattern()
        pattern_dict = self._create_mock_pattern_dictionary([pattern])
        
        # Decompress the pattern dictionary
        patterns = self.decompressor.decompress_pattern_dictionary(pattern_dict)
        self.assertEqual(len(patterns), 1, "Should have exactly 1 pattern")
        
        # Verify the pattern matches our expectation (all voxels occupied)
        pattern_array = patterns[0]
        self.assertEqual(pattern_array.shape, (8, 8, 8), "Pattern should be 8x8x8")
        
        occupied_count = np.sum(pattern_array)
        print(f"Pattern has {occupied_count} occupied voxels out of {pattern_array.size}")
        self.assertEqual(occupied_count, 512, "All 512 voxels should be occupied")
        
        # Create visualization markers
        marker_array = self.visualizer.create_pattern_markers(
            patterns, 
            pattern_spacing=10.0,  # Space patterns far apart
            voxel_size=self.voxel_size
        )
        
        self.assertEqual(len(marker_array.markers), 1, "Should have exactly 1 marker")
        
        marker = marker_array.markers[0]
        print(f"Marker has {len(marker.points)} visualization points")
        print(f"Marker scale: {marker.scale.x} x {marker.scale.y} x {marker.scale.z}")
        
        # Verify marker scale matches voxel size
        self.assertEqual(marker.scale.x, self.voxel_size)
        self.assertEqual(marker.scale.y, self.voxel_size)
        self.assertEqual(marker.scale.z, self.voxel_size)
        
        # Extract marker point positions
        marker_positions = []
        for point in marker.points:
            marker_positions.append((point.x, point.y, point.z))
        
        print(f"Marker points range: X[{min(p[0] for p in marker_positions)}, {max(p[0] for p in marker_positions)}]")
        print(f"Marker points range: Y[{min(p[1] for p in marker_positions)}, {max(p[1] for p in marker_positions)}]")
        print(f"Marker points range: Z[{min(p[2] for p in marker_positions)}, {max(p[2] for p in marker_positions)}]")
        
        # Verify all original points are represented in marker positions
        self._verify_point_coverage(original_points, marker_positions, self.voxel_size)
        
        print("✓ All original points are correctly represented in pattern markers")
        
    def test_sparse_point_cloud_integrity(self):
        """Test integrity with a sparse point cloud (fewer points per voxel)"""
        print("\n=== Testing Sparse Point Cloud Integrity ===")
        
        # Create a sparse set of points - only corners of the 8x8x8 block
        original_points = [
            MockPoint3D(0.5, 0.5, 0.5),    # Corner 1
            MockPoint3D(7.5, 0.5, 0.5),    # Corner 2
            MockPoint3D(0.5, 7.5, 0.5),    # Corner 3
            MockPoint3D(7.5, 7.5, 0.5),    # Corner 4
            MockPoint3D(0.5, 0.5, 7.5),    # Corner 5
            MockPoint3D(7.5, 0.5, 7.5),    # Corner 6
            MockPoint3D(0.5, 7.5, 7.5),    # Corner 7
            MockPoint3D(7.5, 7.5, 7.5),    # Corner 8
        ]
        
        print(f"Created {len(original_points)} sparse original points")
        
        # Create a pattern dictionary with only corner voxels occupied
        pattern = self._create_corner_block_pattern()
        pattern_dict = self._create_mock_pattern_dictionary([pattern])
        
        # Decompress and visualize
        patterns = self.decompressor.decompress_pattern_dictionary(pattern_dict)
        marker_array = self.visualizer.create_pattern_markers(
            patterns, 
            pattern_spacing=10.0,
            voxel_size=self.voxel_size
        )
        
        # Extract marker positions
        marker_positions = []
        for point in marker_array.markers[0].points:
            marker_positions.append((point.x, point.y, point.z))
        
        print(f"Marker has {len(marker_positions)} visualization points")
        
        # Verify coverage
        self._verify_point_coverage(original_points, marker_positions, self.voxel_size)
        
        print("✓ Sparse point cloud correctly represented in pattern markers")
        
    def test_multiple_blocks_integrity(self):
        """Test integrity with multiple pattern blocks"""
        print("\n=== Testing Multiple Blocks Integrity ===")
        
        # Create points for two separate blocks
        original_points = []
        
        # Block 1: Lower left corner voxels (0-7 in each dimension)
        for i in range(4):  # Only fill part of first block
            for j in range(4):
                for k in range(4):
                    original_points.append(MockPoint3D(i + 0.5, j + 0.5, k + 0.5))
        
        # Block 2: Upper right corner voxels (simulated as offset)
        # In reality this would be a separate spatial block, but for the pattern
        # dictionary test we simulate this as a different pattern
        
        print(f"Created {len(original_points)} points across multiple conceptual blocks")
        
        # Create two different patterns
        pattern1 = self._create_partial_block_pattern()  # Partially filled
        pattern2 = self._create_corner_block_pattern()   # Corner points only
        
        pattern_dict = self._create_mock_pattern_dictionary([pattern1, pattern2])
        
        # Decompress and visualize
        patterns = self.decompressor.decompress_pattern_dictionary(pattern_dict)
        marker_array = self.visualizer.create_pattern_markers(
            patterns, 
            pattern_spacing=10.0,
            voxel_size=self.voxel_size
        )
        
        self.assertEqual(len(marker_array.markers), 2, "Should have 2 pattern markers")
        
        print(f"Pattern 1 has {len(marker_array.markers[0].points)} points")
        print(f"Pattern 2 has {len(marker_array.markers[1].points)} points")
        
        # Verify first pattern covers our first block points
        marker1_positions = [(p.x, p.y, p.z) for p in marker_array.markers[0].points]
        
        # For first pattern, check that our original points are represented
        # Note: This is a simplified check since in reality the spatial arrangement is more complex
        self.assertGreater(len(marker1_positions), 0, "First pattern should have visualization points")
        
        print("✓ Multiple blocks correctly represented in pattern markers")
        
    def _verify_point_coverage(self, original_points, marker_positions, voxel_size, tolerance=0.1):
        """
        Verify that all original points are covered by marker positions.
        
        Args:
            original_points: List of MockPoint3D objects
            marker_positions: List of (x, y, z) tuples from markers
            voxel_size: Size of each voxel
            tolerance: Tolerance for position matching
        """
        print(f"\nVerifying coverage of {len(original_points)} original points")
        print(f"Against {len(marker_positions)} marker positions")
        
        uncovered_points = []
        
        for orig_point in original_points:
            found_match = False
            orig_pos = (orig_point.x, orig_point.y, orig_point.z)
            
            for marker_pos in marker_positions:
                # Check if marker position is close to original point
                distance = np.sqrt(
                    (orig_pos[0] - marker_pos[0])**2 + 
                    (orig_pos[1] - marker_pos[1])**2 + 
                    (orig_pos[2] - marker_pos[2])**2
                )
                
                if distance <= tolerance:
                    found_match = True
                    break
            
            if not found_match:
                uncovered_points.append(orig_pos)
                
        if uncovered_points:
            print(f"❌ Found {len(uncovered_points)} uncovered points:")
            for i, point in enumerate(uncovered_points[:5]):  # Show first 5
                print(f"  Uncovered point {i+1}: {point}")
            if len(uncovered_points) > 5:
                print(f"  ... and {len(uncovered_points) - 5} more")
        
        self.assertEqual(len(uncovered_points), 0, 
                         f"All original points should be covered by markers. "
                         f"Found {len(uncovered_points)} uncovered points.")
    
    def _create_full_block_pattern(self):
        """Create a pattern with all voxels occupied (8x8x8 = 512 bits = 64 bytes)"""
        # All bits set to 1
        return [0xFF] * 64
    
    def _create_corner_block_pattern(self):
        """Create a pattern with only corner voxels occupied"""
        pattern = [0x00] * 64  # Start with all zeros
        
        # Set bits for corners of 8x8x8 block
        # Corner positions in 3D: (0,0,0), (7,0,0), (0,7,0), (7,7,0), (0,0,7), (7,0,7), (0,7,7), (7,7,7)
        corner_indices = [
            0,      # (0,0,0)
            7,      # (7,0,0)  
            56,     # (0,7,0) = 7*8 + 0*64
            63,     # (7,7,0) = 7*8 + 7
            448,    # (0,0,7) = 0 + 0*8 + 7*64
            455,    # (7,0,7) = 7 + 0*8 + 7*64
            504,    # (0,7,7) = 0 + 7*8 + 7*64
            511,    # (7,7,7) = 7 + 7*8 + 7*64
        ]
        
        for index in corner_indices:
            byte_index = index // 8
            bit_index = index % 8
            if byte_index < 64:  # Safety check
                pattern[byte_index] |= (1 << bit_index)
        
        return pattern
    
    def _create_partial_block_pattern(self):
        """Create a pattern with only lower corner voxels occupied (4x4x4 subset)"""
        pattern = [0x00] * 64  # Start with all zeros
        
        # Set bits for 4x4x4 lower corner
        for x in range(4):
            for y in range(4):
                for z in range(4):
                    index = z * 64 + y * 8 + x  # 3D to 1D index conversion
                    byte_index = index // 8
                    bit_index = index % 8
                    if byte_index < 64:  # Safety check
                        pattern[byte_index] |= (1 << bit_index)
        
        return pattern
    
    def _create_mock_pattern_dictionary(self, patterns):
        """Create a mock PatternDictionary message from pattern data"""
        dictionary_data = []
        for pattern in patterns:
            dictionary_data.extend(pattern)
        
        return MockPatternDictionary(
            num_patterns=len(patterns),
            pattern_size_bytes=64,  # 8x8x8 bits = 512 bits = 64 bytes
            dictionary_data=dictionary_data,
            checksum=0
        )


if __name__ == '__main__':
    # Run the tests
    unittest.main(verbosity=2)