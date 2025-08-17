#!/usr/bin/env python3
"""
Test to compare results between Decompressor and PatternMarkerVisualizer
to identify the source of visualization discrepancies.
"""

import unittest
import numpy as np
import sys
import os

# Add the package modules to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from compressed_viewer.decompressor import Decompressor
from compressed_viewer.pattern_dictionary_decompressor import PatternDictionaryDecompressor
from compressed_viewer.pattern_marker_visualizer import PatternMarkerVisualizer

# Mock message types
class MockCompressionSettings:
    def __init__(self, voxel_size=1.0, block_size=8):
        self.voxel_size = voxel_size
        self.block_size = block_size

class MockVector3:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

class MockPatternDictionary:
    def __init__(self, num_patterns=0, pattern_size_bytes=64, dictionary_data=None, checksum=0):
        self.num_patterns = num_patterns
        self.pattern_size_bytes = pattern_size_bytes
        self.dictionary_data = dictionary_data or []
        self.checksum = checksum

class MockCompressedPointCloud:
    def __init__(self):
        self.compression_settings = MockCompressionSettings()
        self.voxel_grid_dimensions = MockVector3()
        self.voxel_grid_origin = MockVector3()
        self.pattern_dictionary = MockPatternDictionary()
        self.block_indices = []

class DecompressorVsVisualizerTest(unittest.TestCase):
    """Compare Decompressor and PatternMarkerVisualizer outputs"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.decompressor = Decompressor()
        self.pattern_decompressor = PatternDictionaryDecompressor()
        self.pattern_visualizer = PatternMarkerVisualizer(frame_id='test')
        
    def test_simple_pattern_comparison(self):
        """Compare outputs for a simple 8x8x8 pattern"""
        print("\n=== Testing Simple Pattern Comparison ===")
        
        voxel_size = 1.0
        block_size = 8
        
        # Create a simple pattern: corners of 8x8x8 block
        pattern_data = self._create_corner_pattern()
        
        # Create mock messages
        pattern_dict = MockPatternDictionary(
            num_patterns=1,
            pattern_size_bytes=64,
            dictionary_data=pattern_data
        )
        
        compressed_msg = MockCompressedPointCloud()
        compressed_msg.compression_settings = MockCompressionSettings(voxel_size, block_size)
        compressed_msg.voxel_grid_dimensions = MockVector3(8.0, 8.0, 8.0)  # 8m = 8 voxels at 1m/voxel
        compressed_msg.voxel_grid_origin = MockVector3(0.0, 0.0, 0.0)
        compressed_msg.pattern_dictionary = pattern_dict
        compressed_msg.block_indices = [0]  # Single block using pattern 0
        
        # Test 1: PatternDictionaryDecompressor
        patterns = self.pattern_decompressor.decompress_pattern_dictionary(pattern_dict)
        print(f"PatternDictionaryDecompressor: {len(patterns)} patterns")
        if patterns:
            pattern = patterns[0]
            occupied_count = np.sum(pattern)
            print(f"  Pattern shape: {pattern.shape}")
            print(f"  Occupied voxels: {occupied_count}")
            print(f"  First few occupied positions:")
            occupied_pos = np.where(pattern)
            for i in range(min(5, len(occupied_pos[0]))):
                print(f"    ({occupied_pos[0][i]}, {occupied_pos[1][i]}, {occupied_pos[2][i]})")
        
        # Test 2: Decompressor (full pipeline)
        decompressed_points = self.decompressor.decompress(compressed_msg)
        print(f"Decompressor: {len(decompressed_points)} points")
        if len(decompressed_points) > 0:
            print(f"  First few decompressed points:")
            for i in range(min(5, len(decompressed_points))):
                point = decompressed_points[i]
                print(f"    ({point[0]:.1f}, {point[1]:.1f}, {point[2]:.1f})")
        
        # Test 3: PatternMarkerVisualizer
        if patterns:
            marker_array = self.pattern_visualizer.create_pattern_markers(
                patterns,
                pattern_spacing=10.0,
                voxel_size=voxel_size
            )
            print(f"PatternMarkerVisualizer: {len(marker_array.markers)} markers")
            if marker_array.markers:
                marker = marker_array.markers[0]
                print(f"  Marker points: {len(marker.points)}")
                print(f"  Marker scale: ({marker.scale.x}, {marker.scale.y}, {marker.scale.z})")
                print(f"  First few marker points:")
                for i in range(min(5, len(marker.points))):
                    point = marker.points[i]
                    print(f"    ({point.x:.1f}, {point.y:.1f}, {point.z:.1f})")
        
        # Verification: All three should agree on number of occupied voxels
        if patterns and len(decompressed_points) > 0:
            pattern_count = np.sum(patterns[0])
            decompressor_count = len(decompressed_points)
            marker_count = len(marker_array.markers[0].points) if marker_array.markers else 0
            
            print(f"\n=== Comparison ===")
            print(f"Pattern voxels: {pattern_count}")
            print(f"Decompressed points: {decompressor_count}")
            print(f"Marker points: {marker_count}")
            
            # They should all be equal
            self.assertEqual(pattern_count, marker_count, 
                           "Pattern and marker counts should match")
            
            # Check if decompressor count matches
            if decompressor_count != pattern_count:
                print(f"❌ MISMATCH: Decompressor produced {decompressor_count} points but pattern has {pattern_count} voxels")
                
                # This indicates the issue is in the Decompressor, not PatternMarkerVisualizer
                print("Issue identified: Decompressor is not correctly reconstructing the voxel grid")
            else:
                print("✓ All counts match")
    
    def test_coordinates_comparison(self):
        """Compare coordinate systems between Decompressor and PatternMarkerVisualizer"""
        print("\n=== Testing Coordinate Systems Comparison ===")
        
        voxel_size = 0.5
        block_size = 8
        
        # Create pattern with specific known voxels
        pattern_data = self._create_specific_pattern()
        
        # Create messages
        pattern_dict = MockPatternDictionary(
            num_patterns=1,
            pattern_size_bytes=64,
            dictionary_data=pattern_data
        )
        
        compressed_msg = MockCompressedPointCloud()
        compressed_msg.compression_settings = MockCompressionSettings(voxel_size, block_size)
        compressed_msg.voxel_grid_dimensions = MockVector3(4.0, 4.0, 4.0)  # 4m = 8 voxels at 0.5m/voxel
        compressed_msg.voxel_grid_origin = MockVector3(1.0, 2.0, 3.0)  # Offset origin
        compressed_msg.pattern_dictionary = pattern_dict
        compressed_msg.block_indices = [0]
        
        # Get pattern positions
        patterns = self.pattern_decompressor.decompress_pattern_dictionary(pattern_dict)
        if patterns:
            pattern = patterns[0]
            occupied_pos = np.where(pattern)
            print(f"Pattern occupied positions (relative to block):")
            for i in range(len(occupied_pos[0])):
                print(f"  ({occupied_pos[0][i]}, {occupied_pos[1][i]}, {occupied_pos[2][i]})")
        
        # Get decompressed points (world coordinates)
        decompressed_points = self.decompressor.decompress(compressed_msg)
        print(f"Decompressed points (world coordinates):")
        for i, point in enumerate(decompressed_points):
            print(f"  Point {i}: ({point[0]:.1f}, {point[1]:.1f}, {point[2]:.1f})")
        
        # Get marker points (pattern-relative coordinates + offset)
        if patterns:
            marker_array = self.pattern_visualizer.create_pattern_markers(
                patterns,
                pattern_spacing=0.0,  # No spacing for coordinate comparison
                voxel_size=voxel_size
            )
            if marker_array.markers:
                marker = marker_array.markers[0]
                print(f"Marker points (pattern coordinates):")
                for i, point in enumerate(marker.points):
                    print(f"  Point {i}: ({point.x:.1f}, {point.y:.1f}, {point.z:.1f})")
        
        # Manual calculation of expected world coordinates
        print(f"Expected world coordinates (manual calculation):")
        if patterns:
            pattern = patterns[0]
            occupied_pos = np.where(pattern)
            origin = (1.0, 2.0, 3.0)  # From compressed_msg.voxel_grid_origin
            
            for i in range(len(occupied_pos[0])):
                vx, vy, vz = occupied_pos[0][i], occupied_pos[1][i], occupied_pos[2][i]
                # World coordinate = origin + (voxel_index + 0.5) * voxel_size
                world_x = origin[0] + (vx + 0.5) * voxel_size
                world_y = origin[1] + (vy + 0.5) * voxel_size
                world_z = origin[2] + (vz + 0.5) * voxel_size
                print(f"  Manual {i}: ({world_x:.1f}, {world_y:.1f}, {world_z:.1f})")
    
    def _create_corner_pattern(self):
        """Create pattern with corners of 8x8x8 block occupied"""
        pattern_data = [0] * 64  # 8^3 = 512 bits = 64 bytes
        
        # Corner positions in 8x8x8 block
        corners = [
            (0, 0, 0), (7, 0, 0), (0, 7, 0), (7, 7, 0),
            (0, 0, 7), (7, 0, 7), (0, 7, 7), (7, 7, 7)
        ]
        
        for x, y, z in corners:
            # Convert to linear index: z * 64 + y * 8 + x
            bit_index = z * 64 + y * 8 + x
            byte_index = bit_index // 8
            bit_offset = bit_index % 8
            
            if byte_index < 64:
                pattern_data[byte_index] |= (1 << bit_offset)
        
        return pattern_data
    
    def _create_specific_pattern(self):
        """Create pattern with specific voxels for coordinate testing"""
        pattern_data = [0] * 64
        
        # Specific positions for testing
        positions = [(0, 0, 0), (1, 0, 0), (0, 1, 0), (1, 1, 1)]
        
        for x, y, z in positions:
            bit_index = z * 64 + y * 8 + x
            byte_index = bit_index // 8
            bit_offset = bit_index % 8
            
            if byte_index < 64:
                pattern_data[byte_index] |= (1 << bit_offset)
        
        return pattern_data


if __name__ == '__main__':
    unittest.main(verbosity=2)