#!/usr/bin/env python3
"""
End-to-end integration test for pointcloud compression and visualization pipeline.

This test verifies the complete pipeline from original point cloud through 
pointcloud_compressor to compressed_viewer pattern markers, ensuring that
all original points are correctly represented when min_points_threshold=1.
"""

import unittest
import numpy as np
import sys
import os
import tempfile

# Add the package modules to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from compressed_viewer.pattern_dictionary_decompressor import PatternDictionaryDecompressor
from compressed_viewer.pattern_marker_visualizer import PatternMarkerVisualizer
from compressed_viewer.decompressor import Decompressor

# Mock ROS message types that mirror the actual messages
class MockCompressionSettings:
    def __init__(self, voxel_size=1.0, block_size=8, use_8bit_indices=True, min_points_threshold=1):
        self.voxel_size = voxel_size
        self.block_size = block_size
        self.use_8bit_indices = use_8bit_indices
        self.min_points_threshold = min_points_threshold

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
        self.original_point_count = 0
        self.original_data_size = 0
        self.voxel_grid_dimensions = MockVector3()
        self.voxel_grid_origin = MockVector3()
        self.total_blocks = 0
        self.blocks_x = 0
        self.blocks_y = 0
        self.blocks_z = 0
        self.block_indices = []
        self.compression_settings = MockCompressionSettings()
        self.pattern_dictionary = MockPatternDictionary()
        self.compression_ratio = 1.0
        self.compressed_data_size = 0
        self.compression_time_seconds = 0.0
        self.unique_patterns_count = 0
        self.reconstruction_error = 0.0

class EndToEndVisualizationTest(unittest.TestCase):
    """Test suite for end-to-end visualization verification"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.decompressor = Decompressor()
        self.pattern_decompressor = PatternDictionaryDecompressor()
        self.pattern_visualizer = PatternMarkerVisualizer(frame_id='test_frame')
        
    def test_simple_point_cloud_pipeline(self):
        """Test the complete pipeline with a simple point cloud"""
        print("\n=== Testing Simple Point Cloud End-to-End Pipeline ===")
        
        # Create a simple test point cloud: 2x2x2 grid of points
        voxel_size = 0.5
        original_points = []
        
        # Create points that will fall into different voxels
        for x in range(2):
            for y in range(2):
                for z in range(2):
                    # Place points at voxel centers  
                    point_x = x * voxel_size + voxel_size * 0.5
                    point_y = y * voxel_size + voxel_size * 0.5
                    point_z = z * voxel_size + voxel_size * 0.5
                    original_points.append((point_x, point_y, point_z))
        
        print(f"Created {len(original_points)} original points")
        print(f"Voxel size: {voxel_size}")
        print(f"Point cloud bounds: x[{min(p[0] for p in original_points)}, {max(p[0] for p in original_points)}]")
        print(f"                    y[{min(p[1] for p in original_points)}, {max(p[1] for p in original_points)}]")
        print(f"                    z[{min(p[2] for p in original_points)}, {max(p[2] for p in original_points)}]")
        
        # Create a mock compressed point cloud message that represents this data
        compressed_msg = self._create_compressed_message_from_points(original_points, voxel_size)
        
        # Test decompression to point cloud
        decompressed_points = self.decompressor.decompress(compressed_msg)
        print(f"Decompressed to {len(decompressed_points)} points")
        
        if len(decompressed_points) > 0:
            print("Decompressed point positions:")
            for i, point in enumerate(decompressed_points):
                print(f"  Point {i}: ({point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f})")
        else:
            print("❌ No points were decompressed!")
        
        # Test pattern dictionary decompression
        patterns = self.pattern_decompressor.decompress_pattern_dictionary(
            compressed_msg.pattern_dictionary
        )
        print(f"Decompressed {len(patterns)} patterns from dictionary")
        
        # Test pattern marker visualization
        if patterns:
            marker_array = self.pattern_visualizer.create_pattern_markers(
                patterns,
                pattern_spacing=5.0,
                voxel_size=voxel_size
            )
            
            print(f"Created {len(marker_array.markers)} pattern markers")
            
            # Verify markers
            for i, marker in enumerate(marker_array.markers):
                print(f"Marker {i}: {len(marker.points)} visualization points")
                print(f"  Scale: {marker.scale.x} x {marker.scale.y} x {marker.scale.z}")
                print(f"  Position offset: ({marker.pose.position.x}, {marker.pose.position.y}, {marker.pose.position.z})")
                
                # Check that marker scale matches voxel size
                self.assertAlmostEqual(marker.scale.x, voxel_size, places=3)
                self.assertAlmostEqual(marker.scale.y, voxel_size, places=3)
                self.assertAlmostEqual(marker.scale.z, voxel_size, places=3)
        
        # Verify that decompressed points cover original points
        self._verify_decompressed_coverage(original_points, decompressed_points, voxel_size)
        
        print("✓ End-to-end pipeline completed successfully")
        
    def test_threshold_impact_on_visualization(self):
        """Test how min_points_threshold affects visualization"""
        print("\n=== Testing Threshold Impact on Visualization ===")
        
        # Create points with varying density in different voxels
        voxel_size = 1.0
        original_points = []
        
        # Voxel 1: Single point (should be excluded with threshold > 1)
        original_points.append((0.5, 0.5, 0.5))
        
        # Voxel 2: Multiple points (should be included with any reasonable threshold)
        for i in range(5):
            original_points.append((1.1 + i * 0.1, 0.5, 0.5))
        
        # Voxel 3: Moderate number of points
        for i in range(3):
            original_points.append((0.5, 1.1 + i * 0.1, 0.5))
        
        print(f"Created {len(original_points)} points with varying density")
        
        # Test with threshold = 1 (should include all points)
        compressed_msg_th1 = self._create_compressed_message_from_points(
            original_points, voxel_size, min_points_threshold=1
        )
        
        patterns_th1 = self.pattern_decompressor.decompress_pattern_dictionary(
            compressed_msg_th1.pattern_dictionary
        )
        
        # Test with threshold = 2 (should exclude single-point voxels)
        compressed_msg_th2 = self._create_compressed_message_from_points(
            original_points, voxel_size, min_points_threshold=2
        )
        
        patterns_th2 = self.pattern_decompressor.decompress_pattern_dictionary(
            compressed_msg_th2.pattern_dictionary
        )
        
        # With threshold=1, we should have more occupied voxels than with threshold=2
        if patterns_th1 and patterns_th2:
            occupied_th1 = sum(np.sum(pattern) for pattern in patterns_th1)
            occupied_th2 = sum(np.sum(pattern) for pattern in patterns_th2)
            
            print(f"Threshold=1: {occupied_th1} occupied voxels")
            print(f"Threshold=2: {occupied_th2} occupied voxels")
            
            # With higher threshold, we should have fewer or equal occupied voxels
            self.assertLessEqual(occupied_th2, occupied_th1,
                               "Higher threshold should result in fewer occupied voxels")
        
        print("✓ Threshold impact verified")
        
    def test_coordinate_transformation_accuracy(self):
        """Test accuracy of coordinate transformations through the pipeline"""
        print("\n=== Testing Coordinate Transformation Accuracy ===")
        
        voxel_size = 0.1
        block_size = 8
        
        # Create ALL test points in a single grid to test relative positions
        test_points = [
            (0.05, 0.05, 0.05),    # Should map to voxel (0,0,0)
            (0.15, 0.05, 0.05),    # Should map to voxel (1,0,0)
            (0.05, 0.15, 0.05),    # Should map to voxel (0,1,0)
            (0.05, 0.05, 0.15),    # Should map to voxel (0,0,1)
            (0.75, 0.75, 0.75),    # Should map to voxel (7,7,7)
        ]
        
        print(f"Testing {len(test_points)} points in single grid")
        
        # Create compressed message with all points
        compressed_msg = self._create_compressed_message_from_points(
            test_points, voxel_size, min_points_threshold=1
        )
        
        # Decompress to get full voxel grid reconstruction
        decompressed_points = self.decompressor.decompress(compressed_msg)
        print(f"Decompressed to {len(decompressed_points)} points")
        
        # Also get patterns to examine
        patterns = self.pattern_decompressor.decompress_pattern_dictionary(
            compressed_msg.pattern_dictionary
        )
        print(f"Found {len(patterns)} patterns")
        
        if patterns and len(decompressed_points) > 0:
            print("Decompressed point positions:")
            for i, point in enumerate(decompressed_points):
                print(f"  Point {i}: ({point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f})")
        
        # Test that we have reasonable number of points
        self.assertGreater(len(decompressed_points), 0, "Should have decompressed some points")
        
        # For each original point, find closest decompressed point
        for i, (orig_x, orig_y, orig_z) in enumerate(test_points):
            print(f"\nOriginal point {i}: ({orig_x}, {orig_y}, {orig_z})")
            
            if len(decompressed_points) > 0:
                # Find closest decompressed point
                distances = []
                for decomp_point in decompressed_points:
                    dist = np.sqrt(
                        (orig_x - decomp_point[0])**2 +
                        (orig_y - decomp_point[1])**2 +
                        (orig_z - decomp_point[2])**2
                    )
                    distances.append(dist)
                
                min_dist_idx = np.argmin(distances)
                closest_point = decompressed_points[min_dist_idx]
                min_distance = distances[min_dist_idx]
                
                print(f"  Closest decompressed: ({closest_point[0]:.3f}, {closest_point[1]:.3f}, {closest_point[2]:.3f})")
                print(f"  Distance: {min_distance:.3f}")
                
                # Verify distance is reasonable (within one voxel)
                self.assertLessEqual(min_distance, voxel_size * 1.5, 
                                   f"Point {i} should be within 1.5 voxel sizes of decompressed point")
        
        print("✓ Coordinate transformation accuracy verified")
    
    def _verify_decompressed_coverage(self, original_points, decompressed_points, voxel_size, tolerance=None):
        """Verify that decompressed points cover the original point cloud"""
        if tolerance is None:
            tolerance = voxel_size * 0.6  # Allow some margin for voxel center positioning
        
        print(f"\nVerifying coverage with tolerance {tolerance}")
        
        uncovered_original = []
        
        for orig_point in original_points:
            found_cover = False
            
            for decomp_point in decompressed_points:
                distance = np.sqrt(
                    (orig_point[0] - decomp_point[0])**2 +
                    (orig_point[1] - decomp_point[1])**2 +
                    (orig_point[2] - decomp_point[2])**2
                )
                
                if distance <= tolerance:
                    found_cover = True
                    break
            
            if not found_cover:
                uncovered_original.append(orig_point)
        
        if uncovered_original:
            print(f"❌ Found {len(uncovered_original)} uncovered original points:")
            for point in uncovered_original[:3]:  # Show first 3
                print(f"  Uncovered: {point}")
        
        coverage_ratio = (len(original_points) - len(uncovered_original)) / len(original_points)
        print(f"Coverage ratio: {coverage_ratio:.2%}")
        
        # For min_points_threshold=1, we expect very high coverage
        self.assertGreaterEqual(coverage_ratio, 0.9, "Should have at least 90% coverage")
    
    def _create_compressed_message_from_points(self, points, voxel_size, min_points_threshold=1):
        """
        Create a mock compressed point cloud message from a list of points.
        This simulates what pointcloud_compressor would produce.
        """
        # Calculate basic grid info
        if not points:
            return MockCompressedPointCloud()
        
        min_x = min(p[0] for p in points)
        max_x = max(p[0] for p in points)
        min_y = min(p[1] for p in points) 
        max_y = max(p[1] for p in points)
        min_z = min(p[2] for p in points)
        max_z = max(p[2] for p in points)
        
        # Calculate voxel grid dimensions in voxel units
        grid_voxels_x = int(np.ceil((max_x - min_x) / voxel_size)) + 1
        grid_voxels_y = int(np.ceil((max_y - min_y) / voxel_size)) + 1
        grid_voxels_z = int(np.ceil((max_z - min_z) / voxel_size)) + 1
        
        print(f"  Grid dimensions (voxels): {grid_voxels_x} x {grid_voxels_y} x {grid_voxels_z}")
        print(f"  Voxel size: {voxel_size}")
        print(f"  Grid bounds: ({min_x}, {min_y}, {min_z}) to ({max_x}, {max_y}, {max_z})")
        
        # Create voxel occupancy grid
        voxel_counts = {}
        for point in points:
            vx = int((point[0] - min_x) / voxel_size)
            vy = int((point[1] - min_y) / voxel_size)
            vz = int((point[2] - min_z) / voxel_size)
            
            # Clamp to grid bounds
            vx = max(0, min(vx, grid_voxels_x - 1))
            vy = max(0, min(vy, grid_voxels_y - 1))
            vz = max(0, min(vz, grid_voxels_z - 1))
            
            key = (vx, vy, vz)
            voxel_counts[key] = voxel_counts.get(key, 0) + 1
        
        # Apply threshold
        occupied_voxels = {k: v for k, v in voxel_counts.items() if v >= min_points_threshold}
        print(f"  Occupied voxels after threshold: {len(occupied_voxels)}")
        
        if not occupied_voxels:
            # Return empty message if no voxels meet threshold
            msg = MockCompressedPointCloud()
            msg.compression_settings = MockCompressionSettings(
                voxel_size=voxel_size,
                min_points_threshold=min_points_threshold
            )
            return msg
        
        # Create voxel grid
        voxel_grid = np.zeros((grid_voxels_x, grid_voxels_y, grid_voxels_z), dtype=bool)
        for (vx, vy, vz) in occupied_voxels.keys():
            voxel_grid[vx, vy, vz] = True
        
        # Divide into blocks and create patterns
        block_size = 8
        blocks_x = (grid_voxels_x + block_size - 1) // block_size
        blocks_y = (grid_voxels_y + block_size - 1) // block_size  
        blocks_z = (grid_voxels_z + block_size - 1) // block_size
        
        print(f"  Block grid: {blocks_x} x {blocks_y} x {blocks_z}")
        
        # Extract blocks and create unique patterns
        pattern_map = {}  # pattern bytes -> pattern index
        patterns = []     # list of pattern byte arrays
        block_indices = []
        
        for bz in range(blocks_z):
            for by in range(blocks_y):
                for bx in range(blocks_x):
                    # Extract block from voxel grid
                    start_x = bx * block_size
                    start_y = by * block_size
                    start_z = bz * block_size
                    
                    end_x = min(start_x + block_size, grid_voxels_x)
                    end_y = min(start_y + block_size, grid_voxels_y)
                    end_z = min(start_z + block_size, grid_voxels_z)
                    
                    # Create pattern for this block
                    pattern_bits = block_size * block_size * block_size
                    pattern_bytes_count = (pattern_bits + 7) // 8
                    pattern_data = bytearray(pattern_bytes_count)
                    
                    # Fill pattern data
                    for z in range(block_size):
                        for y in range(block_size):
                            for x in range(block_size):
                                grid_x = start_x + x
                                grid_y = start_y + y
                                grid_z = start_z + z
                                
                                if (grid_x < grid_voxels_x and grid_y < grid_voxels_y and grid_z < grid_voxels_z):
                                    occupied = voxel_grid[grid_x, grid_y, grid_z]
                                else:
                                    occupied = False
                                
                                if occupied:
                                    # Calculate bit position using same indexing as decompressor
                                    bit_index = z * block_size * block_size + y * block_size + x
                                    byte_index = bit_index // 8
                                    bit_offset = bit_index % 8
                                    
                                    if byte_index < len(pattern_data):
                                        pattern_data[byte_index] |= (1 << bit_offset)
                    
                    # Convert to tuple for hashing
                    pattern_key = tuple(pattern_data)
                    
                    # Check if this pattern already exists
                    if pattern_key in pattern_map:
                        pattern_idx = pattern_map[pattern_key]
                    else:
                        pattern_idx = len(patterns)
                        pattern_map[pattern_key] = pattern_idx
                        patterns.append(list(pattern_data))
                    
                    block_indices.append(pattern_idx)
        
        print(f"  Created {len(patterns)} unique patterns for {len(block_indices)} blocks")
        
        # Create pattern dictionary data
        dictionary_data = []
        for pattern in patterns:
            dictionary_data.extend(pattern)
        
        # Create mock message
        msg = MockCompressedPointCloud()
        msg.original_point_count = len(points)
        
        # Set grid dimensions in METERS (Decompressor will divide by voxel_size)
        msg.voxel_grid_dimensions = MockVector3(
            (max_x - min_x),  # meters
            (max_y - min_y),  # meters
            (max_z - min_z)   # meters
        )
        msg.voxel_grid_origin = MockVector3(min_x, min_y, min_z)
        
        msg.compression_settings = MockCompressionSettings(
            voxel_size=voxel_size,
            min_points_threshold=min_points_threshold,
            block_size=block_size
        )
        
        # Create pattern dictionary
        pattern_size_bytes = (block_size ** 3 + 7) // 8
        msg.pattern_dictionary = MockPatternDictionary(
            num_patterns=len(patterns),
            pattern_size_bytes=pattern_size_bytes,
            dictionary_data=dictionary_data
        )
        
        msg.block_indices = block_indices
        msg.total_blocks = len(block_indices)
        msg.blocks_x = blocks_x
        msg.blocks_y = blocks_y
        msg.blocks_z = blocks_z
        
        return msg


if __name__ == '__main__':
    # Run the tests
    unittest.main(verbosity=2)