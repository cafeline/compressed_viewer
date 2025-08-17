#!/usr/bin/env python3
"""
Component-level test for verifying point cloud coverage
when min_points_threshold=1
"""

import numpy as np
import sys
import os

# Add path for compressed_viewer imports
sys.path.insert(0, '/home/ryo/image_compressor_ws/src/compressed_viewer')

from compressed_viewer.decompressor import Decompressor
from compressed_viewer.pattern_dictionary_decompressor import PatternDictionaryDecompressor
from compressed_viewer.pattern_marker_visualizer import PatternMarkerVisualizer


class MockVector3:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


class MockCompressionSettings:
    def __init__(self, voxel_size=0.1, min_points_threshold=1, block_size=8):
        self.voxel_size = voxel_size
        self.min_points_threshold = min_points_threshold
        self.block_size = block_size


class MockPatternDictionary:
    def __init__(self, patterns, block_size=8):
        self.num_patterns = len(patterns)
        self.pattern_size_bytes = (block_size ** 3 + 7) // 8
        # Flatten patterns to bytes
        self.dictionary_data = []
        for pattern in patterns:
            pattern_bytes = self._pattern_to_bytes(pattern, block_size)
            self.dictionary_data.extend(pattern_bytes)
    
    def _pattern_to_bytes(self, pattern, block_size):
        """Convert 3D boolean pattern to byte array"""
        total_bits = block_size ** 3
        bytes_needed = (total_bits + 7) // 8
        result = bytearray(bytes_needed)
        
        bit_idx = 0
        for z in range(block_size):
            for y in range(block_size):
                for x in range(block_size):
                    if pattern[x, y, z]:
                        byte_idx = bit_idx // 8
                        bit_offset = bit_idx % 8
                        result[byte_idx] |= (1 << bit_offset)
                    bit_idx += 1
        
        return list(result)


class MockCompressedPointCloud:
    def __init__(self, points, voxel_size=0.1, min_points_threshold=1, block_size=8):
        """Create mock compressed message from points"""
        self.original_point_count = len(points)
        
        # Calculate bounds
        points_array = np.array(points)
        min_bounds = np.min(points_array, axis=0)
        max_bounds = np.max(points_array, axis=0)
        
        # Set grid dimensions and origin
        # Calculate actual grid size in voxels first
        voxel_counts_x = int(np.ceil((max_bounds[0] - min_bounds[0]) / voxel_size)) + 1
        voxel_counts_y = int(np.ceil((max_bounds[1] - min_bounds[1]) / voxel_size)) + 1
        voxel_counts_z = int(np.ceil((max_bounds[2] - min_bounds[2]) / voxel_size)) + 1
        
        # Grid dimensions should be in meters (voxel_counts * voxel_size)
        self.voxel_grid_dimensions = MockVector3(
            voxel_counts_x * voxel_size,
            voxel_counts_y * voxel_size,
            voxel_counts_z * voxel_size
        )
        self.voxel_grid_origin = MockVector3(
            min_bounds[0], min_bounds[1], min_bounds[2]
        )
        
        # Compression settings
        self.compression_settings = MockCompressionSettings(
            voxel_size, min_points_threshold, block_size
        )
        
        # Voxelize points (use same voxel counts as calculated above)
        grid_voxels_x = voxel_counts_x
        grid_voxels_y = voxel_counts_y
        grid_voxels_z = voxel_counts_z
        
        voxel_grid = np.zeros((grid_voxels_x, grid_voxels_y, grid_voxels_z), dtype=bool)
        
        # Mark occupied voxels
        for point in points:
            vx = int((point[0] - min_bounds[0]) / voxel_size)
            vy = int((point[1] - min_bounds[1]) / voxel_size)
            vz = int((point[2] - min_bounds[2]) / voxel_size)
            
            # Clamp to grid bounds
            vx = max(0, min(vx, grid_voxels_x - 1))
            vy = max(0, min(vy, grid_voxels_y - 1))
            vz = max(0, min(vz, grid_voxels_z - 1))
            
            voxel_grid[vx, vy, vz] = True
        
        # Divide into blocks
        blocks_x = (grid_voxels_x + block_size - 1) // block_size
        blocks_y = (grid_voxels_y + block_size - 1) // block_size
        blocks_z = (grid_voxels_z + block_size - 1) // block_size
        
        self.blocks_x = blocks_x
        self.blocks_y = blocks_y
        self.blocks_z = blocks_z
        
        # Extract patterns from blocks
        patterns = []
        pattern_map = {}
        self.block_indices = []
        
        for bz in range(blocks_z):
            for by in range(blocks_y):
                for bx in range(blocks_x):
                    # Extract block pattern
                    pattern = np.zeros((block_size, block_size, block_size), dtype=bool)
                    
                    for z in range(block_size):
                        for y in range(block_size):
                            for x in range(block_size):
                                gx = bx * block_size + x
                                gy = by * block_size + y
                                gz = bz * block_size + z
                                
                                if gx < grid_voxels_x and gy < grid_voxels_y and gz < grid_voxels_z:
                                    pattern[x, y, z] = voxel_grid[gx, gy, gz]
                    
                    # Get or create pattern index
                    pattern_key = pattern.tobytes()
                    if pattern_key not in pattern_map:
                        pattern_map[pattern_key] = len(patterns)
                        patterns.append(pattern)
                    
                    self.block_indices.append(pattern_map[pattern_key])
        
        # Create pattern dictionary
        self.pattern_dictionary = MockPatternDictionary(patterns, block_size)
        self.total_blocks = len(self.block_indices)
        
        # Store for verification
        self._voxel_grid = voxel_grid
        self._occupied_voxels = np.sum(voxel_grid)


def test_simple_grid():
    """Test 1: Simple 2x2x2 grid coverage"""
    print("="*60)
    print("TEST 1: Simple 2x2x2 Grid Coverage")
    print("="*60)
    
    # Create 8 points
    points = [
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [1.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 1.0],
        [0.0, 1.0, 1.0],
        [1.0, 1.0, 1.0]
    ]
    
    voxel_size = 1.0  # Each point in separate voxel
    block_size = 8
    
    # Create mock compressed message
    msg = MockCompressedPointCloud(points, voxel_size, min_points_threshold=1, block_size=block_size)
    
    print(f"Original points: {len(points)}")
    print(f"Occupied voxels: {msg._occupied_voxels}")
    print(f"Total blocks: {msg.total_blocks}")
    print(f"Unique patterns: {msg.pattern_dictionary.num_patterns}")
    
    # Test decompressor
    decompressor = Decompressor()
    decompressed_points = decompressor.decompress(msg)
    
    print(f"Decompressed points: {len(decompressed_points)}")
    
    # Test pattern visualizer
    pattern_decompressor = PatternDictionaryDecompressor()
    patterns = pattern_decompressor.decompress_pattern_dictionary(msg.pattern_dictionary)
    
    visualizer = PatternMarkerVisualizer()
    markers = visualizer.create_pattern_markers(patterns, pattern_spacing=2.0, voxel_size=voxel_size)
    
    total_marker_points = sum(len(m.points) for m in markers.markers)
    print(f"Visualization points: {total_marker_points}")
    
    # Verify coverage
    if msg._occupied_voxels == len(decompressed_points) == total_marker_points == len(points):
        print("✅ PASS: All points correctly represented")
        return True
    else:
        print(f"❌ FAIL: Point count mismatch")
        print(f"  Expected: {len(points)}")
        print(f"  Occupied voxels: {msg._occupied_voxels}")
        print(f"  Decompressed: {len(decompressed_points)}")
        print(f"  Visualized: {total_marker_points}")
        return False


def test_single_point_threshold():
    """Test 2: Single point with different thresholds"""
    print("\n" + "="*60)
    print("TEST 2: Single Point with min_points_threshold")
    print("="*60)
    
    points = [[0.5, 0.5, 0.5]]
    voxel_size = 0.1
    block_size = 8
    
    # Test with threshold=1
    msg1 = MockCompressedPointCloud(points, voxel_size, min_points_threshold=1, block_size=block_size)
    
    print(f"Single point with min_points_threshold=1:")
    print(f"  Occupied voxels: {msg1._occupied_voxels}")
    
    # Decompress and verify
    decompressor = Decompressor()
    decompressed1 = decompressor.decompress(msg1)
    
    if msg1._occupied_voxels == 1 and len(decompressed1) == 1:
        print("✅ PASS: Single point captured with threshold=1")
        return True
    else:
        print(f"❌ FAIL: Expected 1 voxel and 1 point, got {msg1._occupied_voxels} voxels and {len(decompressed1)} points")
        return False


def test_dense_cluster():
    """Test 3: Dense cluster coverage"""
    print("\n" + "="*60)
    print("TEST 3: Dense Cluster Coverage")
    print("="*60)
    
    # Create 100 points in small area
    points = []
    for i in range(10):
        for j in range(10):
            points.append([i * 0.01, j * 0.01, 0.0])
    
    voxel_size = 0.05  # Should create 4 voxels (2x2)
    block_size = 8
    
    msg = MockCompressedPointCloud(points, voxel_size, min_points_threshold=1, block_size=block_size)
    
    print(f"Dense cluster: {len(points)} points in 0.1x0.1 area")
    print(f"Voxel size: {voxel_size}")
    print(f"Occupied voxels: {msg._occupied_voxels}")
    
    # Calculate expected voxels
    expected_voxels = 4  # 2x2 grid at this voxel size
    
    # Decompress
    decompressor = Decompressor()
    decompressed = decompressor.decompress(msg)
    
    # Visualize
    pattern_decompressor = PatternDictionaryDecompressor()
    patterns = pattern_decompressor.decompress_pattern_dictionary(msg.pattern_dictionary)
    
    visualizer = PatternMarkerVisualizer()
    markers = visualizer.create_pattern_markers(patterns, pattern_spacing=2.0, voxel_size=voxel_size)
    total_marker_points = sum(len(m.points) for m in markers.markers)
    
    print(f"Decompressed points: {len(decompressed)}")
    print(f"Visualization points: {total_marker_points}")
    
    if msg._occupied_voxels == expected_voxels and len(decompressed) == expected_voxels:
        print("✅ PASS: Dense cluster correctly voxelized")
        return True
    else:
        print(f"❌ FAIL: Expected {expected_voxels} voxels, got {msg._occupied_voxels}")
        return False


def test_pattern_coverage():
    """Test 4: Verify pattern markers cover all voxels"""
    print("\n" + "="*60)
    print("TEST 4: Pattern Marker Coverage")
    print("="*60)
    
    # Create points in specific pattern
    points = [
        [0.0, 0.0, 0.0],
        [0.1, 0.0, 0.0],
        [0.0, 0.1, 0.0],
        [0.1, 0.1, 0.0],
        [0.5, 0.5, 0.5],  # Isolated point
        [1.0, 1.0, 1.0],  # Another isolated point
    ]
    
    voxel_size = 0.2
    block_size = 8
    
    msg = MockCompressedPointCloud(points, voxel_size, min_points_threshold=1, block_size=block_size)
    
    print(f"Test points: {len(points)}")
    print(f"Occupied voxels: {msg._occupied_voxels}")
    
    # Get patterns from decompressor
    pattern_decompressor = PatternDictionaryDecompressor()
    patterns = pattern_decompressor.decompress_pattern_dictionary(msg.pattern_dictionary)
    
    print(f"Unique patterns: {len(patterns)}")
    
    # Count total occupied voxels in patterns
    total_pattern_voxels = 0
    for i, pattern in enumerate(patterns):
        occupied = np.sum(pattern)
        total_pattern_voxels += occupied
        if occupied > 0:
            print(f"  Pattern {i}: {occupied} occupied voxels")
    
    # Get visualization
    pattern_decompressor_vis = PatternDictionaryDecompressor()
    patterns_vis = pattern_decompressor_vis.decompress_pattern_dictionary(msg.pattern_dictionary)
    
    visualizer = PatternMarkerVisualizer()
    markers = visualizer.create_pattern_markers(patterns_vis, pattern_spacing=2.0, voxel_size=voxel_size)
    
    total_marker_points = sum(len(m.points) for m in markers.markers)
    print(f"Total visualization points: {total_marker_points}")
    
    # Decompress
    decompressor = Decompressor()
    decompressed = decompressor.decompress(msg)
    
    print(f"Decompressed points: {len(decompressed)}")
    
    # All should match
    if len(decompressed) == total_marker_points == msg._occupied_voxels:
        print("✅ PASS: Pattern markers correctly cover all voxels")
        return True
    else:
        print("❌ FAIL: Coverage mismatch")
        print(f"  Occupied voxels: {msg._occupied_voxels}")
        print(f"  Decompressed: {len(decompressed)}")
        print(f"  Visualized: {total_marker_points}")
        return False


def test_boundary_conditions():
    """Test 5: Boundary conditions and edge cases"""
    print("\n" + "="*60)
    print("TEST 5: Boundary Conditions")
    print("="*60)
    
    # Test with points at block boundaries
    block_size = 4
    voxel_size = 1.0
    
    # Points that should result in 3 occupied voxels
    points = [
        [0.5, 0.5, 0.5],  # Center of voxel (0,0,0)
        [2.5, 2.5, 2.5],  # Center of voxel (2,2,2)
        [4.5, 4.5, 4.5],  # Center of voxel (4,4,4)
    ]
    
    msg = MockCompressedPointCloud(points, voxel_size, min_points_threshold=1, block_size=block_size)
    
    print(f"Boundary test points: {len(points)}")
    print(f"Block size: {block_size}")
    print(f"Occupied voxels: {msg._occupied_voxels}")
    print(f"Total blocks: {msg.total_blocks}")
    
    # Decompress and verify
    decompressor = Decompressor()
    decompressed = decompressor.decompress(msg)
    
    # Visualize
    pattern_decompressor = PatternDictionaryDecompressor()
    patterns = pattern_decompressor.decompress_pattern_dictionary(msg.pattern_dictionary)
    
    visualizer = PatternMarkerVisualizer()
    markers = visualizer.create_pattern_markers(patterns, pattern_spacing=2.0, voxel_size=voxel_size)
    total_marker_points = sum(len(m.points) for m in markers.markers)
    
    print(f"Decompressed points: {len(decompressed)}")
    print(f"Visualization points: {total_marker_points}")
    
    if len(decompressed) == total_marker_points == msg._occupied_voxels == len(points):
        print("✅ PASS: Boundary conditions handled correctly")
        return True
    else:
        print("❌ FAIL: Boundary condition error")
        return False


def main():
    """Run all tests"""
    print("\n🧪 POINT CLOUD COVERAGE TEST SUITE 🧪")
    print("Testing min_points_threshold=1 with pattern markers")
    print("="*60 + "\n")
    
    results = []
    
    # Run tests
    results.append(("Simple Grid", test_simple_grid()))
    results.append(("Single Point", test_single_point_threshold()))
    results.append(("Dense Cluster", test_dense_cluster()))
    results.append(("Pattern Coverage", test_pattern_coverage()))
    results.append(("Boundary Conditions", test_boundary_conditions()))
    
    # Summary
    print("\n" + "="*60)
    print("TEST SUMMARY")
    print("="*60)
    
    passed = sum(1 for _, r in results if r)
    failed = sum(1 for _, r in results if not r)
    
    for test_name, result in results:
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"{test_name}: {status}")
    
    print(f"\nTotal: {passed} passed, {failed} failed")
    
    if failed == 0:
        print("\n🎉 All tests passed! Coverage is working correctly with min_points_threshold=1")
        return 0
    else:
        print(f"\n⚠️ {failed} test(s) failed. Issues found in coverage.")
        return 1


if __name__ == "__main__":
    exit(main())