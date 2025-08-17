#!/usr/bin/env python3
"""
Direct test of pointcloud compression and visualization components
without ROS2 infrastructure
"""
import sys
import os
import numpy as np

# Add paths for imports
sys.path.insert(0, '/home/ryo/image_compressor_ws/src')

from pointcloud_compressor.pointcloud_compressor.voxel_processor import VoxelProcessor
from pointcloud_compressor.pointcloud_compressor.pattern_dictionary_builder import PatternDictionaryBuilder
from pointcloud_compressor.pointcloud_compressor.pattern_encoder import PatternEncoder
from compressed_viewer.compressed_viewer.decompressor import Decompressor
from compressed_viewer.compressed_viewer.pattern_marker_visualizer import PatternMarkerVisualizer

def create_test_pointcloud():
    """Create a simple 2x2x2 cube point cloud"""
    points = np.array([
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [1.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 1.0],
        [0.0, 1.0, 1.0],
        [1.0, 1.0, 1.0]
    ], dtype=np.float32)
    return points

def test_compression_visualization_pipeline():
    """Test the complete pipeline with min_points_threshold=1"""
    
    print("=" * 50)
    print("Testing Compression and Visualization Pipeline")
    print("with min_points_threshold=1")
    print("=" * 50)
    
    # Create test point cloud
    points = create_test_pointcloud()
    print(f"\nCreated test point cloud with {len(points)} points")
    for i, p in enumerate(points):
        print(f"  Point {i}: ({p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f})")
    
    # Parameters
    voxel_size = 1.0
    min_points_threshold = 1
    block_size = 8
    
    print(f"\nParameters:")
    print(f"  voxel_size: {voxel_size}")
    print(f"  min_points_threshold: {min_points_threshold}")
    print(f"  block_size: {block_size}")
    
    # Step 1: Voxelize
    print("\n--- Step 1: Voxelization ---")
    voxel_processor = VoxelProcessor(voxel_size, min_points_threshold)
    voxel_grid = voxel_processor.process(points)
    
    print(f"Voxel grid shape: {voxel_grid.shape}")
    occupied_voxels = np.sum(voxel_grid)
    print(f"Occupied voxels: {occupied_voxels}")
    
    # Find occupied positions
    occupied_positions = np.argwhere(voxel_grid)
    print("Occupied voxel positions:")
    for pos in occupied_positions:
        print(f"  ({pos[0]}, {pos[1]}, {pos[2]})")
    
    # Step 2: Build pattern dictionary
    print("\n--- Step 2: Pattern Dictionary ---")
    dict_builder = PatternDictionaryBuilder(block_size=block_size)
    blocks = dict_builder.extract_blocks(voxel_grid)
    print(f"Extracted {len(blocks)} blocks")
    
    pattern_dict = dict_builder.build_dictionary(blocks, target_patterns=128)
    print(f"Dictionary has {len(pattern_dict)} unique patterns")
    
    # Step 3: Encode
    print("\n--- Step 3: Encoding ---")
    encoder = PatternEncoder(pattern_dict, block_size=block_size)
    indices = encoder.encode(blocks)
    print(f"Encoded to {len(indices)} block indices")
    
    # Step 4: Create mock compressed message for visualization
    print("\n--- Step 4: Visualization ---")
    
    # Calculate grid dimensions
    grid_min = np.min(points, axis=0)
    grid_max = np.max(points, axis=0)
    grid_size_meters = grid_max - grid_min
    
    print(f"Grid bounds: ({grid_min[0]:.1f}, {grid_min[1]:.1f}, {grid_min[2]:.1f}) to ({grid_max[0]:.1f}, {grid_max[1]:.1f}, {grid_max[2]:.1f})")
    print(f"Grid size (meters): ({grid_size_meters[0]:.1f}, {grid_size_meters[1]:.1f}, {grid_size_meters[2]:.1f})")
    
    # Create mock message
    class MockVector3:
        def __init__(self, x, y, z):
            self.x = x
            self.y = y
            self.z = z
    
    class MockCompressionSettings:
        def __init__(self):
            self.voxel_size = voxel_size
            self.min_points_threshold = min_points_threshold
            self.block_size = block_size
    
    class MockPatternDictionary:
        def __init__(self):
            self.num_patterns = len(pattern_dict)
            self.pattern_size_bytes = (block_size ** 3 + 7) // 8
            # Flatten dictionary patterns to bytes
            self.dictionary_data = []
            for pattern in pattern_dict:
                pattern_bytes = np.packbits(pattern.flatten())
                self.dictionary_data.extend(pattern_bytes.tolist())
    
    class MockCompressedMessage:
        def __init__(self):
            self.voxel_grid_dimensions = MockVector3(
                grid_size_meters[0], 
                grid_size_meters[1], 
                grid_size_meters[2]
            )
            self.voxel_grid_origin = MockVector3(grid_min[0], grid_min[1], grid_min[2])
            self.compression_settings = MockCompressionSettings()
            self.pattern_dictionary = MockPatternDictionary()
            self.block_indices = indices
            self.blocks_x = (voxel_grid.shape[0] + block_size - 1) // block_size
            self.blocks_y = (voxel_grid.shape[1] + block_size - 1) // block_size
            self.blocks_z = (voxel_grid.shape[2] + block_size - 1) // block_size
    
    msg = MockCompressedMessage()
    
    # Test decompressor
    print("\n--- Testing Decompressor ---")
    decompressor = Decompressor()
    decompressed_points = decompressor.decompress(msg)
    print(f"Decompressed {len(decompressed_points)} points")
    
    if len(decompressed_points) > 0:
        print("Decompressed point positions:")
        for i, p in enumerate(decompressed_points):
            print(f"  Point {i}: ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f})")
    
    # Test visualizer
    print("\n--- Testing PatternMarkerVisualizer ---")
    visualizer = PatternMarkerVisualizer()
    markers = visualizer.create_pattern_markers(msg)
    
    total_marker_points = 0
    for marker in markers:
        total_marker_points += len(marker.points)
    
    print(f"Created {len(markers)} markers with {total_marker_points} total points")
    
    # Verify results
    print("\n--- Verification ---")
    print(f"Original points: {len(points)}")
    print(f"Occupied voxels: {occupied_voxels}")
    print(f"Decompressed points: {len(decompressed_points)}")
    print(f"Visualization points: {total_marker_points}")
    
    if occupied_voxels == len(decompressed_points) == total_marker_points:
        print("✓ SUCCESS: All counts match!")
    else:
        print("✗ FAILURE: Count mismatch detected")
        
    # Check if all original points are represented
    if occupied_voxels >= len(points):
        print(f"✓ All {len(points)} original points are represented in {occupied_voxels} voxels")
    else:
        print(f"✗ Only {occupied_voxels} voxels for {len(points)} points")
    
    return occupied_voxels == len(decompressed_points) == total_marker_points

if __name__ == "__main__":
    success = test_compression_visualization_pipeline()
    sys.exit(0 if success else 1)