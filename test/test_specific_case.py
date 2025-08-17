#!/usr/bin/env python3
"""
Test specific case to debug the issue
"""

import numpy as np
import sys
sys.path.insert(0, '/home/ryo/image_compressor_ws/src/compressed_viewer')

from compressed_viewer.decompressor import Decompressor
from test_compression_coverage import MockCompressedPointCloud


def test_specific_case():
    """Test the exact case that's failing"""
    print("="*60)
    print("Testing Simple 2x2x2 Grid")
    print("="*60)
    
    # Same points as in failing test
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
    
    voxel_size = 1.0
    block_size = 8
    
    # Create mock message
    msg = MockCompressedPointCloud(points, voxel_size, min_points_threshold=1, block_size=block_size)
    
    print(f"Original points: {len(points)}")
    print(f"Voxel grid dimensions (meters): ({msg.voxel_grid_dimensions.x:.2f}, {msg.voxel_grid_dimensions.y:.2f}, {msg.voxel_grid_dimensions.z:.2f})")
    print(f"Voxel grid origin: ({msg.voxel_grid_origin.x:.2f}, {msg.voxel_grid_origin.y:.2f}, {msg.voxel_grid_origin.z:.2f})")
    print(f"Voxel size: {msg.compression_settings.voxel_size}")
    print(f"Block size: {msg.compression_settings.block_size}")
    print(f"Occupied voxels: {msg._occupied_voxels}")
    print(f"Total blocks: {msg.total_blocks}")
    print(f"Block indices: {msg.block_indices[:10]}...")  # Show first 10
    print(f"Unique patterns: {msg.pattern_dictionary.num_patterns}")
    
    # Test decompressor
    decompressor = Decompressor()
    
    # Check what decompressor calculates for grid dimensions
    grid_dims = (
        max(1, int(msg.voxel_grid_dimensions.x / msg.compression_settings.voxel_size)),
        max(1, int(msg.voxel_grid_dimensions.y / msg.compression_settings.voxel_size)),
        max(1, int(msg.voxel_grid_dimensions.z / msg.compression_settings.voxel_size))
    )
    print(f"\nDecompressor grid dimensions (voxels): {grid_dims}")
    
    # Extract patterns manually
    decompressor.block_size = msg.compression_settings.block_size
    patterns = decompressor._extract_patterns(msg.pattern_dictionary)
    print(f"Extracted {len(patterns)} patterns")
    
    for i, pattern in enumerate(patterns[:3]):  # Show first 3
        occupied = np.sum(pattern)
        print(f"  Pattern {i}: occupied={occupied}")
        if occupied > 0:
            occupied_pos = np.where(pattern)
            for j in range(min(5, len(occupied_pos[0]))):  # Show first 5
                x, y, z = occupied_pos[0][j], occupied_pos[1][j], occupied_pos[2][j]
                print(f"    Occupied at: ({x}, {y}, {z})")
    
    # Reconstruct voxel grid
    voxel_grid = decompressor._reconstruct_voxel_grid(
        patterns,
        msg.block_indices,
        grid_dims,
        msg.compression_settings.block_size
    )
    
    print(f"\nReconstructed voxel grid shape: {voxel_grid.shape}")
    print(f"Occupied voxels in reconstructed grid: {np.sum(voxel_grid)}")
    
    # Show occupied positions
    occupied_pos = np.where(voxel_grid)
    print(f"Occupied positions in grid:")
    for i in range(len(occupied_pos[0])):
        x, y, z = occupied_pos[0][i], occupied_pos[1][i], occupied_pos[2][i]
        print(f"  ({x}, {y}, {z})")
    
    # Full decompress
    decompressed = decompressor.decompress(msg)
    print(f"\nDecompressed points: {len(decompressed)}")
    for i, p in enumerate(decompressed):
        print(f"  Point {i}: ({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f})")
    
    # Compare with expected
    print(f"\n--- Analysis ---")
    print(f"Expected 8 points, got {len(decompressed)}")
    
    if len(decompressed) != 8:
        print("❌ PROBLEM IDENTIFIED!")
        print(f"  Grid dimensions mismatch or block indexing issue")
        print(f"  Mock created {msg._occupied_voxels} occupied voxels")
        print(f"  Decompressor found {np.sum(voxel_grid)} occupied voxels")
        print(f"  Number of blocks: {msg.total_blocks}")
        print(f"  blocks_x={msg.blocks_x}, blocks_y={msg.blocks_y}, blocks_z={msg.blocks_z}")
    else:
        print("✅ Working correctly!")


if __name__ == "__main__":
    test_specific_case()