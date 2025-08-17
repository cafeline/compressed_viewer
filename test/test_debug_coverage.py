#!/usr/bin/env python3
"""
Debug test to understand the decompression issue
"""

import numpy as np
import sys
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


class SimpleCompressedMessage:
    """Simplified mock for debugging"""
    def __init__(self):
        # Create a simple 2x2x2 voxel grid
        voxel_size = 1.0
        block_size = 8
        
        # Grid dimensions in meters
        self.voxel_grid_dimensions = MockVector3(2.0, 2.0, 2.0)  # 2x2x2 meters
        self.voxel_grid_origin = MockVector3(0.0, 0.0, 0.0)
        
        self.compression_settings = MockCompressionSettings(voxel_size, 1, block_size)
        
        # Create a simple pattern with 8 occupied voxels (2x2x2)
        pattern = np.zeros((block_size, block_size, block_size), dtype=bool)
        pattern[0, 0, 0] = True
        pattern[1, 0, 0] = True
        pattern[0, 1, 0] = True
        pattern[1, 1, 0] = True
        pattern[0, 0, 1] = True
        pattern[1, 0, 1] = True
        pattern[0, 1, 1] = True
        pattern[1, 1, 1] = True
        
        # Convert pattern to bytes
        pattern_bytes = []
        for z in range(block_size):
            for y in range(block_size):
                for x in range(block_size):
                    bit_idx = z * block_size * block_size + y * block_size + x
                    byte_idx = bit_idx // 8
                    bit_offset = bit_idx % 8
                    
                    while len(pattern_bytes) <= byte_idx:
                        pattern_bytes.append(0)
                    
                    if pattern[x, y, z]:
                        pattern_bytes[byte_idx] |= (1 << bit_offset)
        
        # Create pattern dictionary
        self.pattern_dictionary = type('obj', (object,), {
            'num_patterns': 1,
            'pattern_size_bytes': len(pattern_bytes),
            'dictionary_data': pattern_bytes
        })()
        
        # Single block using pattern 0
        self.block_indices = [0]
        self.total_blocks = 1
        self.blocks_x = 1
        self.blocks_y = 1
        self.blocks_z = 1


def debug_decompression():
    """Debug the decompression process step by step"""
    print("="*60)
    print("DEBUG: Decompression Process")
    print("="*60)
    
    # Create simple message
    msg = SimpleCompressedMessage()
    
    print(f"Grid dimensions (meters): ({msg.voxel_grid_dimensions.x}, {msg.voxel_grid_dimensions.y}, {msg.voxel_grid_dimensions.z})")
    print(f"Voxel size: {msg.compression_settings.voxel_size}")
    print(f"Block size: {msg.compression_settings.block_size}")
    print(f"Number of patterns: {msg.pattern_dictionary.num_patterns}")
    print(f"Block indices: {msg.block_indices}")
    
    # Test pattern decompressor
    print("\n--- Pattern Decompression ---")
    pattern_decompressor = PatternDictionaryDecompressor()
    patterns = pattern_decompressor.decompress_pattern_dictionary(msg.pattern_dictionary)
    
    print(f"Decompressed {len(patterns)} patterns")
    for i, pattern in enumerate(patterns):
        occupied = np.sum(pattern)
        print(f"  Pattern {i}: shape={pattern.shape}, occupied={occupied}")
        
        # Show occupied positions
        occupied_positions = np.where(pattern)
        for j in range(len(occupied_positions[0])):
            x = occupied_positions[0][j]
            y = occupied_positions[1][j]
            z = occupied_positions[2][j]
            print(f"    Occupied at: ({x}, {y}, {z})")
    
    # Test decompressor
    print("\n--- Full Decompression ---")
    decompressor = Decompressor()
    
    # Manually set block_size before decompression
    decompressor.block_size = msg.compression_settings.block_size
    
    # Extract patterns manually
    manual_patterns = decompressor._extract_patterns(msg.pattern_dictionary)
    print(f"Manually extracted {len(manual_patterns)} patterns")
    
    # Calculate grid dimensions in voxels
    grid_dims = (
        max(1, int(msg.voxel_grid_dimensions.x / msg.compression_settings.voxel_size)),
        max(1, int(msg.voxel_grid_dimensions.y / msg.compression_settings.voxel_size)),
        max(1, int(msg.voxel_grid_dimensions.z / msg.compression_settings.voxel_size))
    )
    print(f"Grid dimensions (voxels): {grid_dims}")
    
    # Reconstruct voxel grid
    voxel_grid = decompressor._reconstruct_voxel_grid(
        manual_patterns,
        msg.block_indices,
        grid_dims,
        msg.compression_settings.block_size
    )
    
    print(f"Voxel grid shape: {voxel_grid.shape}")
    print(f"Occupied voxels in grid: {np.sum(voxel_grid)}")
    
    # Show occupied positions in grid
    occupied_grid = np.where(voxel_grid)
    print(f"Occupied grid positions:")
    for i in range(len(occupied_grid[0])):
        x = occupied_grid[0][i]
        y = occupied_grid[1][i]
        z = occupied_grid[2][i]
        print(f"  Grid voxel at: ({x}, {y}, {z})")
    
    # Convert to points
    origin = (msg.voxel_grid_origin.x, msg.voxel_grid_origin.y, msg.voxel_grid_origin.z)
    points = decompressor._voxel_to_points(voxel_grid, msg.compression_settings.voxel_size, origin)
    
    print(f"\nFinal points: {len(points)}")
    for i, p in enumerate(points):
        print(f"  Point {i}: ({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f})")
    
    # Now test full decompress method
    print("\n--- Using decompress() method ---")
    decompressed = decompressor.decompress(msg)
    print(f"Decompressed points: {len(decompressed)}")
    for i, p in enumerate(decompressed):
        print(f"  Point {i}: ({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f})")
    
    # Test visualization
    print("\n--- Pattern Visualization ---")
    visualizer = PatternMarkerVisualizer()
    markers = visualizer.create_pattern_markers(patterns, pattern_spacing=2.0, voxel_size=msg.compression_settings.voxel_size)
    
    total_marker_points = sum(len(m.points) for m in markers.markers)
    print(f"Total visualization points: {total_marker_points}")
    
    # Summary
    print("\n--- SUMMARY ---")
    print(f"Expected: 8 occupied voxels")
    print(f"Pattern has: {np.sum(patterns[0])} occupied voxels")
    print(f"Grid has: {np.sum(voxel_grid)} occupied voxels")
    print(f"Decompressed: {len(decompressed)} points")
    print(f"Visualized: {total_marker_points} points")
    
    if len(decompressed) == 8:
        print("✅ SUCCESS: Decompression working correctly!")
    else:
        print("❌ FAILURE: Decompression issue detected")


if __name__ == "__main__":
    debug_decompression()