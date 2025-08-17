#!/usr/bin/env python3
"""
Decompressor module for reconstructing point clouds from compressed data
"""

import numpy as np
from typing import Tuple, List, Optional


class Decompressor:
    """Handles decompression of compressed point cloud data"""
    
    def __init__(self, voxel_size: float = 0.01, block_size: int = 8):
        """
        Initialize the decompressor
        
        Args:
            voxel_size: Size of each voxel in meters
            block_size: Size of each block (e.g., 8x8x8)
        """
        self.voxel_size = voxel_size
        self.block_size = block_size
        
    def decompress(self, compressed_msg) -> np.ndarray:
        """
        Decompress a CompressedPointCloud message into points
        
        Args:
            compressed_msg: CompressedPointCloud ROS message
            
        Returns:
            numpy array of shape (N, 3) containing point coordinates
        """
        if not compressed_msg.block_indices:
            return np.array([])
            
        # Update parameters from message BEFORE extracting patterns
        self.voxel_size = compressed_msg.compression_settings.voxel_size
        self.block_size = compressed_msg.compression_settings.block_size
        
        # Extract pattern dictionary (now block_size is updated)
        patterns = self._extract_patterns(compressed_msg.pattern_dictionary)
        
        # Get grid dimensions - must be in voxel units, not meters
        grid_dims = (
            max(1, int(compressed_msg.voxel_grid_dimensions.x / self.voxel_size)),
            max(1, int(compressed_msg.voxel_grid_dimensions.y / self.voxel_size)),
            max(1, int(compressed_msg.voxel_grid_dimensions.z / self.voxel_size))
        )
        
        # Get grid origin
        origin = (
            compressed_msg.voxel_grid_origin.x,
            compressed_msg.voxel_grid_origin.y,
            compressed_msg.voxel_grid_origin.z
        )
        
        # Reconstruct voxel grid from blocks
        voxel_grid = self._reconstruct_voxel_grid(
            patterns, 
            compressed_msg.block_indices,
            grid_dims,
            self.block_size
        )
        
        # Convert voxel grid to points
        points = self._voxel_to_points(voxel_grid, self.voxel_size, origin)
        
        return points
        
    def _extract_patterns(self, pattern_dict) -> List[np.ndarray]:
        """
        Extract block patterns from dictionary
        
        Args:
            pattern_dict: PatternDictionary message
            
        Returns:
            List of numpy arrays representing block patterns
        """
        patterns = []
        
        if pattern_dict.num_patterns == 0:
            return patterns
            
        pattern_size = pattern_dict.pattern_size_bytes
        dict_data = bytes(pattern_dict.dictionary_data)
        
        # Extract each pattern
        for i in range(pattern_dict.num_patterns):
            start_idx = i * pattern_size
            end_idx = start_idx + pattern_size
            pattern_bytes = dict_data[start_idx:end_idx]
            
            # Convert bytes to 3D boolean array
            pattern_block = self._bytes_to_block(pattern_bytes)
            patterns.append(pattern_block)
            
        return patterns
        
    def _bytes_to_block(self, pattern_bytes: bytes) -> np.ndarray:
        """
        Convert byte pattern to 3D boolean block
        
        Args:
            pattern_bytes: Bytes representing the pattern
            
        Returns:
            3D numpy boolean array
        """
        block = np.zeros((self.block_size, self.block_size, self.block_size), dtype=bool)
        
        total_voxels = self.block_size ** 3
        
        for i in range(min(len(pattern_bytes) * 8, total_voxels)):
            byte_idx = i // 8
            bit_idx = i % 8
            
            if byte_idx < len(pattern_bytes):
                is_occupied = (pattern_bytes[byte_idx] >> bit_idx) & 1
                
                # Convert linear index to 3D coordinates
                z = i // (self.block_size * self.block_size)
                y = (i % (self.block_size * self.block_size)) // self.block_size
                x = i % self.block_size
                
                if x < self.block_size and y < self.block_size and z < self.block_size:
                    block[x, y, z] = bool(is_occupied)
                    
        return block
        
    def _reconstruct_voxel_grid(self, 
                                patterns: List[np.ndarray],
                                indices: List[int],
                                grid_shape: Tuple[int, int, int],
                                block_size: int) -> np.ndarray:
        """
        Reconstruct full voxel grid from block patterns and indices
        
        Args:
            patterns: List of block patterns
            indices: Block indices referencing patterns
            grid_shape: Shape of the voxel grid
            block_size: Size of each block
            
        Returns:
            3D boolean numpy array representing voxel grid
        """
        voxel_grid = np.zeros(grid_shape, dtype=bool)
        
        if not patterns or not indices:
            return voxel_grid
            
        # Calculate number of blocks in each dimension
        blocks_x = (grid_shape[0] + block_size - 1) // block_size
        blocks_y = (grid_shape[1] + block_size - 1) // block_size
        blocks_z = (grid_shape[2] + block_size - 1) // block_size
        
        # Place each block in the grid
        for block_idx, pattern_idx in enumerate(indices):
            if pattern_idx >= len(patterns):
                continue
                
            # Calculate block position in grid
            bz = block_idx // (blocks_x * blocks_y)
            by = (block_idx % (blocks_x * blocks_y)) // blocks_x
            bx = block_idx % blocks_x
            
            # Calculate voxel coordinates
            start_x = bx * block_size
            start_y = by * block_size
            start_z = bz * block_size
            
            end_x = min(start_x + block_size, grid_shape[0])
            end_y = min(start_y + block_size, grid_shape[1])
            end_z = min(start_z + block_size, grid_shape[2])
            
            # Copy pattern to grid with bounds checking
            pattern = patterns[pattern_idx]
            copy_x = end_x - start_x
            copy_y = end_y - start_y
            copy_z = end_z - start_z
            
            # Ensure we don't copy more than the pattern or grid allows
            if copy_x > 0 and copy_y > 0 and copy_z > 0:
                pattern_copy_x = min(copy_x, pattern.shape[0])
                pattern_copy_y = min(copy_y, pattern.shape[1])
                pattern_copy_z = min(copy_z, pattern.shape[2])
                
                voxel_grid[start_x:start_x+pattern_copy_x, 
                          start_y:start_y+pattern_copy_y, 
                          start_z:start_z+pattern_copy_z] = \
                    pattern[:pattern_copy_x, :pattern_copy_y, :pattern_copy_z]
                
        return voxel_grid
        
    def _voxel_to_points(self, 
                        voxel_grid: np.ndarray,
                        voxel_size: float,
                        origin: Tuple[float, float, float]) -> np.ndarray:
        """
        Convert voxel grid to point cloud
        
        Args:
            voxel_grid: 3D boolean array
            voxel_size: Size of each voxel
            origin: Origin of the voxel grid
            
        Returns:
            Nx3 numpy array of point coordinates
        """
        # Find occupied voxels
        occupied = np.where(voxel_grid)
        
        if len(occupied[0]) == 0:
            return np.array([])
            
        # Calculate point positions (voxel centers)
        points = np.zeros((len(occupied[0]), 3))
        points[:, 0] = origin[0] + (occupied[0] + 0.5) * voxel_size
        points[:, 1] = origin[1] + (occupied[1] + 0.5) * voxel_size
        points[:, 2] = origin[2] + (occupied[2] + 0.5) * voxel_size
        
        return points