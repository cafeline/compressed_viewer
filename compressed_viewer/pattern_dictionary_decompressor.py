#!/usr/bin/env python3
"""
PatternDictionaryDecompressor module for extracting patterns from PatternDictionary messages
"""

import numpy as np
import zlib
from typing import List, Optional


class PatternDictionaryDecompressor:
    """Handles decompression of PatternDictionary messages into 3D boolean arrays"""
    
    def __init__(self):
        """Initialize the PatternDictionaryDecompressor"""
        pass
        
    def decompress_pattern_dictionary(self, 
                                    pattern_dict, 
                                    block_size: int = 8,
                                    validate_checksum: bool = False) -> List[np.ndarray]:
        """
        Decompress a PatternDictionary message into a list of 3D boolean arrays
        
        Args:
            pattern_dict: PatternDictionary ROS message
            block_size: Size of each block (e.g., 8 for 8x8x8 blocks)
            validate_checksum: Whether to validate CRC32 checksum
            
        Returns:
            List of numpy arrays representing block patterns
            
        Raises:
            ValueError: If pattern data is invalid or checksum validation fails
        """
        if block_size <= 0:
            raise ValueError("Block size must be positive")
            
        if pattern_dict.num_patterns == 0:
            return []
            
        # Validate checksum if requested
        if validate_checksum and hasattr(pattern_dict, 'checksum'):
            pattern_data = bytes(pattern_dict.dictionary_data)
            calculated_checksum = zlib.crc32(pattern_data) & 0xffffffff
            if calculated_checksum != pattern_dict.checksum:
                raise ValueError(f"Checksum validation failed: expected {pattern_dict.checksum}, got {calculated_checksum}")
        
        # Validate data size
        expected_total_size = pattern_dict.num_patterns * pattern_dict.pattern_size_bytes
        if len(pattern_dict.dictionary_data) < expected_total_size:
            raise ValueError(f"Insufficient pattern data: expected {expected_total_size} bytes, got {len(pattern_dict.dictionary_data)}")
        
        patterns = []
        pattern_data = bytes(pattern_dict.dictionary_data)
        
        # Extract each pattern
        for i in range(pattern_dict.num_patterns):
            start_idx = i * pattern_dict.pattern_size_bytes
            end_idx = start_idx + pattern_dict.pattern_size_bytes
            pattern_bytes = pattern_data[start_idx:end_idx]
            
            # Convert bytes to 3D boolean array
            pattern_block = self._bytes_to_block(pattern_bytes, block_size)
            patterns.append(pattern_block)
            
        return patterns
        
    def _bytes_to_block(self, pattern_bytes: bytes, block_size: int) -> np.ndarray:
        """
        Convert byte pattern to 3D boolean block
        
        Args:
            pattern_bytes: Bytes representing the pattern
            block_size: Size of the block (e.g., 8 for 8x8x8)
            
        Returns:
            3D numpy boolean array of shape (block_size, block_size, block_size)
        """
        block = np.zeros((block_size, block_size, block_size), dtype=bool)
        
        total_voxels = block_size ** 3
        
        # Process each bit in the byte array
        for i in range(min(len(pattern_bytes) * 8, total_voxels)):
            byte_idx = i // 8
            bit_idx = i % 8
            
            if byte_idx < len(pattern_bytes):
                is_occupied = (pattern_bytes[byte_idx] >> bit_idx) & 1
                
                # Convert linear index to 3D coordinates
                z = i // (block_size * block_size)
                y = (i % (block_size * block_size)) // block_size
                x = i % block_size
                
                if x < block_size and y < block_size and z < block_size:
                    block[x, y, z] = bool(is_occupied)
                    
        return block