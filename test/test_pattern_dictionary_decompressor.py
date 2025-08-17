#!/usr/bin/env python3
"""
Test suite for PatternDictionaryDecompressor using Test-Driven Development (TDD)
"""

import unittest
import numpy as np
from unittest.mock import Mock
import sys
import os

# Add the package to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from compressed_viewer.pattern_dictionary_decompressor import PatternDictionaryDecompressor


class TestPatternDictionaryDecompressor(unittest.TestCase):
    """Test cases for the PatternDictionaryDecompressor class"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.decompressor = PatternDictionaryDecompressor()
        
    def test_init(self):
        """Test PatternDictionaryDecompressor initialization"""
        self.assertIsNotNone(self.decompressor)
        
    def test_decompress_empty_pattern_dictionary(self):
        """Test decompression with empty pattern dictionary"""
        mock_dict = Mock()
        mock_dict.num_patterns = 0
        mock_dict.pattern_size_bytes = 0
        mock_dict.dictionary_data = []
        
        result = self.decompressor.decompress_pattern_dictionary(mock_dict)
        self.assertIsNotNone(result)
        self.assertEqual(len(result), 0)
        
    def test_decompress_single_pattern(self):
        """Test decompression with a single pattern"""
        # Create a simple 8x8x8 pattern with one voxel set
        pattern_data = bytearray(64)  # 512 bits / 8 = 64 bytes for 8x8x8 voxels
        pattern_data[0] = 1  # Set first bit (voxel at position 0,0,0)
        
        mock_dict = Mock()
        mock_dict.num_patterns = 1
        mock_dict.pattern_size_bytes = 64
        mock_dict.dictionary_data = pattern_data
        
        result = self.decompressor.decompress_pattern_dictionary(mock_dict, block_size=8)
        
        self.assertIsNotNone(result)
        self.assertEqual(len(result), 1)
        self.assertIsInstance(result[0], np.ndarray)
        self.assertEqual(result[0].shape, (8, 8, 8))
        self.assertTrue(result[0][0, 0, 0])  # First voxel should be True
        
    def test_decompress_multiple_patterns(self):
        """Test decompression with multiple patterns"""
        # Create two patterns: first empty, second with all voxels set
        pattern_size = 64  # 8x8x8 voxels = 512 bits = 64 bytes
        pattern_data = bytearray(2 * pattern_size)
        
        # First pattern: all zeros (already initialized)
        # Second pattern: all ones
        for i in range(pattern_size, 2 * pattern_size):
            pattern_data[i] = 0xFF
        
        mock_dict = Mock()
        mock_dict.num_patterns = 2
        mock_dict.pattern_size_bytes = pattern_size
        mock_dict.dictionary_data = pattern_data
        
        result = self.decompressor.decompress_pattern_dictionary(mock_dict, block_size=8)
        
        self.assertIsNotNone(result)
        self.assertEqual(len(result), 2)
        
        # First pattern should be all False
        self.assertFalse(np.any(result[0]))
        
        # Second pattern should be all True
        self.assertTrue(np.all(result[1]))
        
    def test_bytes_to_block_conversion(self):
        """Test conversion from bytes to 3D boolean block"""
        # Create a simple pattern: first 8 bits set
        pattern_bytes = bytearray([0xFF, 0x00])  # First byte all 1s, second all 0s
        
        block = self.decompressor._bytes_to_block(pattern_bytes, block_size=4)
        
        self.assertEqual(block.shape, (4, 4, 4))
        
        # Check that first 8 voxels are True
        linear_idx = 0
        for z in range(4):
            for y in range(4):
                for x in range(4):
                    if linear_idx < 8:
                        self.assertTrue(block[x, y, z], f"Voxel at {x},{y},{z} should be True")
                    else:
                        self.assertFalse(block[x, y, z], f"Voxel at {x},{y},{z} should be False")
                    linear_idx += 1
                    
    def test_different_block_sizes(self):
        """Test pattern decompression with different block sizes"""
        # Test with 4x4x4 block (64 voxels = 8 bytes)
        pattern_size = 8
        pattern_data = bytearray(pattern_size)
        pattern_data[0] = 0x01  # Set first bit
        
        mock_dict = Mock()
        mock_dict.num_patterns = 1
        mock_dict.pattern_size_bytes = pattern_size
        mock_dict.dictionary_data = pattern_data
        
        result = self.decompressor.decompress_pattern_dictionary(mock_dict, block_size=4)
        
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0].shape, (4, 4, 4))
        self.assertTrue(result[0][0, 0, 0])
        
    def test_checksum_validation(self):
        """Test CRC32 checksum validation"""
        import zlib
        
        pattern_data = bytearray([0xFF, 0x00, 0xAA, 0x55])
        expected_checksum = zlib.crc32(pattern_data) & 0xffffffff
        
        mock_dict = Mock()
        mock_dict.num_patterns = 1
        mock_dict.pattern_size_bytes = 4
        mock_dict.dictionary_data = pattern_data
        mock_dict.checksum = expected_checksum
        
        # Should succeed with correct checksum
        result = self.decompressor.decompress_pattern_dictionary(mock_dict, validate_checksum=True)
        self.assertIsNotNone(result)
        
        # Should fail with incorrect checksum
        mock_dict.checksum = 0x12345678
        with self.assertRaises(ValueError):
            self.decompressor.decompress_pattern_dictionary(mock_dict, validate_checksum=True)
            
    def test_invalid_pattern_data(self):
        """Test handling of invalid pattern data"""
        # Test with insufficient data
        mock_dict = Mock()
        mock_dict.num_patterns = 2
        mock_dict.pattern_size_bytes = 64
        mock_dict.dictionary_data = bytearray(32)  # Only enough for 1 pattern
        
        with self.assertRaises(ValueError):
            self.decompressor.decompress_pattern_dictionary(mock_dict)
            
    def test_zero_block_size(self):
        """Test handling of invalid block size"""
        mock_dict = Mock()
        mock_dict.num_patterns = 1
        mock_dict.pattern_size_bytes = 1
        mock_dict.dictionary_data = bytearray([0x01])
        
        with self.assertRaises(ValueError):
            self.decompressor.decompress_pattern_dictionary(mock_dict, block_size=0)


if __name__ == '__main__':
    unittest.main()