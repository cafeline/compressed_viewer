#!/usr/bin/env python3
"""
Test coverage with real PLY file: tsudanuma-challenge-all.ply
This test verifies that all original points are represented in pattern_markers
when min_points_threshold=1
"""

import subprocess
import time
import numpy as np
import re
import os


def get_ply_stats(ply_file):
    """Get statistics about PLY file"""
    print(f"Analyzing PLY file: {ply_file}")
    
    # Read PLY header to get point count
    with open(ply_file, 'rb') as f:
        header = []
        vertex_count = 0
        
        for line in f:
            header_line = line.decode('utf-8', errors='ignore').strip()
            header.append(header_line)
            
            if header_line.startswith('element vertex'):
                vertex_count = int(header_line.split()[-1])
            
            if header_line == 'end_header':
                break
        
        print(f"  PLY vertex count: {vertex_count:,}")
        return vertex_count


def run_compression_pipeline(ply_file, voxel_size, min_points_threshold, timeout=60):
    """Run compression pipeline and capture statistics"""
    print(f"\nRunning compression pipeline:")
    print(f"  Input: {ply_file}")
    print(f"  Voxel size: {voxel_size}")
    print(f"  Min points threshold: {min_points_threshold}")
    
    cmd = f"""
    source /opt/ros/humble/setup.bash
    source /home/ryo/image_compressor_ws/install/setup.bash 2>/dev/null
    
    timeout {timeout}s ros2 run pointcloud_compressor pointcloud_compressor_node \\
        --ros-args \\
        -p input_file:={ply_file} \\
        -p voxel_size:={voxel_size} \\
        -p min_points_threshold:={min_points_threshold} \\
        -p block_size:=8 \\
        -p publish_once:=true \\
        -p verbose_output:=true 2>&1
    """
    
    start_time = time.time()
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    elapsed_time = time.time() - start_time
    
    print(f"  Compression time: {elapsed_time:.1f} seconds")
    
    # Parse output
    stats = {
        'success': False,
        'original_points': 0,
        'occupied_voxels': 0,
        'unique_patterns': 0,
        'total_blocks': 0,
        'compression_ratio': 0.0,
        'grid_dimensions': None
    }
    
    for line in result.stdout.split('\n'):
        if 'Compression completed' in line:
            stats['success'] = True
        elif 'Original points:' in line:
            match = re.search(r'Original points:\s*(\d+)', line)
            if match:
                stats['original_points'] = int(match.group(1))
        elif 'Occupied voxels:' in line:
            match = re.search(r'Occupied voxels:\s*(\d+)', line)
            if match:
                stats['occupied_voxels'] = int(match.group(1))
        elif 'Unique patterns:' in line:
            match = re.search(r'Unique patterns:\s*(\d+)', line)
            if match:
                stats['unique_patterns'] = int(match.group(1))
        elif 'Total blocks:' in line:
            match = re.search(r'Total blocks:\s*(\d+)', line)
            if match:
                stats['total_blocks'] = int(match.group(1))
        elif 'Compression ratio:' in line:
            match = re.search(r'Compression ratio:\s*([\d.]+)', line)
            if match:
                stats['compression_ratio'] = float(match.group(1))
        elif 'Voxel grid dimensions:' in line:
            match = re.search(r'dimensions:\s*\((\d+),\s*(\d+),\s*(\d+)\)', line)
            if match:
                stats['grid_dimensions'] = (int(match.group(1)), int(match.group(2)), int(match.group(3)))
    
    return stats


def calculate_theoretical_coverage(original_points, voxel_size, occupied_voxels):
    """Calculate theoretical coverage based on voxel occupancy"""
    # Each occupied voxel represents at least one point when min_points_threshold=1
    # The coverage depends on how many points fall into the same voxel
    
    if occupied_voxels == 0:
        return 0.0
    
    # Theoretical maximum: each voxel could contain multiple points
    # Minimum coverage: occupied_voxels / original_points (if perfectly distributed)
    min_coverage = (occupied_voxels / original_points) * 100 if original_points > 0 else 0
    
    return min_coverage


def test_with_different_voxel_sizes(ply_file):
    """Test with different voxel sizes to understand coverage"""
    print("\n" + "="*70)
    print("Testing with different voxel sizes")
    print("="*70)
    
    # Get original point count
    original_count = get_ply_stats(ply_file)
    
    # Test with different voxel sizes
    voxel_sizes = [0.1, 0.5, 1.0, 2.0, 5.0]
    results = []
    
    for voxel_size in voxel_sizes:
        print(f"\n--- Voxel size: {voxel_size} meters ---")
        stats = run_compression_pipeline(ply_file, voxel_size, min_points_threshold=1)
        
        if stats['success']:
            coverage = calculate_theoretical_coverage(
                stats['original_points'],
                voxel_size,
                stats['occupied_voxels']
            )
            
            results.append({
                'voxel_size': voxel_size,
                'occupied_voxels': stats['occupied_voxels'],
                'unique_patterns': stats['unique_patterns'],
                'total_blocks': stats['total_blocks'],
                'compression_ratio': stats['compression_ratio'],
                'theoretical_coverage': coverage,
                'grid_dimensions': stats['grid_dimensions']
            })
            
            print(f"  Occupied voxels: {stats['occupied_voxels']:,}")
            print(f"  Unique patterns: {stats['unique_patterns']:,}")
            print(f"  Total blocks: {stats['total_blocks']:,}")
            print(f"  Compression ratio: {stats['compression_ratio']:.4f}")
            print(f"  Theoretical min coverage: {coverage:.2f}%")
            if stats['grid_dimensions']:
                print(f"  Grid dimensions: {stats['grid_dimensions']}")
        else:
            print("  ❌ Compression failed")
    
    return results


def analyze_coverage_issue(ply_file):
    """Analyze why coverage might be incomplete"""
    print("\n" + "="*70)
    print("Coverage Analysis")
    print("="*70)
    
    # Use a reasonable voxel size for testing
    voxel_size = 1.0
    
    print(f"\nTesting with voxel_size={voxel_size}, min_points_threshold=1")
    stats = run_compression_pipeline(ply_file, voxel_size, min_points_threshold=1)
    
    if stats['success']:
        print("\n--- Analysis ---")
        print(f"Original points: {stats['original_points']:,}")
        print(f"Occupied voxels: {stats['occupied_voxels']:,}")
        
        ratio = stats['occupied_voxels'] / stats['original_points'] if stats['original_points'] > 0 else 0
        print(f"Voxel-to-point ratio: {ratio:.4f}")
        
        if ratio < 1.0:
            print("\n⚠️ COVERAGE ISSUE DETECTED:")
            print(f"  - Only {stats['occupied_voxels']:,} voxels are occupied")
            print(f"  - But there are {stats['original_points']:,} original points")
            print(f"  - This means multiple points are sharing voxels")
            print(f"  - Average points per voxel: {stats['original_points']/stats['occupied_voxels']:.2f}")
            
            # Calculate point density
            if stats['grid_dimensions']:
                gx, gy, gz = stats['grid_dimensions']
                volume = gx * gy * gz * (voxel_size ** 3)
                density = stats['original_points'] / volume
                print(f"  - Point density: {density:.2f} points/m³")
        else:
            print("\n✅ Good coverage: Each point is in a separate voxel")
    
    # Test with smaller voxel size for better coverage
    print(f"\nTesting with smaller voxel_size=0.1 for better coverage")
    stats_small = run_compression_pipeline(ply_file, 0.1, min_points_threshold=1)
    
    if stats_small['success']:
        print(f"Occupied voxels with 0.1m voxels: {stats_small['occupied_voxels']:,}")
        coverage_small = (stats_small['occupied_voxels'] / stats_small['original_points']) * 100
        print(f"Coverage: {coverage_small:.2f}%")
        
        if coverage_small > 90:
            print("✅ Good coverage with smaller voxel size")
        else:
            print("⚠️ Still incomplete coverage even with smaller voxels")


def test_decompression_coverage(ply_file, voxel_size=1.0):
    """Test actual decompression to see how many points are recovered"""
    print("\n" + "="*70)
    print("Decompression Coverage Test")
    print("="*70)
    
    # This would require running the full pipeline with compressed_viewer
    # For now, we'll analyze the compression statistics
    
    print(f"\nCompression with voxel_size={voxel_size}, min_points_threshold=1")
    stats = run_compression_pipeline(ply_file, voxel_size, min_points_threshold=1)
    
    if stats['success']:
        print("\n--- Decompression Analysis ---")
        print(f"Original points: {stats['original_points']:,}")
        print(f"Occupied voxels: {stats['occupied_voxels']:,}")
        print(f"Expected decompressed points: {stats['occupied_voxels']:,}")
        
        # Each occupied voxel becomes one point in decompression
        recovery_rate = (stats['occupied_voxels'] / stats['original_points']) * 100
        print(f"Point recovery rate: {recovery_rate:.2f}%")
        
        if recovery_rate < 100:
            loss = stats['original_points'] - stats['occupied_voxels']
            print(f"\n⚠️ DATA LOSS:")
            print(f"  - Lost points: {loss:,} ({100-recovery_rate:.2f}%)")
            print(f"  - Reason: Multiple points in same voxel")
            print(f"  - Solution: Use smaller voxel_size")
            
            # Suggest optimal voxel size
            suggested_voxel = voxel_size * 0.1
            print(f"\n💡 Suggestion: Try voxel_size={suggested_voxel} for better coverage")
        else:
            print("\n✅ Full coverage: All points preserved")


def main():
    """Main test function"""
    print("\n" + "🔍"*35)
    print("REAL PLY FILE COVERAGE TEST")
    print("Testing: /home/ryo/tsudanuma/maps/tsudanuma-challenge-all.ply")
    print("🔍"*35 + "\n")
    
    ply_file = "/home/ryo/tsudanuma/maps/tsudanuma-challenge-all.ply"
    
    # Check if file exists
    if not os.path.exists(ply_file):
        print(f"❌ File not found: {ply_file}")
        return 1
    
    # Run tests
    print("1. Testing with different voxel sizes...")
    results = test_with_different_voxel_sizes(ply_file)
    
    print("\n2. Analyzing coverage issue...")
    analyze_coverage_issue(ply_file)
    
    print("\n3. Testing decompression coverage...")
    test_decompression_coverage(ply_file, voxel_size=1.0)
    
    # Summary
    print("\n" + "="*70)
    print("SUMMARY")
    print("="*70)
    
    if results:
        print("\nVoxel Size vs Coverage:")
        print("Voxel Size | Occupied Voxels | Theoretical Coverage")
        print("-"*50)
        for r in results:
            print(f"{r['voxel_size']:10.1f} | {r['occupied_voxels']:15,} | {r['theoretical_coverage']:18.2f}%")
    
    print("\n📊 CONCLUSION:")
    print("When min_points_threshold=1, the coverage depends on voxel_size:")
    print("- Larger voxels = fewer occupied voxels = lower coverage")
    print("- Smaller voxels = more occupied voxels = higher coverage")
    print("- For full coverage, voxel_size should be small enough that")
    print("  each point falls into its own voxel")
    
    return 0


if __name__ == "__main__":
    exit(main())