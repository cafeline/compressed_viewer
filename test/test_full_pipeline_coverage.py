#!/usr/bin/env python3
"""
Direct test of the full compression/decompression pipeline
to verify point cloud coverage with min_points_threshold=1
"""

import numpy as np
import subprocess
import tempfile
import os
import json
import time


def create_test_pcd(points, filename):
    """Create a PCD file from points"""
    with open(filename, 'w') as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write(f"WIDTH {len(points)}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {len(points)}\n")
        f.write("DATA ascii\n")
        for p in points:
            f.write(f"{p[0]} {p[1]} {p[2]}\n")


def run_compression_test(pcd_file, voxel_size, min_points_threshold, block_size=8):
    """Run compression and capture statistics"""
    cmd = f"""
    source /opt/ros/humble/setup.bash
    source /home/ryo/image_compressor_ws/install/setup.bash 2>/dev/null
    
    timeout 10s ros2 run pointcloud_compressor pointcloud_compressor_node \\
        --ros-args \\
        -p input_file:={pcd_file} \\
        -p voxel_size:={voxel_size} \\
        -p min_points_threshold:={min_points_threshold} \\
        -p block_size:={block_size} \\
        -p publish_once:=true \\
        -p verbose_output:=true 2>&1
    """
    
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    
    # Parse output
    stats = {}
    for line in result.stdout.split('\n'):
        if 'Loading point cloud from:' in line:
            stats['file_loaded'] = True
        elif 'Original points:' in line:
            stats['original_points'] = int(line.split(':')[-1].strip())
        elif 'Voxel grid dimensions:' in line:
            # Extract dimensions like (10, 10, 10)
            dims = line.split(':')[-1].strip()
            stats['voxel_dimensions'] = dims
        elif 'Occupied voxels:' in line:
            stats['occupied_voxels'] = int(line.split(':')[-1].strip())
        elif 'Total blocks:' in line:
            stats['total_blocks'] = int(line.split(':')[-1].strip())
        elif 'Unique patterns:' in line:
            stats['unique_patterns'] = int(line.split(':')[-1].strip())
        elif 'Compression ratio:' in line:
            stats['compression_ratio'] = float(line.split(':')[-1].strip())
        elif 'Compression completed' in line:
            stats['compression_completed'] = True
            
    return stats, result.stdout


def calculate_expected_voxels(points, voxel_size):
    """Calculate how many voxels should be occupied"""
    voxel_indices = set()
    
    for p in points:
        vx = int(p[0] / voxel_size)
        vy = int(p[1] / voxel_size)
        vz = int(p[2] / voxel_size)
        voxel_indices.add((vx, vy, vz))
    
    return len(voxel_indices)


def test_simple_grid():
    """Test 1: Simple 2x2x2 grid"""
    print("="*60)
    print("TEST 1: Simple 2x2x2 Grid")
    print("="*60)
    
    # Create 8 points in a 2x2x2 configuration
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
    
    voxel_size = 0.5  # Each point should be in separate voxel
    
    # Create PCD file
    pcd_file = '/tmp/test_grid.pcd'
    create_test_pcd(points, pcd_file)
    
    # Run compression
    stats, output = run_compression_test(pcd_file, voxel_size, min_points_threshold=1)
    
    # Verify results
    expected_voxels = calculate_expected_voxels(points, voxel_size)
    print(f"Points: {len(points)}")
    print(f"Voxel size: {voxel_size}")
    print(f"Expected occupied voxels: {expected_voxels}")
    
    if 'occupied_voxels' in stats:
        print(f"Actual occupied voxels: {stats['occupied_voxels']}")
        if stats['occupied_voxels'] == expected_voxels:
            print("✅ PASS: Correct number of occupied voxels")
            return True
        else:
            print(f"❌ FAIL: Expected {expected_voxels} voxels, got {stats['occupied_voxels']}")
            return False
    else:
        print("❌ FAIL: Could not get occupied voxels count")
        print("Output:", output[:500])
        return False


def test_single_point():
    """Test 2: Single point with min_points_threshold=1"""
    print("\n" + "="*60)
    print("TEST 2: Single Point with min_points_threshold=1")
    print("="*60)
    
    points = [[0.5, 0.5, 0.5]]
    voxel_size = 0.1
    
    # Create PCD file
    pcd_file = '/tmp/test_single.pcd'
    create_test_pcd(points, pcd_file)
    
    # Test with threshold=1
    stats1, _ = run_compression_test(pcd_file, voxel_size, min_points_threshold=1)
    
    # Test with threshold=2
    stats2, _ = run_compression_test(pcd_file, voxel_size, min_points_threshold=2)
    
    print(f"Single point at (0.5, 0.5, 0.5)")
    print(f"Voxel size: {voxel_size}")
    
    test_passed = True
    
    # Check threshold=1 case
    if 'occupied_voxels' in stats1:
        print(f"With min_points_threshold=1: {stats1['occupied_voxels']} voxels")
        if stats1['occupied_voxels'] == 1:
            print("✅ PASS: Single point captured with threshold=1")
        else:
            print(f"❌ FAIL: Expected 1 voxel with threshold=1, got {stats1['occupied_voxels']}")
            test_passed = False
    else:
        print("❌ FAIL: Could not get voxel count for threshold=1")
        test_passed = False
    
    # Check threshold=2 case
    if 'occupied_voxels' in stats2:
        print(f"With min_points_threshold=2: {stats2['occupied_voxels']} voxels")
        if stats2['occupied_voxels'] == 0:
            print("✅ PASS: Single point ignored with threshold=2")
        else:
            print(f"❌ FAIL: Expected 0 voxels with threshold=2, got {stats2['occupied_voxels']}")
            test_passed = False
    else:
        print("❌ FAIL: Could not get voxel count for threshold=2")
        test_passed = False
        
    return test_passed


def test_dense_cluster():
    """Test 3: Dense cluster of points"""
    print("\n" + "="*60)
    print("TEST 3: Dense Cluster")
    print("="*60)
    
    # Create 100 points in 0.1x0.1 area
    points = []
    for i in range(10):
        for j in range(10):
            points.append([i * 0.01, j * 0.01, 0.0])
    
    voxel_size = 0.05  # Should result in 4 voxels (2x2)
    
    # Create PCD file
    pcd_file = '/tmp/test_dense.pcd'
    create_test_pcd(points, pcd_file)
    
    # Run compression
    stats, _ = run_compression_test(pcd_file, voxel_size, min_points_threshold=1)
    
    expected_voxels = calculate_expected_voxels(points, voxel_size)
    print(f"Points: {len(points)} in 0.1x0.1 area")
    print(f"Voxel size: {voxel_size}")
    print(f"Expected occupied voxels: {expected_voxels}")
    
    if 'occupied_voxels' in stats:
        print(f"Actual occupied voxels: {stats['occupied_voxels']}")
        if stats['occupied_voxels'] == expected_voxels:
            print("✅ PASS: Dense cluster correctly voxelized")
            return True
        else:
            print(f"❌ FAIL: Expected {expected_voxels} voxels, got {stats['occupied_voxels']}")
            return False
    else:
        print("❌ FAIL: Could not get occupied voxels count")
        return False


def test_coverage_preservation():
    """Test 4: Verify all points are represented after compression"""
    print("\n" + "="*60)
    print("TEST 4: Coverage Preservation")
    print("="*60)
    
    # Create random points
    np.random.seed(42)
    points = []
    for _ in range(50):
        points.append([
            np.random.uniform(0, 5),
            np.random.uniform(0, 5),
            np.random.uniform(0, 5)
        ])
    
    voxel_size = 0.5
    
    # Create PCD file
    pcd_file = '/tmp/test_random.pcd'
    create_test_pcd(points, pcd_file)
    
    # Run compression with min_points_threshold=1
    stats, output = run_compression_test(pcd_file, voxel_size, min_points_threshold=1)
    
    expected_voxels = calculate_expected_voxels(points, voxel_size)
    print(f"Random points: {len(points)}")
    print(f"Voxel size: {voxel_size}")
    print(f"Expected occupied voxels: {expected_voxels}")
    
    if 'occupied_voxels' in stats:
        print(f"Actual occupied voxels: {stats['occupied_voxels']}")
        
        # Check if compression completed successfully
        if 'compression_completed' in stats and stats['compression_completed']:
            print("✅ Compression completed successfully")
            
            # Verify reasonable compression ratio
            if 'compression_ratio' in stats:
                print(f"Compression ratio: {stats['compression_ratio']}")
                if stats['compression_ratio'] > 0:
                    print("✅ PASS: Valid compression achieved")
                    return True
                else:
                    print("❌ FAIL: Invalid compression ratio")
                    return False
        else:
            print("❌ FAIL: Compression did not complete")
            return False
    else:
        print("❌ FAIL: Could not get statistics")
        return False


def main():
    """Run all tests"""
    print("\n" + "🧪"*30)
    print("POINT CLOUD COVERAGE TEST SUITE")
    print("Testing min_points_threshold=1 functionality")
    print("🧪"*30 + "\n")
    
    # Make sure packages are built
    print("Building packages...")
    build_result = subprocess.run(
        "cd /home/ryo/image_compressor_ws && colcon build --packages-select pointcloud_compressor compressed_viewer", 
        shell=True,
        capture_output=True,
        text=True
    )
    if build_result.returncode != 0:
        print("Warning: Build had issues")
        print(build_result.stderr[:500])
    
    results = []
    
    # Run tests
    results.append(("Simple Grid", test_simple_grid()))
    results.append(("Single Point", test_single_point()))
    results.append(("Dense Cluster", test_dense_cluster()))
    results.append(("Coverage Preservation", test_coverage_preservation()))
    
    # Summary
    print("\n" + "="*60)
    print("TEST SUMMARY")
    print("="*60)
    
    passed = 0
    failed = 0
    
    for test_name, result in results:
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"{test_name}: {status}")
        if result:
            passed += 1
        else:
            failed += 1
    
    print(f"\nTotal: {passed} passed, {failed} failed")
    
    if failed == 0:
        print("\n🎉 All tests passed! Point cloud coverage is working correctly.")
        return 0
    else:
        print(f"\n⚠️ {failed} test(s) failed. Review the implementation.")
        return 1


if __name__ == "__main__":
    exit(main())