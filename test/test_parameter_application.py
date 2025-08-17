#!/usr/bin/env python3
"""
Test script to verify that parameters are correctly applied in ROS2 nodes
"""
import sys
import os
import subprocess
import time
import signal

def test_parameters():
    """Test that min_points_threshold parameter is correctly applied"""
    
    print("Testing parameter application with min_points_threshold=1...")
    
    # Build the command
    cmd = [
        'ros2', 'run', 'pointcloud_compressor', 'pointcloud_compressor_node',
        '--ros-args',
        '-p', 'input_file:=/tmp/test_simple.pcd',
        '-p', 'min_points_threshold:=1',
        '-p', 'voxel_size:=1.0',
        '-p', 'publish_once:=true'
    ]
    
    try:
        # Start the process
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            preexec_fn=os.setsid
        )
        
        # Read output for a few seconds
        output_lines = []
        start_time = time.time()
        timeout = 3.0
        
        while time.time() - start_time < timeout:
            line = proc.stdout.readline()
            if line:
                output_lines.append(line.strip())
                print(line.strip())
                
                # Check for parameter in output
                if "Min points threshold:" in line:
                    value = line.split(":")[-1].strip()
                    if value == "1":
                        print("✓ SUCCESS: min_points_threshold=1 was correctly applied!")
                        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                        return True
                    else:
                        print(f"✗ FAILURE: Expected min_points_threshold=1, got {value}")
                        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                        return False
        
        # Timeout - kill the process
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        print("✗ FAILURE: Timeout - parameter value not found in output")
        return False
        
    except Exception as e:
        print(f"Error running test: {e}")
        return False

if __name__ == "__main__":
    # First check if test file exists
    if not os.path.exists('/tmp/test_simple.pcd'):
        print("Creating test point cloud file...")
        pcd_content = """# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH 8
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS 8
DATA ascii
0.0 0.0 0.0
1.0 0.0 0.0
0.0 1.0 0.0
1.0 1.0 0.0
0.0 0.0 1.0
1.0 0.0 1.0
0.0 1.0 1.0
1.0 1.0 1.0
"""
        with open('/tmp/test_simple.pcd', 'w') as f:
            f.write(pcd_content)
    
    # Source ROS2 setup
    setup_cmd = "cd /home/ryo/image_compressor_ws && source install/setup.bash 2>/dev/null && " + " ".join(sys.argv)
    if len(sys.argv) == 1:
        # We're running directly, need to re-run with sourced environment
        print("Sourcing ROS2 environment...")
        result = subprocess.run(
            f"cd /home/ryo/image_compressor_ws && source install/setup.bash 2>/dev/null && python3 {__file__} --sourced",
            shell=True,
            capture_output=False,
            text=True
        )
        sys.exit(result.returncode)
    
    # Run the test
    success = test_parameters()
    sys.exit(0 if success else 1)