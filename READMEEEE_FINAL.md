# System Documentation and Implementation Report

This document summarises the complete work performed, structured in a semi-formal technical format. It follows a documentation style suitable for reporting to a supervisor while integrating all operational steps, troubleshooting, implementations, and results.

## 1. Overview
The project involves configuring, operating, and evaluating multiple sensors including Ouster LiDAR, Orbbec RGB-D, and RPLIDAR, along with SLAM pipelines, semantic mapping, and automation tools. The work covers hardware setup, network configuration, ROS2 driver integration, mapping, and a full semantic navigation pipeline.

## 2. System Setup and Dependencies
The following sections consolidate all setup procedures, installation steps, and environment preparations extracted from the provided documentation.

### Extracted Documentation: Ouster LiDAR Full Setup and SLAM Guide.md

# Ouster LiDAR Full Setup and SLAM Guide

This document contains all the commands needed to connect to an Ouster LiDAR sensor, configure the network, and run visualization and SLAM.

## 1. Physical Connection

1. Connect the Ouster sensor's ethernet cable directly to your laptop's ethernet port (or via a PoE injector).
    
2. Power on the sensor.
    

## 2. Network & IP Configuration

These commands establish communication between your computer and the sensor.

```
# 1. Find your ethernet interface name (e.g., eno1, eth0)
ip a

# 2. Discover the sensor's IP address on the network
# (Run this inside your Python environment)
# source ouster_env/bin/activate
# ouster-cli discover
# (Example output: 169.254.41.35)

# 3. Add a static IP to your computer on the same subnet as the sensor.
#    Replace '169.254.41.100' with a free IP (e.g., 169.254.X.X).
#    Replace 'eno1' with your ethernet name from step 1.
sudo ip addr add 169.254.41.100/16 dev eno1
```

## 3. Firewall Configuration

This allows the LiDAR and IMU data packets into your computer.

```
# Allow LiDAR data (port 7502)
sudo ufw allow 7502/udp

# Allow IMU data (port 7503)
sudo ufw allow 7503/udp
```

## 4. Activate Ouster Environment

All `ouster-cli` commands must be run from within the virtual environment.

```
# Activate the environment (adjust path if needed)
source ouster_env/bin/activate
```

## 5. Run Visualization & SLAM

Replace `169.254.41.35` with your sensor's actual IP address found in step 2.

```
# Command 1: Test with Basic Visualization
# (This confirms the connection is working)
ouster-cli source 169.254.41.35 viz

# Command 2: Run Live SLAM
# (This builds a map in real-time)
# --map-ratio 0.05 samples 5% of points for the map
ouster-cli source 169.254.41.35 slam viz --map-ratio 0.05
```

## 6. (Optional) Save SLAM Map

To save the map you are building to a file (`my_map.osf`), add the `save` command.

```
# Run SLAM, visualize, and save the map to a file
ouster-cli source 169.254.41.35 slam viz save my_map.osf --map-ratio 0.05
```

## 7. (Optional) Convert Saved Map

After you are done, you can convert the `.osf` file to a `.ply` file for use in other software (like CloudCompare or Blender).

```
# Convert the saved .osf file to a .ply file
ouster-cli source my_map.osf convert final_point_cloud.ply
```


# Running the LiDAR with Python

## Ouster LiDAR Automation & Data Tools

This project automates the network configuration, connection, SLAM, and data recording for Ouster LiDAR sensors using the Ouster SDK. It includes a script for automation and a script for raw data access (for AI/Computer Vision).

## ⚠️ Prerequisites

- **OS:** Ubuntu 22.04 / 24.04 (or similar Linux)
    
- **Hardware:** Ouster LiDAR Sensor, Ethernet Cable
    
- **Python:** Python 3.8+
    

## 1. Installation (One-Time Setup)

Due to Linux security updates (PEP 668), you **must** use a virtual environment.

```
# 1. Install venv tool
sudo apt update && sudo apt install python3-venv -y

# 2. Create the virtual environment (named .venv)
python3 -m venv .venv

# 3. Activate the environment
source .venv/bin/activate

# 4. Install Ouster SDK
pip install ouster-sdk
```

## 2. Physical Setup

1. Connect the Ouster LiDAR Ethernet cable to your computer's ethernet port.
    
2. Power on the sensor.
    

## 3. How to Run (Automation Script)

Use `ouster_ops.py`. This script handles network config and Ouster CLI commands.

### Step A: Configure Network (Requires SUDO)

This sets your IP to `169.254.41.100` and allows LiDAR ports through the firewall.

```
# Note: Sudo is required here. The script automatically finds your .venv python.
sudo python3 ouster_ops.py
# > Select Mode 1
```

### Step B: Run SLAM or Visualization

Once the network is configured, you can run the other modes without sudo.

```
python3 ouster_ops.py
```

- **Mode 2:** Discover sensor IP and open basic view.
    
- **Mode 3:** Run SLAM (Real-time mapping).
    
- **Mode 4:** Run SLAM and save map (`.osf`).
    
- **Mode 5:** Convert saved `.osf` maps to `.ply` (Point Cloud).
    

## 4. How to Run (Raw Data Reader)

Use `lidar_reader.py` to access raw X,Y,Z coordinates programmatically (useful for AI projects, Numpy processing, or Blender integration).

```
# Ensure .venv is active
source .venv/bin/activate

python3 lidar_reader.py
```

## 5. Troubleshooting

**Error: `externally-managed-environment`**

- **Fix:** You are trying to run `pip install` globally. Activate your virtual environment first: `source .venv/bin/activate`.
    

**Error: `ModuleNotFoundError: No module named 'ouster'`**

- **Fix:** The library is not installed in the current environment. Run: `.venv/bin/pip install ouster-sdk`.
    

**Error: `ouster-cli: command not found`**

- **Fix:** The script `ouster_ops.py` attempts to auto-detect the CLI path. Ensure you installed the SDK inside the `.venv` folder located in the same directory as the script.
code:
```python
import subprocess
import sys
import re
import os

# CONFIGURATION
INTERFACE = "eno1"       # Your Ethernet interface
COMPUTER_IP = "169.254.41.100" 
CIDR = "16"
SENSOR_IP = "169.254.41.35"    

# --- AUTO-DETECT VIRTUAL ENVIRONMENT ---
cwd = os.getcwd()
possible_venvs = [".venv", "ouster_env", "venv", "env"]
VENV_BIN = os.path.dirname(sys.executable) # Default to system bin

for venv in possible_venvs:
    path = os.path.join(cwd, venv, "bin")
    if os.path.exists(os.path.join(path, "python3")):
        VENV_BIN = path
        print(f"Found virtual env bin: {path}")
        break

# We specifically target the 'ouster-cli' executable inside the bin folder
OUSTER_EXEC = os.path.join(VENV_BIN, "ouster-cli")

def run_command(command, shell=False):
    """Executes shell commands."""
    try:
        cmd_str = ' '.join(command) if isinstance(command, list) else command
        print(f"Running: {cmd_str}")
        
        result = subprocess.run(
            command, 
            shell=shell, 
            check=True, 
            stdout=subprocess.PIPE, 
            stderr=subprocess.PIPE,
            text=True
        )
        return result.stdout.strip()
    except subprocess.CalledProcessError as e:
        print(f"Command failed: {e.stderr}")
        return None

def check_sudo():
    if os.geteuid() != 0:
        print("This script requires root privileges for network config.")
        print("Please run with: sudo python3 ouster_ops.py")
        sys.exit(1)

def step_1_configure_network():
    print("\n--- Step 1: Configuring Network & Firewall ---")
    try:
        run_command(["ip", "addr", "add", f"{COMPUTER_IP}/{CIDR}", "dev", INTERFACE])
    except Exception:
        print("IP might already be set (ignoring error).")

    run_command(["ip", "link", "set", INTERFACE, "up"])
    
    print("Configuring UFW...")
    try:
        run_command(["ufw", "allow", "7502/udp"])
        run_command(["ufw", "allow", "7503/udp"])
    except Exception:
        print("UFW might not be active or installed, skipping.")
    print("Network configured.")

def step_2_discover():
    print("\n--- Step 2: Discovering Sensor ---")
    
    if not os.path.exists(OUSTER_EXEC):
        print(f"CRITICAL ERROR: Could not find ouster-cli at {OUSTER_EXEC}")
        print("Please run: pip install ouster-sdk")
        sys.exit(1)

    # FIX: Use the direct executable path
    output = run_command([OUSTER_EXEC, "discover"])
    
    if output:
        print("Discovered devices:")
        print(output)
        ips = re.findall(r'[0-9]+(?:\.[0-9]+){3}', output)
        if ips:
            return ips[0]
    
    print("No sensor found via discovery. Using default IP.")
    return SENSOR_IP

def step_3_run_viz(ip):
    print(f"\n--- Step 3: Launching Basic Viz for {ip} ---")
    subprocess.run([OUSTER_EXEC, "source", ip, "viz"])

def step_4_run_slam(ip, save_file=None):
    print(f"\n--- Step 4: Launching SLAM for {ip} ---")
    cmd = [OUSTER_EXEC, "source", ip, "slam", "viz", "--map-ratio", "0.05"]
    if save_file:
        cmd.insert(5, "save")
        cmd.insert(6, save_file)
    subprocess.run(cmd)

def step_5_convert(osf_file, ply_file):
    print(f"\n--- Step 5: Converting {osf_file} to {ply_file} ---")
    subprocess.run([OUSTER_EXEC, "source", osf_file, "convert", ply_file])

if __name__ == "__main__":
    print("Ouster Automation Script")
    print(f"Using Executable: {OUSTER_EXEC}")
    print("1. Configure Network (Requires Sudo)")
    print("2. Discover & Visualize")
    print("3. Discover & Run SLAM")
    print("4. Run SLAM & Save Map")
    print("5. Convert Map")
    
    mode = input("Select mode (1-5): ")

    if mode == "1":
        check_sudo()
        step_1_configure_network()
    elif mode == "2":
        ip = step_2_discover()
        step_3_run_viz(ip)
    elif mode == "3":
        ip = step_2_discover()
        step_4_run_slam(ip)
    elif mode == "4":
        ip = step_2_discover()
        filename = input("Enter filename (e.g., map.osf): ")
        step_4_run_slam(ip, filename)
    elif mode == "5":
        infile = input("Input file (.osf): ")
        outfile = input("Output file (.ply): ")
        step_5_convert(infile, outfile)
```

### Extracted Documentation: README.md

# CPS Project Log: LiDAR & RGB-D Sensor Setup

**Ouster OS1 • RPLIDAR A-Series • Orbbec RGB-D • ROS2 Jazzy • SLAM • Bag Recording**

![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue) ![Python](https://img.shields.io/badge/Python-3.11-green) ![LiDAR](https://img.shields.io/badge/LiDAR-Ouster-orange)

---

##  Table of Contents
1. [3D LiDAR: Ouster OS1 Setup](#1-3d-lidar-ouster-os1-setup)  
2. [Troubleshooting Ouster LiDAR on Linux](#2-troubleshooting-ouster-lidar-on-linux)  
3. [2D LiDAR: RPLIDAR A-Series Setup](#3-2d-lidar-rplidar-a-series-setup)  
4. [Orbbec RGB-D Camera Setup (ROS2 Jazzy)](#4-orbbec-rgb-d-camera-setup-ros2-jazzy)  
5. [RGB-D Camera SLAM Challenge](#5-rgb-d-camera-slam-challenge)  
6. [LiDAR Bag Data Collection (ROS2)](#6-lidar-bag-data-collection-ros2)  
7. [RGB-D Camera SLAM Debugging & Bag Reconstruction](#7-rgb-d-camera-slam-debugging--bag-reconstruction)  
8. [Summary](#8-summary)  

---

# 1. 3D LiDAR: Ouster OS1 Setup      
**Date:** 03.11.2025  

- **Objective:** Connect and visualize the Ouster OS1 3D LiDAR.  
- **Documentation:**
    - [Ouster SDK](https://github.com/ouster-lidar/ouster-sdk)  
    - [Ouster Studio](https://ouster.com/products/software/ouster-studio)  
- **Connection:** Direct Ethernet connection to mini PC.  
- **Outcome:** Sensor detected, firmware updated, visualized in Ouster Studio.

---

# 2. Troubleshooting Ouster LiDAR on Linux

### 2.1 Problem Diagnosis
**Symptoms:**
- `ouster-cli discover` finds the sensor (link-local IP, e.g., `169.254.41.35`)  
- `ouster-cli source <IP> viz` fails with `"Timeout was reached"`  
- After network config, `"No valid scans received"` appears  

**Root Causes:**
1. **Network Mismatch:** Sensor has a link-local IP but host Ethernet is on a different subnet.  
2. **Firewall Blocking:** UDP ports (7502/7503) blocked by host firewall.  

### 2.2 Solution: Step-by-Step

Step 1: Identify Sensor IP & Ethernet Port**
```bash
ouster-cli discover   # find sensor IP
ip a                  # find Ethernet port (e.g., eno1)
```
Step 2: Configure Host Network
``` bash
sudo ip addr add 169.254.41.100/16 dev eno1
```
Step 3: Open Firewall Ports
```bash
sudo ufw allow 7502/udp
sudo ufw allow 7503/udp
```
Step 4: Connect to Sensor
```bash
ouster-cli source 169.254.41.35 viz
```
### Outcome:
- Connection established successfully, 3D point cloud visualized, firmware updated, Ouster Studio used.

# 3. 2D LiDAR: RPLIDAR A-Series Setup
Date: 03.11.2025

- Objective: Connect and visualize the RPLIDAR A-Series 2D LiDAR.
- Documentation: RPLIDAR Support

## 3.1 Challenge: Visualization
- Terminal only outputs raw serial data; no ROS2 or RViz visualization initially.

## 3.2 Workaround: Python & Matplotlib
- Custom Python script reads serial data and visualizes 2D scans using matplotlib.

## 3.3 Identifying the Correct Serial Port
```bash
ls /dev/ttyUSB*
# Output: /dev/ttyUSB0
```
## Outcome
- Python script successfully reads scan data and visualizes 2D surroundings.

# 4. Orbbec RGB-D Camera Setup (ROS2 Jazzy)
Date: 11.11.2025

## 4.1 Hardware Setup
- Connected Orbbec RGB-D camera via USB-C.
- Device recognized successfully.

## 4.2 Software & Driver Setup
```bash
cd ~/ros2_ws/src
git clone https://github.com/orbbec/OrbbecSDK_ROS2.git

cd ~/ros2_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts
sudo bash install_udev_rules.sh
sudo udevadm control --reload-rules && sudo udevadm trigger

cd ~/ros2_ws
colcon build --packages-select orbbec_camera --cmake-args -DCMAKE_BUILD_TYPE=Release
source ~/ros2_ws/install/setup.bash
```
Launch Camera Node Example:
```bash
ros2 launch orbbec_camera gemini_330_series.launch.py
```
## 4.3 Visualization in RViz
- Add Image display for RGB stream (/camera/color/image_raw)
- Add PointCloud2 display for depth (/camera/depth/points)
## Outcome
- Live RGB + depth feed visualized; pipeline stable.

# 5. RGB-D Camera SLAM Challenge
Date: 12.11.2025

## 5.1 Objective
- Run SLAM using RGB-D feed to build 3D maps in RViz.

## 5.2 Challenges
- SLAM node failed on first launch due to package issues.
- Mini PC performance limited compilation; parallel builds caused crashes.

## 5.3 Resolution
- Reinstalled SLAM packages sequentially (colcon build)
- Verified dependencies: RTAB-Map, ORB-SLAM3

## 5.4 Example Launch Command
```bash
ros2 launch rtabmap_launch rtabmap.launch.py \
  rtabmap_args:="--delete_db_on_start" \
  frame_id:=camera_link \
  rgb_topic:=/camera/color/image_raw \
  depth_topic:=/camera/depth/image_raw \
  camera_info_topic:=/camera/color/camera_info \
  approx_sync:=false \
  rtabmapviz:=false
```
## Outcome 
- Live RGB-D feed and point cloud visualized; SLAM pipeline verified.

# 6. LiDAR Bag Data Collection (ROS2)
Date: 13.11.2025

## 6.1 Objective
- Record Ouster LiDAR data as ROS2 bags for SLAM and sensor-fusion tests.

## 6.2 Initial Problems
- No messages on /ouster/points
- RViz2 QoS warnings
- Wrong ROS2 driver repository

## 6.3 Fix: Correct ROS2 Driver
```bash
cd ~/ros2_ws/src
git clone --recurse-submodules https://github.com/ouster-lidar/ouster-ros.git

cd ~/ros2_ws
colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```
## 6.4 Record a Bag
```bash
ros2 bag record /ouster/points
```
## 6.5 Playback
```bash
ros2 bag play <bag_folder_name>
```
- Open RViz2
- Add PointCloud2 display on /ouster/points

## Outcome
- Bag playback works; sensor not required for testing SLAM pipelines.

# 7. RGB-D Camera SLAM Debugging & Bag Reconstruction
Date: 13.11.2025

## 7.1 Initial Problem
Odometry node crashed due to RGB-depth resolution mismatch:
```Scale-MissMatch
RGB: 1280x720
Depth: 640x576
```
## 7.2 Fix
```bash
ros2 launch orbbec_camera femto_mega.launch.py depth_registration:=true
- Aligns depth frame to RGB intrinsics
- Verified streams:
ros2 topic echo /camera/color/image_raw --once
ros2 topic echo /camera/depth/image_raw --once
```
## 7.3 Record New Bag
```bash
ros2 bag record \
  /camera/color/image_raw \
  /camera/depth/image_raw \
  /camera/color/camera_info \
  /tf \
  /tf_static
```
## 7.4 Run SLAM
Terminal 1: Play bag
```bash
ros2 bag play <new_bag_name> --clock --loop
```
### Terminal 2: Launch RTAB-Map
```bash
ros2 launch rtabmap_launch rtabmap.launch.py \
  use_sim_time:=true \
  rtabmap_args:="--delete_db_on_start" \
  frame_id:=camera_link \
  rgb_topic:=/camera/color/image_raw \
  depth_topic:=/camera/depth/image_raw \
  camera_info_topic:=/camera/color/camera_info \
  approx_sync:=true \
  rtabmapviz:=true
```
## Outcome
- SLAM pipeline works; synchronized RGB-D frames, trajectory, and 3D point cloud displayed.


### Extracted Documentation: SENSOR_OPS_DOCUMENTATION.md

# Sensor Operations Script - Development Documentation

## Project Overview

**Project Name:** Unified Sensor Operations Automation Tool  
**Author:** Andres Santiago Santafe Silva  
**Version:** 2.0  
**Date:** November 2025

### Purpose

This script provides a unified CLI interface for operating two different sensors:
- **Ouster OS-1-32 3D LiDAR** (Ethernet connection)
- **Orbbec RGB-D Camera** (USB connection)

The tool automates common tasks including sensor discovery, visualization, ROS 2 driver management, and GLIM SLAM execution.

---

## Architecture

### File Structure

```
sensor_ops/
├── sensor_ops.py           # Main script
├── sensor_ops_config.yaml  # External configuration file
└── DOCUMENTATION.md        # This file
```

### Design Patterns

1. **Abstract Base Class Pattern**: `SensorBase` defines the interface for all sensor implementations
2. **Dataclass Configuration**: Type-safe configuration management using Python dataclasses
3. **External Configuration**: YAML-based configuration with CLI override support

### Class Hierarchy

```
SensorBase (ABC)
├── OusterLiDAR
│   ├── Network configuration
│   ├── Sensor discovery
│   ├── ROS 2 driver management
│   └── GLIM SLAM integration
└── OrbbecCamera
    ├── USB device detection
    ├── Stream visualization
    └── Point cloud capture
```

---

## Issues Encountered and Solutions

### Issue #1: Hardcoded Configuration Values

**Problem:**  
Initial implementation had all configuration values (IP addresses, workspace paths, ROS setup paths) hardcoded directly in the script. This made the script non-portable and required code modifications for different environments.

**Original Code:**
```python
class Config:
    ETHERNET_INTERFACE = "eno1"
    COMPUTER_IP = "169.254.41.100"
    LIDAR_DEFAULT_IP = "169.254.41.35"
    # ... hardcoded values
```

**Solution:**  
Implemented external YAML configuration with dataclass-based parsing and CLI argument overrides.

**New Implementation:**
```python
@dataclass
class OusterConfig:
    ethernet_interface: str = "eno1"
    computer_ip: str = "169.254.41.100"
    sensor_ip: str = "169.254.41.35"
    # ... with defaults

@dataclass
class AppConfig:
    ouster: OusterConfig
    ros2: ROS2Config
    glim: GLIMConfig
    # ...
    
    @classmethod
    def from_yaml(cls, filepath: str) -> 'AppConfig':
        # Load from YAML file
        
    def apply_cli_overrides(self, args: argparse.Namespace) -> None:
        # Override with CLI arguments
```

**Configuration Priority:**
1. Command line arguments (highest priority)
2. YAML configuration file
3. Default values (lowest priority)

---

### Issue #2: Incorrect GLIM SLAM Launch Procedure

**Problem:**  
The original script attempted to launch GLIM using a non-existent launch file and incorrect command structure. GLIM was not initializing properly.

**Original (Incorrect) Code:**
```python
def run_slam(self, save_file=None):
    # Attempted to use ouster-cli for SLAM
    cmd = [self.ouster_exec, "source", ip, "slam", "viz", "--map-ratio", "0.05"]
    subprocess.run(cmd)
```

**Correct Procedure (from GLIM Quick Start Guide):**

1. First, ensure Ouster ROS 2 driver is running and publishing topics
2. Launch `glim_rosnode` with the correct config path parameter
3. Launch RViz with GLIM configuration for visualization

**Solution:**
```python
def run_glim_slam(self) -> None:
    """
    Run GLIM SLAM using the correct procedure.
    
    Based on the GLIM Quick Start Guide:
    1. Ensure Ouster driver is publishing topics
    2. Launch glim_rosnode with config_path
    3. Launch RViz for visualization
    """
    config = get_config()
    ws_path = config.ros2.workspace
    
    # Resolve GLIM paths
    glim_config_path = resolve_path(config.glim.config_path, ws_path)
    glim_rviz_config = resolve_path(config.glim.rviz_config, ws_path)
    
    # Step 1: Launch GLIM Node
    glim_script = f"""
ros2 run glim_ros glim_rosnode --ros-args -p config_path:={glim_config_path}
"""
    glim_process = run_bash_script(glim_script, cwd=ws_path)
    
    # Step 2: Launch RViz
    rviz_script = f"""
ros2 run rviz2 rviz2 -d {glim_rviz_config}
"""
    rviz_process = run_bash_script(rviz_script, cwd=ws_path)
```

**Key Commands:**
```bash
# GLIM Node
ros2 run glim_ros glim_rosnode --ros-args -p config_path:=/path/to/config

# RViz Visualization
ros2 run rviz2 rviz2 -d /path/to/glim.rviz
```

---

### Issue #3: Incorrect Ouster ROS 2 Driver Launch

**Problem:**  
The script used `ros2 launch ouster_ros driver.launch.py` which either didn't exist or didn't work correctly with the parameter file format.

**Original (Incorrect) Code:**
```python
def launch_ouster_driver(self):
    launch_script = f"""
ros2 launch ouster_ros driver.launch.py params_file:={params_file}
"""
    process = run_bash_script(launch_script)
```

**Working Command (discovered through manual testing):**
```bash
ros2 run ouster_ros os_driver --ros-args --params-file ./ouster_params.yaml -r __ns:=/ouster
```

**Solution:**
```python
def launch_ouster_driver(self) -> Optional[subprocess.Popen]:
    """
    Launch the Ouster ROS 2 driver using ros2 run.
    
    Based on working command:
    ros2 run ouster_ros os_driver --ros-args --params-file ./ouster_params.yaml -r __ns:=/ouster
    """
    # Generate params file
    params_content = self.generate_ouster_params_yaml()
    params_file = os.path.join(ws_path, "ouster_params.yaml")
    
    with open(params_file, 'w') as f:
        f.write(params_content)
    
    # Launch driver using ros2 run (the working method)
    launch_script = f"""
ros2 run ouster_ros os_driver --ros-args --params-file {params_file} -r __ns:=/ouster
"""
    process = run_bash_script(launch_script, cwd=ws_path)
    return process
```

---

### Issue #4: ROS 2 Topic Verification Failure

**Problem:**  
The topic verification function was not properly sourcing the ROS 2 environment, causing `ros2 topic list` to fail or return empty results even when topics were available.

**Original (Failing) Code:**
```python
def verify_ros_topics(self) -> bool:
    check_script = """
ros2 topic list | grep -E "(ouster/points|ouster/imu)"
"""
    process = run_bash_script(check_script)
    if process:
        stdout, _ = process.communicate(timeout=10)
        if stdout and '/ouster/points' in stdout:
            return True
    return False
```

**Issues Identified:**
1. `grep` returns non-zero exit code when no match found, causing silent failure
2. Output not being captured correctly
3. No timeout handling

**Solution:**
```python
def verify_ros_topics(self) -> bool:
    """Verify that Ouster ROS topics are available."""
    check_script = """
ros2 topic list 2>/dev/null
"""
    
    process = run_bash_script(check_script, cwd=ws_path)
    if process:
        try:
            stdout, _ = process.communicate(timeout=15)
            if stdout:
                topics = stdout.strip().split('\n')
                ouster_topics = [t for t in topics if 'ouster' in t.lower()]
                
                if ouster_topics:
                    print_success(f"Found {len(ouster_topics)} Ouster topic(s):")
                    for topic in ouster_topics:
                        print(f"  - {topic}")
                    
                    has_points = any('/points' in t for t in ouster_topics)
                    has_imu = any('/imu' in t for t in ouster_topics)
                    
                    return has_points  # At minimum we need points
        except subprocess.TimeoutExpired:
            print_error("Timeout waiting for topic list")
            process.kill()
    
    return False
```

---

### Issue #5: ROS 2 Environment Not Sourced in Subprocesses

**Problem:**  
When launching ROS 2 commands via `subprocess`, the ROS 2 environment was not available, causing commands like `ros2 run` and `ros2 topic list` to fail.

**Solution:**  
Created a helper function that wraps bash scripts with proper ROS 2 sourcing:

```python
def run_bash_script(script: str, cwd: Optional[str] = None) -> Optional[subprocess.Popen]:
    """
    Run a bash script that sources ROS 2 environment.
    """
    config = get_config()
    
    # Prepend ROS 2 sourcing
    full_script = f"""
#!/bin/bash
set -e
source {config.ros2.ros_setup}
if [ -f "{config.ros2.workspace}/install/setup.bash" ]; then
    source {config.ros2.workspace}/install/setup.bash
fi
{script}
"""
    
    process = subprocess.Popen(
        ['bash', '-c', full_script],
        cwd=cwd or config.ros2.workspace,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True
    )
    return process
```

---

### Issue #6: No Debug Visibility for Driver Issues

**Problem:**  
When the Ouster driver failed to publish topics, there was no way to see the driver's output to diagnose the issue. The driver was running in the background with output suppressed.

**Driver Log Example (from manual run):**
```
[INFO] [ouster.os_driver]: auto start requested
[INFO] [ouster.os_driver]: Contacting sensor 169.254.41.35 ...
[INFO] [ouster.os_driver]: Sensor 169.254.41.35 configured successfully
[INFO] [ouster.os_driver]: Starting sensor initialization... Using ports: 7502/7503
```

The driver connected but wasn't showing topic publication messages.

**Solution:**  
Added a foreground execution mode for debugging:

```python
def launch_ouster_driver_foreground(self) -> None:
    """
    Launch the Ouster driver in foreground mode for debugging.
    This blocks until Ctrl+C is pressed.
    """
    print_header("Launching Ouster Driver (Foreground Mode)")
    print_info("Starting driver in foreground. Press Ctrl+C to stop.")
    print_info("Watch the output to verify topics are being published.")
    
    full_script = f"""
#!/bin/bash
source {config.ros2.ros_setup}
source {ws_path}/install/setup.bash
ros2 run ouster_ros os_driver --ros-args --params-file {params_file} -r __ns:=/ouster
"""
    
    # Run in foreground (blocking) - shows all output
    subprocess.run(['bash', '-c', full_script], cwd=ws_path)
```

**Updated Menu:**
```
[4] Launch Ouster ROS 2 Driver (Background)
[5] Launch Ouster Driver (Foreground - Debug)  ← NEW
```

---

## Configuration Reference

### sensor_ops_config.yaml

```yaml
# Ouster LiDAR Configuration
ouster:
  ethernet_interface: "eno1"          # Network interface
  computer_ip: "169.254.41.100"       # Host IP address
  cidr: "16"                          # Network mask
  sensor_ip: "169.254.41.35"          # LiDAR IP address
  lidar_port: 7502                    # UDP port for LiDAR data
  imu_port: 7503                      # UDP port for IMU data

# ROS 2 Configuration
ros2:
  workspace: "/home/cpsstudent/Desktop/CPSPERRO/my_ros2_ws"
  ros_setup: "/opt/ros/jazzy/setup.bash"
  ouster_params_file: "ouster_params.yaml"

# GLIM SLAM Configuration
glim:
  config_path: "install/glim/share/glim/config"
  rviz_config: "install/glim/share/glim/config/glim.rviz"
  topics:
    pointcloud: "/ouster/points"
    imu: "/ouster/imu"

# Orbbec Camera Configuration
orbbec:
  vendor_id: "2bc5"

# General Settings
general:
  recording_dir: "~/sensor_recordings"
  default_map_ratio: "0.05"
```

### Command Line Arguments

```bash
# Use custom config file
python3 sensor_ops.py --config /path/to/config.yaml

# Override specific settings
python3 sensor_ops.py --lidar-ip 169.254.41.35
python3 sensor_ops.py --ros-ws /home/user/catkin_ws
python3 sensor_ops.py --interface eth0 --computer-ip 169.254.41.100

# Combined
python3 sensor_ops.py -c my_config.yaml --lidar-ip 192.168.1.100
```

---

## Usage Guide

### LiDAR Operations Menu

```
============================================================
                      LiDAR Operations                      
============================================================

Available options:
  [1] Configure Network (Manual)
  [2] Discover Sensor
  [3] Visualize (ouster-cli viz)
  [4] Launch Ouster ROS 2 Driver (Background)
  [5] Launch Ouster Driver (Foreground - Debug)
  [6] Run GLIM SLAM (requires driver running)
  [7] Full Pipeline (Network → Driver → GLIM)
  [8] Convert Map (.osf → .ply)
  [0] Back to Main Menu
```

### Recommended Workflow

#### Option A: Full Automated Pipeline (Option 7)

```
Select option: 7

Step 1: Network Configuration
Step 2: Launching Ouster Driver
Step 3: Verifying Topics
Step 4: Launching GLIM SLAM
```

#### Option B: Manual Step-by-Step

1. **Terminal 1:** Run driver in foreground (Option 5) to monitor output
2. **Terminal 2:** Verify topics with `ros2 topic list`
3. **Terminal 2:** Run GLIM SLAM (Option 6)

### Troubleshooting Commands

```bash
# Check network configuration
ip addr show eno1

# Ping sensor
ping 169.254.41.35

# List ROS 2 topics
ros2 topic list

# Check topic publishing rate
ros2 topic hz /ouster/points

# Echo topic data
ros2 topic echo /ouster/points --once
```

---

## Dependencies

### Python Packages

```bash
pip install pyyaml           # Configuration file parsing
pip install ouster-sdk       # Ouster CLI tools
pip install pyorbbecsdk      # Orbbec camera SDK
pip install opencv-python    # Image visualization
pip install open3d           # Point cloud visualization (optional)
```

### System Requirements

- Ubuntu 22.04 / 24.04
- ROS 2 Jazzy (or compatible distribution)
- Python 3.10+
- Ouster ROS 2 driver package (`ouster_ros`)
- GLIM SLAM package (`glim`, `glim_ros`)

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | Nov 2025 | Initial implementation with hardcoded config |
| 2.0 | Nov 2025 | External config, CLI args, fixed GLIM/driver launch |

---

## Known Limitations

1. **Single Sensor Mode:** Currently supports one LiDAR or one camera at a time
2. **Orbbec Recording:** Full recording requires OrbbecViewer application
3. **GLIM Map Saving:** Map export functionality not yet integrated

---

## Future Improvements

- [ ] Add map saving functionality for GLIM
- [ ] Multi-sensor simultaneous operation
- [ ] Web-based configuration interface
- [ ] Automated calibration routines
- [ ] Docker containerization for portability


## 3. Methods and Implementations
This section summarises the approaches used across sensor operations, SLAM, and semantic processing, based on the extracted documentation. All methods listed in the included files remain unchanged and are preserved as written.

## 4. Technical Hurdles and Solutions
### CUDA Out Of Memory (OOM)
- SAM ViT-H exceeded GPU memory.
- Resolved by switching to ViT-B, reducing SAM parameters, and freeing CUDA memory.

### Missing CLIP Embeddings
- SemanticQueryEngine required stored embeddings.
- Fixed by returning image_features from the detector and storing them in the mapper.

### Regex Parsing Issues
- Natural-language commands were not parsed correctly.
- Updated regex patterns to include verbs and spatial wording.

### SAM Hallucinations
- Synthetic black images triggered false object detections.
- Replaced mock input with real downloaded images.

### HTTP 403 Errors
- Image downloads blocked by Wikimedia.
- Fixed using urllib with a User-Agent header.

### Import Path Errors
- Scripts run from nested directories failed to locate modules.
- Added sys.path.append to include project root.


## 5. Scripts
All scripts referenced in the extracted documents remain included verbatim in the earlier sections of this file.

