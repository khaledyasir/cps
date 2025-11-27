# Project Documentation: Sensor Setup, SLAM, and Semantic Navigation System

## 1. Overview
This document summarizes the completed work on LiDAR and RGB-D sensor integration, SLAM pipelines, data acquisition, semantic perception, and the development of a language-grounded navigation system. The content consolidates all procedures, configurations, and challenges encountered throughout the project. It follows the structure of the GET-STARTED plan while excluding the plan text itself.

The objective is to provide a clear record of the implementation work carried out, covering sensor setup, environment preparation, system debugging, semantic model integration, and navigation command parsing. The document is written in a semi-formal tone suitable for technical reporting.

---

## 2. System Setup and Dependencies

### 2.1 Operating System and Tools
- Ubuntu 22.04 / 24.04  
- ROS2 Jazzy  
- Python 3.10+  
- CUDA-enabled GPU (4 GB available)  
- Required packages installed inside isolated virtual environments  

### 2.2 Python Packages
```
pip install ouster-sdk
pip install pyorbbecsdk
pip install torch torchvision torchaudio
pip install open3d
pip install opencv-python
pip install transformers timm
pip install git+https://github.com/openai/CLIP.git
pip install git+https://github.com/IDEA-Research/GroundingDINO.git
pip install git+https://github.com/facebookresearch/segment-anything.git
```

### 2.3 ROS2 Packages
```
sudo apt install ros-jazzy-pcl-ros
sudo apt install ros-jazzy-image-transport
sudo apt install ros-jazzy-vision-opencv
sudo apt install ros-jazzy-nav2-bringup
```

---

## 3. Sensor Setup and Data Acquisition

### 3.1 Ouster OS1 LiDAR Setup
Content sourced from: Ouster LiDAR Full Setup and SLAM Guide.md

Steps completed:
- Physical Ethernet connection established  
- Sensor IP discovered with `ouster-cli discover`  
- Network interface configured using:  
```
sudo ip addr add 169.254.41.100/16 dev eno1
```
- Firewall ports opened:  
```
sudo ufw allow 7502/udp
sudo ufw allow 7503/udp
```
- Visualization executed:  
```
ouster-cli source <IP> viz
```

SLAM executed using:
```
ouster-cli source <IP> slam viz --map-ratio 0.05
```

### 3.2 Orbbec RGB-D Camera Setup
Content sourced from: README.md

Steps completed:
```
git clone https://github.com/orbbec/OrbbecSDK_ROS2.git
sudo bash install_udev_rules.sh
colcon build
ros2 launch orbbec_camera gemini_330_series.launch.py
```

Topics verified in RViz:
- /camera/color/image_raw  
- /camera/depth/points  

### 3.3 Recording ROS2 Bag Files
Examples used:
```
ros2 bag record /ouster/points
ros2 bag record /camera/color/image_raw /camera/depth/image_raw
```

All bags were later used for SLAM reconstruction and semantic mapping.

---

## 4. SLAM Pipeline Implementation

### 4.1 Ouster-Based SLAM
- Driver troubleshooting performed due to mismatched ROS2 parameters  
- Correct driver launch determined:
```
ros2 run ouster_ros os_driver --ros-args --params-file ouster_params.yaml -r __ns:=/ouster
```
- GLIM SLAM integration validated:
```
ros2 run glim_ros glim_rosnode --ros-args -p config_path:=<path>
```
- RViz launched with provided GLIM configuration

### 4.2 RGB-D SLAM (RTAB-Map)
Configuration commands:
```
ros2 launch rtabmap_launch rtabmap.launch.py   rtabmap_args:="--delete_db_on_start"   frame_id:=camera_link   rgb_topic:=/camera/color/image_raw   depth_topic:=/camera/depth/image_raw   camera_info_topic:=/camera/color/camera_info   approx_sync:=true
```

Depth registration issues resolved with:
```
ros2 launch orbbec_camera femto_mega.launch.py depth_registration:=true
```

---

## 5. Sensor Operations Automation Tool

Content sourced from: SENSOR_OPS_DOCUMENTATION.md

### 5.1 Purpose
The unified tool automates:
- Ouster network setup  
- Sensor discovery  
- ROS2 driver launch  
- GLIM SLAM execution  
- Orbbec USB device detection  

### 5.2 Issues Resolved
- Removed hardcoded configuration in favor of YAML  
- Corrected GLIM launch method  
- Corrected Ouster ROS2 driver launch method  
- Improved topic verification logic  
- Ensured ROS2 environment sourcing inside subprocesses  
- Added foreground debugging mode for driver inspection  

### 5.3 Example Commands (Automatic Tool)
```
python3 sensor_ops.py --config sensor_ops_config.yaml
```
Menu includes:
- Network configuration  
- Sensor discovery  
- Visualization  
- Driver debugging  
- GLIM SLAM  

---

## 6. Semantic Perception and Navigation System

### 6.1 Overview
The semantic navigation system integrates:
- Segment Anything Model (SAM)  
- CLIP image-text embeddings  
- GroundingDINO object detection  
- A 3D semantic map  
- A query engine for natural-language grounding  

### 6.2 Perception Module
Functions implemented:
- Object segmentation  
- CLIP embedding extraction  
- Storing embeddings inside SemanticMapper  
- Averaging object embeddings across frames  

### 6.3 Mapping Module
Features:
- Integration of RGB-D or LiDAR frames  
- Projection of 2D masks into 3D  
- Embedding-based object association  
- Global map storage  

### 6.4 Query Engine
Capabilities:
- Flexible regex-based command parsing  
- Handling natural verbs (“go”, “navigate”, “move to”)  
- Understanding spatial prepositions (“near”, “on”, “next to”)  
- CLIP-based similarity search  

---

## 7. Technical Challenges and Solutions

### 7.1 CUDA Out-of-Memory with SAM
- SAM ViT-H required too much VRAM  
- Solution: switched to ViT-B, reduced parameters, added cache clearing  

### 7.2 Missing Embeddings
- CLIP embeddings were not stored in the mapper  
- Solution: modified detector to return `image_features`  

### 7.3 Regex Failures
- Commands like “go to the chair” were ignored  
- Solution: expanded regex patterns  

### 7.4 Black Image “Table” Hallucination
- Caused by noise from `np.zeros`  
- Solution: real image downloader with HTTP spoofing  

### 7.5 403 Errors in Image Download
- Resolved with custom User-Agent  

### 7.6 Import Errors
- Originated from running scripts inside `src/scripts`  
- Solved with `sys.path.append(<root>)`  

---

## 8. Scripts

### 8.1 Ouster Automation Script
The full `ouster_ops.py` script remains included in the original documentation. No changes were made except for import fixes and environment detection.

---

## 9. Summary of Work Completed
The following tasks were fully implemented:

- Complete Ouster LiDAR setup and SLAM  
- Full RGB-D camera setup and bag recording  
- Debugging of network, driver, QoS, and topic issues  
- Full GLIM SLAM integration  
- Build and refine semantic mapping pipeline  
- Implement open-vocabulary detection  
- Design and test language-grounded query parsing  
- Resolve GPU memory and data-handling problems  
- Develop the unified sensor operations automation tool  

This document serves as a complete record of all work carried out.

