# **3D Semantic Mapping & Language-Grounded Navigation System**
**CPS Robotics Lab – Full Sensor + SLAM + Semantic Integration Pipeline**  
**Author:** Khaled Abuawwad  
**Date:** November 2025  

# **📌 Overview**
This project builds a full 3D semantic navigation system combining:
- Ouster OS1 LiDAR  
- Orbbec RGB-D camera  
- GLIM and RTAB-Map SLAM  
- GroundingDINO + SAM + CLIP  
- Natural-language query engine  

The robot can detect objects, build maps, answer questions, and navigate via language.

# **📂 Repository Structure**
```
open_vocab_nav/
├── configs/
├── data/
├── models/
├── src/
├── scripts/
├── results/
└── README.md
```

# **🛰️ Sensor Setup**
## Ouster LiDAR
```bash
sudo ip addr add 169.254.41.100/16 dev eno1
sudo ufw allow 7502/udp
sudo ufw allow 7503/udp
ouster-cli discover
ouster-cli source <IP> viz
```

## Orbbec RGB-D
```bash
git clone https://github.com/orbbec/OrbbecSDK_ROS2.git
colcon build --packages-select orbbec_camera
ros2 launch orbbec_camera gemini_330_series.launch.py
```

# **📡 ROS2 Bag Recording**
```bash
ros2 bag record /ouster/points
ros2 bag record /camera/color/image_raw /camera/depth/image_raw /tf /tf_static
ros2 bag play <bag> --clock --loop
```

# **🗺️ SLAM Pipelines**
## GLIM
```bash
ros2 run glim_ros glim_rosnode --ros-args -p config_path:=/path
ros2 run rviz2 rviz2 -d /path/to/glim.rviz
```

## RTAB-Map
```bash
ros2 launch rtabmap_launch rtabmap.launch.py   rgb_topic:=/camera/color/image_raw   depth_topic:=/camera/depth/image_raw
```

# **🧠 Semantic Mapping Pipeline**
Uses GroundingDINO, SAM and CLIP to build a 3D semantic map with object embeddings.

# **🛠️ Key Fixes**
- SAM OOM → switched to ViT-B
- Added CLIP embedding export
- Improved regex parsing
- Replaced mock images
- Added browser user-agent
- Fixed module imports
- Corrected GLIM launch commands
- Added driver foreground mode

# **🚦 Running System**
```bash
python3 scripts/run_system.py
```

# **📈 Evaluation**
```bash
python3 scripts/evaluate_system.py
```

# **📜 License**
Internal academic use – CPS Robotics Lab
