# **3D Semantic Mapping & Language-Grounded Navigation System**

CPS Robotics Lab – Full Sensor + SLAM + Semantic Integration Pipeline  
**Author:** Khaled Abuawwad  
**Date:** November 2025  

---

# **📌 Overview**

This project builds a **full 3D semantic navigation system** combining:

- **Ouster OS1 LiDAR** (large-scale geometry)  
- **Orbbec RGB-D camera** (color + depth fusion)  
- **GLIM and RTAB-Map SLAM** (pose estimation & mapping)  
- **GroundingDINO + SAM + CLIP** (semantic perception)  
- **Natural-language query engine** (human-level commands)

The robot can:

- Detect objects with *open vocabulary*  
- Build a dense 3D semantic map  
- Answer natural language questions  
- Navigate using phrases like *“go to the red chair near the window”*  

This README includes **sensor setup, SLAM, semantic mapping, debugging fixes, and usage instructions**.

---

# **📂 Repository Structure**

```
open_vocab_nav/
├── configs/
│   ├── camera_intrinsics.yaml
│   ├── perception_params.yaml
│   └── navigation_config.yaml
├── data/
│   ├── bags/
│   ├── envodat/
│   └── calibration/
├── models/
├── src/
│   ├── sensors/
│   ├── perception/
│   ├── mapping/
│   ├── navigation/
│   └── utils/
├── scripts/
│   ├── download_models.py
│   ├── run_system.py
│   ├── evaluate_system.py
│   └── visualize_results.py
├── results/
│   ├── semantic_maps/
│   ├── evaluations/
│   └── visualizations/
└── README.md
```

---

# **🚀 System Architecture**

```
SENSORS (LiDAR, RGB-D)
       ↓
SLAM (GLIM, RTAB-Map)
       ↓
SEMANTIC PERCEPTION (GroundingDINO + SAM + CLIP)
       ↓
SEMANTIC MAPPING (3D Fusion + Embeddings)
       ↓
LANGUAGE NAVIGATION (Query Engine + Planner)
```

All semantic models run on GPU.  
SLAM runs on ROS2.

---

# **🛰️ Sensor Setup**

## **1. Ouster OS1 LiDAR Setup**

### **Network Configuration**

```bash
ip a                                  # find interface name
sudo ip addr add 169.254.41.100/16 dev eno1
sudo ufw allow 7502/udp
sudo ufw allow 7503/udp
```

### **Test Connection**

```bash
ouster-cli discover
ouster-cli source <IP> viz
```

### **Run SLAM**

```bash
ouster-cli source <IP> slam viz --map-ratio 0.05
```

---

## **2. Orbbec RGB-D Camera Setup**

### **Install ROS2 driver**

```bash
cd ~/ros2_ws/src
git clone https://github.com/orbbec/OrbbecSDK_ROS2.git
cd OrbbecSDK_ROS2/orbbec_camera/scripts
sudo bash install_udev_rules.sh
cd ~/ros2_ws
colcon build --packages-select orbbec_camera
source install/setup.bash
```

### **Launch Camera**

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py
```

---

## **3. RPLIDAR (2D LiDAR)**

- Connected over `/dev/ttyUSB0`  
- Visualized using a custom Python + Matplotlib script.

---

# **📡 ROS2 Bag Recording**

### **LiDAR Bags**

```bash
ros2 bag record /ouster/points
```

### **RGB-D Bags**

```bash
ros2 bag record   /camera/color/image_raw   /camera/depth/image_raw   /camera/color/camera_info   /tf /tf_static
```

### **Playback**

```bash
ros2 bag play <bag> --clock --loop
```

---

# **🗺️ SLAM Pipelines**

## **1. GLIM SLAM (LiDAR)**

### **Launch node**

```bash
ros2 run glim_ros glim_rosnode   --ros-args -p config_path:=/path/to/glim/config
```

### **Open RViz**

```bash
ros2 run rviz2 rviz2 -d /path/to/glim.rviz
```

---

## **2. RTAB-Map SLAM (RGB-D)**

```bash
ros2 launch rtabmap_launch rtabmap.launch.py   rtabmap_args:="--delete_db_on_start"   frame_id:=camera_link   rgb_topic:=/camera/color/image_raw   depth_topic:=/camera/depth/image_raw   camera_info_topic:=/camera/color/camera_info   approx_sync:=true   rtabmapviz:=true
```

---

# **🧠 Semantic Mapping Pipeline**

Uses:

- **GroundingDINO** – open-vocabulary detection  
- **SAM** – segmentation  
- **CLIP** – embedding similarity  

### Key components

- `OpenVocabularyDetector`  
- `SemanticMapper`  
- `SemanticQueryEngine`  
- `NavigationController`  

Objects become entries in the semantic map with:

- 3D location  
- class label  
- embedding vector  
- mask / bounding box  

---

# **🛠️ Major Technical Issues & Fixes**

## **1. CUDA OOM – SAM too large**

**Cause:** ViT-H uses ~7GB VRAM → crashes  
**Fix:**

- Switched to **ViT-B**  
- Disabled multi-crop  
- Lowered `points_per_side`  
- Added:

```python
torch.cuda.empty_cache()
```

---

## **2. Missing CLIP Embeddings**

**Cause:** SemanticMapper never stored embeddings.  
**Fix:**

- Detector now returns `image_features`  
- Mapper averages and stores them per object  

---

## **3. Regex Didn’t Understand Commands**

Commands like **“go to the chair near the window”** failed.  

**Fix:**

- Added flexible verb & preposition matching in `query_engine.py`

---

## **4. “Table” Hallucinations from Black Images**

**Cause:** All-zero mock frames → CLIP forced nearest label (“table”).  

**Fix:**

- Replaced mock generator with real image download → real pixels  

---

## **5. HTTP 403 When Downloading Test Images**

**Fix:**

```python
Request(url, headers={'User-Agent': 'Mozilla/5.0'})
```

---

## **6. Python Import Errors (src not found)**

**Fix:**

```python
sys.path.append(os.path.abspath(os.path.join(__file__, "..", "..")))
```

---

## **7. GLIM SLAM Launch Incorrect**

**Fix:**

```bash
ros2 run glim_ros glim_rosnode --ros-args -p config_path:=/path
```

---

## **8. Ouster Driver Silent Failure**

Added **foreground mode**:

```bash
ros2 run ouster_ros os_driver --ros-args --params-file params.yaml
```

---

# **🚦 Running the Full System**

### **1. Launch sensors**
- Ouster + Orbbec  

### **2. Record ROS2 bags**  
### **3. Run SLAM**  
### **4. Build semantic map**

```bash
python3 scripts/run_system.py
```

### **5. Example queries**
- *“navigate to the chair”*  
- *“find the monitor near the window”*  

---

# **📈 Evaluation**

Use:

```bash
python3 scripts/evaluate_system.py
```

Measures:

- Object retrieval  
- Navigation success  
- Error cases  
- 3D visualizations  

---

# **📎 Useful Commands**

### **Check Ouster:**

```bash
ros2 topic list | grep ouster
ros2 topic hz /ouster/points
```

### **Check RGB-D:**

```bash
ros2 topic list | grep camera
```

---

# **📜 License**

Internal academic use – CPS Robotics Lab.

---

# **🎉 Completed**

If you'd like:

- PDF export  
- GitHub Pages website  
- Multi-file documentation  

Just ask!

