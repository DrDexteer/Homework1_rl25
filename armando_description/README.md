# Overview
This package contains the URDF/Xacro description of the **Armando** robot, used for both visualization and simulation.

---

# 📂 Contents
- `urdf/arm.urdf.xacro` — Main robot model
- `urdf/armando_hardware_interface.xacro` — Main robot model  
- `meshes/` — Meshes for visual and collision geometry  
- `config/rviz/rviz_config.rviz` — config file for RViz
- `config/controllers.yaml` — controllers configuration file

---

# How to launch
The package can be launched using

```
ros2 launch armando_description armando_display.launch.py 
```

# Inspecting the running system

After launching the robot, you can open another terminal and use ROS 2 tools
to inspect the running nodes, topics, and their connections.

```bash
# Visualize the node graph
rqt_graph

# List all active nodes
ros2 node list

# List all active topics
ros2 topic list
