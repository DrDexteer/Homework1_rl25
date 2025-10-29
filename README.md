# Homework1 

This repo contains three ROS 2 packages developed for the first homework of the Robotics Lab course:

---
### 📂 Package Documentation
Each package includes a dedicated `README.md`:
| Package | Description |
|----------|--------------|
| [`armando_description`](armando_description/) | URDF/Xacro model and RViz visualization |
| [`armando_gazebo`](armando_gazebo/) | Launch and Gazebo simulation setup |
| [`armando_controller`](armando_controller/) | C++ controller node for position and trajectory control |

---

##  Requirements
- ROS 2 Humble
- `colcon`, `rosdep`
- `gazebo_ros` / `gazebo` (Ignition / GZ Sim)
- Standard packages:  
  `robot_state_publisher`, `joint_state_publisher(_gui)`,  
  `trajectory_msgs`, `sensor_msgs`, `std_msgs`

---

##  Build Instructions
Clone this repository inside the `src` folder of your ROS 2 workspace, install dependencies, and build:

```bash
  cd ~/ros2_ws
  git clone https://github.com/DrDexteer/Homework1_rl25.git src/Homework1
  rosdep update
  rosdep install -i --from-path src --rosdistro humble -y
  colcon build --symlink-install
  . install/setup.bash
