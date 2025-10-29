# Homework1 

This repo contains three ROS 2 packages developed for the first homework of the Robotics Lab course:

- **armando_description** — URDF/Xacro model and robot resources  and RViz visualization
- **armando_gazebo** — Launch and configuration files for simulation in Gazebo (Ignition/GZ)  
- **armando_controller** — C++ node for joint control in position or trajectory mode  

---

## 🧩 Requirements
- ROS 2 Humble
- `colcon`, `rosdep`
- `gazebo_ros` / `gazebo` (Ignition / GZ Sim)
- Standard packages:  
  `robot_state_publisher`, `joint_state_publisher(_gui)`,  
  `trajectory_msgs`, `sensor_msgs`, `std_msgs`

---

## ⚙️ Build Instructions
Clone this repository inside the `src` folder of your ROS 2 workspace, install dependencies, and build:

```bash
cd ~/ros2_ws
git clone https://github.com/DrDexteer/Homework1_rl25.git src/Homework1
rosdep update
rosdep install -i --from-path src --rosdistro humble -y
colcon build --symlink-install
. install/setup.bash
