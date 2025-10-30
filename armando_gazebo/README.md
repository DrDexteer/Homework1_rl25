# Overview
This package provides the **launch files** to simulate and visualize the **Armando** robot in Gazebo (Ignition / GZ Sim) and RViz.  
It also handles the spawning of controllers through `ros2_control` and bridges image topics from Gazebo to ROS 2.

When launched, the system:
1. Loads the robot model from `armando_description` using Xacro  
2. Starts **Gazebo** with an empty world (`empty.sdf`)  
3. Spawns the **Armando** robot entity into the world  
4. Loads and starts the following controllers:
   - `joint_state_broadcaster`
   - `position_controller` or `joint_trajectory_controller`
5. Opens **RViz** for visualization  
6. Launches a **camera bridge** between Gazebo and ROS 2 topics

---

# How to launch

```bash
# to run simulation with position_controller configured and activated
ros2 launch armando_gazebo armando_world.launch.py controller_type:=position
```

```bash
# to run simulation with joint_trajectory_controller configured and activated
ros2 launch armando_gazebo armando_world.launch.py controller_type:=trajectory
```


# How to control
If you are using the `position_controller` for the joints, you can set the position references using a message like
```
ros2 topic pub /position_controller/commands std_msgs/msg/Float64MultiArray "{
  data: [0.0, 0.0, 0.0, 0.0]
}"
```

If instead you are using the `joint_trajectory_controller`, the message can be sent from the command line like this
```
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{ 
  joint_names: ['j0', 'j1', 'j2', 'j3'], 
  points: [
    {
      positions: [0.0, 0.0, 0.0, 0.0], 
      time_from_start: { sec: 2, nanosec: 0 }
    }
  ] 
}"
```
You can also control the robot by using `arm_controller_node` within the `armando_controller` package
