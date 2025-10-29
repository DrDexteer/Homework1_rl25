# Overview
This is an example package on how to use sensors and controllers in the ROS2/Gazebo environment.

# Requirements
This package depends on `iiwa_description`, which can be downloaded from the `ros2_iiwa_description` repository. Make sure to also use the updated `Dockerfile`, which includes the packages for `Gazebo Ignition` and for the use of controllers and sensors.

# How to launch
The example can be launched using

```
ros2 launch ros2_sensors_and_actuators iiwa.launch.py
```

# How to control
If you are using the `position_controller` for the joints, you can set the position references using a message like
```
ros2 topic pub /position_controller/commands std_msgs/msg/Float64MultiArray "{
  data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
}"
```

If instead you are using the `joint_trajectory_controller`, the message can be sent from the command line like this
```
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{ 
  joint_names: ['joint_a1', 'joint_a2', 'joint_a3', 'joint_a4', 'joint_a5', 'joint_a6', 'joint_a7'], 
  points: [
    {
      positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
      velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
      time_from_start: { sec: 2, nanosec: 0 }
    }
  ] 
}"
```
