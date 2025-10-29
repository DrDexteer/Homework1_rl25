# Overview
This package contains a C++ ROS 2 node used to control the **Armando** robot in both *position* and *trajectory* mode. It subscribes to `/joint_states` and publishes commands to the corresponding controller topic, depending on the selected control mode.


# How to control
The controller node can be launched using (default parameters controller_type:=position, cmd_i:="[0.0, 0.0, 0.0, 0.0]"):
```
ros2 run armando_controller arm_controller_node
```
Parameters can be passed via the command line using `--ros-args`. For example, to launch the node in **position mode**:
```
ros2 run armando_controller arm_controller_node --ros-args \
  -p controller_type:=position \
  -p cmd1:="[0.00,  0.00,  0.00,  0.00]" \
  -p cmd2:="[0.15,  0.20, -0.10,  0.10]" \
  -p cmd3:="[0.30,  0.35, -0.15,  0.20]" \
  -p cmd4:="[0.45,  0.35,  0.00,  0.25]" \
  -p cmd5:="[0.60,  0.20,  0.10,  0.30]" \
  -p cmd6:="[0.45,  0.05,  0.15,  0.20]" \
  -p cmd7:="[0.30, -0.10,  0.05,  0.10]" \
  -p cmd8:="[0.15, -0.15, -0.05,  0.00]" \
  -p cmd9:="[0.00,  0.00,  0.00, -0.10]" \
  -p cmd10:="[0.00,  0.20,  0.00,  0.00]"


```
To launch the node in **trajectory mode**, simply change the parameter:
```
ros2 run armando_controller arm_controller_node --ros-args \
  -p controller_type:=trajectory \
  -p cmd1:="[0.00,  0.00,  0.00,  0.00]" \
  -p cmd2:="[0.15,  0.20, -0.10,  0.10]" \
  -p cmd3:="[0.30,  0.35, -0.15,  0.20]" \
  -p cmd4:="[0.45,  0.35,  0.00,  0.25]" \
  -p cmd5:="[0.60,  0.20,  0.10,  0.30]" \
  -p cmd6:="[0.45,  0.05,  0.15,  0.20]" \
  -p cmd7:="[0.30, -0.10,  0.05,  0.10]" \
  -p cmd8:="[0.15, -0.15, -0.05,  0.00]" \
  -p cmd9:="[0.00,  0.00,  0.00, -0.10]" \
  -p cmd10:="[0.00,  0.20,  0.00,  0.00]"

```


# Notes
- The node publishes one command every **5 seconds** and cycles through up to **10 commands** (`cmd1` … `cmd10`).
- Each command must be a **vector of 4 elements** corresponding to the four joints of the Armando robot.
- In trajectory mode, each target point is set with a time of **2 seconds**.
- Make sure that the corresponding controller (`position_controller` or `joint_trajectory_controller`) is **active** before running the node.
- You can inspect active controllers and topics with:
```
ros2 control list_controllers
ros2 topic list
rqt_graph
```
