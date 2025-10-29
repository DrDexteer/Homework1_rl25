## Enable acces to the USB port

Ensure you add the `--privileged` flag to the Docker run command to grant the container additional privileges, such as access to hardware resources.

Add your current user to the dialout group
```
sudo usermod -aG dialout <username>
```
This grants your user access to serial devices (e.g., `/dev/ttyUSB0`)

Then, enable access to the port

```
sudo chmod a+rw /dev/ttyUSB0
```

## Build Dynamixel SDK
Build the `dynamixel_sdk` libraries to use methods for reading and writing to the motor registers.

- **dynamixel_sdk**: Build the libraries to use methods for reading and writing to the motor registers.
- **dynamixel_sdk_custom_interfaces**: This library generates custom messages and services that will be used by the `read_write` node.

```
colcon build --packages-select arm_description kdl_arm_dynamixel kdl_arm_dynamixel_custom_interfaces dynamixel_sdk dynamixel_sdk_custom_interfaces dynamixel_sdk_examples 
```

## To test in simulation (the use_hardware parameter defaults to 0, so it is optional in this case)
```
ros2 launch kdl_arm_dynamixel ros2_armando.launch.py use_hardware:=0
```
## To test on the real hardware
```
ros2 launch kdl_arm_dynamixel ros2_armando.launch.py use_hardware:=1
```

## The cartesian goal can be given to the robot from another terminal calling the service /command_cartesian_position
```
ros2 service call /command_cartesian_position kdl_arm_dynamixel_custom_interfaces/srv/CommandCartesianPosition "{x : -0.1, y : 0.0, z : 0.25}"
```

 