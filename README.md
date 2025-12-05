# eut_OnRobot_ROS2_Driver


ROS 2 driver for OnRobot Grippers.


## Features
- ROS 2 driver for OnRobot grippers controlled via Modbus TCP or Serial
- Currently supported grippers:
    - [VGC10](https://onrobot.com/en/products/vgc10-vacuum-gripper)
    - [VG10](https://onrobot.com/en/products/vg10-vacuum-gripper) 
- ROS 2 Control hardware interface plugin

## Dependencies (included in the installation steps below)

- [Modbus](https://github.com/Mazurel/Modbus) C++ library (included as a submodule)

## Installation

### As part of a project
If you want to include it into a bigger project, the best thing would be to add it as a submodule in **dependencies**: 
```bash
git submodule add -b master <https://ice.eurecat.org/gitlab/robotics-automation/eut_onrobot_driver.git> dependencies/eut_onrobot_driver

```

### As a standalone repo

1. Navigate to your ROS 2 workspace and **clone the repository**:
   ```sh
   git clone --recurse-submodules https://ice.eurecat.org/gitlab/robotics-automation/eut_onrobot_driver.git 
   ```
2. Install libnet:
   ```sh
   sudo apt install libnet1-dev
   ```
3. Build using colcon with symlink install:
   ```sh
   colcon build --symlink-install
   ```
4. Source the workspace:
   ```sh
   source install/setup.bash
   ```

## Usage
### Launch the driver
Launch the driver with `onrobot_type` [`vgc10`,`vg10`] and `connection_type` [`serial` (UR Tool I/O) or `tcp` (Control Box)] arguments.
   ```sh
   ros2 launch onrobot_driver onrobot_control.launch.py onrobot_type:=vgc10 connection_type:=tcp 
   ```
Other arguments:
- `use_fake_hardware` (default: `false`): Use mock hardware interface for testing
- `launch_rviz` (default: `true`): Launch RViz with the gripper model
- `launch_rsp` (default: `true`): Launch the Robot State Publisher node (publishes to `/tf`)
- `device` (default: `/tmp/ttyUR`): Virtual Serial device path (if using Modbus Serial)
- `ip_address` (default: `192.168.1.1`): IP address of the Compute Box (if using Modbus TCP)
- `port` (default: `502`): Port of the Compute Box (if using Modbus TCP)

### Get the `vacuum_level` joint state (percentage)
   ```sh
   ros2 topic echo /onrobot/joint_states
   ```
### Control the vacuum level with `vacuum_controller`(JointGroupPositionController)
   ```sh
   ros2 topic pub --once /onrobot/vacuum_controller/commands std_msgs/msg/Float64MultiArray "{data: [80.0]}"
   ```

## TO DOs
### MoveIt integration
- Create an action wrapper around the command topic.

### Implementing other controllers
A [Gripper Action Controller](https://control.ros.org/humble/doc/ros2_controllers/gripper_controllers/doc/userdoc.html) can be implemented to control the gripper with a `gripper_action_interface` and `GripperCommand` action. This will allow for more advanced control of the gripper, such as opening and closing with a specified force and monitoring the action state.

### Adding support for other grippers
To come in the future

## Author & Mantainer
Alejandro González

