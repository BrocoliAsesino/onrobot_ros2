# OnRobot ROS 2 Driver

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-blue)](https://docs.ros.org/en/jazzy/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

A modular ROS 2 Control hardware interface for OnRobot grippers with multi-gripper support, Modbus TCP/Serial communication, and comprehensive URDF models.

<p align="center">
  <img src="https://onrobot.com/hubfs/VGC10-1.png" width="300"/>
  <img src="https://onrobot.com/hubfs/VG10-1.png" width="300"/>
</p>

## 📋 Table of Contents
- [Features](#-features)
- [Supported Grippers](#-supported-grippers)
- [Architecture](#-architecture)
- [Installation](#-installation)
- [Quick Start](#-quick-start)
- [Usage Examples](#-usage-examples)
- [Configuration](#-configuration)
- [Integration with MoveIt](#-integration-with-moveit)
- [Troubleshooting](#-troubleshooting)
- [Contributing](#-contributing)

## Features

- **Plugin-Based Architecture**: Modular design supporting multiple OnRobot gripper families (VG, RG)
- **ROS 2 Control Integration**: Standard `hardware_interface` 
- **Dual Communication**: Modbus TCP (Control Box) and Serial (UR Tool I/O RS485)
- **Mock Hardware Support**: Test without physical hardware using `mock_components/GenericSystem`
- **Complete URDF Models**: Parametric xacro models 
- **Namespaced Operation**: Multi-gripper setups with configurable namespaces

## Supported Grippers

### Vacuum Grippers (VG Series)
| Model | Type | Range | Status |
|-------|------|-------|--------|
| [VGC10](https://onrobot.com/en/products/vgc10-vacuum-gripper) | Compact Vacuum | 0-80 kPa | ✅ Supported |
| [VG10](https://onrobot.com/en/products/vg10-vacuum-gripper) | Standard Vacuum | 0-80 kPa | ✅ Supported |

### Parallel Grippers (RG Series)
| Model | Type | Range | Status |
|-------|------|-------|--------|
| RG2 | 2-Finger Parallel | 0-110 mm | 🚧 Planned |
| RG6 | 6-Finger Parallel | 0-160 mm | 🚧 Planned |


## Installation

### Prerequisites
- **ROS 2**: Jazzy or later
- **Compiler**: C++17 compatible
- **Dependencies**: 
  - `ros2_control`
  - `ros2_controllers`
  - `libnet1-dev` (for Modbus)

### Option 1: Standalone Installation

```bash
# Create workspace
mkdir -p ~/gripper_ws/src
cd ~/gripper_ws/src

# Clone with submodules
git clone --recurse-submodules https://github.com/BrocoliAsesino/onrobot_ros2.git

# Install dependencies
cd ~/gripper_ws
rosdep install --from-paths src --ignore-src -r -y
sudo apt install libnet1-dev

# Build
colcon build --symlink-install

# Source
source install/setup.bash
```

### Option 2: As a Git Submodule

```bash
# In your existing ROS 2 workspace
cd ~/your_workspace/src
git submodule add -b master https://github.com/BrocoliAsesino/onrobot_ros2.git
git submodule update --init --recursive

# Build your workspace
cd ~/your_workspace
colcon build --symlink-install
```

## Quick Start

### 1. Test with Mock Hardware (No Physical Gripper)

```bash
ros2 launch onrobot_drivers onrobot_control.launch.py \
  onrobot_type:=vgc10 \
  connection_type:=tcp \
  use_fake_hardware:=true
```

RViz will open showing the gripper model. Test control:
```bash
# Set vacuum to 60 kPa
ros2 topic pub --once /onrobot/vacuum_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [60.0]}"
```

### 2. Connect to Real Hardware via TCP

```bash
ros2 launch onrobot_drivers onrobot_control.launch.py \
  onrobot_type:=vgc10 \
  connection_type:=tcp \
  ip_address:=192.168.1.121 \
  use_fake_hardware:=false
```

### 3. Connect via Serial (UR Robot Tool I/O)

```bash
ros2 launch onrobot_drivers onrobot_control.launch.py \
  onrobot_type:=vg10 \
  connection_type:=serial \
  device:=/tmp/ttyUR \
  use_fake_hardware:=false
```

## Usage Examples

### Monitor Joint States

```bash
# Real-time joint states (vacuum level in kPa)
ros2 topic echo /onrobot/joint_states

# Specific joint
ros2 topic echo /onrobot/joint_states --field position
```

### Control Vacuum Level

```bash
# Set to 40 kPa
ros2 topic pub --once /onrobot/vacuum_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [40.0]}"

# Full vacuum (80 kPa)
ros2 topic pub --once /onrobot/vacuum_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [80.0]}"

# Release vacuum (0 kPa)
ros2 topic pub --once /onrobot/vacuum_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.0]}"
```

## ⚙️ Configuration

### Launch Arguments

| Argument | Default | Choices | Description |
|----------|---------|---------|-------------|
| `onrobot_type` | *required* | `vgc10`, `vg10`, `rg2`, `rg6` | Gripper model |
| `connection_type` | *required* | `tcp`, `serial` | Communication protocol |
| `ip_address` | `192.168.1.1` | - | TCP IP address |
| `port` | `502` | - | Modbus TCP port |
| `device` | `/tmp/ttyUR` | - | Serial device path |
| `num_cups` | `4` | `1`, `4` | VGC10 suction cups |
| `use_fake_hardware` | `false` | `true`, `false` | Enable mock hardware |
| `ns` | `onrobot` | - | Node namespace |
| `prefix` | `""` | - | Joint name prefix |
| `launch_rviz` | `true` | `true`, `false` | Start RViz |
| `launch_rsp` | `true` | `true`, `false` | Start robot_state_publisher |
| `include_only_plugin` | `false` | `true`, `false` | Minimal URDF (no meshes) |

### Controller Configuration

Edit `onrobot_drivers/config/vg_controllers.yaml`:

```yaml
$(var ns):
  controller_manager:
    ros__parameters:
      update_rate: 50  # Hz (lower if seeing timing warnings)

      joint_state_broadcaster:
        type: joint_state_broadcaster/JointStateBroadcaster

      vacuum_controller:
        type: position_controllers/JointGroupPositionController
        
  vacuum_controller:
    ros__parameters:
      joints:
        - _suction_regulator_joint
```

### Multi-Gripper Setup

```bash
# Left gripper
ros2 launch onrobot_drivers onrobot_control.launch.py \
  ns:=left_gripper \
  ip_address:=192.168.1.121 \
  onrobot_type:=vgc10 \
  connection_type:=tcp \
  launch_rviz:=false

# Right gripper
ros2 launch onrobot_drivers onrobot_control.launch.py \
  ns:=right_gripper \
  ip_address:=192.168.1.122 \
  onrobot_type:=vg10 \
  connection_type:=tcp \
  launch_rviz:=false
```

## Integration with MoveIt

### Current State
The driver provides a `JointGroupPositionController` that can be commanded via topics.

### Recommended Integration (TODO)

1. **Create Action Server**
   - Wrap `/vacuum_controller/commands` topic
   - Use `onrobot_interfaces/action/VGC10GripperCommand.action`
   - Provide feedback and result states

2. **MoveIt Configuration**
   ```yaml
   # moveit_config/config/gripper_controller.yaml
   gripper_controller:
     action_ns: /onrobot/gripper_action
     type: GripperCommand
     joints:
       - _suction_regulator_joint
   ```

3. **MoveIt Task Constructor**
   ```cpp
   auto grasp = std::make_unique<stages::ModifyPlanningScene>("grasp");
   grasp->attachObject("object", "onrobot_vg_base_link");
   
   auto close_gripper = std::make_unique<stages::MoveTo>("close", planner);
   close_gripper->setGroup("gripper");
   close_gripper->setGoal("closed");  // Named pose in SRDF
   ```

## Troubleshooting

### Issue: "Overrun detected! Missed desired rate"
**Cause**: Communication latency exceeds control loop period.

**Solution**: Lower update rate in `vg_controllers.yaml`:
```yaml
update_rate: 20  # Reduce from 50-100 Hz
```

### Issue: "Could not contact service /controller_manager"
**Cause**: Namespace mismatch.

**Solution**: Specify namespace:
```bash
ros2 control list_controllers --controller-manager /onrobot/controller_manager
```
### Issue: "Failed to connect to Modbus TCP"
**Cause**: Network unreachable or wrong IP.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contact 

**Maintainer:** Alejandro González  
**Email:** agonzalez.hdez1@gmail.com


