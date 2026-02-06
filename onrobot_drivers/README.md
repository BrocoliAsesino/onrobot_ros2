# OnRobot Drivers - ROS 2 Control Hardware Interface

**Maintainer:** Alejandro González (alejandro.gonzalez@local.eurecat.org)

A modular ROS 2 Control hardware interface package for OnRobot grippers, supporting multiple gripper types through a plugin-based architecture.

## 📋 Table of Contents

- [Overview](#overview)
- [Architecture](#architecture)
- [Supported Grippers](#supported-grippers)
- [How It Works: ROS 2 Control Deep Dive](#how-it-works-ros-2-control-deep-dive)
- [File Structure and Interconnections](#file-structure-and-interconnections)
- [Creating a New Gripper Plugin](#creating-a-new-gripper-plugin)
- [Building and Testing](#building-and-testing)
- [Troubleshooting](#troubleshooting)

---

## Overview

This package provides **hardware interfaces** for OnRobot grippers in ROS 2 Control. Each gripper type (VG, RG, 2FG, etc.) has its own specialized plugin while sharing common communication infrastructure.

### Key Features

- ✅ **Multi-gripper support** - Separate plugins for VG vacuum, RG parallel, etc.
- ✅ **Dual communication** - TCP and Serial (RS485) support
- ✅ **Modular design** - Shared Modbus communication layer
- ✅ **ROS 2 Control integration** - Full lifecycle management
- ✅ **Thread-safe** - Real-time control loop compatible

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                         ROS 2 CONTROL FRAMEWORK                     │
│  ┌────────────────────┐                    ┌────────────────────┐  │
│  │   Controllers      │                    │  Controller        │  │
│  │  (Position, etc.)  │◄──────────────────►│  Manager           │  │
│  └────────────────────┘                    └──────────┬─────────┘  │
│                                                        │             │
│                                              ┌─────────▼─────────┐  │
│                                              │ Hardware Interface │  │
│                                              │   Plugin Loader    │  │
│                                              └─────────┬─────────┘  │
└──────────────────────────────────────────────────────┼─────────────┘
                                                        │
                          ┌─────────────────────────────┼─────────────────────────┐
                          │        ONROBOT_DRIVERS PACKAGE                        │
                          │                                                        │
         ┌────────────────▼────────────────┐       ┌──────────────────────────┐  │
         │  VG Plugin (Vacuum Grippers)    │       │  RG Plugin (Parallel)    │  │
         │  ┌──────────────────────────┐   │       │  ┌───────────────────┐   │  │
         │  │ VGHardwareInterface      │   │       │  │ RGHardwareInterface│  │  │
         │  │  - on_init()             │   │       │  │  - on_init()       │  │  │
         │  │  - on_configure()        │   │       │  │  - on_configure()  │  │  │
         │  │  - read() / write()      │   │       │  │  - read() / write()│  │  │
         │  │  - export_interfaces()   │   │       │  │  - export_...()    │  │  │
         │  └───────────┬──────────────┘   │       │  └──────────┬─────────┘   │  │
         │              │                   │       │             │             │  │
         │      ┌───────▼────────┐          │       │     ┌───────▼────────┐   │  │
         │      │  VG Class      │          │       │     │  RG Class      │   │  │
         │      │  Device Driver │          │       │     │  Device Driver │   │  │
         │      └───────┬────────┘          │       │     └───────┬────────┘   │  │
         └──────────────┼───────────────────┘       └─────────────┼────────────┘  │
                        │                                          │                │
                        └──────────┬───────────────────────────────┘                │
                                   │                                                │
                          ┌────────▼─────────────────────────────────────┐         │
                          │         COMMON LAYER                         │         │
                          │  ┌────────────────────────────────────┐      │         │
                          │  │  IModbusConnection (Interface)     │      │         │
                          │  └────────┬───────────────────┬───────┘      │         │
                          │           │                   │              │         │
                          │  ┌────────▼─────────┐  ┌──────▼──────────┐  │         │
                          │  │ TCPConnection    │  │ SerialConnection│  │         │
                          │  │ Wrapper          │  │ Wrapper         │  │         │
                          │  └────────┬─────────┘  └──────┬──────────┘  │         │
                          └───────────┼────────────────────┼─────────────┘         │
                                      │                    │                        │
                          ┌───────────▼────────────────────▼─────────────┐         │
                          │        MODBUS LIBRARY (Submodule)            │         │
                          │   - ModbusRequest / ModbusResponse           │         │
                          │   - TCP::Connection / Serial::Connection     │         │
                          │   - CRC, Exception Handling                  │         │
                          └──────────────────┬───────────────────────────┘         │
                                             │                                      │
└─────────────────────────────────────────────┼──────────────────────────────────┘
                                              │
                                              ▼
                                    ┌──────────────────┐
                                    │ Physical Gripper │
                                    │  (Modbus Device) │
                                    └──────────────────┘
```

---

## Supported Grippers

| Gripper | Plugin Name | Device Types | Status |
|---------|-------------|--------------|--------|
| **VG** (Vacuum) | `onrobot_drivers/VGHardwareInterface` | VG10, VGC10 | ✅ Implemented |
| **RG** (Parallel) | `onrobot_drivers/RGHardwareInterface` | RG2, RG6 | ✅ Implemented |
| **2FG** (2-Finger) | `onrobot_drivers/2FGHardwareInterface` | 2FG7, 2FG14 | 🚧 Planned |

---

## How It Works: ROS 2 Control Deep Dive

### The ROS 2 Control Lifecycle

ROS 2 Control uses a **lifecycle-based plugin system** to manage hardware interfaces. Understanding this is key to creating your own plugins.

#### 1. **Launch Time: Plugin Discovery**

When you launch `ros2_control_node`, here's what happens:

```python
# In your launch file
robot_description = Command(['xacro', 'robot.urdf.xacro', ...])
ros2_control_node = Node(
    package='controller_manager',
    executable='ros2_control_node',
    parameters=[robot_description, controller_config]
)
```

**Under the hood:**
```cpp
// ros2_control_node C++ code (simplified)

// 1. Parse URDF and find <ros2_control> tags
urdf::Model model;
model.initString(robot_description);

// 2. Extract plugin information from URDF
<ros2_control name="OnRobotGripper" type="actuator">
    <hardware>
        <plugin>onrobot_drivers/VGHardwareInterface</plugin>  // ← This string
        <param name="ip_address">192.168.1.1</param>
    </hardware>
</ros2_control>

// 3. Use pluginlib to dynamically load the plugin
pluginlib::ClassLoader<hardware_interface::ActuatorInterface> loader(
    "hardware_interface", 
    "hardware_interface::ActuatorInterface"
);

// 4. pluginlib searches for XML descriptor files exported by packages
//    It finds: vg_hardware_interface.xml
<library path="onrobot_vg_plugin">  // ← Shared library name
  <class name="onrobot_drivers/VGHardwareInterface"  // ← Matches URDF
         type="vg_hardware_interface::VGHardwareInterface"  // ← C++ type
         base_class_type="hardware_interface::ActuatorInterface">
  </class>
</library>

// 5. Load the .so library and instantiate the class
auto hw = loader.createInstance("onrobot_drivers/VGHardwareInterface");
// This calls: new vg_hardware_interface::VGHardwareInterface()
```

#### 2. **Initialization Phase**

```cpp
// ros2_control_node calls lifecycle methods in sequence:

// STEP 1: on_init()
hardware_interface::CallbackReturn on_init(const HardwareInfo &info) {
    // Parse URDF parameters
    onrobot_type_ = info.hardware_parameters.at("onrobot_type");
    ip_address_ = info.hardware_parameters.at("ip_address");
    // Store them, but DON'T connect to hardware yet!
    return CallbackReturn::SUCCESS;
}

// STEP 2: on_configure()
hardware_interface::CallbackReturn on_configure(...) {
    // NOW connect to hardware
    gripper_ = std::make_unique<VG>(onrobot_type_, ip_address_, port_);
    // Read initial state
    vacuum_state_ = gripper_->getVacuumLevel();
    return CallbackReturn::SUCCESS;
}

// STEP 3: on_activate()
hardware_interface::CallbackReturn on_activate(...) {
    // Hardware is ready, enable control
    return CallbackReturn::SUCCESS;
}
```

**Why this separation?**
- `on_init()`: Fast, no blocking operations (parse config only)
- `on_configure()`: Can take time (network connections, device initialization)
- `on_activate()`: Final checks before real-time loop starts

#### 3. **Interface Export: Memory Sharing**

Before the control loop starts, the hardware interface **exports pointers** to its internal state:

```cpp
std::vector<StateInterface> export_state_interfaces() {
    std::vector<StateInterface> interfaces;
    
    // Create a StateInterface that points to our member variable
    interfaces.emplace_back(
        "gripper_suction_joint",    // Joint name
        "position",                  // Interface type
        &vacuum_state_              // ← POINTER to our variable
    );
    
    return interfaces;
}

std::vector<CommandInterface> export_command_interfaces() {
    std::vector<CommandInterface> interfaces;
    
    interfaces.emplace_back(
        "gripper_suction_joint",
        "position",
        &vacuum_command_            // ← POINTER to our variable
    );
    
    return interfaces;
}
```

**Key Insight:** Controllers and hardware interface share memory directly through these pointers!

```
┌─────────────────────┐          ┌─────────────────────────┐
│    Controller       │          │  Hardware Interface     │
│                     │          │                         │
│  double* cmd_ptr ───┼─────────►│  double vacuum_command_ │
│                     │  Shared  │                         │
│  double* state_ptr ─┼─────────►│  double vacuum_state_   │
│                     │  Memory  │                         │
└─────────────────────┘          └─────────────────────────┘
```

#### 4. **Real-Time Control Loop**

Once activated, `ros2_control_node` runs this loop at the configured rate (e.g., 50 Hz):

```cpp
while (rclcpp::ok()) {
    auto start = std::chrono::steady_clock::now();
    
    // ══════════════════════════════════════════════════════════
    // PHASE 1: READ FROM HARDWARE
    // ══════════════════════════════════════════════════════════
    for (auto& hw_interface : hardware_interfaces_) {
        hw_interface->read(time, period);
    }
    // Your code executes:
    // vacuum_state_ = gripper_->getVacuumLevel();  // Updates shared memory
    
    
    // ══════════════════════════════════════════════════════════
    // PHASE 2: UPDATE CONTROLLERS
    // ══════════════════════════════════════════════════════════
    for (auto& controller : active_controllers_) {
        controller->update(time, period);
    }
    // Controller reads *state_ptr (your vacuum_state_)
    // Controller computes new command
    // Controller writes to *cmd_ptr (your vacuum_command_)
    
    
    // ══════════════════════════════════════════════════════════
    // PHASE 3: WRITE TO HARDWARE
    // ══════════════════════════════════════════════════════════
    for (auto& hw_interface : hardware_interfaces_) {
        hw_interface->write(time, period);
    }
    // Your code executes:
    // gripper_->setVacuumLevel(vacuum_command_);  // Sends to hardware
    
    
    // Sleep to maintain cycle time
    auto end = std::chrono::steady_clock::now();
    auto elapsed = end - start;
    std::this_thread::sleep_for(period - elapsed);
}
```

**Timeline Example (50 Hz = 20ms period):**

```
Time(ms) | Action
---------|--------------------------------------------------------
0        | read()  → vacuum_state_ = 45.0 (read from gripper)
2        | Controllers update → vacuum_command_ = 50.0 (new target)
4        | write() → gripper_->setVacuum(50.0) (send to gripper)
20       | [Cycle repeats]
```

---

## File Structure and Interconnections

### Directory Layout

```
onrobot_drivers/
├── CMakeLists.txt              # Build configuration
├── package.xml                 # ROS 2 package manifest
├── vg_hardware_interface.xml   # VG plugin descriptor (for pluginlib)
├── rg_hardware_interface.xml   # RG plugin descriptor
│
├── include/onrobot_drivers/
│   ├── common/                 # 🔄 SHARED CODE (reused by all grippers)
│   │   ├── IModbusConnection.hpp         # Interface for TCP/Serial abstraction
│   │   ├── TCPConnectionWrapper.hpp      # TCP implementation
│   │   ├── SerialConnectionWrapper.hpp   # Serial implementation
│   │   └── Modbus/                       # Git submodule (3rd party library)
│   │
│   ├── vg/                     # 🟢 VG GRIPPER SPECIFIC
│   │   ├── vg_hardware_interface.hpp     # ROS 2 Control adapter
│   │   └── VG.hpp                        # Device driver (register mapping)
│   │
│   └── rg/                     # 🟣 RG GRIPPER SPECIFIC
│       ├── rg_hardware_interface.hpp
│       └── RG.hpp
│
├── src/
│   ├── vg/                     # VG implementation files
│   │   ├── vg_hardware_interface.cpp
│   │   └── VG.cpp
│   └── rg/                     # RG implementation files
│       ├── rg_hardware_interface.cpp
│       └── RG.cpp
│
├── config/
│   ├── vg_controllers.yaml     # Controller configuration for VG
│   └── rg_controllers.yaml     # Controller configuration for RG
│
└── launch/
    └── onrobot_control.launch.py   # Launch file
```

### File Interconnections

#### **Layer 1: Plugin Descriptor (XML)**

**Purpose:** Tell pluginlib how to load the plugin

**File:** `vg_hardware_interface.xml`
```xml
<library path="onrobot_vg_plugin">  <!-- Shared library name -->
  <class name="onrobot_drivers/VGHardwareInterface"  <!-- Plugin ID -->
         type="vg_hardware_interface::VGHardwareInterface"  <!-- C++ class -->
         base_class_type="hardware_interface::ActuatorInterface">
  </class>
</library>
```

**Registered in:** `package.xml`
```xml
<export>
  <hardware_interface plugin="${prefix}/vg_hardware_interface.xml"/>
</export>
```

**Built in:** `CMakeLists.txt`
```cmake
add_library(onrobot_vg_plugin SHARED ...)
pluginlib_export_plugin_description_file(hardware_interface vg_hardware_interface.xml)
```

---

#### **Layer 2: Hardware Interface (ROS 2 Control Adapter)**

**Purpose:** Implement ROS 2 Control API and manage gripper lifecycle

**Files:** 
- `include/onrobot_drivers/vg/vg_hardware_interface.hpp`
- `src/vg/vg_hardware_interface.cpp`

**Key Responsibilities:**

1. **Parse URDF Parameters:**
```cpp
hardware_interface::CallbackReturn on_init(const HardwareInfo &info) {
    // Extract from URDF: <param name="ip_address">192.168.1.1</param>
    ip_address_ = info.hardware_parameters.at("ip_address");
}
```

2. **Create Device Driver:**
```cpp
hardware_interface::CallbackReturn on_configure(...) {
    gripper_ = std::make_unique<VG>(onrobot_type_, ip_address_, port_);
}
```

3. **Export Interfaces:**
```cpp
std::vector<StateInterface> export_state_interfaces() {
    return {StateInterface("joint_name", "position", &vacuum_state_)};
}
```

4. **Real-Time Loop:**
```cpp
hardware_interface::return_type read(...) {
    vacuum_state_ = gripper_->getVacuumLevel();  // Hardware → ROS
}

hardware_interface::return_type write(...) {
    gripper_->setVacuumLevel(vacuum_command_);   // ROS → Hardware
}
```

---

#### **Layer 3: Device Driver (Gripper-Specific Logic)**

**Purpose:** Implement device-specific communication protocol

**Files:**
- `include/onrobot_drivers/vg/VG.hpp`
- `src/vg/VG.cpp`

**Key Responsibilities:**

1. **Connection Management:**
```cpp
VG(const std::string &type, const std::string &ip, int port) {
    connection = std::make_unique<TCPConnectionWrapper>(ip, port);
    // Set gripper-specific parameters
    max_vacuum = 80.0f;
}
```

2. **Register Mapping (Hardware-Specific):**
```cpp
class VG {
public:
    static constexpr uint16_t DEVICE_ID = 65;           // Modbus device address
    static constexpr uint16_t VG_REG_CTRL_A = 0;        // Control register
    static constexpr uint16_t STATUS_ADDR_A_VACUUM = 18; // Vacuum level register
    
    float getVacuumLevel(uint16_t channel);
    bool setVacuumLevel(float vacuum_val);
};
```

3. **High-Level Commands:**
```cpp
bool VG::setVacuumLevel(float vacuum_val) {
    uint16_t command = buildCommandWord(MODE_GRIP, vacuum_val);
    return writeRegister(VG_REG_CTRL_A, command);
}
```

4. **Low-Level Modbus Operations:**
```cpp
bool VG::writeRegister(uint16_t address, uint16_t value) {
    MB::ModbusRequest req = MB::ModbusRequest::writeSingleRegister(
        DEVICE_ID, address, value
    );
    MB::ModbusResponse resp = connection->sendRequest(req);
    return !resp.isException();
}
```

---

#### **Layer 4: Communication Abstraction**

**Purpose:** Abstract TCP vs Serial differences

**Files:**
- `include/onrobot_drivers/common/IModbusConnection.hpp` (interface)
- `include/onrobot_drivers/common/TCPConnectionWrapper.hpp`
- `include/onrobot_drivers/common/SerialConnectionWrapper.hpp`

**Pattern:** Strategy Pattern (polymorphism)

```cpp
// Interface
class IModbusConnection {
public:
    virtual MB::ModbusResponse sendRequest(const MB::ModbusRequest &req) = 0;
    virtual void close() = 0;
};

// TCP Implementation
class TCPConnectionWrapper : public IModbusConnection {
private:
    MB::TCP::Connection connection_;
public:
    TCPConnectionWrapper(const std::string &ip, int port) 
        : connection_(ip, port) {}
    
    MB::ModbusResponse sendRequest(const MB::ModbusRequest &req) override {
        return connection_.sendRequest(req);
    }
};

// Usage in VG class
std::unique_ptr<IModbusConnection> connection;  // Can be TCP or Serial
connection->sendRequest(req);  // Works for both!
```

---

#### **Layer 5: Modbus Protocol Library**

**Purpose:** Low-level Modbus protocol implementation

**Location:** `include/onrobot_drivers/common/Modbus/` (git submodule)

**Functionality:**
- Create Modbus packets (function codes 0x03, 0x06, 0x10, etc.)
- Parse responses
- CRC calculation
- Exception handling
- TCP and Serial socket management

---

### Data Flow: Complete Example

**User Command → Gripper Actuation:**

```
1. User Action
   ros2 action send_goal /vacuum_controller/follow_joint_trajectory {...}

2. Controller (vacuum_controller)
   - Receives goal: target_position = 60.0
   - Writes to shared memory: *command_ptr = 60.0
   
3. Hardware Interface (write method)
   vacuum_command_ = 60.0  (controller wrote to this)
   gripper_->setVacuumLevel(60.0)

4. Device Driver (VG class)
   command_word = buildCommandWord(MODE_GRIP, 60)  // 0x013C
   writeRegister(VG_REG_CTRL_A, 0x013C)

5. Communication Layer
   req = ModbusRequest::writeSingleRegister(65, 0, 0x013C)
   connection->sendRequest(req)  // Polymorphic call

6. Modbus Library (TCPConnectionWrapper)
   - Create Modbus TCP packet: [00 01 00 00 00 06 41 06 00 00 01 3C]
   - Send over socket to 192.168.1.1:502
   - Receive response: [00 01 00 00 00 06 41 06 00 00 01 3C]

7. Physical Gripper
   - Receives Modbus command
   - Activates vacuum to 60%
```

---

## Creating a New Gripper Plugin

### Step-by-Step Guide

Let's create a plugin for a fictional **OnRobot 3FG** (3-finger gripper).

---

### **Step 1: Create Directory Structure**

```bash
cd onrobot_drivers

# Create directories
mkdir -p include/onrobot_drivers/3fg
mkdir -p src/3fg
```

---

### **Step 2: Create Device Driver Header**

**File:** `include/onrobot_drivers/3fg/3FG.hpp`

```cpp
#pragma once
#include <memory>
#include <vector>
#include <string>

#include "onrobot_drivers/common/IModbusConnection.hpp"
#include "onrobot_drivers/common/TCPConnectionWrapper.hpp"
#include "onrobot_drivers/common/SerialConnectionWrapper.hpp"

#include "MB/modbusRequest.hpp"
#include "MB/modbusResponse.hpp"
#include "MB/modbusException.hpp"

class ThreeFG {
public:
    // Constructors
    ThreeFG(const std::string &type, const std::string &ip, int port);
    ThreeFG(const std::string &type, const std::string &device);
    ~ThreeFG();

    // High-level commands
    bool setFingerPosition(float pos_mm);
    bool setGripForce(float force_n);
    float getFingerPosition();
    bool isObjectDetected();

    // Register definitions (from gripper datasheet)
    static constexpr uint16_t DEVICE_ID = 66;
    static constexpr uint16_t REG_POSITION_CMD = 0;
    static constexpr uint16_t REG_FORCE_CMD = 1;
    static constexpr uint16_t REG_POSITION_STATUS = 20;
    static constexpr uint16_t REG_GRIP_STATUS = 21;

private:
    bool writeRegister(uint16_t address, uint16_t value);
    uint16_t readRegister(uint16_t address);
    MB::ModbusResponse sendRequest(const MB::ModbusRequest &req);

    std::unique_ptr<IModbusConnection> connection;
    std::string type;
    float max_opening;  // mm
    float max_force;    // N
};
```

---

### **Step 3: Implement Device Driver**

**File:** `src/3fg/3FG.cpp`

```cpp
#include "onrobot_drivers/3fg/3FG.hpp"

ThreeFG::ThreeFG(const std::string &type, const std::string &ip, int port)
    : type(type)
{
    // Validate type
    if (type != "3fg7" && type != "3fg15")
        throw std::invalid_argument("Invalid type. Use '3fg7' or '3fg15'");
    
    // Establish connection
    connection = std::make_unique<TCPConnectionWrapper>(ip, port);
    
    // Set parameters based on model
    if (type == "3fg7") {
        max_opening = 70.0f;  // mm
        max_force = 70.0f;    // N
    } else {
        max_opening = 150.0f;
        max_force = 150.0f;
    }
}

ThreeFG::~ThreeFG() {
    if (connection) connection->close();
}

bool ThreeFG::setFingerPosition(float pos_mm) {
    // Clamp to valid range
    if (pos_mm < 0.0f) pos_mm = 0.0f;
    if (pos_mm > max_opening) pos_mm = max_opening;
    
    // Convert mm to register value (example: 0-1000 range)
    uint16_t reg_value = static_cast<uint16_t>((pos_mm / max_opening) * 1000);
    
    return writeRegister(REG_POSITION_CMD, reg_value);
}

float ThreeFG::getFingerPosition() {
    uint16_t reg_value = readRegister(REG_POSITION_STATUS);
    // Convert register value to mm
    return (static_cast<float>(reg_value) / 1000.0f) * max_opening;
}

bool ThreeFG::writeRegister(uint16_t address, uint16_t value) {
    MB::ModbusRequest req = MB::ModbusRequest::writeSingleRegister(
        DEVICE_ID, address, value
    );
    MB::ModbusResponse resp = sendRequest(req);
    return !resp.isException();
}

uint16_t ThreeFG::readRegister(uint16_t address) {
    MB::ModbusRequest req = MB::ModbusRequest::readHoldingRegisters(
        DEVICE_ID, address, 1
    );
    MB::ModbusResponse resp = sendRequest(req);
    if (resp.isException()) throw MB::ModbusException("Read failed");
    return resp.data()[0];
}

MB::ModbusResponse ThreeFG::sendRequest(const MB::ModbusRequest &req) {
    return connection->sendRequest(req);
}
```

---

### **Step 4: Create Hardware Interface Header**

**File:** `include/onrobot_drivers/3fg/3fg_hardware_interface.hpp`

```cpp
#ifndef THREEFG_HW_INTERFACE_HPP
#define THREEFG_HW_INTERFACE_HPP

#include <memory>
#include <vector>
#include <string>
#include <mutex>

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "onrobot_drivers/3fg/3FG.hpp"

namespace threefg_hardware_interface
{

class ThreeFGHardwareInterface : public hardware_interface::ActuatorInterface
{
public:
    ThreeFGHardwareInterface();
    ~ThreeFGHardwareInterface() override;

    // Lifecycle methods
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo &info) override;
    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

    // Interface export
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    // Real-time loop
    hardware_interface::return_type read(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;
    hardware_interface::return_type write(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
    std::unique_ptr<ThreeFG> gripper_;
    std::string prefix_;

    // State/command variables (shared with controllers)
    double finger_position_state_;
    double finger_position_command_;

    // Parameters from URDF
    std::string onrobot_type_;
    std::string connection_type_;
    std::string ip_address_;
    int port_;
    std::string device_;

    std::mutex hw_interface_mutex_;
};

} // namespace threefg_hardware_interface

#endif // THREEFG_HW_INTERFACE_HPP
```

---

### **Step 5: Implement Hardware Interface**

**File:** `src/3fg/3fg_hardware_interface.cpp`

```cpp
#include "onrobot_drivers/3fg/3fg_hardware_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace threefg_hardware_interface
{

ThreeFGHardwareInterface::ThreeFGHardwareInterface()
    : finger_position_state_(0.0),
      finger_position_command_(0.0)
{
}

ThreeFGHardwareInterface::~ThreeFGHardwareInterface()
{
}

hardware_interface::CallbackReturn ThreeFGHardwareInterface::on_init(
    const hardware_interface::HardwareInfo &info)
{
    // Parse URDF parameters
    if (info.hardware_parameters.find("onrobot_type") != info.hardware_parameters.end()) {
        onrobot_type_ = info.hardware_parameters.at("onrobot_type");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("ThreeFGHardwareInterface"), 
                     "Missing onrobot_type parameter");
        return hardware_interface::CallbackReturn::ERROR;
    }

    if (onrobot_type_ != "3fg7" && onrobot_type_ != "3fg15") {
        RCLCPP_ERROR(rclcpp::get_logger("ThreeFGHardwareInterface"), 
                     "Invalid onrobot_type: %s", onrobot_type_.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Parse connection parameters
    connection_type_ = info.hardware_parameters.at("connection_type");
    
    if (connection_type_ == "tcp") {
        ip_address_ = info.hardware_parameters.at("ip_address");
        port_ = std::stoi(info.hardware_parameters.at("port"));
    } else if (connection_type_ == "serial") {
        device_ = info.hardware_parameters.at("device");
    }

    prefix_ = info.hardware_parameters.at("prefix");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ThreeFGHardwareInterface::on_configure(
    const rclcpp_lifecycle::State &)
{
    try {
        // Create gripper instance
        if (connection_type_ == "tcp") {
            gripper_ = std::make_unique<ThreeFG>(onrobot_type_, ip_address_, port_);
        } else {
            gripper_ = std::make_unique<ThreeFG>(onrobot_type_, device_);
        }
        
        // Read initial state
        finger_position_state_ = gripper_->getFingerPosition();
        finger_position_command_ = finger_position_state_;
        
        RCLCPP_INFO(rclcpp::get_logger("ThreeFGHardwareInterface"), 
                    "3FG gripper configured successfully");
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("ThreeFGHardwareInterface"), 
                     "Failed to configure: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ThreeFGHardwareInterface::on_cleanup(
    const rclcpp_lifecycle::State &)
{
    if (gripper_) {
        gripper_.reset();
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ThreeFGHardwareInterface::on_activate(
    const rclcpp_lifecycle::State &)
{
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ThreeFGHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State &)
{
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> 
ThreeFGHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(
            prefix_ + "_finger_joint", 
            "position", 
            &finger_position_state_
        )
    );
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> 
ThreeFGHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
            prefix_ + "_finger_joint", 
            "position", 
            &finger_position_command_
        )
    );
    return command_interfaces;
}

hardware_interface::return_type ThreeFGHardwareInterface::read(
    const rclcpp::Time &, const rclcpp::Duration &)
{
    std::lock_guard<std::mutex> lock(hw_interface_mutex_);
    
    if (!gripper_) {
        return hardware_interface::return_type::ERROR;
    }
    
    try {
        finger_position_state_ = gripper_->getFingerPosition();
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("ThreeFGHardwareInterface"), 
                     "Read failed: %s", e.what());
        return hardware_interface::return_type::ERROR;
    }
    
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ThreeFGHardwareInterface::write(
    const rclcpp::Time &, const rclcpp::Duration &)
{
    std::lock_guard<std::mutex> lock(hw_interface_mutex_);
    
    if (!gripper_) {
        return hardware_interface::return_type::ERROR;
    }
    
    try {
        gripper_->setFingerPosition(finger_position_command_);
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("ThreeFGHardwareInterface"), 
                     "Write failed: %s", e.what());
        return hardware_interface::return_type::ERROR;
    }
    
    return hardware_interface::return_type::OK;
}

} // namespace threefg_hardware_interface

// Export the plugin
PLUGINLIB_EXPORT_CLASS(
    threefg_hardware_interface::ThreeFGHardwareInterface,
    hardware_interface::ActuatorInterface
)
```

---

### **Step 6: Create Plugin Descriptor**

**File:** `3fg_hardware_interface.xml`

```xml
<?xml version="1.0"?>
<library path="onrobot_3fg_plugin">
  <class name="onrobot_drivers/ThreeFGHardwareInterface"
         type="threefg_hardware_interface::ThreeFGHardwareInterface"
         base_class_type="hardware_interface::ActuatorInterface">
    <description>
        ROS 2 Control hardware interface for the OnRobot 3FG gripper.
    </description>
  </class>
</library>
```

---

### **Step 7: Update CMakeLists.txt**

Add to the existing `CMakeLists.txt`:

```cmake
# Build the 3FG hardware interface as a shared library
add_library(onrobot_3fg_plugin
  SHARED
  src/3fg/3fg_hardware_interface.cpp
  src/3fg/3FG.cpp
  include/onrobot_drivers/common/TCPConnectionWrapper.hpp
  include/onrobot_drivers/common/SerialConnectionWrapper.hpp
  include/onrobot_drivers/common/IModbusConnection.hpp
)
target_link_libraries(onrobot_3fg_plugin
  Modbus
)
target_include_directories(onrobot_3fg_plugin
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE
  include
)
ament_target_dependencies(onrobot_3fg_plugin
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
pluginlib_export_plugin_description_file(hardware_interface 3fg_hardware_interface.xml)

# Add to install targets
install(TARGETS onrobot_vg_plugin onrobot_rg_plugin onrobot_3fg_plugin
  DESTINATION lib
)

# Add to exports
ament_export_libraries(
  onrobot_vg_plugin
  onrobot_rg_plugin
  onrobot_3fg_plugin
)
```

---

### **Step 8: Create Controller Configuration**

**File:** `config/3fg_controllers.yaml`

```yaml
$(var ns):
  controller_manager:
    ros__parameters:
      update_rate: 100  # Hz

      joint_state_broadcaster:
        type: joint_state_broadcaster/JointStateBroadcaster

      finger_controller:
        type: position_controllers/JointGroupPositionController
        
  finger_controller:
    ros__parameters:
      joints:
        - _finger_joint
```

---

### **Step 9: Test Your Plugin**

1. **Build:**
```bash
cd ~/ros2_ws
colcon build --packages-select onrobot_drivers
source install/setup.bash
```

2. **Create test URDF:**
```xml
<ros2_control name="OnRobot3FG" type="actuator">
    <hardware>
        <plugin>onrobot_drivers/ThreeFGHardwareInterface</plugin>
        <param name="onrobot_type">3fg7</param>
        <param name="connection_type">tcp</param>
        <param name="ip_address">192.168.1.1</param>
        <param name="port">502</param>
        <param name="prefix"></param>
    </hardware>
    <joint name="_finger_joint">
        <command_interface name="position"/>
        <state_interface name="position"/>
    </joint>
</ros2_control>
```

3. **Launch:**
```bash
ros2 launch onrobot_drivers onrobot_control.launch.py \
    onrobot_type:=3fg7 \
    connection_type:=tcp \
    ip_address:=192.168.1.1
```

4. **Test command:**
```bash
ros2 topic pub /finger_controller/commands std_msgs/msg/Float64MultiArray \
  "data: [30.0]"  # Open to 30mm
```

---

## Building and Testing

### Build Package

```bash
cd ~/ros2_ws
colcon build --packages-select onrobot_drivers
source install/setup.bash
```

### Verify Plugin Registration

```bash
# List all hardware_interface plugins
ros2 pkg plugins --package hardware_interface

# Should show:
# onrobot_drivers/VGHardwareInterface
# onrobot_drivers/RGHardwareInterface
```

### Test with Fake Hardware

Use `use_fake_hardware:=true` to test without physical gripper:

```bash
ros2 launch onrobot_drivers onrobot_control.launch.py \
    onrobot_type:=vg10 \
    use_fake_hardware:=true
```

---

## Troubleshooting

### Plugin Not Found

**Error:** `Could not load 'onrobot_drivers/VGHardwareInterface'`

**Solutions:**
1. Check `vg_hardware_interface.xml` path matches `CMakeLists.txt`
2. Verify `package.xml` exports the plugin:
   ```xml
   <export>
     <hardware_interface plugin="${prefix}/vg_hardware_interface.xml"/>
   </export>
   ```
3. Rebuild and source:
   ```bash
   colcon build --packages-select onrobot_drivers --cmake-clean-cache
   source install/setup.bash
   ```

### Connection Timeout

**Error:** `Failed to establish TCP connection`

**Solutions:**
1. Verify gripper IP: `ping 192.168.1.1`
2. Check port: `telnet 192.168.1.1 502`
3. Ensure gripper is powered on
4. Check firewall rules

### Read/Write Errors

**Error:** `Modbus exception: Illegal data address`

**Solutions:**
1. Verify register addresses in datasheet
2. Check `DEVICE_ID` matches gripper configuration
3. Enable Modbus debug logging

---

## Additional Resources

- [ROS 2 Control Documentation](https://control.ros.org/)
- [Modbus Protocol Specification](https://modbus.org/specs.php)
- [OnRobot Technical Documentation](https://onrobot.com/en/downloads)

---

## License

MIT License - See LICENSE file

---

## Contributing

Contributions welcome! To add support for a new gripper:

1. Fork the repository
2. Create feature branch: `git checkout -b feature/add-xyz-gripper`
3. Follow the "Creating a New Gripper Plugin" guide
4. Test thoroughly with physical hardware
5. Submit pull request with:
   - Gripper datasheet reference
   - Tested hardware configuration
   - Example launch files

---

**Questions?** Contact: alejandro.gonzalez@local.eurecat.org
