## Good News, Everyone! You’re About to Write a Custom Controller For ros2_control!

![Image](https://github.com/user-attachments/assets/944a5167-43e6-4038-8b45-c8d599f56b53)

---

#### TL;DR

#### _This guide will show you how to write your own controller and hardware interface—from the URDF to the serial stream. You’ll craft a custom controller, export it as a plugin, and actually talk to your motors like a true scholar. Yes, yes—there’ll be launch files, YAML, CMakeLists and other ancient rituals. The robot still won’t thank you. But I will. Probably._
---

### What is a _Controller_ in `ros2_control`?

In the **`ros2_control`** framework, a `controller` isn't a standalone `node`-- it is only an _algorithmic plugin_ that the **Controller Manager (CM)** loads and manages. The plugin runs your control algorithm inside a real-time loop that:

- Reads the robot's current _states_ values
- Executes **_your algorithm_**
- Writes new _commands_ values back to **_your hardware_**

In a nutshell, the controller acts as a bridge between the robot's state interfaces and command interfaces. Every controller lives inside the **Controller Manager** process. The **CM** owns the real-time loop.

Imagine your robot is a car. The **controller** is the driver's brain deciding _how much_ to push the gas/brakes and _when_ to steer. The `ros2_control` package comes packed with many built-in excellent **controllers** that can be used for most of the robotics applications. However, sometimes you'll need to write your own custom controllers if your robot is governed by some custom governing principles or control laws and does some specilized jobs. 

Yes, it might sound daunting. But don't worry! In this tutorial, we'll walk through the steps to write your own **custom controller** and integrate it cleanly with `ros2_control`. We'll package the controller as a plugin, so you can load, unload, and swap control algorithms at run time-- without recompiling your whole system. It's modular, it's flexible, and honestly......it's pretty cool! 

By the way your imaginary robot car's engine and steering can be considered as the **hardware interface**. I will also discuss how to build your own _custom hardware interface_.

![Image](https://github.com/user-attachments/assets/e5ff4952-5dec-4b17-b928-0008413f1260)

Before we move to our main tutorial, here is a brief overview of different `ros2_control` components.

1. **Controller Manager**:

   - Loads/unloads controllers
   - Manages controller lifecycle
   - Coordinates hardware access
   - Provides ROS2 services for controller management

2. **Controllers**:

   - Implement control algorithms
   - Process command inputs
   - Generate hardware commands
   - Publish state information
  
3. **Resource Manager**

   - Manages hardware interfaces
   - Ensures exclusive hardware access
   - Handles hardware communication protocols

4. **Hardware Interfaces**

   - Abstract hardware specific details
   - Provide standardize access methods
   - Handle low-level communications
   


### Steps to write a _custom controller_:

---
### 0. Our Example Robot:
We will use this custom actuator (https://www.nature.com/articles/s44182-025-00023-6) as an example to write your custom controller for ros2_control. 

![Image](https://github.com/user-attachments/assets/9c5ac8d7-0a38-44a7-b399-83998d42f6c5)

From our custom control law, the high level inputs are linear velocity (Vₜ) of the end-effector and the rotational velocity (Ω) of the end-effector. Using inverse kinematics we solve for ω₁ and ω₂, and the controller will send these commands to the motors. Don't worry if your robot isn't identical--this framework is designed to be adaptable. Follow along, and you'll be able to extend the ideas to your own project. Now, Create a `ros2 package`. Let's call it `my_custom_controller`.

- Use `ros2 pkg create`.
- Dependencies:
  - `controller_interface`
  - `hardware_interface`
  - `rclcpp`
  - `pluginlib`
---

### 1. Robot Description:
This is the robot's blueprint. Create a robot `URDF` with `<ros2_control>` tag. This robot has two joints. We only control the state _velocity_. The state _position_ is not really used for now. Let's call this `my_custom_robot_rviz.urdf.xacro`.

```xacro
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_custom_robot">
    <xacro:property name="PI" value="3.1416"/>


    <!-- Define material named "black" with RGBA color values -->
    <material name="black">
        <color rgba="0.0 0.0 0.0 1.0" />
    </material>

    <!-- Define material named "blue" with RGBA color values -->
    <material name="blue">
        <color rgba="0.0 0.0 0.8 1.0" />
    </material>

    <!-- Define material named "green" with RGBA color values -->
    <material name="green">
        <color rgba="0.0 0.8 0.0 1.0" />
    </material>
    
    <!-- Define material named "red" with RGBA color values -->
    <material name="red">
        <color rgba="0.8 0.0 0.0 1.0" />
    </material>



    <!-- Define link named "world" -->
    <link name="world" />

    <!-- Define link named "base" -->
    <link name="base">
        <collision>
            <origin xyz="0 0 0.05" rpy="0 0 0" />
            <geometry>
                <box size="3.0 3.0 0.1" />
            </geometry>
        </collision>
        <visual>
            <origin xyz="0 0 0.05" rpy="0 0 0" />
            <geometry>
                <box size="3.0 3.0 0.1" />
            </geometry>
            <material name="green" />
        </visual>

    </link>

    <!-- Define link named "roller_1" -->
    <link name="roller_1">
        <collision>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <geometry>
                <cylinder radius="0.3" length="0.5" />
            </geometry>
        </collision>
        <visual>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <geometry>
                <cylinder radius="0.3" length="0.5" />
            </geometry>
            <material name="red" />
        </visual>

    </link>

    <!-- Define link named "roller_2" -->
    <link name="roller_2">
        <collision>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <geometry>
                <cylinder radius="0.3" length="0.5" />
            </geometry>
        </collision>
        <visual>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <geometry>
                <cylinder radius="0.3" length="0.5" />
            </geometry>
            <material name="red" />
        </visual>

    </link>


    <!-- Define joint named "joint1" with type "fixed" (the base link is fixed to the world)-->
    <joint name="joint1" type="fixed">
        <parent link="world" />
        <child link="base" />
        <origin xyz="0 0 0" rpy="0 0 0" />
        <axis xyz="0 1 0" />
    </joint>

    <!-- Define joint named "joint2" with type "continuous" -->
    <joint name="joint2" type="continuous">
        <parent link="base" />
        <child link="roller_1" />
        <origin xyz="0.325 0 0.25" rpy="-0.5236 0 0" />
        <axis xyz="0 0 1" />
    </joint>

    <!-- Define joint named "joint3" with type "continuous" -->
    <joint name="joint3" type="continuous">
        <parent link="base" />
        <child link="roller_2" />
        <origin xyz="-0.325 0 0.25" rpy="-0.5236 0 0" />
        <axis xyz="0 0 1" />
    </joint>


    <ros2_control name="custom_controller" type="system">
        <hardware>
            <plugin>my_arduino_controller/MyArduinoInterface</plugin>
            <param name="port">/dev/ttyACM0</param>
            
        </hardware>

    <joint name="joint2">
        <command_interface name="velocity"/>
        <state_interface name="velocity"/>
        <state_interface name="position"/>
    </joint>

    <joint name="joint3">
        <command_interface name="velocity"/>
        <state_interface name="velocity"/>
        <state_interface name="position"/>
    </joint>

    </ros2_control>


</robot>
```
The URDF needs to describe the robot joints. In our case we have two rollers (revolute joints) actively controlled by two motors. The roller domensions and tilt angles do not need to match the real robot for now. This is only to define the controllable joints. Our _custom controller_ will compute the appropriate outputs from the governing equations. You need to use `<ros2_control>` tag in your URDF to describe different hardware components or the hardware setup. Notice:

```xml
        <hardware>
            <plugin>my_arduino_controller/MyArduinoInterface</plugin>
            <param name="port">/dev/ttyACM0</param>
            
        </hardware>
```
This is our _custom hardware interface_ and added as a plugin in the URDF. This gives us incredible flexibility when we change robots. All we need to do is to use a different plugin- the controller code, the launch file script etc. remain mostly the same as they communicate with the standardize interfaces provided by the plugin.

To visualize your URDF while building it, install this package:

`sudo apt-get install ros-jazzy-urdf-tutorial`

The launch your urdf with the following command:

`ros2 launch urdf_tutorial display.launch.py model:=/home/<put your user name>/ros2_ws/src/my_custom_controller/urdf/my_custom_robot.urdf.xacro`

After launch you will see something like this:

![Image](https://github.com/user-attachments/assets/33788928-c91a-4b22-8ae2-714f0bb1dc46)

Now change `Fixed frame` to `world`. Check if you can control the rollers with Joint State Publisher GUI. Now you can save your rviz config file to a folder `rviz`.

---

### 2. Write a _custom hardware interface_:
So, we know that `ros2_control` is a _robot_agnostic_ `'brain'` that ony understands two things:

1. Commands it _sends_ to hardware
2. States it _reads_ from hardware

The _**hardware interface**_ is the translator that:

- Takes the abstract commands coming from `ros2_control` controllers.
- Turns them into the exact bytes your actuators understand (e.g., serial packets).
- Reads raw sensor data (encoders, currents, etc.) and puts them into the abstract state variables `ros2_control` expects.

That’s it—everything else (controllers, URDF, launch files) can stay the same no matter what motors, sensors, or micro-controllers you swap in.

Hardware interfaces follow ROS2's lifecycle pattern:

![Image](https://github.com/user-attachments/assets/addfb01e-768d-48d0-a45c-503ca52518be)

The `SystemInterface` is the base class you inherit from to create your hardware interface. It provides a standardized lifecycle and interface management system:

![Image](https://github.com/user-attachments/assets/904c4520-d8f4-4df0-a357-5e515e50b85e)

**Class Definition and Headers**
```cpp
#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
// ... other includes

namespace my_arduino_controller
{
class MyArduinoInterface : public hardware_interface::SystemInterface
{
public:
    MyArduinoInterface();
    virtual ~MyArduinoInterface();
    
    // Lifecycle methods
    virtual CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;
    virtual CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
    
    // Interface methods
    virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    virtual hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    virtual hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    // Data storage
    std::vector<double> velocity_commands_;
    std::vector<double> velocity_states_;
    std::vector<double> position_states_;
    
    // Serial communication
    int SerialPort;
    struct termios tty;
    
    // Helper methods
    bool setupSerialCommunication();
    // ... other helpers
};
}
```

**Initialization (on_init)**
The `on_init()` method is called when the hardware interface is loaded:

```cpp

CallbackReturn MyArduinoInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
    // Call parent initialization
    CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
    if (result != CallbackReturn::SUCCESS) return result;
    
    // Create ROS2 node for publishing additional data
    node_ = std::make_shared<rclcpp::Node>("velocity_publisher_node");
    current_publisher_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>("current_values", 10);
    
    // Setup hardware communication
    if (!setupSerialCommunication()) {
        RCLCPP_WARN(rclcpp::get_logger("arduino_controller"), "Running in simulation mode");
    }
    
    // Initialize data vectors
    initializeDataVectors();
    
    return CallbackReturn::SUCCESS;
}

```

**Interface Export**

- State Interfaces

```cpp
std::vector<hardware_interface::StateInterface> MyArduinoInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    
    // Export velocity states for each joint
    for (size_t i = 0; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, 
            hardware_interface::HW_IF_VELOCITY, 
            &velocity_states_[i]
        ));
    }
    
    // Export position states for each joint
    for (size_t i = 0; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, 
            hardware_interface::HW_IF_POSITION, 
            &position_states_[i]
        ));
    }
    
    return state_interfaces;
}
```

- Command Interfaces

```cpp
std::vector<hardware_interface::CommandInterface> MyArduinoInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    
    // Export velocity commands for each joint
    for (size_t i = 0; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, 
            hardware_interface::HW_IF_VELOCITY, 
            &velocity_commands_[i]
        ));
    }
    
    return command_interfaces;
}
```

**Communication Loop**

![Image](https://github.com/user-attachments/assets/9673cce8-e58f-4f01-b8cb-435b50b3d92a)

`<Commands sent to Arduino (write() method)>`:

`Format: "velocity1 direction1 velocity2 direction2\n"
Example: "15.5 0 -20.0 1\n"`

`<Data received from Arduino (read() method)>`:

`Format: "current1 current2\n"
Example: "10.6 13.3\n"`

The arduino firmware is included as `my_custom_controller_firmware.ino`.

![Image](https://github.com/user-attachments/assets/aad2eae6-374b-47be-9f24-d90b1e5f7d6d)

---
### 3. Write the **_custom controller_** - The Brain:

The _controller manager_ loads your _custom controller_. The controller receives data:

- State feedback from hardware

- Commands from topics

Then the controller computes the output according to your custom control algorithm and writes them to the hardware commands.

Every controller has:

✅ **Lifecycle** (configured, activated, deactivated)  
✅ **Interfaces** (what it reads and writes)  
✅ **Callback functions** (update loop)  
✅ **Parameters** (gains, limits, co-efficients/constants)

![Image](https://github.com/user-attachments/assets/2a7bbd29-f881-4390-824f-1deb67dac091)


**Header File Setup**

- Class declaration:
  
  The controller inherits from `ControllerInterface`, which provides the lifecycle management and interface definitions.
```cpp
class MyCustomController : public controller_interface::ControllerInterface
```
- Essential includes:

  ```cpp
   #include "controller_interface/controller_interface.hpp"  // Base class
   #include "geometry_msgs/msg/twist.hpp"                   // For velocity commands
   #include "rclcpp_lifecycle/state.hpp"                    // For lifecycle states
  ```
- Configuration parameters:

  ```cpp
   std::vector<std::string> joint_names_;  // Names of joints to control
   std::string interface_name_;            // Type of control (velocity/position)
  ```
- Kinematic parameters:

  ```cpp
   double roller_radius_;  // Physical parameter of the robot
   double d_;             // Tool diameter
   double theta_;         // Tilt angle
  ```
- Communication components:

  ```cpp
   rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
   geometry_msgs::msg::Twist twist_command_;
  ```

**Initialization (on_init)**

```cpp
controller_interface::CallbackReturn MyCustomController::on_init()
{
    // Declare parameters with default values
    joint_names_ = auto_declare<std::vector<std::string>>("joints", {});
    interface_name_ = auto_declare<std::string>("interface_name", "velocity");
    
    // Kinematic parameters
    roller_radius_ = auto_declare<double>("roller_radius", 0.0125);
    d_ = auto_declare<double>("d", 0.025);
    
    return controller_interface::CallbackReturn::SUCCESS;
}
```

We use `auto_declare` to register parameters that can be set via YAML files or launch files. We also include some kinematic constants.

**Configuration (on_configure)**
```cpp
controller_interface::CallbackReturn MyCustomController::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    // Validate configuration
    if (joint_names_.size() != 2) {
        RCLCPP_ERROR(get_node()->get_logger(), 
                     "Expected exactly 2 joint names, but got %zu", joint_names_.size());
        return controller_interface::CallbackReturn::ERROR;
    }
    
    // Set up communication
    auto callback = [this](const geometry_msgs::msg::Twist::SharedPtr msg) -> void {
        twist_command_ = *msg;
    };
    
    cmd_vel_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
        "~/cmd_vel", 10, callback);
        
    return controller_interface::CallbackReturn::SUCCESS;
}
```
We check the robot's configuration and create a subscriber to send commands. The `~/` prefix creates a topic relative to the controller's namespace and lambda functions for simple callbacks.

_**Interface Configuration**_

- **Command Interfaces**
  ```cpp
     controller_interface::InterfaceConfiguration MyCustomController::command_interface_configuration() const
   {
       controller_interface::InterfaceConfiguration config;
       config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
       
       for (const auto& joint_name : joint_names_) {
           config.names.push_back(joint_name + "/" + interface_name_);
       }
       return config;
   }
  ```

- **State Interfaces**
  ```cpp
     controller_interface::InterfaceConfiguration MyCustomController::state_interface_configuration() const
   {
       controller_interface::InterfaceConfiguration config;
       config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
       
       for (const auto& joint_name : joint_names_) {
           config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
           config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
       }
       return config;
   }
  ```

**Activation (on_active)**

```cpp
controller_interface::CallbackReturn MyCustomController::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    // Initialize command to safe state
    twist_command_ = geometry_msgs::msg::Twist();
    
    RCLCPP_INFO(get_node()->get_logger(), "MyCustomController activated.");
    return controller_interface::CallbackReturn::SUCCESS;
}
```
We set commands to zero/safe values to prevent unexpected behavior from the robot at the startup.

**The Main Control Loop (update)**

_"I once designed a control loop that was so tight, it collapsed into a singularity!"_

The `update()` function is called repeatedly (typically at 100-1000 Hz) and contains our main control algorithm.

```cpp
controller_interface::return_type MyCustomController::update(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    // 1. Read input commands
    const double v_linear  = twist_command_.linear.x;
    const double v_angular = twist_command_.angular.z;
    
    // 2. Perform kinematic calculations
    
    
    // 3. Send commands to hardware
    command_interfaces_[0].set_value(left_cmd);
    command_interfaces_[1].set_value(right_cmd);
    
    return controller_interface::return_type::OK;
}
```

The complete code is provided in the file named `my_custom_controller.cpp`.

---

### 4. Export Plugins:

We will have to export both our _custom hardware interface_ and _custom controller_ as `xml` plugins.

- Custom hardware interface plugin (`my_custom_hardware_interface.xml`):

  ```xml
  <library path="my_custom_controller_hardware"> 
    <class name="my_arduino_controller/MyArduinoInterface"
           type="my_arduino_controller::MyArduinoInterface"
           base_class_type="hardware_interface::SystemInterface">
        <description>
            Custom Arduino controller interface for a robot with rollers.
        </description>
    </class>
   </library> 
  ```

- Custom controller plugin (`my_custom_controller.xml`):

  ```xml
  <library path="my_custom_controller_controller">
    <class name="my_custom_controller/MyCustomController"
           type="my_custom_controller::MyCustomController"
           base_class_type="controller_interface::ControllerInterface">
        <description>
            Custom controller for a robot with rollers, designed to interface with hardware.
        </description>
    </class>
   </library>
  ```

Notice that `class name` and `type` for both plugins come from their `namespace` and 'class' when we exported them using `PLUGINLIB_EXPORT_CLASS`

```cpp
PLUGINLIB_EXPORT_CLASS(my_arduino_controller::MyArduinoInterface, hardware_interface::SystemInterface)
```

```cpp
PLUGINLIB_EXPORT_CLASS(my_custom_controller::MyCustomController, controller_interface::ControllerInterface)
```
Also notice that I used `_hardware` and `_controller` tag to differentiate them in CMakeLists file for easier understanding (see the CMakeLists.txt).

_In your `package.xml` add these dependencies:

```xml
  <depend>controller_interface</depend>
  <depend>hardware_interface</depend>
  <depend>pluginlib</depend>
  <depend>rclcpp_lifecycle</depend>
 
  
  

  <exec_depend>urdf</exec_depend>
  <exec_depend>xacro</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher_gui</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <exec_depend>ros2launch</exec_depend>
```

---

### 5. Configuring Our Custom Controller:

We use `my_custom_controller_config.yaml` to configure our custom controller. Pay attenting when doing this. If you do not write this file in the correct format your _controller_ will fail to load.

```yaml
/my_custom_controller/controller_manager:
  ros__parameters:
    update_rate: 10
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

/my_custom_controller/rpm_controller:
  ros__parameters:
    type: my_custom_controller/MyCustomController
    joints:
      - joint2
      - joint3
    interface_name: velocity
    roller_radius: 0.0125
    d: 0.025
    theta: 30.0
    command_interfaces:
      - velocity
    state_interfaces:
      - velocity
      - position
```

Notice that I use namespace `(/rpm_controller)` for my convenience. Using namespace `(/)` will enable you to run multiple **Controller Manager** from the same machine for different robots. The _state interface_ names must match with your _custom controller_ code. So, please double check everything. This part is very important.

---

### 6. CMakeLists.txt:

Very important to install all your libraries/dependencies/plugins. I have added comments to easily understand the structure of this file. And separated things related to the custom hardware interface `(_hardware)` and the custom controller `(_controller)`.

```CMakeLists
# CMakeLists.txt for the 'my_custom_controller' package
# This file tells CMake how to build your custom ROS 2 controller and hardware interface.

# Specify the minimum required CMake version for compatibility
cmake_minimum_required(VERSION 3.8)

# Define the name of your project/package. This MUST match the name in package.xml.
# --- CORRECTED: Project name ---
project(my_custom_controller)

# --- COMPILER SETTINGS ---

# Enable stricter compiler warnings for GNU or Clang compilers to catch potential issues early
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Ensure the project uses C++17 standard, which is required by ROS 2 and modern C++ features
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON) # Require the standard to be available
set(CMAKE_CXX_EXTENSIONS OFF)       # Disable compiler-specific extensions for better portability

# --- FIND DEPENDENCIES ---

# Find the core ROS 2 build system package
find_package(ament_cmake REQUIRED)

# Find required ROS 2 packages for your project
# These correspond to the <depend> tags in your package.xml and provide necessary headers/libs

# Core ROS 2 C++ client library
find_package(rclcpp REQUIRED)

# Framework for creating hardware interfaces (used by your Arduino interface)
find_package(hardware_interface REQUIRED)

# Framework for creating controllers (used by your custom controller)
find_package(controller_interface REQUIRED)

# System for loading/unloading plugins (needed for both hardware and controller plugins)
find_package(pluginlib REQUIRED)

# ROS 2 lifecycle management (often used by nodes, hardware interfaces might use it indirectly)
find_package(rclcpp_lifecycle REQUIRED)

# Standard message types used for publishing/subscribing
find_package(std_msgs REQUIRED) # Needed for Float32MultiArray in Arduino interface

# Message types for geometry (Twist command)
find_package(geometry_msgs REQUIRED) # Needed for geometry_msgs::msg::Twist in controller

# Tools for real-time operations (often needed by controllers)
find_package(realtime_tools REQUIRED)


# --- INCLUDE DIRECTORIES ---

# Add the 'include' directory within your package to the compiler's search path
# This allows you to include your own header files like #include "my_custom_controller/my_custom_hardware_interface.hpp"
include_directories(include)


# --- DEFINE LIBRARIES ---

# 1. HARDWARE INTERFACE PLUGIN LIBRARY
# Create a shared library named 'my_custom_controller_hardware' from the specified source file.
# --- CORRECTED: Source file name ---
add_library(${PROJECT_NAME}_hardware SHARED
  src/my_custom_hardware_interface.cpp # <-- Corrected source file name
)

# Specify the dependencies for the 'my_custom_controller_hardware' library.
ament_target_dependencies(${PROJECT_NAME}_hardware
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  std_msgs # Required for std_msgs::msg::Float32MultiArray
  # Add other dependencies used directly in my_custom_hardware_interface.cpp if needed
)

# Tell pluginlib where to find the description of the hardware interface plugin.
pluginlib_export_plugin_description_file(
  hardware_interface
  my_custom_hardware_interface.xml # Make sure this file exists in your package root
)


# 2. CONTROLLER PLUGIN LIBRARY
#Create a shared library named 'my_custom_controller_controller' from the specified source file.
add_library(${PROJECT_NAME}_controller SHARED
  src/my_custom_controller.cpp # Add the source file for your custom controller
)

# Specify the dependencies for the 'my_custom_controller_controller' library.
ament_target_dependencies(${PROJECT_NAME}_controller
  rclcpp
  controller_interface
  pluginlib
  geometry_msgs # Required for geometry_msgs::msg::Twist
  realtime_tools
  # Add other dependencies used directly in my_custom_controller.cpp if needed
)

# Tell pluginlib where to find the description of the controller plugin.
pluginlib_export_plugin_description_file(
  controller_interface
  my_custom_controller.xml # Make sure this file exists in your package root
)


# --- INSTALLATION RULES ---

# Install the compiled shared libraries (.so files) to the 'lib' directory
install(
  TARGETS
    ${PROJECT_NAME}_hardware    # Install the hardware interface library
    ${PROJECT_NAME}_controller  # Install the controller library
  EXPORT export_${PROJECT_NAME} # Export the targets for use by other packages
  LIBRARY DESTINATION lib       # Destination directory for shared libraries
)

# Install public header files (*.hpp) from the 'include' directory
install(
  DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp" # Only install .hpp files
)

# Install additional data directories (URDF, RViz configs, parameters, launch files)
install(
  DIRECTORY
    urdf
    rviz
    config
    launch
  DESTINATION share/${PROJECT_NAME} # Destination path
)


# --- EXPORT CONFIGURATION ---

# Make the include directories findable by other packages using ament_cmake.
ament_export_include_directories(include)

# Export the compiled targets so other packages can link against them.
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# Export the dependencies so other packages automatically find them.
ament_export_dependencies(
  rclcpp
  hardware_interface
  controller_interface
  pluginlib
  rclcpp_lifecycle
  std_msgs
  geometry_msgs
  realtime_tools
)

# Finalize the package configuration for ament/colcon.
ament_package()
```
---

### 7. Launch File:

We will write a simple python launch file to launch our _custom controller_. See I have used the namespace `(rpm_controller)` matching the `yaml` file.

```python
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    # get urdf via xacro to show actuator movement
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [
                    FindPackageShare('my_custom_controller'),
                    'urdf',
                    'my_custom_robot.urdf.xacro',
                ]
            ),
        ]
    )
    robot_description = {'robot_description': robot_description_content}



    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    # configuration for rviz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('my_custom_controller'), 'rviz', 'my_custom_robot_rviz.rviz']
    )




    # configuration specifying type of controllers for joints and their configurable parameters
    datras_system = PathJoinSubstitution(
        [
            FindPackageShare('my_custom_controller'),
            'config',
            'my_custom_controller_config.yaml',
        ]
    )



    # node for controller manager which manages lifecycle of controllers defined for joints
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace="/my_custom_controller",
        parameters=[robot_description, datras_system],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
       
        remappings=[(
            "/my_custom_controller/my_custom_controller/commands",
            "/velocity_commands",
        )],
    )

    # node which publishes robot state including the robot description
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace="/my_custom_controller",
        output='both',
        parameters=[robot_description],
        
    )

    # node for launching rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
    )

    # node to broadcasting joint states of various joints listed in urdf and configuration
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace="my_custom_controller",
        #arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        arguments=["joint_state_broadcaster"],
    )
    
    # node to start controllers associated with the joints
    motor_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace="my_custom_controller",
        #arguments=['motor_controller', '--controller-manager', '/controller_manager'],
        arguments=["rpm_controller", "--param-file", datras_system],  # controller name -- rpm_controller
    )




    # delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # delay start of robot_controller after `joint_state_broadcaster`
    delay_motor_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[motor_controller_spawner],
        )
    )

    return LaunchDescription([
      
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        #delay_rviz_after_joint_state_broadcaster_spawner,
        delay_motor_controller_spawner_after_joint_state_broadcaster_spawner,
    ])
```
---

Now, it's time to compile our package.

`<colcon build>`

`<source ~/.bashrc>`

_"Don’t worry if it doesn’t compile the first 17 times—that’s just science happening."_ 

Now launch your package with the command:

`ros2 launch my_custom_controller my_custom_controller.launch.py `

Congrats! You have successfully launched your custom controller! 

![Image](https://github.com/user-attachments/assets/a7f367d0-c4c7-4bb8-bc25-1dbdb748bb6b)

Now, we have used namespace to remap our robot description in the launch file- `namespace="/my_custom_controller"`. So, to visualize it properly in the Rviz you'll have to change the description topic to `Description topic: /my_custom_controller/robot_description`.

https://github.com/user-attachments/assets/3c356428-9bb1-457e-80b5-f2e5ae4e082f

Use the following command list all your controllers:

`ros2 control list_controllers --controller-manager /my_custom_controller/controller_manager`

Use the following command to see all your hardware interfaces:

`ros2 control list_hardware_interfaces --controller-manager /my_custom_controller/controller_manager`

Use the following command to send rpm commands to the motors:

`ros2 topic pub -r 10 /my_custom_controller/rpm_controller/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"`

Of course later you can use a joystick/gamepad or any other method to send your rpm commands to the appropirate topic!

**_To Deactivate the controller:_**

`ros2 control set_controller_state rpm_controller inactive --controller-manager /my_custom_controller/controller_manager`

**_To Unload the controller:_**

`ros2 control unload_controller rpm_controller --controller-manager /my_custom_controller/controller_manager`

**_To Reload the controller:_**

`ros2 control load_controller rpm_controller --controller-manager /my_custom_controller/controller_manager`

**_To Reactivate the controller:_**

`ros2 control set_controller_state rpm_controller active --controller-manager /my_custom_controller/controller_manager`

---

## Tips:



1. To stop header files include errors in vscode open up the folder `.vscode` inside the `<vscode>` directory. The click `c_cpp_properties.json`. Under `"includePath":` add `"/home/code/ros2_ws/src/my_custom_controller/include/**"` and save the file.

   ```json
               "includePath": [
                "/opt/ros/jazzy/include/**",
                "/usr/include/**",
                "/opt/ros/jazzy/include/rclcpp",
                "/opt/ros/jazzy/include/rcl_interfaces",
                "/opt/ros/jazzy/include/joint_limits",
                "/opt/ros/jazzy/include/std_msgs",
                "/opt/ros/jazzy/include/tf2_geometry_msgs",
                "/home/code/ros2_ws/src/my_custom_controller/include/**"
            ],
   ```
2. Configure your vscode for C++17, and add these lines to your CMakeLists

   ```CMakeLists.txt
      # Ensure the project uses C++17 standard, which is required by ROS 2 and modern C++ features
   if(NOT CMAKE_CXX_STANDARD)
     set(CMAKE_CXX_STANDARD 17)
   endif()
   set(CMAKE_CXX_STANDARD_REQUIRED ON) # Require the standard to be available
   set(CMAKE_CXX_EXTENSIONS OFF)       # Disable compiler-specific extensions for better portability
   ```
3. Read how to write UART drivers for Linux

   - https://www.kernel.org/doc/Documentation/serial/driver
   - https://www.linux.it/~rubini/docs/serial/serial.html
   - https://www.marcusfolkesson.se/blog/writing-a-uart-driver-for-linux/

4. **Make sure your interface names in your custom controller script match with your YAML parameters. Otherwise the controller manager will fail to load your controller. VERY IMPORTANT.**
   
6. If you are having `colcon build` issues sometimes `rm -rf build install log` can help. 

---

<img width="1536" height="1024" alt="Image" src="https://github.com/user-attachments/assets/22424ede-7680-4bfc-9fd8-79fba6999bf7" />

**"When you do things right, people won’t be sure you’ve done anything at all." —Bender**

_**Written by**_

   **Masum**
   
   **Jul 27, 2025**
   







