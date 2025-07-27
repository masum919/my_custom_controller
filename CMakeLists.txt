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