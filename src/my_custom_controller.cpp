#include "my_custom_controller/my_custom_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory> // For std::make_shared

#include "pluginlib/class_list_macros.hpp"
#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

// Include for Twist message
#include "geometry_msgs/msg/twist.hpp"
// Include for Float64 (if needed for other purposes, but not for command interface)
#include "std_msgs/msg/float64.hpp"

namespace my_custom_controller
{

MyCustomController::MyCustomController(): controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn MyCustomController::on_init()
{
  try {
    // Declare parameters with default values
    joint_names_ = auto_declare<std::vector<std::string>>("joints", {});
    interface_name_ = auto_declare<std::string>("interface_name", "velocity");

    // Kinematic parameters
    roller_radius_ = auto_declare<double>("roller_radius", 0.0125); 
    d_ = auto_declare<double>("d", 0.025); // [m] - Tool/end-effector diameter

    double theta_deg = auto_declare<double>("theta", 30.0); // [degrees]
    theta_ = theta_deg * M_PI / 180.0; // Convert degrees to radians

    RCLCPP_INFO(get_node()->get_logger(), "MyCustomController initialized with params: R=%.4fm, d=%.4fm, theta=%.2f deg (%.4f rad)",
                roller_radius_, d_, theta_deg, theta_);

  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MyCustomController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (joint_names_.size() != 2) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Expected exactly 2 joint names, but got %zu", joint_names_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  left_joint_name_ = joint_names_[0];
  right_joint_name_ = joint_names_[1];

  // Subscribe to Twist commands
  auto callback = [this](const geometry_msgs::msg::Twist::SharedPtr msg) -> void {
    // Store the latest Twist command
    twist_command_ = *msg;
    RCLCPP_DEBUG(get_node()->get_logger(), "Received Twist: linear.x=%.4f, angular.z=%.4f", twist_command_.linear.x, twist_command_.angular.z);
  };

  // Create the subscriber for Twist messages
  cmd_vel_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
    "~/cmd_vel", 10, callback); 

  RCLCPP_INFO(get_node()->get_logger(), "MyCustomController configured. Subscribed to ~/cmd_vel.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration MyCustomController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.reserve(joint_names_.size());

  for (const auto& joint_name : joint_names_) {
    config.names.push_back(joint_name + "/" + interface_name_);
  }
  return config;
}

controller_interface::InterfaceConfiguration MyCustomController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.reserve(joint_names_.size() * 2); // Velocity and Position

  for (const auto& joint_name : joint_names_) {
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION); // Claiming for fuuture use
  }
  return config;
}

controller_interface::CallbackReturn MyCustomController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Initialize twist command to zero
  twist_command_ = geometry_msgs::msg::Twist();

  RCLCPP_INFO(get_node()->get_logger(), "MyCustomController activated.");
  RCLCPP_INFO(get_node()->get_logger(), "Controlling joints: %s and %s",
              left_joint_name_.c_str(), right_joint_name_.c_str());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type MyCustomController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Extract linear and angular velocity commands from Twist message
  const double v_linear_mm_s  = twist_command_.linear.x;   // Vt (Translational velocity of the end-effector)
  const double v_angular = twist_command_.angular.z;  // Ω (Rotational velocity of the end-effector)

  // Convert linear velocity from mm/s to m/s
  const double v_linear = v_linear_mm_s / 1000.0;

  // Get kinematic parameters
  const double R = roller_radius_;       // Roller radius
  const double d = d_;                   // Tool/end-effector diameter
  const double theta = theta_;           // Tilt angle in radians

  // Check for potential division by zero
  if (std::abs(std::sin(theta)) < 1e-6 || std::abs(std::cos(theta)) < 1e-6 || std::abs(R) < 1e-6) {
      RCLCPP_ERROR(get_node()->get_logger(), "Invalid kinematic parameters leading to division by zero.");
      return controller_interface::return_type::ERROR;
  }

  // Calculate the inverse kinematics based on DATRAS Equation 3
  // Solve for ω1 and ω2 from [Vt; Ω] = A * [ω1; ω2]
  // A = [ -R/2 * cos(theta)   R/2 * cos(theta) ]
  //     [ -R/d * sin(theta)  -R/d * sin(theta) ]
  // det(A) = (-R/2 * cos(θ)) * (-R/d * sin(θ)) - (R/2 * cos(θ)) * (-R/d * sin(θ))
  //        = (R^2 / (2*d)) * cos(θ) * sin(θ) + (R^2 / (2*d)) * cos(θ) * sin(θ)
  //        = 2 * (R^2 / (2*d)) * cos(θ) * sin(θ) = (R^2 / d) * cos(θ) * sin(θ)
  // A_inv = (1/det(A)) * [ -R/d * sin(θ)   -R/2 * cos(θ) ]
  //                      [  R/d * sin(θ)   -R/2 * cos(θ) ]
  // [ω1]   = (d / (R^2 * cos(θ) * sin(θ))) * [ -R/d * sin(θ)   -R/2 * cos(θ) ] * [Vt]
  // [ω2]                                    [  R/d * sin(θ)   -R/2 * cos(θ) ]   [Ω]
  //
  // ω1 = (d / (R^2 * cos(θ) * sin(θ))) * [ (-R/d * sin(θ)) * Vt + (-R/2 * cos(θ)) * Ω ]
  //    = (1 / (R * cos(θ))) * [ -sin(θ)/d * Vt - cos(θ)/2 * Ω ]
  //    = - (sin(θ) / (R * d * cos(θ))) * Vt - (1 / (2 * R * cos(θ))) * Ω
  //
  // ω2 = (d / (R^2 * cos(θ) * sin(θ))) * [ (R/d * sin(θ)) * Vt + (-R/2 * cos(θ)) * Ω ]
  //    = (1 / (R * cos(θ))) * [ sin(θ)/d * Vt - cos(θ)/2 * Ω ]
  //    = (sin(θ) / (R * d * cos(θ))) * Vt - (1 / (2 * R * cos(θ))) * Ω
  //
  // Simplify coefficients:
  const double k_v_angular = 1.0 / (2.0 * R * std::cos(theta)); // Coefficient for Ω term
  const double k_cross = std::sin(theta) / (R * d * std::cos(theta)); // Cross-term coefficient

  // Calculate angular velocities (rad/s)
  const double omega1_rad = -k_cross * v_linear - k_v_angular * v_angular; // ω1 (rad/s)
  const double omega2_rad =  k_cross * v_linear - k_v_angular * v_angular; // ω2 (rad/s)

  // Convert angular velocities from rad/s to RPM
  const double rad_per_sec_to_rpm = 60.0 / (2.0 * M_PI);
  const double omega1_rpm = omega1_rad * rad_per_sec_to_rpm;
  const double omega2_rpm = omega2_rad * rad_per_sec_to_rpm;

  RCLCPP_DEBUG(get_node()->get_logger(), "Calculated RPMs: Left=%.2f, Right=%.2f", omega1_rpm, omega2_rpm);

  const double left_cmd = omega1_rpm;
  const double right_cmd = omega2_rpm;

  // Send commands to the hardware interface command interfaces
  // Assuming command_interfaces_[0] corresponds to left_joint_name_
  // and command_interfaces_[1] corresponds to right_joint_name_
  if (command_interfaces_.size() >= 2) {
    if (!command_interfaces_[0].set_value(left_cmd)) {
      RCLCPP_WARN(get_node()->get_logger(), "Failed to set command for left joint");
    }
    if (!command_interfaces_[1].set_value(right_cmd)) {
      RCLCPP_WARN(get_node()->get_logger(), "Failed to set command for right joint");
    }
    RCLCPP_DEBUG(get_node()->get_logger(), "Sent commands to hardware: %.2f RPM, %.2f RPM", left_cmd, right_cmd);
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Insufficient command interfaces available.");
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}


controller_interface::CallbackReturn MyCustomController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Set safe stop command: zero velocity to both joints
  if (command_interfaces_.size() >= 2) {
    if (!command_interfaces_[0].set_value(0.0)) {
      RCLCPP_WARN(get_node()->get_logger(), "Failed to set zero command for left joint during deactivation");
    }
    if (!command_interfaces_[1].set_value(0.0)) {
      RCLCPP_WARN(get_node()->get_logger(), "Failed to set zero command for right joint during deactivation");
    }
  }

  RCLCPP_INFO(get_node()->get_logger(), "MyCustomController deactivated. Motors set to zero.");
  return controller_interface::CallbackReturn::SUCCESS;
}

} // namespace my_custom_controller

PLUGINLIB_EXPORT_CLASS(my_custom_controller::MyCustomController, controller_interface::ControllerInterface)