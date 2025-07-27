// my_custom_controller.hpp
#ifndef MY_CUSTOM_CONTROLLER_HPP
#define MY_CUSTOM_CONTROLLER_HPP

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include <string>
#include <vector>

// Include necessary message types
#include "geometry_msgs/msg/twist.hpp" // For Twist command input
#include "std_msgs/msg/float64_multi_array.hpp" // If needed elsewhere, though not for basic command/state interfaces

namespace my_custom_controller
{

class MyCustomController : public controller_interface::ControllerInterface
{
public:
  MyCustomController();

  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  // Member variables for parameters and joint names
  std::vector<std::string> joint_names_;
  std::string interface_name_;
  std::string left_joint_name_;
  std::string right_joint_name_;

  // --- Kinematic Parameters ---
  double roller_radius_;
  
  double d_;     //Tool/end-effector diameter        

  double theta_;  // Tilt angle in RADIANS (converted from degrees in on_init)    

  // --- Subscriber and Command Storage ---
  // Subscriber for Twist commands
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
  // Storage for the latest received Twist command
  geometry_msgs::msg::Twist twist_command_;
};

} // namespace my_custom_controller

#endif // MY_CUSTOM_CONTROLLER_HPP