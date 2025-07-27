#ifndef MY_CUSTOM_HARDWARE_INTERFACE_H
#define MY_CUSTOM_HARDWARE_INTERFACE_H

#include <rclcpp/rclcpp.hpp>
#include "rclcpp/macros.hpp"
#include <hardware_interface/system_interface.hpp>

#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include <vector>
#include <string>
#include <memory>
#include <termios.h>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>  
#include <sstream>
#include <sys/select.h>



namespace my_arduino_controller
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class MyArduinoInterface : public hardware_interface::SystemInterface
{
public:
  MyArduinoInterface();
  virtual ~MyArduinoInterface();

  // Implementing rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface
  virtual CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
  virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  // Implementing hardware_interface::SystemInterface
  virtual CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;
  virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  virtual hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  virtual hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

  std::vector<double> velocity_commands_;
  std::vector<double> prev_velocity_commands_;
  std::vector<double> velocity_states_;
  std::vector<double> position_states_;

  int SerialPort = -1;
  struct termios tty;
  int WriteToSerial(const unsigned char* buf, int nBytes);
  int ReadSerial(unsigned char* buf, int nBytes);


  // Serial communication setup helpers
  bool setupSerialCommunication();
  void configureSerialPort();
  void waitForArduinoInitialization();
  void initializeDataVectors();
    
  // Data processing helpers
  bool readArduinoSensorData();
  bool processArduinoData(unsigned char* buffer, ssize_t bytes_read);




rclcpp::Node::SharedPtr node_;  
rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr current_publisher_;









  


};
}  // namespace arduino_controller


#endif  // MY_CUSTOM_HARDWARE_INTERFACE_H