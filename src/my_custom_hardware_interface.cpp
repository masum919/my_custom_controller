#include "my_custom_controller/my_custom_hardware_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include <string>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <cmath>
#include <std_msgs/msg/float32.hpp>

namespace my_arduino_controller
{

// Constructor
MyArduinoInterface::MyArduinoInterface() 
{
    // Constructor intentionally left empty - initialization happens in on_init()
}

// Destructor
MyArduinoInterface::~MyArduinoInterface()
{
    // Close serial port if it was opened
    if (SerialPort != -1)
    {
        close(SerialPort);
    }
}

// Lifecycle methods

CallbackReturn MyArduinoInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
    // Call parent class initialization
    CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
    if (result != CallbackReturn::SUCCESS)
    {
        return result;
    }

    // Initialize ROS2 node for publishing data
    node_ = std::make_shared<rclcpp::Node>("velocity_publisher_node"); // Standalone node for publishing
    // SystemInterface itself does not inherit from rclcpp::Node, so you must manually create a Node instance.
    current_publisher_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>("current_values", 10);

    try
    {
        // Setup serial communication with Arduino
        if (!setupSerialCommunication())
        {
            RCLCPP_WARN(rclcpp::get_logger("arduino_controller_interface"), 
                        "Controller will run in simulation mode.");
            // Continue execution even if Arduino is not connected
        }
    }
    catch(std::exception &e)
    {
        RCLCPP_WARN(
            rclcpp::get_logger("arduino_actuator_interface"),
            "Error during initialization: %s. Running in simulation mode.", e.what()
        );
        // Continue execution even if there's an error
    }

    // Initialize data storage vectors based on number of joints
    initializeDataVectors();

    return CallbackReturn::SUCCESS;
}


CallbackReturn MyArduinoInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("ArduinoInterface"), "Starting robot hardware ...");

    // Initialize all command and state vectors with proper sizes
    velocity_commands_.resize(info_.joints.size(), 0.0);
    prev_velocity_commands_.resize(info_.joints.size(), 0.0);
    velocity_states_.resize(info_.joints.size(), 0.0);
    position_states_.resize(info_.joints.size(), 0.0);

    RCLCPP_INFO(rclcpp::get_logger("ArduinoInterface"),
                "Hardware started, ready to take commands");
    return CallbackReturn::SUCCESS;
}


CallbackReturn MyArduinoInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    // Only attempt to close if serial port was successfully opened
    if(SerialPort == -1)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    }
    
    // Flush any remaining data and close the port
    tcflush(SerialPort, TCIFLUSH);
    close(SerialPort);
    return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> MyArduinoInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    // Export velocity state interfaces for all joints
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
    }

    // Export position state interfaces for all joints
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
    }

    return state_interfaces;
}


std::vector<hardware_interface::CommandInterface> MyArduinoInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    // Export velocity command interfaces for all joints
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
    }

    return command_interfaces;
}




hardware_interface::return_type MyArduinoInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) 
{
    try {
        
        float rpmValue1 = static_cast<float>(velocity_commands_.at(0));
        int dirValue1 = (rpmValue1 >= 0) ? 0 : 1;  // 0 = forward, 1 = reverse

        float rpmValue2 = static_cast<float>(velocity_commands_.at(1));
        int dirValue2 = (rpmValue2 >= 0) ? 0 : 1;  // 0 = forward, 1 = reverse

        // std::string data = std::to_string(rpmValue1) + " " + std::to_string(dirValue1) + " " +
        //                    std::to_string(rpmValue2) + " " + std::to_string(dirValue2) + "\n ";

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2)
            << rpmValue1 << " " << dirValue1 << " "
            << rpmValue2 << " " << dirValue2 << "\n";

        std::string data = oss.str();
                           
        // Send command to Arduino via serial port
        WriteToSerial(reinterpret_cast<const unsigned char*>(data.c_str()), data.length());
        RCLCPP_INFO(rclcpp::get_logger("arduino_actuator_interface"), "Writing %s", data.c_str());

        // Note: Sleep removed to let Arduino handle timing
        // std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    catch (const std::exception& e) {
        // Handle any exceptions that occur during the write process
        RCLCPP_FATAL(rclcpp::get_logger("arduino_actuator_interface"), "Error: %s", e.what());
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}


hardware_interface::return_type MyArduinoInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  

    try 
    {
        // Read and process Arduino sensor data
        if (readArduinoSensorData())
        {
            // Data successfully read and published
        }
        else
        {
            // Handle case where no data was available or read failed
            RCLCPP_WARN(rclcpp::get_logger("arduino_actuator_interface"), "No data available from Arduino.");
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("arduino_actuator_interface"), 
                     "Exception in read method: %s", e.what());
    }

    return hardware_interface::return_type::OK;
}

// Helpers for serial communication

bool MyArduinoInterface::setupSerialCommunication()
{
    std::string port = "/dev/ttyACM0"; // Define Arduino port

    // Check if port parameter is defined in hardware info
    auto port_param = info_.hardware_parameters.find("port");
    if (port_param != info_.hardware_parameters.end())
    {
        port = port_param->second;
        RCLCPP_INFO(rclcpp::get_logger("arduino_controller_interface"), 
                    "Using port from configuration: %s", port.c_str());
    }
    else
    {
        RCLCPP_WARN(rclcpp::get_logger("arduino_controller_interface"), 
                    "Port parameter not found in configuration, using default: %s", port.c_str());
    }
    
    
    // Attempt to open serial port
    SerialPort = open(port.c_str(), O_RDWR);
    if (SerialPort < 0)
    {
        RCLCPP_WARN(rclcpp::get_logger("arduino_controller_interface"), 
                    "Unable to open serial port %s. Error: %s", port.c_str(), strerror(errno));
        return false;
    }

    // Get current terminal settings
    if (tcgetattr(SerialPort, &tty) != 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("arduino_controller_interface"), 
                     "Error %i from tcgetattr: %s", errno, strerror(errno));
        close(SerialPort);
        return false;
    }

    // Configure serial port settings
    configureSerialPort();

    // Apply settings to serial port
    if (tcsetattr(SerialPort, TCSANOW, &tty) != 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("CustomHardware"), 
                     "Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return false;
    }

    RCLCPP_INFO(rclcpp::get_logger("CustomHardware"), 
                "SERIAL PORT OPENED: %d! WAITING...", SerialPort);

    // Wait for Arduino to initialize (3 second delay)
    waitForArduinoInitialization();

    return true;
}


void MyArduinoInterface::configureSerialPort()
{
    // Control flags configuration
    tty.c_cflag &= ~PARENB;         // Clear parity bit (no parity)
    tty.c_cflag &= ~CSTOPB;         // Clear stop field (1 stop bit)
    tty.c_cflag &= ~CSIZE;          // Clear data size bits
    tty.c_cflag |= CS8;             // 8 bits per byte
    tty.c_cflag &= ~CRTSCTS;        // Disable RTS/CTS hardware flow control
    tty.c_cflag |= CREAD | CLOCAL;  // Turn on READ & ignore ctrl lines

    // Local flags configuration
    tty.c_lflag &= ~ICANON;         // Disable canonical mode
    tty.c_lflag &= ~ECHO;           // Disable echo
    tty.c_lflag &= ~ECHOE;          // Disable erasure
    tty.c_lflag &= ~ECHONL;         // Disable new-line echo
    tty.c_lflag &= ~ISIG;           // Disable interpretation of INTR, QUIT and SUSP

    // Input flags configuration
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);                              // Turn off software flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);   // Disable special handling

    // Output flags configuration
    tty.c_oflag &= ~OPOST;          // Prevent special interpretation of output bytes
    tty.c_oflag &= ~ONLCR;          // Prevent conversion of newline to carriage return/line feed

    // Control characters configuration
    tty.c_cc[VTIME] = 1;            // Wait for up to 0.1s (1 decisecond)
    tty.c_cc[VMIN] = 0;             // Return as soon as any data is received

    // Set baud rate to 115200
    speed_t speed = B115200;
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    // Flush any existing data
    tcflush(SerialPort, TCIFLUSH);
}


void MyArduinoInterface::waitForArduinoInitialization()
{
    auto t_start = std::chrono::high_resolution_clock::now();
    while(true)
    {
        auto t_end = std::chrono::high_resolution_clock::now();
        double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
        if(elapsed_time_ms > 3000)  // Wait 3 seconds
        {
            break;
        }
    }
}


void MyArduinoInterface::initializeDataVectors()
{
    velocity_commands_.reserve(info_.joints.size());
    velocity_states_.reserve(info_.joints.size());
    prev_velocity_commands_.reserve(info_.joints.size());
    position_states_.reserve(info_.joints.size());
}


bool MyArduinoInterface::readArduinoSensorData()
{
    // Prepare buffer for reading Arduino data
    unsigned char buffer[128];
    std::memset(buffer, 0, sizeof(buffer));

    // Setup non-blocking read with timeout using select()
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(SerialPort, &read_fds);

    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 10000; // 10ms timeout

    // Check if data is available to read
    int ready = select(SerialPort + 1, &read_fds, NULL, NULL, &timeout);

    if (ready > 0)
    {
        // Data is available, read it
        ssize_t bytes_read = ::read(SerialPort, buffer, sizeof(buffer) - 1);

        if (bytes_read > 0)
        {
            return processArduinoData(buffer, bytes_read);
        }
        else
        {
            RCLCPP_WARN(rclcpp::get_logger("arduino_actuator_interface"), 
                        "No data received or read error.");
            return false;
        }
    }
    else
    {
        RCLCPP_WARN(rclcpp::get_logger("arduino_actuator_interface"), 
                    "Timeout waiting for data.");
        return false;
    }
}


bool MyArduinoInterface::processArduinoData(unsigned char* buffer, ssize_t /*bytes_read*/)
{
    // Convert buffer to string for parsing
    std::string data_str(reinterpret_cast<char*>(buffer));
    std::istringstream iss(data_str);
    
    float current1, current2;  // Current values for 3 motors
    
    // Parse the expected format: "current1 current2 current3"
    if (iss >> current1 >> current2)
    {
        // Create and publish current values message
        std_msgs::msg::Float32MultiArray current_msg;
        current_msg.data.push_back(current1);
        current_msg.data.push_back(current2);
        // current_msg.data.push_back(current3);

        current_publisher_->publish(current_msg);

        RCLCPP_INFO(rclcpp::get_logger("arduino_actuator_interface"), 
                    "Current Values - Motor1: %.2f mA, Motor2: %.2f mA", 
                    current1, current2);
        return true;
    }
    else
    {
        RCLCPP_WARN(rclcpp::get_logger("arduino_actuator_interface"), 
                    "Malformed data received from Arduino: %s", data_str.c_str());
        return false;
    }
}


int MyArduinoInterface::WriteToSerial(const unsigned char* buf, int nBytes)
{
    return ::write(SerialPort, const_cast<unsigned char*>(buf), nBytes);
}


int MyArduinoInterface::ReadSerial(unsigned char* buf, int nBytes)
{
    auto t_start = std::chrono::high_resolution_clock::now();
    int n = 0;
    
    // Read bytes one by one until we have enough or timeout
    while(n < nBytes)
    {
        int ret = ::read(SerialPort, &buf[n], 1);
        if(ret < 0)
        {
            return ret;  // Error occurred
        }

        n += ret;
        
        // Check for timeout (10 seconds)
        auto t_end = std::chrono::high_resolution_clock::now();
        double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end-t_start).count();
        if(elapsed_time_ms > 10000)
        {
            break;  // Timeout reached
        }
    }
    return n;
}

}  // namespace my_arduino_controller

// Export the plugin for ros2_control to use
PLUGINLIB_EXPORT_CLASS(my_arduino_controller::MyArduinoInterface, hardware_interface::SystemInterface)