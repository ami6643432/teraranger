#ifndef TERARANGER_EVO_NODE_HPP_
#define TERARANGER_EVO_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <serial/serial.h>
#include <memory>
#include <string>

namespace teraranger_evo
{

class TerarangerEvoNode : public rclcpp::Node
{
public:
  TerarangerEvoNode();
  virtual ~TerarangerEvoNode();

private:
  // Constants
  static constexpr float MAX_RANGE = 60.0;  // meters
  static constexpr float MIN_RANGE = 0.5;   // meters
  static constexpr float FIELD_OF_VIEW = 0.0349066f;  // radians
  static constexpr int BUFFER_SIZE = 4;
  static constexpr float VALUE_TO_METER_FACTOR = 0.001f;
  static constexpr int SERIAL_SPEED = 115200;
  static constexpr int SERIAL_TIMEOUT_MS = 1000;

  // Binary mode command
  const std::vector<uint8_t> BINARY_MODE = {0x00, 0x11, 0x02, 0x4C};

  // Parameters
  std::string portname_;
  std::string frame_id_;
  
  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_publisher_;
  
  // Serial port
  std::unique_ptr<serial::Serial> serial_port_;
  
  // Timer for reading serial data
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Methods
  void initializeSerial();
  void timerCallback();
  void processSerialData();
  float processRange(uint16_t range_raw);
  void initializeRangeMessage();
  void sendCommand(const std::vector<uint8_t>& cmd);
  
  // Message
  sensor_msgs::msg::Range range_msg_;
  
  // Parameter callback
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  rcl_interfaces::msg::SetParametersResult paramCallback(
    const std::vector<rclcpp::Parameter>& parameters);

  // Add after the existing constants
  static const uint8_t crc_table[256];

  // Add these commands
  const std::vector<uint8_t> ENABLE_CMD = {0x00, 0x52, 0x02, 0x01, 0xDF};

  // Add these method declarations in the private section
  void initializeDevice();
  static uint8_t crc8(uint8_t *p, uint8_t len);
};

}  // namespace teraranger_evo

#endif  // TERARANGER_EVO_NODE_HPP_