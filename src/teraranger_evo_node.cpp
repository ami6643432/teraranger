#include "teraranger/teraranger_evo_node.hpp"
#include <thread>

namespace teraranger_evo
{
  // Add the CRC table initialization here
  const uint8_t TerarangerEvoNode::crc_table[256] = {
      0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31, 0x24, 0x23,
      0x2a, 0x2d, 0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65, 0x48, 0x4f, 0x46, 0x41,
      0x54, 0x53, 0x5a, 0x5d, 0xe0, 0xe7, 0xee, 0xe9, 0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf,
      0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd, 0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85,
      0xa8, 0xaf, 0xa6, 0xa1, 0xb4, 0xb3, 0xba, 0xbd, 0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc,
      0xd5, 0xd2, 0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea, 0xb7, 0xb0, 0xb9, 0xbe,
      0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a, 0x27, 0x20,
      0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0d, 0x0a,
      0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42, 0x6f, 0x68, 0x61, 0x66, 0x73, 0x74,
      0x7d, 0x7a, 0x89, 0x8e, 0x87, 0x80, 0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8,
      0xad, 0xaa, 0xa3, 0xa4, 0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6,
      0xcf, 0xc8, 0xdd, 0xda, 0xd3, 0xd4, 0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c,
      0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44, 0x19, 0x1e, 0x17, 0x10, 0x05, 0x02,
      0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34, 0x4e, 0x49, 0x40, 0x47,
      0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f, 0x6a, 0x6d, 0x64, 0x63, 0x3e, 0x39,
      0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b, 0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13,
      0xae, 0xa9, 0xa0, 0xa7, 0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d,
      0x84, 0x83, 0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef,
      0xfa, 0xfd, 0xf4, 0xf3
  };

TerarangerEvoNode::TerarangerEvoNode()
: Node("teraranger_evo")
{
  // Declare parameters
  this->declare_parameter("portname", "/dev/ttyACM0");
  this->declare_parameter("frame_id", "teraranger_evo_frame");

  // Get parameters
  portname_ = this->get_parameter("portname").as_string();
  frame_id_ = this->get_parameter("frame_id").as_string();

  // Create publisher
  range_publisher_ = this->create_publisher<sensor_msgs::msg::Range>(
    "teraranger_evo/range", 10);

  // Initialize range message
  initializeRangeMessage();

  // Initialize serial port and device
  try {
    initializeSerial();
    initializeDevice();
  } catch (const serial::IOException& e) {
    RCLCPP_ERROR(this->get_logger(), "Unable to open serial port %s. Error: %s", 
      portname_.c_str(), e.what());
    return;
  }

  // Create timer for reading serial data
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(10),
    std::bind(&TerarangerEvoNode::timerCallback, this));

  // Setup parameter callback
  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&TerarangerEvoNode::paramCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "TeraRanger Evo initialized on port %s", portname_.c_str());
}

TerarangerEvoNode::~TerarangerEvoNode()
{
  if (serial_port_ && serial_port_->isOpen()) {
    serial_port_->close();
  }
}

void TerarangerEvoNode::initializeDevice()
{
  // Enable sensor
  sendCommand(ENABLE_CMD);
  
  // Short delay to ensure command is processed
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  // Set binary mode
  sendCommand(BINARY_MODE);
}

uint8_t TerarangerEvoNode::crc8(uint8_t *p, uint8_t len)
{
  uint16_t i;
  uint16_t crc = 0x0;
  
  while (len--) {
    i = (crc ^ *p++) & 0xFF;
    crc = (crc_table[i] ^ (crc << 8)) & 0xFF;
  }
  return crc & 0xFF;
}

void TerarangerEvoNode::initializeSerial()
{
  serial_port_ = std::make_unique<serial::Serial>();
  serial_port_->setPort(portname_);
  serial_port_->setBaudrate(SERIAL_SPEED);
  serial_port_->setParity(serial::parity_none);
  serial_port_->setStopbits(serial::stopbits_one);
  serial_port_->setBytesize(serial::eightbits);
  serial::Timeout timeout = serial::Timeout::simpleTimeout(SERIAL_TIMEOUT_MS);
  serial_port_->setTimeout(timeout);

  serial_port_->open();
}

void TerarangerEvoNode::initializeRangeMessage()
{
  range_msg_.radiation_type = sensor_msgs::msg::Range::INFRARED;
  range_msg_.field_of_view = FIELD_OF_VIEW;
  range_msg_.min_range = MIN_RANGE;
  range_msg_.max_range = MAX_RANGE;
  range_msg_.header.frame_id = frame_id_;
}

void TerarangerEvoNode::sendCommand(const std::vector<uint8_t>& cmd)
{
  if (serial_port_ && serial_port_->isOpen()) {
    serial_port_->write(cmd);
    serial_port_->flushOutput();
  }
}

float TerarangerEvoNode::processRange(uint16_t range_raw)
{
  float range_meters = range_raw * VALUE_TO_METER_FACTOR;

  if (range_raw == 0) {  // Too close
    return -std::numeric_limits<float>::infinity();
  } else if (range_raw == 0xFFFF) {  // Out of range
    return std::numeric_limits<float>::infinity();
  } else if (range_meters > MAX_RANGE) {
    return std::numeric_limits<float>::infinity();
  } else if (range_meters < MIN_RANGE) {
    return -std::numeric_limits<float>::infinity();
  }

  return range_meters;
}

void TerarangerEvoNode::processSerialData()
{
  if (!serial_port_ || !serial_port_->isOpen()) {
    return;
  }

  try {
    uint8_t buffer[BUFFER_SIZE];
    size_t bytes_read = serial_port_->read(buffer, BUFFER_SIZE);

    if (bytes_read != BUFFER_SIZE) {
      return;
    }

    // Check if we have a valid frame starting with 'T'
    if (buffer[0] != 'T') {
      return;
    }

    // Check CRC
    uint8_t crc = crc8(buffer, BUFFER_SIZE-1);
    if(crc != buffer[BUFFER_SIZE-1]) {
      RCLCPP_DEBUG(this->get_logger(), "CRC mismatch");
      return;
    }

    // Extract range data
    uint16_t range_raw = (buffer[1] << 8) + buffer[2];
    
    // Update message
    range_msg_.header.stamp = this->now();
    range_msg_.range = processRange(range_raw);
    
    // Publish
    range_publisher_->publish(range_msg_);

  } catch (const serial::IOException& e) {
    RCLCPP_ERROR(this->get_logger(), "Serial error: %s", e.what());
    // Try to reinitialize
    try {
      initializeSerial();
      initializeDevice();
    } catch (const serial::IOException& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to reinitialize: %s", e.what());
    }
  }
}

void TerarangerEvoNode::timerCallback()
{
  processSerialData();
}

rcl_interfaces::msg::SetParametersResult TerarangerEvoNode::paramCallback(
  const std::vector<rclcpp::Parameter>& parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto& param : parameters) {
    if (param.get_name() == "frame_id") {
      frame_id_ = param.as_string();
      range_msg_.header.frame_id = frame_id_;
      RCLCPP_INFO(this->get_logger(), "Updated frame_id to: %s", frame_id_.c_str());
    }
  }

  return result;
}

}  // namespace teraranger_evo

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<teraranger_evo::TerarangerEvoNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}