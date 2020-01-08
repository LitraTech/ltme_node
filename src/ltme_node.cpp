#include "ltme_node/ltme_node.h"

#include <arpa/inet.h>

const std::string LidarDriver::DEFAULT_FRAME_ID = "laser";
const double LidarDriver::ANGLE_MIN_LIMIT = -2.356;
const double LidarDriver::ANGLE_MAX_LIMIT = 2.356;
const double LidarDriver::DEFAULT_ANGLE_EXCLUDED_MIN = -3.142;
const double LidarDriver::DEFAULT_ANGLE_EXCLUDED_MAX = -3.142;
const double LidarDriver::RANGE_MIN_LIMIT = 0.2;
const double LidarDriver::RANGE_MAX_LIMIT = 30;

LidarDriver::LidarDriver()
  : nh_private_("~")
{
  if (!nh_private_.getParam("device_model", device_model_)) {
    ROS_ERROR("Missing required parameter \"device_model\"");
    exit(-1);
  }
  if (!nh_private_.getParam("device_address", device_address_)) {
    ROS_ERROR("Missing required parameter \"device_address\"");
    exit(-1);
  }
  nh_private_.param<std::string>("frame_id", frame_id_, DEFAULT_FRAME_ID);
  nh_private_.param<double>("angle_min", angle_min_, ANGLE_MIN_LIMIT);
  nh_private_.param<double>("angle_max", angle_max_, ANGLE_MAX_LIMIT);
  nh_private_.param<double>("angle_excluded_min", angle_excluded_min_, DEFAULT_ANGLE_EXCLUDED_MIN);
  nh_private_.param<double>("angle_excluded_max", angle_excluded_max_, DEFAULT_ANGLE_EXCLUDED_MAX);
  nh_private_.param<double>("range_min", range_min_, RANGE_MIN_LIMIT);
  nh_private_.param<double>("range_max", range_max_, RANGE_MAX_LIMIT);

  if (!(angle_min_ < angle_max_)) {
    ROS_ERROR("angle_min (%f) can't be larger than or equal to angle_max (%f)", angle_min_, angle_max_);
    exit(-1);
  }
  if (angle_min_ < ANGLE_MIN_LIMIT) {
    ROS_ERROR("angle_min is set to %f while its minimum allowed value is %f", angle_min_, ANGLE_MIN_LIMIT);
    exit(-1);
  }
  if (angle_max_ > ANGLE_MAX_LIMIT) {
    ROS_ERROR("angle_max is set to %f while its maximum allowed value is %f", angle_max_, ANGLE_MAX_LIMIT);
    exit(-1);
  }
  if (!(range_min_ < range_max_)) {
    ROS_ERROR("range_min (%f) can't be larger than or equal to range_max (%f)", range_min_, range_max_);
    exit(-1);
  }
  if (range_min_ < RANGE_MIN_LIMIT) {
    ROS_ERROR("range_min is set to %f while its minimum allowed value is %f", range_min_, RANGE_MIN_LIMIT);
    exit(-1);
  }
  if (range_max_ > RANGE_MAX_LIMIT) {
    ROS_ERROR("range_max is set to %f while its maximum allowed value is %f", range_max_, RANGE_MAX_LIMIT);
    exit(-1);
  }

  laser_scan_publisher_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 16);
}

void LidarDriver::run()
{
  while (nh_.ok()) {
    std::unique_ptr<ldcp_sdk::Device> device = waitForDevice();
    if (device) {
      ROS_INFO("Device opened");

      device->startStreaming();

      int beam_count = 2048;
      int beam_index_min = std::ceil(angle_min_ * beam_count / (2 * M_PI));
      int beam_index_max = std::floor(angle_max_ * beam_count / (2 * M_PI));
      int beam_index_excluded_min = std::ceil(angle_excluded_min_ * beam_count / (2 * M_PI));
      int beam_index_excluded_max = std::floor(angle_excluded_max_ * beam_count / (2 * M_PI));

      laser_scan_.header.frame_id = frame_id_;
      laser_scan_.angle_min = angle_min_;
      laser_scan_.angle_max = angle_max_;
      laser_scan_.angle_increment = 2 * M_PI / beam_count;
      laser_scan_.time_increment = 1.0 / 15 / beam_count;
      laser_scan_.scan_time = 1.0 / 15;
      laser_scan_.range_min = range_min_;
      laser_scan_.range_max = range_max_;
      laser_scan_.ranges.resize(beam_index_max - beam_index_min + 1);

      auto readScanBlock = [&device](ldcp_sdk::ScanBlock& scan_block) {
        if (device->readScanBlock(scan_block) != ldcp_sdk::no_error)
          throw std::exception();
      };

      auto updateLaserScan = [&](const ldcp_sdk::ScanBlock& scan_block) {
        int block_size = scan_block.layers[0].ranges.size();
        for (int i = 0; i < block_size; i++) {
          int beam_index = block_size * (scan_block.block_id - 4) + i;
          if (beam_index < beam_index_min || beam_index > beam_index_max)
            continue;
          if (beam_index >= beam_index_excluded_min && beam_index <= beam_index_excluded_max)
            continue;
          laser_scan_.ranges[beam_index - beam_index_min] = scan_block.layers[0].ranges[i] * 0.002;
        }
      };

      while (nh_.ok()) {
        std::fill(laser_scan_.ranges.begin(), laser_scan_.ranges.end(), 0.0);

        ldcp_sdk::ScanBlock scan_block;
        try {
          do {
            readScanBlock(scan_block);
          } while (scan_block.block_id != 0);

          laser_scan_.header.stamp = ros::Time::now();

          while (scan_block.block_id != 8 - 1) {
            updateLaserScan(scan_block);
            readScanBlock(scan_block);
          }
          updateLaserScan(scan_block);

          laser_scan_publisher_.publish(laser_scan_);
        }
        catch (const std::exception&) {
          ROS_WARN("Error reading data from device");
          break;
        }
      }

      device->stopStreaming();
      device->close();

      ROS_INFO("Device closed");
    }
    else
      break;
  }
}

std::unique_ptr<ldcp_sdk::Device> LidarDriver::waitForDevice()
{
  std::string address_str = device_address_;
  std::string port_str = "2105";

  size_t position = device_address_.find(':');
  if (position != std::string::npos) {
    address_str = device_address_.substr(0, position);
    port_str = device_address_.substr(position + 1);
  }

  in_addr_t address = INADDR_NONE;
  in_port_t port = 0;
  try {
    address = inet_addr(address_str.c_str());
    if (address == INADDR_NONE)
      throw std::exception();
    port = htons(std::stoi(port_str));
  }
  catch (...) {
    ROS_ERROR("Invalid device address: %s", device_address_.c_str());
    exit(-1);
  }

  ldcp_sdk::NetworkLocation location(address, port);
  std::unique_ptr<ldcp_sdk::Device> device(new ldcp_sdk::Device(location));

  ros::Rate loop_rate(0.3);
  while (nh_.ok()) {
    if (device->open() == ldcp_sdk::no_error)
      return device;
    else {
      ROS_INFO_THROTTLE(5, "Waiting for device... [%s]", device_address_.c_str());
      loop_rate.sleep();
    }
  }

  return nullptr;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "ltme_node");
  ROS_INFO("ltme_node started");

  LidarDriver lidarDriver;
  lidarDriver.run();

  return 0;
}
