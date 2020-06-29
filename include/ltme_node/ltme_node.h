#ifndef LTME_NODE_H
#define LTME_NODE_H

#include "ldcp/device.h"

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class LidarDriver
{
public:
  const static std::string DEFAULT_FRAME_ID;
  const static double ANGLE_MIN_LIMIT;
  const static double ANGLE_MAX_LIMIT;
  const static double DEFAULT_ANGLE_EXCLUDED_MIN;
  const static double DEFAULT_ANGLE_EXCLUDED_MAX;
  const static double RANGE_MIN_LIMIT;
  const static double RANGE_MAX_LIMIT;
  const static int AVERAGE_FACTOR;

public:
  LidarDriver();
  void run();

private:
  std::unique_ptr<ldcp_sdk::Device> waitForDevice();

private:
  ros::NodeHandle nh_, nh_private_;
  ros::Publisher laser_scan_publisher_;
  sensor_msgs::LaserScan laser_scan_;

  std::string device_model_;
  std::string device_address_;
  std::string frame_id_;
  double angle_min_;
  double angle_max_;
  double angle_excluded_min_;
  double angle_excluded_max_;
  double range_min_;
  double range_max_;
  int average_factor_;
};

#endif
