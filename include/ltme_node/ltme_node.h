#ifndef LTME_NODE_H
#define LTME_NODE_H

#include "ltme_node/QueryModel.h"
#include "ltme_node/QuerySerial.h"
#include "ltme_node/QueryFirmwareVersion.h"
#include "ltme_node/QueryHardwareVersion.h"

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include "ldcp/device.h"

#include <mutex>
#include <atomic>

class LidarDriver
{
public:
  const static std::string DEFAULT_ENFORCED_TRANSPORT_MODE;
  const static std::string DEFAULT_FRAME_ID;
  const static bool DEFAULT_INVERT_FRAME;
  const static int DEFAULT_SCAN_FREQUENCY;
  const static double ANGLE_MIN_LIMIT;
  const static double ANGLE_MAX_LIMIT;
  const static double DEFAULT_ANGLE_EXCLUDED_MIN;
  const static double DEFAULT_ANGLE_EXCLUDED_MAX;
  const static double RANGE_MIN_LIMIT;
  const static double RANGE_MAX_LIMIT_02A;
  const static double RANGE_MAX_LIMIT_R1;
  const static double RANGE_MAX_LIMIT_R2;
  const static double RANGE_MAX_LIMIT_I1;
  const static double RANGE_MAX_LIMIT_I2;
  const static int DEFAULT_AVERAGE_FACTOR;
  const static int DEFAULT_SHADOW_FILTER_STRENGTH;
  const static int DEFAULT_RECEIVER_SENSITIVITY_BOOST;

public:
  LidarDriver();
  void run();

private:
  bool queryModelService(ltme_node::QueryModelRequest& request,
                         ltme_node::QueryModelResponse& response);
  bool querySerialService(ltme_node::QuerySerialRequest& request,
                          ltme_node::QuerySerialResponse& response);
  bool queryFirmwareVersion(ltme_node::QueryFirmwareVersionRequest& request,
                            ltme_node::QueryFirmwareVersionResponse& response);
  bool queryHardwareVersion(ltme_node::QueryHardwareVersionRequest& request,
                            ltme_node::QueryHardwareVersionResponse& response);
  bool requestHibernationService(std_srvs::EmptyRequest& request,
                                 std_srvs::EmptyResponse& response);
  bool requestWakeUpService(std_srvs::EmptyRequest& request,
                            std_srvs::EmptyResponse& response);
  bool quitDriverService(std_srvs::EmptyRequest& request,
                         std_srvs::EmptyResponse& response);

private:
  ros::NodeHandle nh_, nh_private_;

  std::string device_model_;
  std::string device_address_;
  std::string enforced_transport_mode_;
  std::string frame_id_;
  bool invert_frame_;
  int scan_frequency_override_;
  double angle_min_;
  double angle_max_;
  double angle_excluded_min_;
  double angle_excluded_max_;
  double range_min_;
  double range_max_;
  int average_factor_;
  int shadow_filter_strength_;
  int receiver_sensitivity_boost_;

  std::unique_ptr<ldcp_sdk::Device> device_;
  std::mutex mutex_;

  std::atomic_bool hibernation_requested_;
  std::atomic_bool quit_driver_;
};

#endif
