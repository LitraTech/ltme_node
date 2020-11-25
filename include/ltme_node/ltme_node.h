#ifndef LTME_NODE_H
#define LTME_NODE_H

#include "ltme_node/QuerySerial.h"
#include "ltme_node/QueryFirmwareVersion.h"
#include "ltme_node/QueryHardwareVersion.h"
#include "ltme_node/GetBackgroundIntensityThreshold.h"
#include "ltme_node/SetBackgroundIntensityThreshold.h"

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
  const static int DEFAULT_SCAN_FREQUENCY;
  const static double ANGLE_MIN_LIMIT;
  const static double ANGLE_MAX_LIMIT;
  const static double DEFAULT_ANGLE_EXCLUDED_MIN;
  const static double DEFAULT_ANGLE_EXCLUDED_MAX;
  const static double RANGE_MIN_LIMIT;
  const static double RANGE_MAX_LIMIT;
  const static int DEFAULT_AVERAGE_FACTOR;
  const static int DEFAULT_BACKGROUND_INTENSITY_THRESHOLD;

public:
  LidarDriver();
  void run();

private:
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
  bool getBackgroundIntensityThresholdService(ltme_node::GetBackgroundIntensityThresholdRequest& request,
                                              ltme_node::GetBackgroundIntensityThresholdResponse& response);
  bool setBackgroundIntensityThresholdService(ltme_node::SetBackgroundIntensityThresholdRequest& request,
                                              ltme_node::SetBackgroundIntensityThresholdResponse& response);
  bool quitDriverService(std_srvs::EmptyRequest& request,
                         std_srvs::EmptyResponse& response);

private:
  ros::NodeHandle nh_, nh_private_;

  std::string device_model_;
  std::string device_address_;
  std::string enforced_transport_mode_;
  std::string frame_id_;
  double angle_min_;
  double angle_max_;
  double angle_excluded_min_;
  double angle_excluded_max_;
  double range_min_;
  double range_max_;
  int average_factor_;
  int background_intensity_threshold_;

  std::unique_ptr<ldcp_sdk::Device> device_;
  std::mutex mutex_;

  std::atomic_bool hibernation_requested_;
  std::atomic_bool quit_driver_;
};

#endif
