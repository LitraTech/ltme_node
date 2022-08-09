#ifndef LTME_NODE_H
#define LTME_NODE_H

#include "ltme_interfaces/srv/query_serial.hpp"
#include "ltme_interfaces/srv/query_firmware_version.hpp"
#include "ltme_interfaces/srv/query_hardware_version.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "ldcp/device.h"

#include <mutex>
#include <atomic>

class LidarDriver : public rclcpp::Node {
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

public:
  LidarDriver(std::string name);
  void run();

private:
  void querySerialService(const ltme_interfaces::srv::QuerySerial::Request::SharedPtr request,
                          const ltme_interfaces::srv::QuerySerial::Response::SharedPtr response);
  void queryFirmwareVersion(const ltme_interfaces::srv::QueryFirmwareVersion::Request::SharedPtr request,
                            const ltme_interfaces::srv::QueryFirmwareVersion::Response::SharedPtr response);
  void queryHardwareVersion(const ltme_interfaces::srv::QueryHardwareVersion::Request::SharedPtr request,
                            const ltme_interfaces::srv::QueryHardwareVersion::Response::SharedPtr response);
  void requestHibernationService(const std_srvs::srv::Empty::Request::SharedPtr request,
                                 const std_srvs::srv::Empty::Response::SharedPtr response);
  void requestWakeUpService(const std_srvs::srv::Empty::Request::SharedPtr request,
                            const std_srvs::srv::Empty::Response::SharedPtr response);
  void quitDriverService(const std_srvs::srv::Empty::Request::SharedPtr request,
                         const std_srvs::srv::Empty::Response::SharedPtr response);

private:
  std::string device_model_;
  std::string device_address_;
  std::string enforced_transport_mode_;
  std::string frame_id_;
  int scan_frequency_override_;
  double angle_min_;
  double angle_max_;
  double angle_excluded_min_;
  double angle_excluded_max_;
  double range_min_;
  double range_max_;
  int average_factor_;

  std::unique_ptr<ldcp_sdk::Device> device_;
  std::mutex mutex_;

  std::atomic_bool hibernation_requested_{false};
  std::atomic_bool quit_driver_{false};

  // Publisher 
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_publisher_;

  // CallBackGroup
  rclcpp::CallbackGroup::SharedPtr callback_group_service_;

  // Server
  rclcpp::Service<ltme_interfaces::srv::QuerySerial>::SharedPtr query_serial_service_;
  rclcpp::Service<ltme_interfaces::srv::QueryFirmwareVersion>::SharedPtr query_firmware_service_;
  rclcpp::Service<ltme_interfaces::srv::QueryHardwareVersion>::SharedPtr query_hardware_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr request_hibernation_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr request_wake_up_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr quit_driver_service_;
};

#endif
