#include "ltme_node/ltme_node.h"

#include <arpa/inet.h>

#include <sensor_msgs/LaserScan.h>

const std::string LidarDriver::DEFAULT_ENFORCED_TRANSPORT_MODE = "none";
const std::string LidarDriver::DEFAULT_FRAME_ID = "laser";
const bool LidarDriver::DEFAULT_INVERT_FRAME = false;
const int LidarDriver::DEFAULT_SCAN_FREQUENCY = 15;
const double LidarDriver::ANGLE_MIN_LIMIT = -3.142;
const double LidarDriver::ANGLE_MAX_LIMIT = 3.142;
const double LidarDriver::DEFAULT_ANGLE_EXCLUDED_MIN = -3.142;
const double LidarDriver::DEFAULT_ANGLE_EXCLUDED_MAX = -3.142;
const double LidarDriver::RANGE_MIN_LIMIT = 0.05;
const double LidarDriver::RANGE_MAX_LIMIT_02A = 30;
const double LidarDriver::RANGE_MAX_LIMIT_R1 = 30;
const double LidarDriver::RANGE_MAX_LIMIT_R2 = 30;
const double LidarDriver::RANGE_MAX_LIMIT_I1 = 100;
const double LidarDriver::RANGE_MAX_LIMIT_I2 = 70;
const int LidarDriver::DEFAULT_AVERAGE_FACTOR = 1;
const int LidarDriver::DEFAULT_SHADOW_FILTER_STRENGTH = 50;
const int LidarDriver::DEFAULT_RECEIVER_SENSITIVITY_BOOST = 0;

LidarDriver::LidarDriver()
  : nh_private_("~")
  , hibernation_requested_(false)
  , quit_driver_(false)
{
  if (!nh_private_.getParam("device_model", device_model_)) {
    ROS_ERROR("Missing required parameter \"device_model\"");
    exit(-1);
  }
  else if (device_model_ != "LTME-02A" &&
      device_model_ != "LT-R1" && device_model_ != "LT-R2" &&
      device_model_ != "LT-I1" && device_model_ != "LT-I2") {
    ROS_ERROR("Unsupported device model %s", device_model_.c_str());
    exit(-1);
  }
  if (!nh_private_.getParam("device_address", device_address_)) {
    ROS_ERROR("Missing required parameter \"device_address\"");
    exit(-1);
  }
  nh_private_.param<std::string>("enforced_transport_mode", enforced_transport_mode_, DEFAULT_ENFORCED_TRANSPORT_MODE);
  nh_private_.param<std::string>("frame_id", frame_id_, DEFAULT_FRAME_ID);
  nh_private_.param<bool>("invert_frame", invert_frame_, DEFAULT_INVERT_FRAME);
  nh_private_.param<int>("scan_frequency_override", scan_frequency_override_, 0);
  nh_private_.param<double>("angle_min", angle_min_, ANGLE_MIN_LIMIT);
  nh_private_.param<double>("angle_max", angle_max_, ANGLE_MAX_LIMIT);
  nh_private_.param<double>("angle_excluded_min", angle_excluded_min_, DEFAULT_ANGLE_EXCLUDED_MIN);
  nh_private_.param<double>("angle_excluded_max", angle_excluded_max_, DEFAULT_ANGLE_EXCLUDED_MAX);
  nh_private_.param<double>("range_min", range_min_, RANGE_MIN_LIMIT);
  if (device_model_ == "LTME-02A")
    nh_private_.param<double>("range_max", range_max_, RANGE_MAX_LIMIT_02A);
  else if (device_model_ == "LT-R1")
    nh_private_.param<double>("range_max", range_max_, RANGE_MAX_LIMIT_R1);
  else if (device_model_ == "LT-R2")
    nh_private_.param<double>("range_max", range_max_, RANGE_MAX_LIMIT_R2);
  else if (device_model_ == "LT-I1")
    nh_private_.param<double>("range_max", range_max_, RANGE_MAX_LIMIT_I1);
  else if (device_model_ == "LT-I2")
    nh_private_.param<double>("range_max", range_max_, RANGE_MAX_LIMIT_I2);
  nh_private_.param<int>("average_factor", average_factor_, DEFAULT_AVERAGE_FACTOR);
  nh_private_.param<int>("shadow_filter_strength", shadow_filter_strength_, DEFAULT_SHADOW_FILTER_STRENGTH);
  nh_private_.param<int>("receiver_sensitivity_boost", receiver_sensitivity_boost_, DEFAULT_RECEIVER_SENSITIVITY_BOOST);

  if (!(enforced_transport_mode_ == "none" || enforced_transport_mode_ == "normal" || enforced_transport_mode_ == "oob")) {
    ROS_ERROR("Transport mode \"%s\" not supported", enforced_transport_mode_.c_str());
    exit(-1);
  }
  if (scan_frequency_override_ != 0 &&
    (scan_frequency_override_ < 10 || scan_frequency_override_ > 30 || scan_frequency_override_ % 5 != 0)) {
    ROS_ERROR("Scan frequency %d not supported", scan_frequency_override_);
    exit(-1);
  }
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
  if (average_factor_ <= 0 || average_factor_ > 8) {
    ROS_ERROR("average_factor is set to %d while its valid value is between 1 and 8", average_factor_);
    exit(-1);
  }
  if (shadow_filter_strength_ < 0 || average_factor_ > 100) {
    ROS_ERROR("shadow_filter_strength is set to %d while its valid value is between 0 and 100 (inclusive)", shadow_filter_strength_);
    exit(-1);
  }
  if (receiver_sensitivity_boost_ < -20 || receiver_sensitivity_boost_ > 10) {
    ROS_ERROR("receiver_sensitivity_boost is set to %d while the valid range is between -20 and 10 (inclusive)", receiver_sensitivity_boost_);
    exit(-1);
  }
}

void LidarDriver::run()
{
  std::unique_lock<std::mutex> lock(mutex_);

  ros::Publisher laser_scan_publisher = nh_.advertise<sensor_msgs::LaserScan>("scan", 16);
  ros::ServiceServer query_model_service = nh_private_.advertiseService<ltme_node::QueryModelRequest, ltme_node::QueryModelResponse>
    ("query_model", std::bind(&LidarDriver::queryModelService, this, std::placeholders::_1, std::placeholders::_2));
  ros::ServiceServer query_serial_service = nh_private_.advertiseService<ltme_node::QuerySerialRequest, ltme_node::QuerySerialResponse>
    ("query_serial", std::bind(&LidarDriver::querySerialService, this, std::placeholders::_1, std::placeholders::_2));
  ros::ServiceServer query_firmware_service = nh_private_.advertiseService<ltme_node::QueryFirmwareVersionRequest, ltme_node::QueryFirmwareVersionResponse>
    ("query_firmware_version", std::bind(&LidarDriver::queryFirmwareVersion, this, std::placeholders::_1, std::placeholders::_2));
  ros::ServiceServer query_hardware_service = nh_private_.advertiseService<ltme_node::QueryHardwareVersionRequest, ltme_node::QueryHardwareVersionResponse>
    ("query_hardware_version", std::bind(&LidarDriver::queryHardwareVersion, this, std::placeholders::_1, std::placeholders::_2));
  ros::ServiceServer request_hibernation_service = nh_private_.advertiseService<std_srvs::EmptyRequest, std_srvs::EmptyResponse>
    ("request_hibernation", std::bind(&LidarDriver::requestHibernationService, this, std::placeholders::_1, std::placeholders::_2));
  ros::ServiceServer request_wake_up_service = nh_private_.advertiseService<std_srvs::EmptyRequest, std_srvs::EmptyResponse>
    ("request_wake_up", std::bind(&LidarDriver::requestWakeUpService, this, std::placeholders::_1, std::placeholders::_2));
  ros::ServiceServer quit_driver_service = nh_private_.advertiseService<std_srvs::EmptyRequest, std_srvs::EmptyResponse>
    ("quit_driver", std::bind(&LidarDriver::quitDriverService, this, std::placeholders::_1, std::placeholders::_2));

  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::string address_str = device_address_;
  std::string port_str = "2105";

  size_t position = device_address_.find(':');
  if (position != std::string::npos) {
    address_str = device_address_.substr(0, position);
    port_str = device_address_.substr(position + 1);
  }

  in_addr_t address = htonl(INADDR_NONE);
  in_port_t port = 0;
  try {
    address = inet_addr(address_str.c_str());
    if (address == htonl(INADDR_NONE))
      throw std::exception();
    port = htons(std::stoi(port_str));
  }
  catch (...) {
    ROS_ERROR("Invalid device address: %s", device_address_.c_str());
    exit(-1);
  }

  ldcp_sdk::NetworkLocation location(address, port);
  device_ = std::unique_ptr<ldcp_sdk::Device>(new ldcp_sdk::Device(location));

  ros::Rate loop_rate(0.3);
  while (nh_.ok() && !quit_driver_.load()) {
    if (device_->open() == ldcp_sdk::no_error) {
      hibernation_requested_ = false;

      lock.unlock();

      ROS_INFO("Device opened");

      bool reboot_required = false;
      if (device_model_ == "LTME-02A" && enforced_transport_mode_ != "none") {
        std::string firmware_version;
        if (device_->queryFirmwareVersion(firmware_version) == ldcp_sdk::no_error) {
          if (firmware_version < "0201")
            ROS_WARN("Firmware version %s supports normal transport mode only, "
              "\"enforced_transport_mode\" parameter will be ignored", firmware_version.c_str());
          else {
            bool oob_enabled = false;
            if (device_->isOobEnabled(oob_enabled) == ldcp_sdk::no_error) {
              if ((enforced_transport_mode_ == "normal" && oob_enabled) ||
                  (enforced_transport_mode_ == "oob" && !oob_enabled)) {
                ROS_INFO("Transport mode will be switched to \"%s\"", oob_enabled ? "normal" : "oob");
                device_->setOobEnabled(!oob_enabled);
                device_->persistSettings();
                reboot_required = true;
              }
            }
            else
              ROS_WARN("Unable to query device for its current transport mode, "
                "\"enforced_transport_mode\" parameter will be ignored");
          }
        }
        else
          ROS_WARN("Unable to query device for firmware version, \"enforced_transport_mode\" parameter will be ignored");
      }

      if (!reboot_required) {
        int scan_frequency = DEFAULT_SCAN_FREQUENCY;
        if (scan_frequency_override_ != 0)
          scan_frequency = scan_frequency_override_;
        else {
          if (device_->getScanFrequency(scan_frequency) != ldcp_sdk::no_error)
            ROS_WARN("Unable to query device for scan frequency and will use %d as the frequency value", scan_frequency);
        }

        if (shadow_filter_strength_ != DEFAULT_SHADOW_FILTER_STRENGTH) {
          if (device_->setShadowFilterStrength(shadow_filter_strength_) == ldcp_sdk::no_error)
            ROS_INFO("Shadow filter strength set to %d", shadow_filter_strength_);
          else
            ROS_WARN("Unable to set shadow filter strength");
        }

        if (receiver_sensitivity_boost_ != DEFAULT_RECEIVER_SENSITIVITY_BOOST) {
          if (device_->setReceiverSensitivityBoost(receiver_sensitivity_boost_) == ldcp_sdk::no_error) {
            ROS_INFO("Receiver sensitivity boost %d applied", receiver_sensitivity_boost_);
            int current_receiver_sensitivity = 0;
            if (device_->getReceiverSensitivityValue(current_receiver_sensitivity) == ldcp_sdk::no_error)
              ROS_INFO("Current receiver sensitivity: %d", current_receiver_sensitivity);
          }
        }

        device_->startMeasurement();
        device_->startStreaming();

        sensor_msgs::LaserScan laser_scan;
        laser_scan.header.frame_id = frame_id_;
        laser_scan.range_min = range_min_;
        laser_scan.range_max = range_max_;

        auto readScanBlock = [&](ldcp_sdk::ScanBlock& scan_block) {
          if (device_->readScanBlock(scan_block) != ldcp_sdk::no_error)
            throw std::exception();
        };

        ros::Time last_frame_end_timestamp;
        ros::Time frame_start_timestamp, frame_end_timestamp;

        while (nh_.ok() && !quit_driver_.load()) {
          ldcp_sdk::ScanBlock scan_block;
          try {
            do {
              readScanBlock(scan_block);
              frame_start_timestamp = ros::Time::now();
            } while (scan_block.block_index != 0);

            double fov_angle_min = 0, fov_angle_max = 0;
            switch (scan_block.angular_fov) {
              case ldcp_sdk::ANGULAR_FOV_270DEG:
                fov_angle_min = -M_PI * 3 / 4;
                fov_angle_max = M_PI * 3 / 4;
                break;
              case ldcp_sdk::ANGULAR_FOV_360DEG:
                fov_angle_min = -M_PI;
                fov_angle_max = M_PI;
                break;
              default:
                ROS_ERROR("Unsupported FoV flag %d", scan_block.angular_fov);
                exit(-1);
            }
            angle_min_ = (angle_min_ > fov_angle_min) ? angle_min_ : fov_angle_min;
            angle_max_ = (angle_max_ < fov_angle_max) ? angle_max_ : fov_angle_max;

            int beam_count = scan_block.block_count * scan_block.block_length * 360 /
              ((scan_block.angular_fov == ldcp_sdk::ANGULAR_FOV_270DEG) ? 270 : 360);
            int beam_index_min = std::ceil(angle_min_ * beam_count / (2 * M_PI));
            int beam_index_max = std::floor(angle_max_ * beam_count / (2 * M_PI));
            int beam_index_excluded_min = std::ceil(angle_excluded_min_ * beam_count / (2 * M_PI));
            int beam_index_excluded_max = std::floor(angle_excluded_max_ * beam_count / (2 * M_PI));

            laser_scan.angle_min = (!invert_frame_) ? angle_min_ : angle_max_;
            laser_scan.angle_max = (!invert_frame_) ? angle_max_ : angle_min_;
            laser_scan.angle_increment = ((!invert_frame_) ? 1 : -1) *
                2 * M_PI / beam_count * average_factor_;

            laser_scan.ranges.resize(beam_index_max - beam_index_min + 1);
            laser_scan.intensities.resize(beam_index_max - beam_index_min + 1);

            std::fill(laser_scan.ranges.begin(), laser_scan.ranges.end(), 0.0);
            std::fill(laser_scan.intensities.begin(), laser_scan.intensities.end(), 0.0);

            auto updateLaserScan = [&](const ldcp_sdk::ScanBlock& scan_block) {
              int block_size = scan_block.layers[0].ranges.size();
              for (int i = 0; i < block_size; i++) {
                int beam_index = (scan_block.block_index - scan_block.block_count / 2) * block_size + i;
                if (beam_index < beam_index_min || beam_index > beam_index_max)
                  continue;
                if (beam_index >= beam_index_excluded_min && beam_index <= beam_index_excluded_max)
                  continue;
                if (scan_block.layers[0].ranges[i] != 0) {
                  laser_scan.ranges[beam_index - beam_index_min] = scan_block.layers[0].ranges[i] * 0.002;
                  laser_scan.intensities[beam_index - beam_index_min] = scan_block.layers[0].intensities[i];
                }
                else {
                  laser_scan.ranges[beam_index - beam_index_min] = std::numeric_limits<float>::infinity();
                  laser_scan.intensities[beam_index - beam_index_min] = 0;
                }
              }
            };

            while (scan_block.block_index != scan_block.block_count - 1) {
              updateLaserScan(scan_block);
              readScanBlock(scan_block);
              frame_end_timestamp = ros::Time::now();
            }
            updateLaserScan(scan_block);

            if (average_factor_ != 1) {
              int final_size = laser_scan.ranges.size() / average_factor_;
              for (int i = 0; i < final_size; i++) {
                double ranges_total = 0, intensities_total = 0;
                int count = 0;
                for (int j = 0; j < average_factor_; j++) {
                  int index = i * average_factor_ + j;
                  if (laser_scan.ranges[index] != 0) {
                    ranges_total += laser_scan.ranges[index];
                    intensities_total += laser_scan.intensities[index];
                    count++;
                  }
                }

                if (count > 0) {
                  laser_scan.ranges[i] = ranges_total / count;
                  laser_scan.intensities[i] = (int)(intensities_total / count);
                }
                else {
                  laser_scan.ranges[i] = 0;
                  laser_scan.intensities[i] = 0;
                }
              }

              laser_scan.ranges.resize(final_size);
              laser_scan.intensities.resize(final_size);
            }

            ros::Duration block_duration = ros::Duration(
              (frame_end_timestamp - frame_start_timestamp).toSec() /
              (scan_block.block_count - 1));
            frame_start_timestamp -= block_duration;

            if (last_frame_end_timestamp.isValid()) {
              if (frame_start_timestamp < last_frame_end_timestamp)
                frame_start_timestamp = last_frame_end_timestamp;
              laser_scan.header.stamp = frame_start_timestamp;
              laser_scan.time_increment = (frame_end_timestamp - frame_start_timestamp).toSec() /
                (scan_block.block_count * scan_block.block_length) * average_factor_;
              laser_scan.scan_time = (frame_end_timestamp - frame_start_timestamp).toSec();
              laser_scan_publisher.publish(laser_scan);
            }
            last_frame_end_timestamp = frame_end_timestamp;

            if (hibernation_requested_.load()) {
              device_->stopMeasurement();
              ROS_INFO("Device brought into hibernation");
              ros::Rate loop_rate(10);
              while (hibernation_requested_.load())
                loop_rate.sleep();
              device_->startMeasurement();
              ROS_INFO("Woken up from hibernation");
            }
          }
          catch (const std::exception&) {
            ROS_WARN("Error reading data from device");
            break;
          }
        }

        device_->stopStreaming();
      }
      else
        device_->reboot();

      lock.lock();
      device_->close();

      if (!reboot_required)
        ROS_INFO("Device closed");
      else
        ROS_INFO("Device rebooted");
    }
    else {
      ROS_INFO_THROTTLE(5, "Waiting for device... [%s]", device_address_.c_str());
      loop_rate.sleep();
    }
  }
}

bool LidarDriver::queryModelService(ltme_node::QueryModelRequest& request, ltme_node::QueryModelResponse& response)
{
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (lock.owns_lock()) {
    std::string model;
    if (device_->queryModel(model) == ldcp_sdk::no_error) {
      response.model = model;
      return true;
    }
  }
  return false;
}

bool LidarDriver::querySerialService(ltme_node::QuerySerialRequest& request,
                                     ltme_node::QuerySerialResponse& response)
{
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (lock.owns_lock()) {
    std::string serial;
    if (device_->querySerial(serial) == ldcp_sdk::no_error) {
      response.serial = serial;
      return true;
    }
  }
  return false;
}

bool LidarDriver::queryFirmwareVersion(ltme_node::QueryFirmwareVersionRequest& request,
                                       ltme_node::QueryFirmwareVersionResponse& response)
{
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (lock.owns_lock()) {
    std::string firmware_version;
    if (device_->queryFirmwareVersion(firmware_version) == ldcp_sdk::no_error) {
      response.firmware_version = firmware_version;
      return true;
    }
  }
  return false;
}

bool LidarDriver::queryHardwareVersion(ltme_node::QueryHardwareVersionRequest& request, ltme_node::QueryHardwareVersionResponse& response)
{
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (lock.owns_lock()) {
    std::string hardware_version;
    if (device_->queryHardwareVersion(hardware_version) == ldcp_sdk::no_error) {
      response.hardware_version = hardware_version;
      return true;
    }
  }
  return false;
}

bool LidarDriver::requestHibernationService(std_srvs::EmptyRequest& request,
                                            std_srvs::EmptyResponse& response)
{
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (lock.owns_lock()) {
    hibernation_requested_ = true;
    return true;
  }
  return false;
}

bool LidarDriver::requestWakeUpService(std_srvs::EmptyRequest& request,
                                       std_srvs::EmptyResponse& response)
{
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (lock.owns_lock()) {
    hibernation_requested_ = false;
    return true;
  }
  return false;
}

bool LidarDriver::quitDriverService(std_srvs::EmptyRequest& request,
                                    std_srvs::EmptyResponse& response)
{
  quit_driver_ = true;
  return true;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "ltme_node");
  ROS_INFO("ltme_node started");

  LidarDriver lidarDriver;
  lidarDriver.run();

  return 0;
}
