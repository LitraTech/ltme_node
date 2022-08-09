#ifndef LDCP_SDK_DEVICE_H_
#define LDCP_SDK_DEVICE_H_

#include "ldcp/device_base.h"

namespace ldcp_sdk
{

class Session;

class Device : public DeviceBase
{
public:
  Device(const DeviceInfo& device_info);
  Device(const Location& location);
  Device(DeviceBase&& other);

  virtual error_t open();

  error_t queryModel(std::string& model);
  error_t querySerial(std::string& serial);
  error_t queryFirmwareVersion(std::string& firmware_version);
  error_t queryHardwareVersion(std::string& hardware_version);
  error_t queryState(std::string& state);
  error_t queryMotorFrequency(double& motor_frequency);

  error_t readTimestamp(uint32_t& timestamp);
  error_t resetTimestamp();

  error_t startMeasurement();
  error_t stopMeasurement();
  error_t startStreaming();
  error_t stopStreaming();

  error_t readScanFrame(ScanFrame& scan_frame);
  error_t readScanBlock(ScanBlock& scan_block);

  error_t getNetworkAddress(in_addr_t& address);
  error_t getSubnetMask(in_addr_t& subnet);
  error_t getScanFrequency(int& frequency);
  error_t isOobEnabled(bool& enabled);
  error_t getOobTargetAddress(in_addr_t& address);
  error_t getOobTargetPort(in_port_t& port);
  error_t setNetworkAddress(in_addr_t address);
  error_t setSubnetMask(in_addr_t subnet);
  error_t setScanFrequency(int frequency);
  error_t setOobEnabled(bool enabled);
  error_t setOobTargetAddress(in_addr_t address);
  error_t setOobTargetPort(in_port_t port);
  error_t persistSettings();

  void rebootToBootloader();
};

}

#endif
