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

  error_t queryModel(std::string& model);
  error_t querySerial(std::string& serial);
  error_t queryFirmwareVersion(std::string& firmware_version);
  error_t queryHardwareVersion(std::string& hardware_version);
  error_t queryState(std::string& state);

  error_t readTimestamp(uint32_t& timestamp);
  error_t resetTimestamp();

  error_t startMeasurement();
  error_t stopMeasurement();
  error_t startStreaming();
  error_t stopStreaming();

  error_t readScanBlock(ScanBlock& scan_block);

  error_t getNetworkAddress(in_addr_t& address);
  error_t getSubnetMask(in_addr_t& subnet);
  error_t getScanFrequency(int& frequency);
  error_t setNetworkAddress(in_addr_t address);
  error_t setSubnetMask(in_addr_t subnet);
  error_t persistSettings();
};

}

#endif
