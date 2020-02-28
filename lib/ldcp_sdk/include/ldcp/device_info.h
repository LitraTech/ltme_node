#ifndef LDCP_SDK_DEVICE_INFO_H_
#define LDCP_SDK_DEVICE_INFO_H_

#include "ldcp/location.h"

#include <memory>
#include <map>
#include <string>

namespace ldcp_sdk
{

class DeviceInfo
{
public:
  DeviceInfo(const std::string& id, const Location& location);
  DeviceInfo(const DeviceInfo& other);

  DeviceInfo& operator=(const DeviceInfo& other);
  bool operator==(const DeviceInfo& other) const;

  const std::string& id() const;
  const Location& location() const;
  std::map<std::string, std::string>& metadata();

private:
  std::string id_;
  std::unique_ptr<Location> location_;
  std::map<std::string, std::string> metadata_;
};

}

#endif
