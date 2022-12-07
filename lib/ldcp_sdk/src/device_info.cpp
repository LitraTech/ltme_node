#include "ldcp/device_info.h"

namespace ldcp_sdk
{

DeviceInfo::DeviceInfo(const std::string& id, const Location& location)
  : id_(id)
{
  if (typeid(location) == typeid(NetworkLocation))
    location_.reset(new NetworkLocation(dynamic_cast<const NetworkLocation&>(location)));
}

DeviceInfo::DeviceInfo(const DeviceInfo& other)
  : id_(other.id_)
  , metadata_(other.metadata_)
{
  const Location& location = *other.location_;
  if (typeid(location) == typeid(NetworkLocation))
    location_.reset(new NetworkLocation(dynamic_cast<const NetworkLocation&>(location)));
}

DeviceInfo& DeviceInfo::operator=(const DeviceInfo& other)
{
  if (&other != this) {
    id_ = other.id_;
    const Location& location = *other.location_;
    if (typeid(location) == typeid(NetworkLocation))
      location_.reset(new NetworkLocation(dynamic_cast<const NetworkLocation&>(location)));
    metadata_ = other.metadata_;
  }
  return *this;
}

bool DeviceInfo::operator==(const DeviceInfo& other) const
{
  if (typeid(*location_) == typeid(NetworkLocation) &&
      typeid(*other.location_) == typeid(NetworkLocation))
    return ((NetworkLocation&)*location_ == (NetworkLocation&)*other.location_);
  else
    return false;
}

const std::string& DeviceInfo::id() const
{
  return id_;
}

const Location& DeviceInfo::location() const
{
  return *location_;
}

std::map<std::string, std::string>& DeviceInfo::metadata()
{
  return metadata_;
}

}
