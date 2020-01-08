#ifndef LDCP_SDK_DEVICE_NOTIFIER_H_
#define LDCP_SDK_DEVICE_NOTIFIER_H_

#include "ldcp/device_manager.h"

#include <thread>

namespace ldcp_sdk
{

typedef std::function<void(const DeviceInfo& new_entry)> DeviceAttachHandler;
typedef std::function<void(const std::string& id)> DeviceDetachHandler;

class DeviceNotifier
{
  friend class DeviceManager;

public:
  virtual ~DeviceNotifier() = default;

protected:
  DeviceNotifier() = default;

  virtual bool init() = 0;
  virtual void start() = 0;
  virtual void stop() = 0;

protected:
  std::thread thread_;

  DeviceAttachHandler device_attach_handler_;
  DeviceDetachHandler device_detach_handler_;
};

class NetworkDeviceNotifier : public DeviceNotifier
{
public:
  static NetworkDeviceNotifier& instance();
};

}

#endif
