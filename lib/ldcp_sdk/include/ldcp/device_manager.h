#ifndef LDCP_SDK_DEVICE_MANAGER_H_
#define LDCP_SDK_DEVICE_MANAGER_H_

#include "ldcp/device_info.h"

#include <functional>
#include <vector>
#include <mutex>
#include <list>

namespace ldcp_sdk
{

enum DeviceEvent
{
  EVENT_DEVICE_ATTACHED,
  EVENT_DEVICE_DETACHED
};
typedef std::function<void(DeviceEvent, const DeviceInfo&)> DeviceEventCallback;

class DeviceNotifier;

class DeviceManager
{
public:
  static DeviceManager& instance();

  ~DeviceManager();

  void registerDeviceEventCallback(DeviceEventCallback callback);

  bool addDeviceNotifier(DeviceNotifier* notifier);
  void startMonitoring();
  void stopMonitoring();

  std::vector<DeviceInfo> deviceEntries();

private:
  DeviceManager();

  void onDeviceAttached(const DeviceInfo& new_entry);
  void onDeviceDetached(const std::string& id);

private:
  DeviceEventCallback callback_;

  std::vector<DeviceNotifier*> notifiers_;
  bool monitoring_started_;

  std::mutex event_mutex_, data_mutex_;
  std::list<DeviceInfo> device_entries_;
};

}

#endif
