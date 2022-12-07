#include "ldcp/device_manager.h"
#include "ldcp/device_notifier.h"

#include <algorithm>

namespace ldcp_sdk
{

DeviceManager& DeviceManager::instance()
{
  static DeviceManager instance;
  return instance;
}

DeviceManager::~DeviceManager()
{
  if (monitoring_started_)
    stopMonitoring();
}

void DeviceManager::registerDeviceEventCallback(DeviceEventCallback callback)
{
  callback_ = callback;
}

bool DeviceManager::addDeviceNotifier(DeviceNotifier* notifier)
{
  if (!notifier->init())
    return false;
  notifier->device_attach_handler_ = std::bind(&DeviceManager::onDeviceAttached, this, std::placeholders::_1);
  notifier->device_detach_handler_ = std::bind(&DeviceManager::onDeviceDetached, this, std::placeholders::_1);
  notifiers_.push_back(notifier);
  return true;
}

void DeviceManager::startMonitoring()
{
  monitoring_started_ = true;
  for (DeviceNotifier* notifier : notifiers_)
    notifier->start();
}

void DeviceManager::stopMonitoring()
{
  if (monitoring_started_) {
    for (DeviceNotifier* notifier : notifiers_)
      notifier->stop();
    monitoring_started_ = false;
  }
}

std::vector<DeviceInfo> DeviceManager::deviceEntries()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return std::vector<DeviceInfo>(device_entries_.begin(), device_entries_.end());
}

DeviceManager::DeviceManager()
  : monitoring_started_(false)
{
}

void DeviceManager::onDeviceAttached(const DeviceInfo& new_entry)
{
  std::lock_guard<std::mutex> event_lock(event_mutex_);
  {
    std::lock_guard<std::mutex> data_lock(data_mutex_);
    device_entries_.push_back(new_entry);
  }
  if (callback_)
    callback_(EVENT_DEVICE_ATTACHED, new_entry);
}

void DeviceManager::onDeviceDetached(const std::string& id)
{
  std::lock_guard<std::mutex> event_lock(event_mutex_);
  std::unique_ptr<DeviceInfo> entry;
  {
    std::lock_guard<std::mutex> data_lock(data_mutex_);
    auto position = std::find_if(device_entries_.begin(), device_entries_.end(), [&id](const DeviceInfo& entry) {
      return (entry.id() == id);
    });
    if (position != device_entries_.end()) {
      entry.reset(new DeviceInfo(*position));
      device_entries_.erase(position);
    }
  }
  if (entry && callback_)
    callback_(EVENT_DEVICE_DETACHED, *entry);
}

}
