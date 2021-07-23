#ifndef LDCP_SDK_DEVICE_BASE_H_
#define LDCP_SDK_DEVICE_BASE_H_

#include "ldcp/device_info.h"
#include "ldcp/error.h"
#include "ldcp/data_types.h"

#include <thread>
#include <atomic>
#include <mutex>

namespace ldcp_sdk
{

class Session;

class DeviceBase
{
public:
  DeviceBase(const DeviceInfo& device_info);
  DeviceBase(const Location& location);
  DeviceBase(DeviceBase&& other);
  virtual ~DeviceBase();

  const Location& location() const;

  void setTimeout(int timeout);

  virtual error_t open();
  bool isOpened() const;
  void close();

  void setLogMessageCallback(LogMessageCallback callback);

  error_t queryOperationMode(std::string& mode);
  void reboot();

private:
  void logMessageHandlerLoop();

protected:
  std::unique_ptr<Location> location_;
  std::unique_ptr<Session> session_;

  std::thread log_message_handler_thread_;
  std::atomic<bool> quit_log_message_handler_;
  LogMessageCallback log_message_callback_;
  std::mutex log_message_mutex_;
};

}

#endif
