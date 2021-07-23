#include "ldcp/device_base.h"
#include "ldcp/session.h"

namespace ldcp_sdk
{

DeviceBase::DeviceBase(const ldcp_sdk::DeviceInfo& device_info)
  : DeviceBase(device_info.location())
{
}

DeviceBase::DeviceBase(const Location& location)
  : session_(new Session())
{
  if (typeid(location) == typeid(NetworkLocation))
    location_ = std::unique_ptr<NetworkLocation>(new NetworkLocation((const NetworkLocation&)location));
}

DeviceBase::DeviceBase(DeviceBase&& other)
  : location_(std::move(other.location_))
  , session_(std::move(other.session_))
{
}

DeviceBase::~DeviceBase()
{
  if (session_ && session_->isOpened())
    session_->close();
}

const Location& DeviceBase::location() const
{
  return *location_;
}

void DeviceBase::setTimeout(int timeout)
{
  session_->setTimeout(timeout);
}

error_t DeviceBase::open()
{
  error_t result = session_->open(*location_);
  if (result == error_t::no_error) {
    quit_log_message_handler_ = false;
    log_message_handler_thread_ = std::thread(
      std::bind(&DeviceBase::logMessageHandlerLoop, this));
  }
  return result;
}

bool DeviceBase::isOpened() const
{
  return session_->isOpened();
}

void DeviceBase::close()
{
  if (log_message_handler_thread_.joinable()) {
    quit_log_message_handler_ = true;
    log_message_handler_thread_.join();
  }
  session_->close();
}

void DeviceBase::setLogMessageCallback(LogMessageCallback callback)
{
  std::unique_lock<std::mutex> log_message_lock(log_message_mutex_);
  log_message_callback_ = callback;
}

error_t DeviceBase::queryOperationMode(std::string& mode)
{
  rapidjson::Document request = session_->createEmptyRequestObject(), response;
  rapidjson::Document::AllocatorType& allocator = request.GetAllocator();
  request["method"].SetString("device/queryInfo");
  request.AddMember("params",
                    rapidjson::Value().SetObject()
                      .AddMember("entry", "mode", allocator), allocator);

  error_t result = session_->executeCommand(std::move(request), response);

  if (result == error_t::no_error)
    mode = response["result"].GetString();

  return result;
}

void DeviceBase::reboot()
{
  rapidjson::Document request = session_->createEmptyRequestObject();
  request["method"].SetString("device/reboot");
  session_->executeCommand(std::move(request));
}

void DeviceBase::logMessageHandlerLoop()
{
  while (!quit_log_message_handler_.load()) {
    rapidjson::Document notification;
    if (session_->pollForLogMessage(notification) == error_t::no_error) {
      std::unique_lock<std::mutex> log_message_lock(log_message_mutex_);
      if (log_message_callback_) {
        std::string level = notification["params"]["level"].GetString();
        std::string message = notification["params"]["message"].GetString();

        log_level_t log_level = log_level_t::debug;
        if (level == "debug")
          log_level = log_level_t::debug;
        else if (level == "info")
          log_level = log_level_t::info;
        else if (level == "warn")
          log_level = log_level_t::warn;
        else if (level == "error")
          log_level = log_level_t::error;

        log_message_callback_(log_level, message);
      }
    }
  }
}

}
