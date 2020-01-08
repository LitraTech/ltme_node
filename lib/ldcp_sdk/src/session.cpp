#include "ldcp/session.h"

#include <algorithm>

namespace ldcp_sdk
{

typedef enum {
  ldcp_no_error = 0,
  ldcp_error_invalid_message = -1,
  ldcp_error_checksum_mismatch = -2,
  ldcp_error_json_rpc_parse_error = -32700,
  ldcp_error_json_rpc_invalid_request = -32600,
  ldcp_error_json_rpc_method_not_found = -32601,
  ldcp_error_json_rpc_invalid_params = -32602,
  ldcp_error_json_rpc_internal_error = -32603
} ldcp_error_t;

Session::Session()
  : timeout_(DEFAULT_TIMEOUT)
  , id_(-1)
{
}

void Session::setTimeout(int timeout)
{
  timeout_ = timeout;
}

error_t Session::open(const Location& location)
{
  transport_ = Transport::create(location);
#if defined(_MSC_VER) && (_MSC_VER <= 1800)
  transport_->setReceivedMessageCallback(
    [this](rapidjson::Document message) { onMessageReceived(std::move(message)); }
  );
#else
  transport_->setReceivedMessageCallback(std::bind(&Session::onMessageReceived, this, std::placeholders::_1));
#endif
  error_t connect_result = transport_->connect(timeout_);
  if (connect_result != error_t::no_error)
    transport_ = nullptr;
  return connect_result;
}

void Session::close()
{
  if (transport_) {
    if (transport_->isConnected())
      transport_->disconnect();
    transport_ = nullptr;
  }

  response_queue_.clear();
  laser_scan_queue_.clear();
}

bool Session::isOpened() const
{
  return (transport_ && transport_->isConnected());
}

void Session::executeCommand(rapidjson::Document request)
{
  request.AddMember("id", ++id_, request.GetAllocator());
  transport_->transmitMessage(std::move(request));
}

error_t Session::executeCommand(rapidjson::Document request, rapidjson::Document& response)
{
  std::unique_lock<std::mutex> lock(response_queue_mutex_);

  request.AddMember("id", ++id_, request.GetAllocator());
  transport_->transmitMessage(std::move(request));

  bool wait_result = response_queue_cv_.wait_for(lock, std::chrono::milliseconds(timeout_), [&]() {
    std::remove_if(response_queue_.begin(), response_queue_.end(), [&](const rapidjson::Document& document) {
      return (document["id"].GetInt() < id_);
    });
    return (response_queue_.size() > 0 && response_queue_.front()["id"] == id_);
  });
  if (wait_result) {
    rapidjson::Document& message = response_queue_.front();

    if (message.HasMember("result")) {
      response = std::move(response_queue_.front());
      response_queue_.pop_front();
      return error_t::no_error;
    }
    else {
      if (message["error"].IsObject() &&
          message["error"].HasMember("code") && message["error"]["code"].IsInt()) {
        int error_code = message["error"]["code"].GetInt();
        if (error_code == ldcp_error_t::ldcp_error_invalid_message ||
            error_code == ldcp_error_t::ldcp_error_checksum_mismatch ||
            error_code == ldcp_error_t::ldcp_error_json_rpc_parse_error ||
            error_code == ldcp_error_t::ldcp_error_json_rpc_invalid_request)
          return error_t::protocol_error;
        else if (error_code == ldcp_error_t::ldcp_error_json_rpc_method_not_found)
          return error_t::not_supported;
        else if (error_code == ldcp_error_t::ldcp_error_json_rpc_invalid_params)
          return error_t::invalid_params;
        else if (error_code == ldcp_error_t::ldcp_error_json_rpc_internal_error)
          return error_t::device_error;
        else
          return error_t::unknown;
      }
      else
        return error_t::unknown;
    }
  }
  else
    return error_t::timed_out;
}

error_t Session::pollForScanData(rapidjson::Document& notification)
{
  std::unique_lock<std::mutex> lock(laser_scan_mutex_);
  bool wait_result = laser_scan_queue_cv_.wait_for(lock, std::chrono::milliseconds(timeout_), [&]() {
    return laser_scan_queue_.size() > 0;
  });
  if (wait_result) {
    notification = std::move(laser_scan_queue_.front());
    laser_scan_queue_.pop_front();
    return error_t::no_error;
  }
  else
    return error_t::timed_out;
}

void Session::onMessageReceived(rapidjson::Document message)
{
  if (!(message.HasMember("jsonrpc") && message["jsonrpc"] == "2.0"))
    return;

  if ((message.HasMember("result") || message.HasMember("error")) &&
      message.HasMember("id")) {
    std::lock_guard<std::mutex> lock(response_queue_mutex_);
    if (message["id"] == id_) {
      response_queue_.clear();
      response_queue_.push_back(std::move(message));
      response_queue_cv_.notify_one();
    }
  }
  else if ((message.HasMember("method") && message["method"] == "notification/laserScan") &&
           message.HasMember("params") && !message.HasMember("id")) {
    std::lock_guard<std::mutex> lock(laser_scan_mutex_);
    if (laser_scan_queue_.size() == LASER_SCAN_NOTIFICATION_BUFFERING_COUNT)
      laser_scan_queue_.pop_front();
    laser_scan_queue_.push_back(std::move(message));
    laser_scan_queue_cv_.notify_one();
  }
}

}
