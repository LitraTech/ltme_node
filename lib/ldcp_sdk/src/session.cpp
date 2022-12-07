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
  transport_->setReceivedOobPacketCallback(std::bind(&Session::onOobPacketReceived, this, std::placeholders::_1));
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
  scan_block_queue_primary_.clear();
  scan_block_queue_oob_.clear();
}

bool Session::isOpened() const
{
  return (transport_ && transport_->isConnected());
}

rapidjson::Document Session::createEmptyRequestObject()
{
  rapidjson::Document request;
  rapidjson::Document::AllocatorType& allocator = request.GetAllocator();

  request.SetObject()
      .AddMember("jsonrpc", "2.0", allocator)
      .AddMember("method", rapidjson::Value(), allocator);

  return request;
}

void Session::executeCommand(rapidjson::Document request)
{
  std::lock_guard<std::mutex> command_lock(command_mutex_);
  request.AddMember("id", ++id_, request.GetAllocator());
  transport_->transmitMessage(std::move(request));
}

error_t Session::executeCommand(rapidjson::Document request, rapidjson::Document& response)
{
  std::lock_guard<std::mutex> command_lock(command_mutex_);
  std::unique_lock<std::mutex> response_queue_lock(response_queue_mutex_);

  request.AddMember("id", ++id_, request.GetAllocator());
  transport_->transmitMessage(std::move(request));

  bool wait_result = response_queue_cv_.wait_for(response_queue_lock, std::chrono::milliseconds(timeout_), [&]() {
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

error_t Session::enableOobTransport(const Location& location)
{
  return transport_->enableOob(location);
}

error_t Session::pollForScanBlock(rapidjson::Document& notification, std::vector<uint8_t>& oob_data)
{
  std::unique_lock<std::mutex> scan_block_queue_lock(scan_block_queue_mutex_);
  bool wait_result = scan_block_queue_cv_.wait_for(scan_block_queue_lock, std::chrono::milliseconds(timeout_), [&]() {
    return (scan_block_queue_primary_.size() > 0) || (scan_block_queue_oob_.size() > 0);
  });
  if (wait_result) {
    if (scan_block_queue_primary_.size() > 0) {
      notification = std::move(scan_block_queue_primary_.front());
      scan_block_queue_primary_.pop_front();
    }
    else if (scan_block_queue_oob_.size() > 0) {
      oob_data = std::move(scan_block_queue_oob_.front());
      scan_block_queue_oob_.pop_front();
    }
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
    std::lock_guard<std::mutex> response_queue_lock(response_queue_mutex_);
    if (message["id"] == id_) {
      response_queue_.clear();
      response_queue_.push_back(std::move(message));
      response_queue_cv_.notify_one();
    }
  }
  else if ((message.HasMember("method") && message["method"] == "notification/laserScan") &&
           message.HasMember("params") && !message.HasMember("id")) {
    std::lock_guard<std::mutex> scan_block_queue_lock(scan_block_queue_mutex_);
    if (scan_block_queue_primary_.size() == SCAN_BLOCK_BUFFERING_COUNT)
      scan_block_queue_primary_.pop_front();
    scan_block_queue_primary_.push_back(std::move(message));
    scan_block_queue_cv_.notify_one();
  }
}

void Session::onOobPacketReceived(std::vector<uint8_t> oob_packet)
{
  std::lock_guard<std::mutex> scan_block_queue_lock(scan_block_queue_mutex_);
  if (scan_block_queue_oob_.size() == SCAN_BLOCK_BUFFERING_COUNT)
    scan_block_queue_oob_.pop_front();
  scan_block_queue_oob_.push_back(std::move(oob_packet));
  scan_block_queue_cv_.notify_one();
}

}
