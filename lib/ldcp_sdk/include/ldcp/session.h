#ifndef LDCP_SDK_SESSION_H_
#define LDCP_SDK_SESSION_H_

#include "ldcp/location.h"
#include "ldcp/transport.h"

#include <rapidjson/document.h>

#include <deque>
#include <condition_variable>

namespace ldcp_sdk
{

class Session
{
public:
  Session();

  void setTimeout(int timeout);

  error_t open(const Location& location);
  void close();
  bool isOpened() const;

  rapidjson::Document createEmptyRequestObject();
  void executeCommand(rapidjson::Document request);
  error_t executeCommand(rapidjson::Document request, rapidjson::Document& response);

  error_t enableOobTransport(const Location& location);

  error_t pollForScanBlock(rapidjson::Document& notification,
                           std::vector<uint8_t>& oob_data);

private:
  void onMessageReceived(rapidjson::Document message);
  void onOobPacketReceived(std::vector<uint8_t> oob_packet);

private:
  static const int DEFAULT_TIMEOUT = 3000;
  static const int SCAN_BLOCK_BUFFERING_COUNT = 32;

private:
  int timeout_;

  std::unique_ptr<Transport> transport_;

  int id_;
  std::mutex command_mutex_;

  std::deque<rapidjson::Document> response_queue_;
  std::mutex response_queue_mutex_;
  std::condition_variable response_queue_cv_;

  std::deque<rapidjson::Document> scan_block_queue_primary_;
  std::deque<std::vector<uint8_t>> scan_block_queue_oob_;
  std::mutex scan_block_queue_mutex_;
  std::condition_variable scan_block_queue_cv_;
};

}

#endif
