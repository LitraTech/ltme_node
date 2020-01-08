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

  void executeCommand(rapidjson::Document request);
  error_t executeCommand(rapidjson::Document request, rapidjson::Document& response);
  error_t pollForScanData(rapidjson::Document& notification);

private:
  void onMessageReceived(rapidjson::Document message);

private:
  static const int DEFAULT_TIMEOUT = 3000;
  static const int LASER_SCAN_NOTIFICATION_BUFFERING_COUNT = 32;

private:
  int timeout_;

  std::unique_ptr<Transport> transport_;

  int id_;
  std::deque<rapidjson::Document> response_queue_;
  std::mutex response_queue_mutex_;
  std::condition_variable response_queue_cv_;

  std::deque<rapidjson::Document> laser_scan_queue_;
  std::mutex laser_scan_mutex_;
  std::condition_variable laser_scan_queue_cv_;
};

}

#endif
