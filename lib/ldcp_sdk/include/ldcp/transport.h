#ifndef LDCP_SDK_TRANSPORT_H_
#define LDCP_SDK_TRANSPORT_H_

#include <functional>
#include <memory>

#include <rapidjson/document.h>

#include "ldcp/error.h"
#include "ldcp/location.h"

namespace ldcp_sdk
{

class Transport
{
public:
  typedef std::function<void(rapidjson::Document)> ReceivedMessageCallback;
  typedef std::function<void(const error_t)> TransmitErrorCallback;
  typedef std::function<void(const error_t)> ReceiveErrorCallback;

public:
  static std::unique_ptr<Transport> create(const Location& location);

public:
  virtual ~Transport() = default;

  virtual error_t connect(int timeout) = 0;
  virtual void disconnect() = 0;
  virtual bool isConnected() const = 0;

  virtual void transmitMessage(rapidjson::Document message) = 0;

  void setReceivedMessageCallback(ReceivedMessageCallback callback);
  void setTransmitErrorCallback(TransmitErrorCallback callback);
  void setReceiveErrorCallback(ReceiveErrorCallback callback);

protected:
  ReceivedMessageCallback received_message_callback_;
  TransmitErrorCallback transmit_error_callback_;
  ReceiveErrorCallback receive_error_callback_;
};

}

#endif
