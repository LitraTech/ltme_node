#include "ldcp/transport.h"
#include "ldcp/utility.h"

#include <rapidjson/ostreamwrapper.h>
#include <rapidjson/writer.h>
#include <rapidjson/istreamwrapper.h>

#include <asio.hpp>

#include <thread>
#include <condition_variable>
#include <deque>
#include <iomanip>

namespace ldcp_sdk
{

class NetworkTransport : public Transport
{
public:
  NetworkTransport(const NetworkLocation& location);

  virtual error_t connect(int timeout);
  virtual void disconnect();
  virtual bool isConnected() const;

  virtual void transmitMessage(rapidjson::Document message);

private:
  void incomingMessageHandler(const asio::error_code& error, size_t bytes_transferred);
  void outgoingMessageHandler(const asio::error_code& error, size_t);

  rapidjson::Document parseIncomingMessage(size_t length);
  void encapsulateOutgoingMessage(rapidjson::Document& message);

private:
  std::thread worker_thread_;

  asio::io_service io_service_;

  asio::ip::tcp::endpoint device_address_;
  asio::ip::tcp::socket socket_;

  asio::streambuf incoming_message_buffer_;
  asio::streambuf outgoing_message_buffers_[3];
  std::deque<rapidjson::Document> outgoing_message_queue_;
};

NetworkTransport::NetworkTransport(const NetworkLocation& location)
  : device_address_(asio::ip::address_v4(ntohl(location.address())), ntohs(location.port()))
  , socket_(io_service_)
{
}

error_t NetworkTransport::connect(int timeout)
{
  std::mutex mutex;
  std::unique_lock<std::mutex> lock(mutex);
  std::condition_variable cv;

  asio::error_code connect_result = asio::error::would_block;
  socket_.async_connect(device_address_, [&](const asio::error_code& error) {
    if (error != asio::error::operation_aborted) {
      {
        std::lock_guard<std::mutex> lock_guard(mutex);
        connect_result = error;
      }
      cv.notify_one();
    }

    if (!error)
      asio::async_read_until(socket_, incoming_message_buffer_, "\r\n",
                             std::bind(&NetworkTransport::incomingMessageHandler,
                                       this, std::placeholders::_1, std::placeholders::_2));
  });

  worker_thread_ = std::thread([&]() {
    io_service_.run();
  });

  bool wait_result = cv.wait_for(lock, std::chrono::milliseconds(timeout), [&]() {
    return (connect_result != asio::error::would_block);
  });
  if (wait_result && !connect_result)
    return error_t::no_error;
  else {
    socket_.close();
    worker_thread_.join();

    if (!wait_result)
      return error_t::timed_out;
    else if (connect_result == asio::error::connection_refused)
      return error_t::connection_refused;
    else
      return error_t::unknown;
  }
}

void NetworkTransport::disconnect()
{
  if (socket_.is_open()) {
    io_service_.dispatch([&]() {
      socket_.shutdown(asio::ip::tcp::socket::shutdown_both);
      socket_.close();
    });
    worker_thread_.join();
  }
}

bool NetworkTransport::isConnected() const
{
  return socket_.is_open();
}

void NetworkTransport::transmitMessage(rapidjson::Document message)
{
  std::shared_ptr<rapidjson::Document> wrapped_message = std::make_shared<rapidjson::Document>(std::move(message));
  io_service_.dispatch([this, wrapped_message]() {
    bool transmit_in_progress = !outgoing_message_queue_.empty();
    outgoing_message_queue_.push_back(std::move(*wrapped_message));
    if (!transmit_in_progress) {
      rapidjson::Document& document = outgoing_message_queue_.front();
      encapsulateOutgoingMessage(document);

      std::vector<asio::streambuf::const_buffers_type> buffers = { outgoing_message_buffers_[0].data(),
                                                                   outgoing_message_buffers_[1].data(),
                                                                   outgoing_message_buffers_[2].data() };
      asio::async_write(socket_, buffers,
                        std::bind(&NetworkTransport::outgoingMessageHandler,
                                  this, std::placeholders::_1, std::placeholders::_2));
    }
  });
}

void NetworkTransport::incomingMessageHandler(const asio::error_code& error, size_t bytes_transferred)
{
  if (!error) {
    if (received_message_callback_) {
      rapidjson::Document message = parseIncomingMessage(bytes_transferred);
      if (!message.IsNull())
        received_message_callback_(std::move(message));
    }
    asio::async_read_until(socket_, incoming_message_buffer_, "\r\n",
                           std::bind(&NetworkTransport::incomingMessageHandler,
                                     this, std::placeholders::_1, std::placeholders::_2));
  }
  else if (error != asio::error::operation_aborted && receive_error_callback_)
    receive_error_callback_(error_t::unknown);
}

void NetworkTransport::outgoingMessageHandler(const asio::error_code& error, size_t)
{
  if (!error) {
    for (int i = 0; i < 3; i++) {
      asio::streambuf& streambuf = outgoing_message_buffers_[i];
      streambuf.consume(streambuf.size());
    }
    outgoing_message_queue_.pop_front();

    if (!outgoing_message_queue_.empty()) {
      rapidjson::Document& document = outgoing_message_queue_.front();
      encapsulateOutgoingMessage(document);

      std::vector<asio::streambuf::const_buffers_type> buffers = { outgoing_message_buffers_[0].data(),
                                                                   outgoing_message_buffers_[1].data(),
                                                                   outgoing_message_buffers_[2].data() };
      asio::async_write(socket_, buffers,
                        std::bind(&NetworkTransport::outgoingMessageHandler,
                                  this, std::placeholders::_1, std::placeholders::_2));
    }
  }
  else if (error != asio::error::operation_aborted && transmit_error_callback_)
    transmit_error_callback_(error_t::unknown);
}

rapidjson::Document NetworkTransport::parseIncomingMessage(size_t length)
{
  rapidjson::Document message;

  size_t bytes_buffered = incoming_message_buffer_.size();

  std::istream istream(&incoming_message_buffer_);
  if (istream.peek() == '{') {
    rapidjson::IStreamWrapper istream_wrapper(istream);
    message.ParseStream<rapidjson::kParseStopWhenDoneFlag>(istream_wrapper);
  }
  else {
    try {
      int expected_checksum = std::numeric_limits<int>::max();
      bool end_of_headers = false;

      while (true) {
        size_t character_count = std::string::npos;
        char colon = '\0', comma = '\0';
        std::string character_sequence;

        istream >> character_count >> colon;
        if (!end_of_headers) {
          character_sequence.resize(character_count);
          istream.read(&character_sequence[0], character_count);
          istream >> comma;
        }
        else {
          auto iter = asio::buffers_begin(incoming_message_buffer_.data());
          uint16_t actual_checksum = Utility::CalculateCRC16(iter, iter + character_count);
          if (actual_checksum != expected_checksum)
            throw std::runtime_error("");
          comma = *(iter + character_count);
        }

        if (character_count == std::string::npos || !istream.good() || colon != ':' || comma != ',')
          throw std::runtime_error("");

        if (character_count == 0) {
          if (!end_of_headers) {
            end_of_headers = true;
            continue;
          }
          else
            break;
        }
        else if (!end_of_headers) {
          std::string key = character_sequence.substr(0, character_sequence.find('='));
          std::string value = character_sequence.substr(key.length() + 1);
          if (key == "checksum")
            expected_checksum = std::stoi(value, nullptr, 16);
        }
        else {
          rapidjson::IStreamWrapper istream_wrapper(istream);
          message.ParseStream<rapidjson::kParseStopWhenDoneFlag>(istream_wrapper);
          break;
        }
      }
    }
    catch (...) {
    }
  }

  size_t bytes_consumed = bytes_buffered - incoming_message_buffer_.size();

  istream.clear();
  istream.ignore(length - bytes_consumed);

  if (message.HasParseError() || !message.IsObject())
    message.SetNull();

  return message;
}

void NetworkTransport::encapsulateOutgoingMessage(rapidjson::Document& message)
{
  std::ostream ostream_main_part(&outgoing_message_buffers_[1]);
  rapidjson::OStreamWrapper ostream_wrapper(ostream_main_part);
  rapidjson::Writer<rapidjson::OStreamWrapper> writer(ostream_wrapper);
  message.Accept(writer);

  size_t length = outgoing_message_buffers_[1].size();
  uint16_t checksum = Utility::CalculateCRC16(asio::buffers_begin(outgoing_message_buffers_[1].data()),
      asio::buffers_end(outgoing_message_buffers_[1].data()));

  std::ostream ostream_leading_part(&outgoing_message_buffers_[0]);
  ostream_leading_part << "15:checksum=0x"
                       << std::hex << std::setw(4) << std::setfill('0') << std::uppercase
                       << checksum << ",";
  ostream_leading_part << "0:,";
  ostream_leading_part << std::dec << length << ":";

  std::ostream ostream_trailing_part(&outgoing_message_buffers_[2]);
  ostream_trailing_part << ",\r\n";
}

std::unique_ptr<Transport> Transport::create(const Location& location)
{
  if (typeid(location) == typeid(NetworkLocation))
    return std::unique_ptr<NetworkTransport>(
      new NetworkTransport(dynamic_cast<const NetworkLocation&>(location)));
  else
    return nullptr;
}

void Transport::setReceivedMessageCallback(Transport::ReceivedMessageCallback callback)
{
  received_message_callback_ = callback;
}

void Transport::setTransmitErrorCallback(Transport::TransmitErrorCallback callback)
{
  transmit_error_callback_ = callback;
}

void Transport::setReceiveErrorCallback(Transport::ReceiveErrorCallback callback)
{
  receive_error_callback_ = callback;
}

}
