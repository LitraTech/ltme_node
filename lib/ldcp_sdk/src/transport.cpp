#include "ldcp/transport.h"
#include "ldcp/utility.h"
#include "ldcp/data_types.h"

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

  virtual error_t enableOob(const Location& location);

private:
  void incomingMessageHandler(const asio::error_code& error, size_t bytes_transferred);
  void outgoingMessageHandler(const asio::error_code& error, size_t);
  void oobPacketHandler(const asio::error_code& error, size_t bytes_transferred);

  rapidjson::Document parseIncomingMessage(size_t length);
  void encapsulateOutgoingMessage(rapidjson::Document& message);
  bool verifyOobPacket(uint8_t* data, int length);

private:
  std::thread worker_thread_;

  asio::io_service io_service_;

  asio::ip::tcp::socket primary_socket_;
  asio::ip::tcp::endpoint device_address_;

  asio::streambuf incoming_message_buffer_;
  asio::streambuf outgoing_message_buffers_[3];
  std::deque<rapidjson::Document> outgoing_message_queue_;

  asio::ip::udp::socket oob_socket_;
  asio::ip::udp::endpoint sender_address_;

  std::array<uint8_t, OOB_PACKET_LENGTH_MAX> oob_packet_buffer_;
};

NetworkTransport::NetworkTransport(const NetworkLocation& location)
  : device_address_(asio::ip::address_v4(ntohl(location.address())), ntohs(location.port()))
  , primary_socket_(io_service_)
  , oob_socket_(io_service_)
{
}

error_t NetworkTransport::connect(int timeout)
{
  error_t result = error_t::no_error;

  std::mutex mutex;
  {
    std::condition_variable cv;

    asio::error_code connect_result = asio::error::would_block;
    primary_socket_.async_connect(device_address_, [&](const asio::error_code& error) {
      std::lock_guard<std::mutex> lock_guard(mutex);
      if (result == error_t::no_error) {
        if (error != asio::error::operation_aborted) {
          connect_result = error;
          cv.notify_one();
        }
        if (!error) {
          asio::async_read_until(primary_socket_, incoming_message_buffer_, "\r\n",
                                 std::bind(&NetworkTransport::incomingMessageHandler,
                                           this, std::placeholders::_1, std::placeholders::_2));
        }
      }
    });

    worker_thread_ = std::thread([&]() {
      io_service_.run();
    });

    std::unique_lock<std::mutex> lock(mutex);
    bool wait_result = cv.wait_for(lock, std::chrono::milliseconds(timeout), [&]() {
      return (connect_result != asio::error::would_block);
    });

    if (wait_result && !connect_result)
      result = error_t::no_error;
    else {
      if (!wait_result)
        result = error_t::timed_out;
      else if (connect_result == asio::error::connection_refused)
        result = error_t::connection_refused;
      else
        result = error_t::unknown;
    }
  }

  if (result != error_t::no_error) {
    primary_socket_.close();
    worker_thread_.join();
  }

  return result;
}

void NetworkTransport::disconnect()
{
  if (primary_socket_.is_open()) {
    io_service_.dispatch([&]() {
      primary_socket_.shutdown(asio::ip::tcp::socket::shutdown_both);
      primary_socket_.close();
      if (oob_socket_.is_open())
        oob_socket_.close();
    });
    worker_thread_.join();
  }
}

bool NetworkTransport::isConnected() const
{
  return primary_socket_.is_open();
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
      asio::async_write(primary_socket_, buffers,
                        std::bind(&NetworkTransport::outgoingMessageHandler,
                                  this, std::placeholders::_1, std::placeholders::_2));
    }
  });
}

error_t NetworkTransport::enableOob(const Location& location)
{
  oob_socket_.open(asio::ip::udp::v4());
  oob_socket_.set_option(asio::ip::udp::socket::reuse_address(true));

  const NetworkLocation& network_location = dynamic_cast<const NetworkLocation&>(location);
  asio::ip::udp::endpoint local_address(asio::ip::address_v4(ntohl(network_location.address())),
                                        ntohs(network_location.port()));
  asio::error_code bind_result;
  oob_socket_.bind(local_address, bind_result);

  if (!bind_result) {
    oob_socket_.async_receive_from(asio::buffer(oob_packet_buffer_),
                                   sender_address_,
                                   std::bind(&NetworkTransport::oobPacketHandler,
                                             this, std::placeholders::_1, std::placeholders::_2));
    return error_t::no_error;
  }
  else {
    oob_socket_.close();

    if (bind_result == asio::error::address_in_use)
      return error_t::address_in_use;
    else
      return error_t::unknown;
  }
}

void NetworkTransport::incomingMessageHandler(const asio::error_code& error, size_t bytes_transferred)
{
  if (!error) {
    if (received_message_callback_) {
      rapidjson::Document message = parseIncomingMessage(bytes_transferred);
      if (!message.IsNull())
        received_message_callback_(std::move(message));
    }
    asio::async_read_until(primary_socket_, incoming_message_buffer_, "\r\n",
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
      asio::async_write(primary_socket_, buffers,
                        std::bind(&NetworkTransport::outgoingMessageHandler,
                                  this, std::placeholders::_1, std::placeholders::_2));
    }
  }
  else if (error != asio::error::operation_aborted && transmit_error_callback_)
    transmit_error_callback_(error_t::unknown);
}

void NetworkTransport::oobPacketHandler(const asio::error_code& error, size_t bytes_transferred)
{
  if (!error) {
    if ((sender_address_.address() == device_address_.address() &&
         sender_address_.port() == device_address_.port()) &&
        received_oob_packet_callback_) {
      if (verifyOobPacket(oob_packet_buffer_.data(), bytes_transferred)) {
        std::vector<uint8_t> oob_data(oob_packet_buffer_.data(),
                                      oob_packet_buffer_.data() + bytes_transferred);
        received_oob_packet_callback_(std::move(oob_data));
      }
    }
    oob_socket_.async_receive_from(asio::buffer(oob_packet_buffer_),
                                   sender_address_,
                                   std::bind(&NetworkTransport::oobPacketHandler,
                                             this, std::placeholders::_1, std::placeholders::_2));
  }
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
      int expected_checksum = -1;
      bool end_of_headers = false;

      while (true) {
        size_t character_count = std::string::npos;
        char colon = '\0', comma = '\0';
        std::string character_sequence;

        istream >> character_count >> colon;
        if (!istream.good() || character_count > MESSAGE_LENGTH_MAX || colon != ':')
          throw std::runtime_error("");

        if (!end_of_headers) {
          character_sequence.resize(character_count);
          istream.read(&character_sequence[0], character_count);
          istream >> comma;
        }
        else {
          if (!(asio::buffers_end(incoming_message_buffer_.data()) -
                asio::buffers_begin(incoming_message_buffer_.data()) >= character_count + 1))
            throw std::runtime_error("");
          auto iter = asio::buffers_begin(incoming_message_buffer_.data());
          int actual_checksum = Utility::CalculateCRC16(iter, iter + character_count);
          if (actual_checksum != expected_checksum)
            throw std::runtime_error("");
          comma = *(iter + character_count);
        }

        if (!istream.good() || comma != ',')
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

bool NetworkTransport::verifyOobPacket(uint8_t* data, int length)
{
  OobPacketHeader* oob_packet_header = reinterpret_cast<OobPacketHeader*>(data);

  if (length < sizeof(OobPacketHeader) || oob_packet_header->signature != 0xFFFF)
    return false;

  uint16_t saved_checksum = oob_packet_header->checksum;
  oob_packet_header->checksum = 0;
  bool packet_valid = (Utility::CalculateCRC16(data, data + length) == saved_checksum);
  oob_packet_header->checksum = saved_checksum;
  return packet_valid;
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

void Transport::setReceivedOobPacketCallback(Transport::ReceivedOobPacketCallback callback)
{
  received_oob_packet_callback_ = callback;
}

}
