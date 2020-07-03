#include "ldcp/device.h"
#include "ldcp/session.h"
#include "ldcp/utility.h"

#include <asio.hpp>

namespace ldcp_sdk
{

Device::Device(const DeviceInfo& device_info)
  : DeviceBase(device_info)
{
}

Device::Device(const Location& location)
  : DeviceBase(location)
{
}

Device::Device(DeviceBase&& other)
  : DeviceBase(std::move(other))
{
}

error_t Device::queryModel(std::string& model)
{
  rapidjson::Document request = session_->createEmptyRequestObject(), response;
  rapidjson::Document::AllocatorType& allocator = request.GetAllocator();
  request["method"].SetString("device/queryInfo");
  request.AddMember("params",
                    rapidjson::Value().SetObject()
                      .AddMember("entry", "identity.model", allocator), allocator);

  error_t result = session_->executeCommand(std::move(request), response);

  if (result == error_t::no_error)
    model = response["result"].GetString();

  return result;
}

error_t Device::querySerial(std::string& serial)
{
  rapidjson::Document request = session_->createEmptyRequestObject(), response;
  rapidjson::Document::AllocatorType& allocator = request.GetAllocator();
  request["method"].SetString("device/queryInfo");
  request.AddMember("params",
                    rapidjson::Value().SetObject()
                      .AddMember("entry", "identity.serial", allocator), allocator);

  error_t result = session_->executeCommand(std::move(request), response);

  if (result == error_t::no_error)
    serial = response["result"].GetString();

  return result;
}

error_t Device::queryFirmwareVersion(std::string& firmware_version)
{
  rapidjson::Document request = session_->createEmptyRequestObject(), response;
  rapidjson::Document::AllocatorType& allocator = request.GetAllocator();
  request["method"].SetString("device/queryInfo");
  request.AddMember("params",
                    rapidjson::Value().SetObject()
                      .AddMember("entry", "version.firmware", allocator), allocator);

  error_t result = session_->executeCommand(std::move(request), response);

  if (result == error_t::no_error)
    firmware_version = response["result"].GetString();

  return result;
}

error_t Device::queryHardwareVersion(std::string& hardware_version)
{
  rapidjson::Document request = session_->createEmptyRequestObject(), response;
  rapidjson::Document::AllocatorType& allocator = request.GetAllocator();
  request["method"].SetString("device/queryInfo");
  request.AddMember("params",
                    rapidjson::Value().SetObject()
                      .AddMember("entry", "version.hardware", allocator), allocator);

  error_t result = session_->executeCommand(std::move(request), response);

  if (result == error_t::no_error)
    hardware_version = response["result"].GetString();

  return result;
}

error_t Device::queryState(std::string& state)
{
  rapidjson::Document request = session_->createEmptyRequestObject(), response;
  rapidjson::Document::AllocatorType& allocator = request.GetAllocator();
  request["method"].SetString("device/queryInfo");
  request.AddMember("params",
                    rapidjson::Value().SetObject()
                      .AddMember("entry", "status.state", allocator), allocator);

  error_t result = session_->executeCommand(std::move(request), response);

  if (result == error_t::no_error)
    state = response["result"].GetString();

  return result;
}

error_t Device::readTimestamp(uint32_t& timestamp)
{
  rapidjson::Document request = session_->createEmptyRequestObject(), response;
  request["method"].SetString("device/readTimestamp");
  error_t result = session_->executeCommand(std::move(request), response);
  if (result == error_t::no_error)
    timestamp = (uint32_t)response["result"].GetInt64();
  return result;
}

error_t Device::resetTimestamp()
{
  rapidjson::Document request = session_->createEmptyRequestObject(), response;
  request["method"].SetString("device/resetTimestamp");
  error_t result = session_->executeCommand(std::move(request), response);
  return result;
}

error_t Device::startMeasurement()
{
  rapidjson::Document request = session_->createEmptyRequestObject(), response;
  request["method"].SetString("scan/startMeasurement");
  error_t result = session_->executeCommand(std::move(request), response);
  return result;
}

error_t Device::stopMeasurement()
{
  rapidjson::Document request = session_->createEmptyRequestObject(), response;
  request["method"].SetString("scan/stopMeasurement");
  error_t result = session_->executeCommand(std::move(request), response);
  return result;
}

error_t Device::startStreaming()
{
  rapidjson::Document request = session_->createEmptyRequestObject(), response;
  request["method"].SetString("scan/startStreaming");
  error_t result = session_->executeCommand(std::move(request), response);
  return result;
}

error_t Device::stopStreaming()
{
  rapidjson::Document request = session_->createEmptyRequestObject(), response;
  request["method"].SetString("scan/stopStreaming");
  error_t result = session_->executeCommand(std::move(request), response);
  return result;
}

error_t Device::readScanBlock(ScanBlock& scan_block)
{
  rapidjson::Document notification;
  error_t result = session_->pollForScanData(notification);

  if (result == error_t::no_error) {
    scan_block.block_id = notification["params"]["block"].GetInt();
    scan_block.timestamp = (uint32_t)notification["params"]["timestamp"].GetInt64();
    scan_block.layers.resize(notification["params"]["layers"].Size());
    for (size_t i = 0; i < scan_block.layers.size(); i++) {
      const rapidjson::Value& layer = notification["params"]["layers"][i];
      if (layer.IsNull())
        continue;

      const rapidjson::Value& ranges = layer["ranges"];
      if (!ranges.IsNull()) {
        int decoded_length = Utility::CalculateBase64DecodedLength(ranges.GetString(), ranges.GetStringLength());
        scan_block.layers[i].ranges.resize(decoded_length / sizeof(uint16_t));
        Utility::Base64Decode(ranges.GetString(), ranges.GetStringLength(), (uint8_t*)&scan_block.layers[i].ranges[0]);
      }
      const rapidjson::Value& intensities = layer["intensities"];
      if (!intensities.IsNull()) {
        int decoded_length = Utility::CalculateBase64DecodedLength(intensities.GetString(), intensities.GetStringLength());
        scan_block.layers[i].intensities.resize(decoded_length);
        Utility::Base64Decode(intensities.GetString(), intensities.GetStringLength(), &scan_block.layers[i].intensities[0]);
      }
    }
  }

  return result;
}

error_t Device::getNetworkAddress(in_addr_t& address)
{
  rapidjson::Document request = session_->createEmptyRequestObject(), response;
  rapidjson::Document::AllocatorType& allocator = request.GetAllocator();
  request["method"].SetString("settings/get");
  request.AddMember("params",
                    rapidjson::Value().SetObject()
                      .AddMember("entry", "connectivity.network.ipv4.address", allocator), allocator);

  error_t result = session_->executeCommand(std::move(request), response);

  if (result == error_t::no_error)
    address = htonl(asio::ip::address_v4::from_string(response["result"].GetString()).to_uint());

  return result;
}

error_t Device::getSubnetMask(in_addr_t& subnet)
{
  rapidjson::Document request = session_->createEmptyRequestObject(), response;
  rapidjson::Document::AllocatorType& allocator = request.GetAllocator();
  request["method"].SetString("settings/get");
  request.AddMember("params",
                    rapidjson::Value().SetObject()
                      .AddMember("entry", "connectivity.network.ipv4.subnet", allocator), allocator);

  error_t result = session_->executeCommand(std::move(request), response);

  if (result == error_t::no_error)
    subnet = htonl(asio::ip::address_v4::from_string(response["result"].GetString()).to_uint());

  return result;
}

error_t Device::getScanFrequency(int& frequency)
{
  rapidjson::Document request = session_->createEmptyRequestObject(), response;
  rapidjson::Document::AllocatorType& allocator = request.GetAllocator();
  request["method"].SetString("settings/get");
  request.AddMember("params",
                    rapidjson::Value().SetObject()
                      .AddMember("entry", "scan.frequency", allocator), allocator);

  error_t result = session_->executeCommand(std::move(request), response);

  if (result == error_t::no_error)
    frequency = response["result"].GetInt();

  return result;
}

error_t Device::setNetworkAddress(in_addr_t address)
{
  rapidjson::Document request = session_->createEmptyRequestObject(), response;
  rapidjson::Document::AllocatorType& allocator = request.GetAllocator();
  request["method"].SetString("settings/set");
  request.AddMember("params",
                    rapidjson::Value().SetObject()
                      .AddMember("entry", "connectivity.network.ipv4.address", allocator)
                      .AddMember("value", rapidjson::Value().SetString(
                        asio::ip::address_v4(ntohl(address)).to_string().c_str(), allocator), allocator),
                    allocator);

  error_t result = session_->executeCommand(std::move(request), response);

  return result;
}

error_t Device::setSubnetMask(in_addr_t subnet)
{
  rapidjson::Document request = session_->createEmptyRequestObject(), response;
  rapidjson::Document::AllocatorType& allocator = request.GetAllocator();
  request["method"].SetString("settings/set");
  request.AddMember("params",
                    rapidjson::Value().SetObject()
                      .AddMember("entry", "connectivity.network.ipv4.subnet", allocator)
                      .AddMember("value", rapidjson::Value().SetString(
                        asio::ip::address_v4(ntohl(subnet)).to_string().c_str(), allocator), allocator),
                    allocator);

  error_t result = session_->executeCommand(std::move(request), response);

  return result;
}

error_t Device::persistSettings()
{
  rapidjson::Document request = session_->createEmptyRequestObject(), response;
  rapidjson::Document::AllocatorType& allocator = request.GetAllocator();
  request["method"].SetString("settings/persist");

  error_t result = session_->executeCommand(std::move(request), response);

  return result;
}

}
