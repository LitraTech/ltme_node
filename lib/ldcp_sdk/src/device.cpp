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

error_t Device::open()
{
  error_t result = DeviceBase::open();
  if (result == error_t::no_error) {
    bool oob_enabled = false;
    if (isOobEnabled(oob_enabled) == error_t::no_error && oob_enabled) {
      in_port_t target_port = 0;
      if (getOobTargetPort(target_port) == error_t::no_error) {
        result = session_->enableOobTransport(NetworkLocation(htonl(INADDR_ANY), target_port));
        if (result != error_t::no_error)
          close();
      }
      else {
        close();
        result = error_t::unknown;
      }
    }
  }
  return result;
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

error_t Device::queryMotorFrequency(double& motor_frequency)
{
  rapidjson::Document request = session_->createEmptyRequestObject(), response;
  rapidjson::Document::AllocatorType& allocator = request.GetAllocator();
  request["method"].SetString("device/queryInfo");
  request.AddMember("params",
                    rapidjson::Value().SetObject()
                      .AddMember("entry", "status.motorFrequency", allocator), allocator);

  error_t result = session_->executeCommand(std::move(request), response);

  if (result == error_t::no_error)
    motor_frequency = response["result"].GetDouble();

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

error_t Device::readScanFrame(ScanFrame& scan_frame)
{
  ScanBlock scan_block;

  int index = 0;
  while (index < 8) {
    error_t result = readScanBlock(scan_block);
    if (result != error_t::no_error)
      return result;

    if (scan_block.block_id != index) {
      index = 0;
      continue;
    }
    else {
      int count = scan_block.layers[0].ranges.size();

      if (index == 0) {
        scan_frame.timestamp = scan_block.timestamp;
        scan_frame.layers.resize(1);
        scan_frame.layers[0].ranges.resize(count * 8);
        scan_frame.layers[0].intensities.resize(count * 8);
      }

      for (int i = 0; i < count; i++) {
        scan_frame.layers[0].ranges[index * count + i] = scan_block.layers[0].ranges[i];
        scan_frame.layers[0].intensities[index * count + i] = scan_block.layers[0].intensities[i];
      }

      index++;
    }
  }

  return error_t::no_error;
}

error_t Device::readScanBlock(ScanBlock& scan_block)
{
  rapidjson::Document notification;
  std::vector<uint8_t> oob_data;
  error_t result = session_->pollForScanBlock(notification, oob_data);

  if (result == error_t::no_error) {
    if (!notification.IsNull()) {
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
    else if (oob_data.size() > 0) {
      const OobPacket* oob_packet = reinterpret_cast<const OobPacket*>(oob_data.data());

      int block_length = oob_packet->count;
      const uint16_t* ranges = nullptr;
      const uint8_t* intensities = nullptr;

      switch (block_length) {
        case LASER_SCAN_BLOCK_LENGTH_10HZ:
          ranges = oob_packet->payload.data_10hz.ranges;
          intensities = oob_packet->payload.data_10hz.intensities;
          break;
        case LASER_SCAN_BLOCK_LENGTH_15HZ:
          ranges = oob_packet->payload.data_15hz.ranges;
          intensities = oob_packet->payload.data_15hz.intensities;
          break;
        case LASER_SCAN_BLOCK_LENGTH_20HZ:
          ranges = oob_packet->payload.data_20hz.ranges;
          intensities = oob_packet->payload.data_20hz.intensities;
          break;
        case LASER_SCAN_BLOCK_LENGTH_25HZ_30HZ:
          ranges = oob_packet->payload.data_25hz_30hz.ranges;
          intensities = oob_packet->payload.data_25hz_30hz.intensities;
          break;
      }

      scan_block.block_id = oob_packet->block_num;
      scan_block.timestamp = oob_packet->timestamp;
      scan_block.layers.resize(1);
      scan_block.layers[0].ranges.resize(block_length);
      scan_block.layers[0].intensities.resize(block_length);
      for (int i = 0; i < block_length; i++) {
        scan_block.layers[0].ranges[i] = ranges[i];
        scan_block.layers[0].intensities[i] = intensities[i];
      }
    }
  }

  return result;
}

error_t Device::getUserMacAddress(uint8_t address[])
{
  rapidjson::Document request = session_->createEmptyRequestObject(), response;
  rapidjson::Document::AllocatorType& allocator = request.GetAllocator();
  request["method"].SetString("settings/get");
  request.AddMember("params",
                    rapidjson::Value().SetObject()
                      .AddMember("entry", "connectivity.network.mac", allocator), allocator);

  error_t result = session_->executeCommand(std::move(request), response);

  if (result == error_t::no_error) {
    std::string value = response["result"].GetString();
    for (int i = 0; i < 6; i++)
      address[i] = std::stoi(value.substr(i * 3, 2), nullptr, 16);
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

error_t Device::isShadowFilterEnabled(bool& enabled)
{
  rapidjson::Document request = session_->createEmptyRequestObject(), response;
  rapidjson::Document::AllocatorType& allocator = request.GetAllocator();
  request["method"].SetString("settings/get");
  request.AddMember("params",
                    rapidjson::Value().SetObject()
                      .AddMember("entry", "filters.shadowFilter.enabled", allocator), allocator);

  error_t result = session_->executeCommand(std::move(request), response);
  if (result == error_t::no_error)
    enabled = response["result"].GetBool();

  return result;
}

error_t Device::getShadowFilterStrength(int& strength)
{
  rapidjson::Document request = session_->createEmptyRequestObject(), response;
  rapidjson::Document::AllocatorType& allocator = request.GetAllocator();
  request["method"].SetString("settings/get");
  request.AddMember("params",
                    rapidjson::Value().SetObject()
                      .AddMember("entry", "filters.shadowFilter.strength", allocator), allocator);

  error_t result = session_->executeCommand(std::move(request), response);
  if (result == error_t::no_error)
    strength = response["result"].GetInt();

  return result;
}

error_t Device::isOobEnabled(bool& enabled)
{
  rapidjson::Document request = session_->createEmptyRequestObject(), response;
  rapidjson::Document::AllocatorType& allocator = request.GetAllocator();
  request["method"].SetString("settings/get");
  request.AddMember("params",
                    rapidjson::Value().SetObject()
                      .AddMember("entry", "transport.oob.enabled", allocator), allocator);

  error_t result = session_->executeCommand(std::move(request), response);

  if (result == error_t::no_error)
    enabled = response["result"].GetBool();

  return result;
}

error_t Device::getOobTargetAddress(in_addr_t& address)
{
  rapidjson::Document request = session_->createEmptyRequestObject(), response;
  rapidjson::Document::AllocatorType& allocator = request.GetAllocator();
  request["method"].SetString("settings/get");
  request.AddMember("params",
                    rapidjson::Value().SetObject()
                      .AddMember("entry", "transport.oob.targetAddress", allocator), allocator);

  error_t result = session_->executeCommand(std::move(request), response);

  if (result == error_t::no_error)
    address = htonl(asio::ip::address_v4::from_string(response["result"].GetString()).to_uint());

  return result;
}

error_t Device::getOobTargetPort(in_port_t& port)
{
  rapidjson::Document request = session_->createEmptyRequestObject(), response;
  rapidjson::Document::AllocatorType& allocator = request.GetAllocator();
  request["method"].SetString("settings/get");
  request.AddMember("params",
                    rapidjson::Value().SetObject()
                      .AddMember("entry", "transport.oob.targetPort", allocator), allocator);

  error_t result = session_->executeCommand(std::move(request), response);

  if (result == error_t::no_error)
    port = htons(response["result"].GetInt());

  return result;
}

error_t Device::setUserMacAddress(const uint8_t address[])
{
  std::string value(17 + 1, '\0');
  std::snprintf(&value[0], value.length(), "%02X:%02X:%02X:%02X:%02X:%02X",
    address[0], address[1], address[2], address[3], address[4], address[5]);

  rapidjson::Document request = session_->createEmptyRequestObject(), response;
  rapidjson::Document::AllocatorType& allocator = request.GetAllocator();
  request["method"].SetString("settings/set");
  request.AddMember("params",
                    rapidjson::Value().SetObject()
                      .AddMember("entry", "connectivity.network.mac", allocator)
                      .AddMember("value", rapidjson::Value().SetString(
                        value.c_str(), allocator), allocator),
                    allocator);

  error_t result = session_->executeCommand(std::move(request), response);

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

error_t Device::setScanFrequency(int frequency)
{
  rapidjson::Document request = session_->createEmptyRequestObject(), response;
  rapidjson::Document::AllocatorType& allocator = request.GetAllocator();
  request["method"].SetString("settings/set");
  request.AddMember("params",
                    rapidjson::Value().SetObject()
                      .AddMember("entry", "scan.frequency", allocator)
                      .AddMember("value", frequency, allocator),
                    allocator);

  error_t result = session_->executeCommand(std::move(request), response);

  return result;
}

error_t Device::setShadowFilterEnabled(bool enabled)
{
  rapidjson::Document request = session_->createEmptyRequestObject(), response;
  rapidjson::Document::AllocatorType& allocator = request.GetAllocator();
  request["method"].SetString("settings/set");
  request.AddMember("params",
                    rapidjson::Value().SetObject()
                      .AddMember("entry", "filters.shadowFilter.enabled", allocator)
                      .AddMember("value", enabled, allocator),
                    allocator);

  error_t result = session_->executeCommand(std::move(request), response);

  return result;
}

error_t Device::setShadowFilterStrength(int strength)
{
  rapidjson::Document request = session_->createEmptyRequestObject(), response;
  rapidjson::Document::AllocatorType& allocator = request.GetAllocator();
  request["method"].SetString("settings/set");
  request.AddMember("params",
                    rapidjson::Value().SetObject()
                      .AddMember("entry", "filters.shadowFilter.strength", allocator)
                      .AddMember("value", strength, allocator),
                    allocator);

  error_t result = session_->executeCommand(std::move(request), response);

  return result;
}

error_t Device::setOobEnabled(bool enabled)
{
  rapidjson::Document request = session_->createEmptyRequestObject(), response;
  rapidjson::Document::AllocatorType& allocator = request.GetAllocator();
  request["method"].SetString("settings/set");
  request.AddMember("params",
                    rapidjson::Value().SetObject()
                      .AddMember("entry", "transport.oob.enabled", allocator)
                      .AddMember("value", enabled, allocator),
                    allocator);

  error_t result = session_->executeCommand(std::move(request), response);

  return result;
}

error_t Device::setOobTargetAddress(in_addr_t address)
{
  rapidjson::Document request = session_->createEmptyRequestObject(), response;
  rapidjson::Document::AllocatorType& allocator = request.GetAllocator();
  request["method"].SetString("settings/set");
  request.AddMember("params",
                    rapidjson::Value().SetObject()
                      .AddMember("entry", "transport.oob.targetAddress", allocator)
                      .AddMember("value", rapidjson::Value().SetString(
                        asio::ip::address_v4(ntohl(address)).to_string().c_str(), allocator), allocator),
                    allocator);

  error_t result = session_->executeCommand(std::move(request), response);

  return result;
}

error_t Device::setOobTargetPort(in_port_t port)
{
  rapidjson::Document request = session_->createEmptyRequestObject(), response;
  rapidjson::Document::AllocatorType& allocator = request.GetAllocator();
  request["method"].SetString("settings/set");
  request.AddMember("params",
                    rapidjson::Value().SetObject()
                      .AddMember("entry", "transport.oob.targetPort", allocator)
                      .AddMember("value", ntohs(port), allocator),
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

void Device::rebootToBootloader()
{
  rapidjson::Document request = session_->createEmptyRequestObject();
  request["method"].SetString("device/rebootToBootloader");
  session_->executeCommand(std::move(request));
}

}
