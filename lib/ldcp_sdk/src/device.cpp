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

  int expected_block_index = 0;
  int block_count = INT_MAX, block_length = 0;
  while (expected_block_index < block_count) {
    error_t result = readScanBlock(scan_block);
    if (result != error_t::no_error)
      return result;

    if (scan_block.block_index != expected_block_index) {
      expected_block_index = 0;
      continue;
    }
    else {
      if (expected_block_index == 0) {
        block_count = scan_block.block_count;
        block_length = scan_block.block_length;

        scan_frame.timestamp = scan_block.timestamp;
        scan_frame.angular_fov = scan_block.angular_fov;
        scan_frame.layers.resize(1);
        scan_frame.layers[0].ranges.resize(block_count * block_length);
        scan_frame.layers[0].intensities.resize(block_count * block_length);
      }

      for (int i = 0; i < block_length; i++) {
        scan_frame.layers[0].ranges[expected_block_index * block_length + i] =
            scan_block.layers[0].ranges[i];
        scan_frame.layers[0].intensities[expected_block_index * block_length + i] =
            scan_block.layers[0].intensities[i];
      }

      expected_block_index++;
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
      scan_block.block_index = notification["params"]["block"].GetInt();
      scan_block.block_count = 8;
      scan_block.timestamp = (uint32_t)notification["params"]["timestamp"].GetInt64();
      scan_block.angular_fov = ANGULAR_FOV_270DEG;
      scan_block.layers.resize(notification["params"]["layers"].Size());
      for (size_t i = 0; i < scan_block.layers.size(); i++) {
        const rapidjson::Value& layer = notification["params"]["layers"][i];
        if (layer.IsNull())
          continue;

        std::vector<uint8_t> decode_buffer;
        const rapidjson::Value& ranges = layer["ranges"];
        if (!ranges.IsNull()) {
          int byte_count = Utility::CalculateBase64DecodedLength(ranges.GetString(), ranges.GetStringLength());
          if (decode_buffer.size() < byte_count)
            decode_buffer.resize(byte_count);
          Utility::Base64Decode(ranges.GetString(), ranges.GetStringLength(), &decode_buffer[0]);
          scan_block.block_length = byte_count / sizeof(uint16_t);
          scan_block.layers[i].ranges.resize(scan_block.block_length);
          for (int j = 0; j < scan_block.block_length; j++)
            scan_block.layers[i].ranges[j] = ((uint16_t*)&decode_buffer[0])[j];
        }
        const rapidjson::Value& intensities = layer["intensities"];
        if (!intensities.IsNull()) {
          int byte_count = Utility::CalculateBase64DecodedLength(intensities.GetString(), intensities.GetStringLength());
          if (decode_buffer.size() < byte_count)
            decode_buffer.resize(byte_count);
          Utility::Base64Decode(intensities.GetString(), intensities.GetStringLength(), &decode_buffer[0]);
          scan_block.layers[i].intensities.resize(byte_count);
          for (int j = 0; j < byte_count; j++)
            scan_block.layers[i].intensities[j] = decode_buffer[j];
        }
      }
    }
    else if (oob_data.size() > 0) {
      const OobPacketHeader* oob_packet_header = reinterpret_cast<const OobPacketHeader*>(
          oob_data.data());

      scan_block.block_index = oob_packet_header->block_index;
      scan_block.block_count = (oob_packet_header->block_count != 0) ? oob_packet_header->block_count : 8;
      scan_block.block_length = oob_packet_header->block_length;
      scan_block.timestamp = oob_packet_header->timestamp;
      scan_block.angular_fov = (oob_packet_header->flags.angular_fov == 0) ?
        ANGULAR_FOV_270DEG : ANGULAR_FOV_360DEG;

      scan_block.layers.resize(1);
      scan_block.layers[0].ranges.resize(oob_packet_header->block_length);
      scan_block.layers[0].intensities.resize(oob_packet_header->block_length);
      for (int i = 0; i < oob_packet_header->block_length; i++) {
        if (oob_packet_header->flags.payload_layout.intensity_width == INTENSITY_WIDTH_8BIT) {
          const uint16_t* ranges = (uint16_t*)(oob_packet_header + 1);
          const uint8_t* intensities = (uint8_t*)(ranges + oob_packet_header->block_length);
          scan_block.layers[0].ranges[i] = ranges[i];
          scan_block.layers[0].intensities[i] = intensities[i];
        }
        else if (oob_packet_header->flags.payload_layout.intensity_width == INTENSITY_WIDTH_16BIT) {
          const uint16_t* ranges = (uint16_t*)(oob_packet_header + 1);
          const uint16_t* intensities = ranges + oob_packet_header->block_length;
          scan_block.layers[0].ranges[i] = ranges[i];
          scan_block.layers[0].intensities[i] = intensities[i];
        }
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

error_t Device::getHostName(std::string& host_name)
{
  rapidjson::Document request = session_->createEmptyRequestObject(), response;
  rapidjson::Document::AllocatorType& allocator = request.GetAllocator();
  request["method"].SetString("settings/get");
  request.AddMember("params",
                    rapidjson::Value().SetObject()
                      .AddMember("entry", "connectivity.network.hostName", allocator), allocator);

  error_t result = session_->executeCommand(std::move(request), response);

  if (result == error_t::no_error)
    host_name = response["result"].GetString();

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

error_t Device::getOobAutoStartStreaming(bool& enabled)
{
  rapidjson::Document request = session_->createEmptyRequestObject(), response;
  rapidjson::Document::AllocatorType& allocator = request.GetAllocator();
  request["method"].SetString("settings/get");
  request.AddMember("params",
                    rapidjson::Value().SetObject()
                      .AddMember("entry", "transport.oob.autoStartStreaming", allocator), allocator);

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

error_t Device::setHostName(const std::string& host_name)
{
  rapidjson::Document request = session_->createEmptyRequestObject(), response;
  rapidjson::Document::AllocatorType& allocator = request.GetAllocator();
  request["method"].SetString("settings/set");
  request.AddMember("params",
                    rapidjson::Value().SetObject()
                      .AddMember("entry", "connectivity.network.hostName", allocator)
                      .AddMember("value", rapidjson::Value().SetString(
                        host_name.c_str(), allocator), allocator),
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

error_t Device::setOobAutoStartStreaming(bool enabled)
{
  rapidjson::Document request = session_->createEmptyRequestObject(), response;
  rapidjson::Document::AllocatorType& allocator = request.GetAllocator();
  request["method"].SetString("settings/set");
  request.AddMember("params",
                    rapidjson::Value().SetObject()
                      .AddMember("entry", "transport.oob.autoStartStreaming", allocator)
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

error_t Device::setReceiverSensitivityBoost(int sensitivity_boost)
{
  int receiver_sensitivity_backup_value = 0;
  error_t result = getReceiverSensitivityBackupValue(receiver_sensitivity_backup_value);
  if (result != error_t::no_error)
    return result;

  if (receiver_sensitivity_backup_value == 4095) {
    error_t result = getReceiverSensitivityValue(receiver_sensitivity_backup_value);
    if (result != error_t::no_error)
      return result;
    result = setReceiverSensitivityBackupValue(receiver_sensitivity_backup_value);
    if (result != error_t::no_error)
      return result;
  }

  return setReceiverSensitivityValue(receiver_sensitivity_backup_value + sensitivity_boost);
}

error_t Device::getReceiverSensitivityBackupValue(int& sensitivity_backup_value)
{
  rapidjson::Document request = session_->createEmptyRequestObject(), response;
  rapidjson::Document::AllocatorType& allocator = request.GetAllocator();
  request["method"].SetString("calibration/get");
  request.AddMember("params",
                    rapidjson::Value().SetObject()
                      .AddMember("entry", "rangefinder.intensityThresholds.m2l", allocator), allocator);
  request.AddMember("vendor", true, allocator);

  error_t result = session_->executeCommand(std::move(request), response);

  if (result == error_t::no_error)
    sensitivity_backup_value = response["result"].GetInt();

  return result;
}

error_t Device::setReceiverSensitivityBackupValue(int sensitivity_backup_value)
{
  rapidjson::Document request = session_->createEmptyRequestObject(), response;
  rapidjson::Document::AllocatorType& allocator = request.GetAllocator();
  request["method"].SetString("calibration/set");
  request.AddMember("params",
                    rapidjson::Value().SetObject()
                      .AddMember("entry", "rangefinder.intensityThresholds.m2l", allocator)
                      .AddMember("value", sensitivity_backup_value, allocator),
                    allocator);
  request.AddMember("vendor", true, allocator);

  error_t result = session_->executeCommand(std::move(request), response);

  return result;
}

error_t Device::getReceiverSensitivityValue(int& sensitivity_value)
{
  rapidjson::Document request = session_->createEmptyRequestObject(), response;
  rapidjson::Document::AllocatorType& allocator = request.GetAllocator();
  request["method"].SetString("settings/get");
  request.AddMember("params",
                    rapidjson::Value().SetObject()
                      .AddMember("entry", "APD.voltageOffset", allocator), allocator);
  request.AddMember("vendor", true, allocator);

  error_t result = session_->executeCommand(std::move(request), response);
  if (result == error_t::no_error)
    sensitivity_value = response["result"].GetInt();

  return result;
}

error_t Device::setReceiverSensitivityValue(int sensitivity_value)
{
  rapidjson::Document request = session_->createEmptyRequestObject(), response;
  rapidjson::Document::AllocatorType& allocator = request.GetAllocator();
  request["method"].SetString("settings/set");
  request.AddMember("params",
                    rapidjson::Value().SetObject()
                      .AddMember("entry", "APD.voltageOffset", allocator)
                      .AddMember("value", sensitivity_value, allocator),
                    allocator);
  request.AddMember("vendor", true, allocator);

  error_t result = session_->executeCommand(std::move(request), response);

  return result;
}

}
