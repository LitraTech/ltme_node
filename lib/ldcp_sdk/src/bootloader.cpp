#include "ldcp/bootloader.h"
#include "ldcp/session.h"
#include "ldcp/utility.h"

namespace ldcp_sdk
{

Bootloader::Bootloader(const Location& location)
  : DeviceBase(location)
{
}

Bootloader::Bootloader(DeviceBase&& other)
  : DeviceBase(std::move(other))
{
}

error_t Bootloader::beginUpdate()
{
  rapidjson::Document request = session_->createEmptyRequestObject(), response;
  request["method"].SetString("firmware/beginUpdate");
  error_t result = session_->executeCommand(std::move(request), response);
  return result;
}

error_t Bootloader::writeData(const uint8_t content[], int length)
{
  static std::string encoded;
  encoded.resize(Utility::CalculateBase64EncodedLength(length));
  Utility::Base64Encode(content, length, &encoded[0]);

  rapidjson::Document request = session_->createEmptyRequestObject(), response;
  rapidjson::Document::AllocatorType& allocator = request.GetAllocator();
  request["method"].SetString("firmware/writeData");
  request.AddMember("params",
                    rapidjson::Value().SetObject()
                      .AddMember("content", rapidjson::StringRef(encoded.c_str()), allocator),
                    allocator);

  error_t result = session_->executeCommand(std::move(request), response);

  return result;
}

error_t Bootloader::endUpdate()
{
  rapidjson::Document request = session_->createEmptyRequestObject(), response;
  request["method"].SetString("firmware/endUpdate");
  error_t result = session_->executeCommand(std::move(request), response);
  return result;
}

error_t Bootloader::verifyHash(const uint8_t expected[], bool& passed)
{
  static std::string encoded;
  encoded.resize(Utility::CalculateBase64EncodedLength(HASH_LENGTH));
  Utility::Base64Encode(expected, HASH_LENGTH, &encoded[0]);

  rapidjson::Document request = session_->createEmptyRequestObject(), response;
  rapidjson::Document::AllocatorType& allocator = request.GetAllocator();
  request["method"].SetString("firmware/verifyHash");
  request.AddMember("params",
                    rapidjson::Value().SetObject()
                      .AddMember("expected", rapidjson::StringRef(encoded.c_str()), allocator),
                    allocator);

  error_t result = session_->executeCommand(std::move(request), response);
  if (result == error_t::no_error)
    passed = response["result"].GetBool();

  return result;
}

error_t Bootloader::commitUpdate()
{
  rapidjson::Document request = session_->createEmptyRequestObject(), response;
  request["method"].SetString("firmware/commitUpdate");
  error_t result = session_->executeCommand(std::move(request), response);
  return result;
}

}
