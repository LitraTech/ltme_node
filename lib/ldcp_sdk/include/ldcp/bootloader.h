#ifndef LDCP_SDK_BOOTLOADER_H_
#define LDCP_SDK_BOOTLOADER_H_

#include "ldcp/device_base.h"

namespace ldcp_sdk
{

class Bootloader : public DeviceBase
{
public:
  static const int HASH_LENGTH = 16;

public:
  Bootloader(const Location& location);
  Bootloader(DeviceBase&& other);

  error_t beginUpdate();
  error_t writeData(const uint8_t content[], int length);
  error_t endUpdate();
  error_t verifyHash(const uint8_t expected[], bool& passed);
  error_t commitUpdate();
};

}

#endif
