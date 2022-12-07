#ifndef LDCP_SDK_UTILITY_H_
#define LDCP_SDK_UTILITY_H_

#include <cinttypes>

namespace ldcp_sdk
{

class Utility
{
public:
  template <class InputIt>
  static uint16_t CalculateCRC16(InputIt begin, InputIt end)
  {
    uint16_t checksum = 0xFFFF;
    for (InputIt iter = begin; iter < end; iter++) {
      uint8_t value = checksum >> 8 ^ *iter;
      value ^= value >> 4;
      checksum = (checksum << 8) ^ (uint16_t)(value << 12) ^ (uint16_t)(value << 5) ^ (uint16_t)value;
    }
    return checksum;
  }

  static int CalculateBase64EncodedLength(int src_len)
  {
    return (src_len + 2) / 3 * 4;
  }

  static int Base64Encode(const uint8_t* src, int src_len, char* dest)
  {
    static const char basis_64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

    int i;

    char* p = dest;
    for (i = 0; i < src_len - 2; i += 3) {
      *p++ = basis_64[(src[i] >> 2) & 0x3F];
      *p++ = basis_64[((src[i] & 0x3) << 4) | ((int)(src[i + 1] & 0xF0) >> 4)];
      *p++ = basis_64[((src[i + 1] & 0xF) << 2) | ((int)(src[i + 2] & 0xC0) >> 6)];
      *p++ = basis_64[src[i + 2] & 0x3F];
    }
    if (i < src_len) {
      *p++ = basis_64[(src[i] >> 2) & 0x3F];
      if (i == (src_len - 1)) {
        *p++ = basis_64[((src[i] & 0x3) << 4)];
        *p++ = '=';
      }
      else {
        *p++ = basis_64[((src[i] & 0x3) << 4) | ((int)(src[i + 1] & 0xF0) >> 4)];
        *p++ = basis_64[((src[i + 1] & 0xF) << 2)];
      }
      *p++ = '=';
    }

    return p - dest;
  }

  static int CalculateBase64DecodedLength(const char* src, int src_len)
  {
    int padding_length = 0;
    const char* p = src + src_len - 1;
    while (*p-- == '=')
      padding_length++;
    return (src_len * 3 / 4 - padding_length);
  }

  static int Base64Decode(const char* src, int src_len, uint8_t* dest)
  {
    static const unsigned char lut[256] = {
      64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
      64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
      64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 62, 64, 64, 64, 63,
      52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 64, 64, 64, 64, 64, 64,
      64,  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14,
      15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 64, 64, 64, 64, 64,
      64, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40,
      41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 64, 64, 64, 64, 64,
      64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
      64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
      64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
      64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
      64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
      64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
      64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
      64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64
    };

    int bytes_read = 0, bytes_written = 0;

    int value = 0, count = 0;
    while (bytes_read < src_len) {
      char c = src[bytes_read++];
      if (c == '=')
        break;
      else {
        value = value << 6 | lut[c];
        if (++count == 4) {
          dest[bytes_written++] = (value >> 16) & 0xFF;
          dest[bytes_written++] = (value >> 8) & 0xFF;
          dest[bytes_written++] = value & 0xFF;

          value = count = 0;
        }
      }
    }

    if (count == 3) {
      dest[bytes_written++] = (value >> 10) & 0xFF;
      dest[bytes_written++] = (value >> 2) & 0xFF;
    }
    else if (count == 2)
      dest[bytes_written++] = (value >> 4) & 0xFF;

    return bytes_written;
  }
};

}

#endif
