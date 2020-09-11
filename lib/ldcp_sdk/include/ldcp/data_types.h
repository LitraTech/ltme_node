#ifndef LDCP_SDK_DATA_TYPES_H_
#define LDCP_SDK_DATA_TYPES_H_

#include <vector>
#include <cstdint>
#include <memory>

namespace ldcp_sdk
{

class ScanBlock
{
public:
  class BlockData
  {
  public:
    std::vector<uint16_t> ranges;
    std::vector<uint8_t> intensities;
  };

  uint8_t block_id;
  uint32_t timestamp;
  std::vector<BlockData> layers;
};

const int LASER_SCAN_BLOCK_LENGTH_10HZ = 288;
const int LASER_SCAN_BLOCK_LENGTH_15HZ = 192;
const int LASER_SCAN_BLOCK_LENGTH_20HZ = 144;
const int LASER_SCAN_BLOCK_LENGTH_25HZ_30hz = 96;

#pragma pack(push, 1)
struct OobPacket
{
  uint16_t signature;
  uint16_t frame_num;
  uint8_t block_num;
  uint8_t flags;
  uint16_t count;
  uint32_t timestamp;
  uint16_t checksum;
  uint16_t reserved;
  union {
    struct {
      uint16_t ranges[LASER_SCAN_BLOCK_LENGTH_10HZ];
      uint8_t intensities[LASER_SCAN_BLOCK_LENGTH_10HZ];
    } payload_10hz;
    struct {
      uint16_t ranges[LASER_SCAN_BLOCK_LENGTH_15HZ];
      uint8_t intensities[LASER_SCAN_BLOCK_LENGTH_15HZ];
    } payload_15hz;
    struct {
      uint16_t ranges[LASER_SCAN_BLOCK_LENGTH_20HZ];
      uint8_t intensities[LASER_SCAN_BLOCK_LENGTH_20HZ];
    } payload_20hz;
    struct {
      uint16_t ranges[LASER_SCAN_BLOCK_LENGTH_25HZ_30hz];
      uint8_t intensities[LASER_SCAN_BLOCK_LENGTH_25HZ_30hz];
    } payload_25hz_30hz;
  };
};
#pragma pack(pop)

class ScanFrame
{
public:
  class FrameData
  {
  public:
    std::vector<uint16_t> ranges;
    std::vector<uint8_t> intensities;
  };

  uint32_t timestamp;
  std::vector<FrameData> layers;
};

}

#endif
