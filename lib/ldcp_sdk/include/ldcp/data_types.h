#ifndef LDCP_SDK_DATA_TYPES_H_
#define LDCP_SDK_DATA_TYPES_H_

#include <vector>
#include <cstdint>
#include <memory>

namespace ldcp_sdk
{

typedef enum {
  ANGULAR_FOV_270DEG,
  ANGULAR_FOV_360DEG
} angular_fov_t;

enum {
  INTENSITY_WIDTH_8BIT,
  INTENSITY_WIDTH_16BIT
};

class ScanBlock
{
public:
  class BlockData
  {
  public:
    std::vector<int> ranges;
    std::vector<int> intensities;
  };

  int block_index;
  int block_count;
  int block_length;
  unsigned int timestamp;
  angular_fov_t angular_fov;
  std::vector<BlockData> layers;
};

#pragma pack(push, 1)
struct OobPacketHeader
{
  uint16_t signature;
  uint16_t frame_index;
  uint8_t block_index;
  uint8_t block_count;
  uint16_t block_length;
  uint32_t timestamp;
  uint16_t checksum;
  union {
    struct {
      uint16_t intensity_width : 1;
    } payload_layout;
    struct {
      uint16_t : 8;
      uint16_t angular_fov : 1;
    };
  } flags;
};
#pragma pack(pop)

class ScanFrame
{
public:
  class FrameData
  {
  public:
    std::vector<int> ranges;
    std::vector<int> intensities;
  };

  unsigned int timestamp;
  angular_fov_t angular_fov;
  std::vector<FrameData> layers;
};

}

#endif
