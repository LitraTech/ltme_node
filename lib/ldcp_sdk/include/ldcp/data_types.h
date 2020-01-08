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

class Geometry
{
public:
  virtual ~Geometry() = default;
};

class Polygon : public Geometry
{
public:
  std::vector<std::pair<double, double>> vertices;
};

class Field
{
public:
  Field() = default;

  Field(Field&& other)
  {
    geometry.reset(other.geometry.release());
  }

  std::unique_ptr<Geometry> geometry;
};

class FieldSet
{
public:
  std::vector<Field> fields;
};

}

#endif
