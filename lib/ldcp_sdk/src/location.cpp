#include "ldcp/location.h"

namespace ldcp_sdk
{

NetworkLocation::NetworkLocation(in_addr_t address, in_port_t port)
  : address_(address)
  , port_(port)
{
}

bool NetworkLocation::operator==(const NetworkLocation& other) const
{
  return (address_ == other.address_) && (port_ == other.port_);
}

in_addr_t NetworkLocation::address() const
{
  return address_;
}

in_port_t NetworkLocation::port() const
{
  return port_;
}

}
