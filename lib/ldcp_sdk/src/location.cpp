#include "ldcp/location.h"

#include "asio.hpp"

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

std::string NetworkLocation::label() const
{
  return asio::ip::address_v4(ntohl(address_)).to_string() + ":" +
      std::to_string(ntohs(port_));
}

}
