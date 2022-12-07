#ifndef LDCP_SDK_LOCATION_H_
#define LDCP_SDK_LOCATION_H_

#ifdef __linux__
#include <netinet/in.h>
#elif _WIN32
#include <WinSock2.h>
typedef ULONG in_addr_t;
typedef USHORT in_port_t;
#endif

#include <string>

namespace ldcp_sdk
{

class Location
{
public:
  virtual ~Location() = default;

  virtual std::string label() const = 0;

protected:
  Location() = default;
};

class NetworkLocation : public Location
{
public:
  NetworkLocation(in_addr_t address, in_port_t port);
  NetworkLocation(const NetworkLocation&) = default;

  NetworkLocation& operator=(const NetworkLocation&) = default;
  bool operator==(const NetworkLocation& other) const;

  in_addr_t address() const;
  in_port_t port() const;

  virtual std::string label() const;

private:
  in_addr_t address_;
  in_port_t port_;
};

}

#endif
