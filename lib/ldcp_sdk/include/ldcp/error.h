#ifndef LDCP_SDK_ERROR_H_
#define LDCP_SDK_ERROR_H_

namespace ldcp_sdk
{

enum error_t {
  no_error = 0,
  connection_refused,
  timed_out,
  address_in_use,
  protocol_error,
  not_supported,
  invalid_params,
  device_error,
  unknown
};

}

#endif
