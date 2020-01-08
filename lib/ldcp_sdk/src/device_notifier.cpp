#include "ldcp/device_notifier.h"

#ifdef __linux__

#include <avahi-client/client.h>
#include <avahi-client/lookup.h>

#include <avahi-common/simple-watch.h>

namespace ldcp_sdk
{

class NetworkDeviceNotifierLinux : public NetworkDeviceNotifier
{
public:
  NetworkDeviceNotifierLinux();

  virtual bool init();
  virtual void start();
  virtual void stop();

private:
  static void clientCallback(AvahiClient *s, AvahiClientState state, void* userdata);
  static void serviceBrowserCallback(AvahiServiceBrowser *b, AvahiIfIndex interface,
                                     AvahiProtocol protocol, AvahiBrowserEvent event,
                                     const char *name, const char *type,
                                     const char *domain, AvahiLookupResultFlags flags,
                                     void *userdata);
  static void serviceResolverCallback(AvahiServiceResolver *r, AvahiIfIndex interface,
                                      AvahiProtocol protocol, AvahiResolverEvent event,
                                      const char *name, const char *type,
                                      const char *domain, const char *host_name,
                                      const AvahiAddress *a, uint16_t port,
                                      AvahiStringList *txt, AvahiLookupResultFlags flags,
                                      void *userdata);

private:
  AvahiClient* client_;
  AvahiServiceBrowser* service_browser_;
  AvahiSimplePoll* simple_poll_;
};

NetworkDeviceNotifierLinux::NetworkDeviceNotifierLinux()
  : client_(NULL)
  , service_browser_(NULL)
  , simple_poll_(NULL)
{
}

bool NetworkDeviceNotifierLinux::init()
{
  simple_poll_ = avahi_simple_poll_new();
  if (!simple_poll_)
    goto FAIL_CLEAN_UP;
  client_ = avahi_client_new(avahi_simple_poll_get(simple_poll_), (AvahiClientFlags)0,
                             clientCallback, NULL, NULL);
  if (!client_)
    goto FAIL_CLEAN_UP;
  service_browser_ = avahi_service_browser_new(client_, AVAHI_IF_UNSPEC, AVAHI_PROTO_INET, "_ldcp._tcp",
                                               NULL, (AvahiLookupFlags)0, serviceBrowserCallback, this);
  if (!service_browser_)
    goto FAIL_CLEAN_UP;
  return true;

FAIL_CLEAN_UP:
  if (service_browser_)
    avahi_service_browser_free(service_browser_);
  if (client_)
    avahi_client_free(client_);
  if (simple_poll_)
    avahi_simple_poll_free(simple_poll_);
  return false;
}

void NetworkDeviceNotifierLinux::start()
{
  thread_ = std::thread([&]() { avahi_simple_poll_loop(simple_poll_); });
}

void NetworkDeviceNotifierLinux::stop()
{
  avahi_simple_poll_quit(simple_poll_);
  thread_.join();
}

void NetworkDeviceNotifierLinux::clientCallback(AvahiClient* s, AvahiClientState state, void* userdata)
{
  return;
}

void NetworkDeviceNotifierLinux::serviceBrowserCallback(AvahiServiceBrowser* b, AvahiIfIndex interface,
                                                        AvahiProtocol protocol, AvahiBrowserEvent event,
                                                        const char* name, const char* type,
                                                        const char* domain, AvahiLookupResultFlags flags,
                                                        void* userdata)
{
  NetworkDeviceNotifierLinux* notifier = (NetworkDeviceNotifierLinux*)userdata;
  switch (event) {
    case AVAHI_BROWSER_NEW:
      avahi_service_resolver_new(notifier->client_, interface, protocol, name,
                                 type, domain, AVAHI_PROTO_INET, (AvahiLookupFlags)0,
                                 serviceResolverCallback, notifier);
      break;
    case AVAHI_BROWSER_REMOVE:
      if (notifier->device_detach_handler_)
        notifier->device_detach_handler_(name);
      break;
    default:
      break;
  }
}

void NetworkDeviceNotifierLinux::serviceResolverCallback(AvahiServiceResolver* r, AvahiIfIndex interface,
                                                         AvahiProtocol protocol, AvahiResolverEvent event,
                                                         const char* name, const char* type,
                                                         const char* domain, const char* host_name,
                                                         const AvahiAddress* a, uint16_t port,
                                                         AvahiStringList* txt, AvahiLookupResultFlags flags,
                                                         void* userdata)
{
  NetworkDeviceNotifierLinux* notifier = (NetworkDeviceNotifierLinux*)userdata;
  switch (event) {
    case AVAHI_RESOLVER_FOUND: {
      DeviceInfo new_entry(name, NetworkLocation(a->data.ipv4.address, htons(port)));
      while (txt) {
        std::string key_value_pair = (char*)avahi_string_list_get_text(txt);
        size_t position = key_value_pair.find('=');
        if (position != std::string::npos) {
          int length = avahi_string_list_get_size(txt);
          std::string key = key_value_pair.substr(0, position);
          std::string value = key_value_pair.substr(position + 1, length - position);
          new_entry.metadata()[key] = value;
        }
        txt = avahi_string_list_get_next(txt);
      }
      if (notifier->device_attach_handler_)
        notifier->device_attach_handler_(new_entry);
      break;
    }
    default:
      break;
  }
  avahi_service_resolver_free(r);
}

}

#elif _WIN32

#include <dns_sd.h>

#include <algorithm>
#include <array>

namespace ldcp_sdk
{

struct mDNSRecord
{
  std::string service_name;
  std::string host_name;
  int port;
  std::string txt_record;
};

class NetworkDeviceNotifierWindows : public NetworkDeviceNotifier
{
public:
  NetworkDeviceNotifierWindows();

  virtual bool init();
  virtual void start();
  virtual void stop();

private:
  static void DNSSD_API serviceBrowseCallback(DNSServiceRef sdRef, DNSServiceFlags flags,
                                              uint32_t interfaceIndex, DNSServiceErrorType errorCode,
                                              const char *serviceName, const char *regtype,
                                              const char *replyDomain, void *context);
  static void DNSSD_API serviceResolveCallback(DNSServiceRef sdRef, DNSServiceFlags flags,
                                               uint32_t interfaceIndex, DNSServiceErrorType errorCode,
                                               const char *fullname, const char *hosttarget,
                                               uint16_t port, uint16_t txtLen,
                                               const unsigned char *txtRecord, void *context);
  static void DNSSD_API serviceGetAddrInfoCallback(DNSServiceRef sdRef, DNSServiceFlags flags,
                                                   uint32_t interfaceIndex, DNSServiceErrorType errorCode,
                                                   const char *hostname, const struct sockaddr *address,
                                                   uint32_t ttl, void *context);

private:
  bool active_flag_;
  std::list<DNSServiceRef> pending_requests_;
  FD_SET read_set_;
  std::list<mDNSRecord> mdns_records_;
};

NetworkDeviceNotifierWindows::NetworkDeviceNotifierWindows()
  : active_flag_(false)
{
}

bool NetworkDeviceNotifierWindows::init()
{
  DNSServiceRef request = NULL;
  DNSServiceErrorType error = DNSServiceBrowse(&request, 0, 0, "_ldcp._tcp", "",
                                               serviceBrowseCallback, this);
  if (error != 0)
    return false;
  else {
    pending_requests_.clear();
    pending_requests_.push_front(request);
    FD_ZERO(&read_set_);
    FD_SET(DNSServiceRefSockFD(request), &read_set_);

    return true;
  }
}

void NetworkDeviceNotifierWindows::start()
{
  thread_ = std::thread([&]() {
    active_flag_ = true;
    while (active_flag_) {
      FD_SET copyed_read_set_ = read_set_;
      struct timeval timeout = { 0, 1000 };

      int result = select(0, &copyed_read_set_, NULL, NULL, &timeout);
      if (result > 0) {
        std::list<DNSServiceRef>::iterator iter = pending_requests_.begin();
        while (iter != pending_requests_.end()) {
          DNSServiceRef current_request = *iter++;
          if (FD_ISSET(DNSServiceRefSockFD(current_request), &copyed_read_set_))
            DNSServiceProcessResult(current_request);
        }
      }
    }
  });
}

void NetworkDeviceNotifierWindows::stop()
{
  if (active_flag_) {
    active_flag_ = false;
    thread_.join();
  }
}

void NetworkDeviceNotifierWindows::serviceBrowseCallback(DNSServiceRef sdRef, DNSServiceFlags flags,
                                                         uint32_t interfaceIndex, DNSServiceErrorType errorCode,
                                                         const char* serviceName, const char* regtype,
                                                         const char* replyDomain, void* context)
{
  NetworkDeviceNotifierWindows* notifier = (NetworkDeviceNotifierWindows*)context;

  std::string id = serviceName;
  if (flags & kDNSServiceFlagsAdd) {
    DNSServiceRef request = NULL;
    DNSServiceErrorType error = DNSServiceResolve(&request, 0, interfaceIndex, serviceName,
                                                  regtype, replyDomain, serviceResolveCallback, context);
    if (error == 0) {
      notifier->pending_requests_.push_front(request);
      FD_SET(DNSServiceRefSockFD(request), &notifier->read_set_);
    }
  }
  else {
    if (notifier->device_detach_handler_)
      notifier->device_detach_handler_(id);
  }
}

void NetworkDeviceNotifierWindows::serviceResolveCallback(DNSServiceRef sdRef, DNSServiceFlags flags,
                                                          uint32_t interfaceIndex, DNSServiceErrorType errorCode,
                                                          const char* fullname, const char* hosttarget,
                                                          uint16_t port, uint16_t txtLen,
                                                          const unsigned char* txtRecord, void* context)
{
  NetworkDeviceNotifierWindows* notifier = (NetworkDeviceNotifierWindows*)context;

  DNSServiceRef request = NULL;
  DNSServiceErrorType result = DNSServiceGetAddrInfo(&request, kDNSServiceFlagsTimeout,
                                                     interfaceIndex, kDNSServiceProtocol_IPv4,
                                                     hosttarget, serviceGetAddrInfoCallback, context);
  if (result == 0) {
    std::string id(fullname, strchr(fullname, '.') - fullname);
    notifier->mdns_records_.push_back({ id, hosttarget, port, std::string((const char*)txtRecord, txtLen) });

    notifier->pending_requests_.push_front(request);
    FD_SET(DNSServiceRefSockFD(request), &notifier->read_set_);
  }

  if (!(flags & kDNSServiceFlagsMoreComing)) {
    notifier->pending_requests_.remove(sdRef);
    FD_CLR(DNSServiceRefSockFD(sdRef), &notifier->read_set_);
  }
}

void NetworkDeviceNotifierWindows::serviceGetAddrInfoCallback(DNSServiceRef sdRef, DNSServiceFlags flags,
                                                              uint32_t interfaceIndex, DNSServiceErrorType errorCode,
                                                              const char* hostname, const sockaddr* address,
                                                              uint32_t ttl, void* context)
{
  NetworkDeviceNotifierWindows* notifier = (NetworkDeviceNotifierWindows*)context;

  auto iter = std::find_if(notifier->mdns_records_.begin(), notifier->mdns_records_.end(), [hostname](const mDNSRecord& record) {
    return (record.host_name == hostname);
  });
  if (iter != notifier->mdns_records_.end()) {
    DeviceInfo new_entry(iter->service_name,
                         NetworkLocation(((const struct sockaddr_in*)address)->sin_addr.s_addr, iter->port));
    int count = TXTRecordGetCount(iter->txt_record.length(), iter->txt_record.c_str());
    for (int i = 0; i < count; i++) {
      std::array<char, 256> key_buf;
      uint8_t value_len = 0, *value_ptr = NULL;
      if (TXTRecordGetItemAtIndex(iter->txt_record.length(), iter->txt_record.c_str(), i,
                                  key_buf.size(), key_buf.data(), &value_len, (const void**)&value_ptr) == kDNSServiceErr_NoError) {
        if (value_len > 0)
          new_entry.metadata()[key_buf.data()] = std::string((char*)value_ptr, value_len);
      }
    }
    notifier->mdns_records_.erase(iter);
    if (notifier->device_attach_handler_)
      notifier->device_attach_handler_(new_entry);
  }

  if (!(flags & kDNSServiceFlagsMoreComing)) {
    notifier->pending_requests_.remove(sdRef);
    FD_CLR(DNSServiceRefSockFD(sdRef), &notifier->read_set_);
  }
}

}

#endif

namespace ldcp_sdk
{

NetworkDeviceNotifier& NetworkDeviceNotifier::instance()
{
  #ifdef __linux__
  static NetworkDeviceNotifierLinux instance;
  #elif _WIN32
  static NetworkDeviceNotifierWindows instance;
  #endif
  return instance;
}

}
