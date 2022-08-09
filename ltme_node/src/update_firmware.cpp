#include <iostream>
#include <fstream>

#include "ldcp/device.h"
#include "ldcp/bootloader.h"

#include <arpa/inet.h>
#include <openssl/md5.h>

static const int FIRMWARE_BLOCK_SIZE = 512;

int main(int argc, char* argv[])
{
  if (argc != 3) {
    std::string path(argv[0]), file_name;
    auto index = path.find_last_of('/');
    file_name = path.substr((index != std::string::npos) ? index + 1 : 0);

    std::cout << "Usage: " << file_name << " <device address> <firmware file>" << std::endl;
    std::cout << "Example: " << file_name << " 192.168.10.160 firmware.bin" << std::endl;
    return 0;
  }

  std::string device_address = argv[1];
  std::string address_str;
  std::string port_str;

  auto position = device_address.find(':');
  address_str = device_address.substr(0, position);
  if (position == std::string::npos)
    port_str = "2105";
  else
    port_str = device_address.substr(position + 1);

  in_addr_t address = htonl(INADDR_NONE);
  in_port_t port = 0;
  try {
    address = inet_addr(address_str.c_str());
    if (address == htonl(INADDR_NONE))
      throw std::exception();
    port = htons(std::stoi(port_str));
  }
  catch (...) {
    std::cerr << "Invalid device address: " << device_address << std::endl;
    return -1;
  }

  std::string firmware_path = argv[2];
  std::ifstream firmware_file(firmware_path, std::ifstream::binary);
  if (!firmware_file.is_open()) {
    std::cerr << "Unable to open firmware file " << firmware_path << std::endl;
    return -1;
  }

  std::cout << "Connecting device at "
            << inet_ntoa({ address }) << ":" << std::to_string(ntohs(port))
            << "..." << std::flush;
  ldcp_sdk::NetworkLocation device_location(address, port);
  ldcp_sdk::Device device(device_location);
  if (device.open() != ldcp_sdk::no_error) {
    std::cerr << "failed" << std::endl;
    return -1;
  }
  std::cout << "done" << std::endl;

  bool wait_for_bring_up = true;

  std::string operation_mode;
  device.queryOperationMode(operation_mode);
  if (operation_mode == "normal") {
    std::cout << "Device is running in normal mode" << std::endl;
    std::cout << "Rebooting to bootloader..." << std::flush;
    device.rebootToBootloader();
    std::cout << "done" << std::endl;
  }
  else if (operation_mode == "bootloader") {
    wait_for_bring_up = false;
    std::cout << "Device is already in bootloader mode" << std::endl;
  }

  device.close();

  ldcp_sdk::NetworkLocation bootloader_location(
    inet_addr("192.168.10.161"), ntohs(2105));
  ldcp_sdk::Bootloader bootloader(bootloader_location);
  if (wait_for_bring_up) {
    bootloader.setTimeout(1000);

    std::cout << "Connecting bootloader at 192.168.10.161:2105" << std::flush;
    for (int i = 0; i < 16; i++) {
      std::cout << "." << std::flush;
      if (bootloader.open() == ldcp_sdk::no_error)
        break;
    }

    if (bootloader.isOpened())
      std::cout << "done" << std::endl;
    else {
      std::cout << "timed out" << std::endl;
      return -1;
    }
  }
  else {
    if (bootloader.open() != ldcp_sdk::no_error) {
      std::cout << "Failed to establish connection to bootloader" << std::endl;
      return -1;
    }
  }

  uint8_t md5_hash[MD5_DIGEST_LENGTH];
  MD5_CTX md5_context;

  bootloader.setTimeout(5000);
  try {
    std::cout << "Preparing device for firmware update..." << std::flush;
    if (bootloader.beginUpdate() != ldcp_sdk::no_error)
      throw std::runtime_error("");
    std::cout << "done" << std::endl;
    std::cout << "Writing firmware to device" << std::flush;

    MD5_Init(&md5_context);

    std::vector<char> buffer(FIRMWARE_BLOCK_SIZE, 0);
    while (!firmware_file.eof()) {
      firmware_file.read(buffer.data(), FIRMWARE_BLOCK_SIZE);
      int count = firmware_file.gcount();
      if (bootloader.writeData((const uint8_t*)buffer.data(), count) != ldcp_sdk::no_error)
        throw std::runtime_error("");
      MD5_Update(&md5_context, buffer.data(), count);
      std::cout << "." << std::flush;
    }
    std::cout << std::endl;
    if (bootloader.endUpdate() != ldcp_sdk::no_error)
      throw std::runtime_error("");
  }
  catch (...) {
    std::cerr << "Failed to write firmware data to device" << std::endl;
    return -1;
  }

  MD5_Final(md5_hash, &md5_context);

  bool passed = false;
  if (bootloader.verifyHash(md5_hash, passed) == ldcp_sdk::no_error &&
      passed) {
    bootloader.commitUpdate();
    bootloader.reboot();
    std::cout << "Firmware updated successfully" << std::endl;
  }
  else
    std::cout << "Firmware verification failed" << std::endl;

  bootloader.close();

  return 0;
}
