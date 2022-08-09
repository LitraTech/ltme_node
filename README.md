# Introduction
**ROS2 foxy** driver for LitraTech's latest generation of mechanical 2D LiDARs running LDCP (**L**iDAR **D**ata and **C**ontrol **P**rotocol). Supported models are:
* LTME-02A

## Differences
DO NOT SUPPORT UPDATE FIRMWARE

# Build and Install

## Dependencies
* ROS2 Foxy
* C++ 14

## Build the Package

Clone or extract package source to your catkin workspace's `src` directory, then build the workspace:

```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/TonyTnT/ltme_node.git -b ros2-foxy
cd .. && colcon build --symlink-install
```

# Nodes

## ltme_node

Reads measurement data (ranges & intensities) from connected device and publishes `LaserScan` messages. Additionally, this node exposes several services for other nodes to query information about the device and control its operation mode.

### Published Topics

`scan` ([sensor_msgs/LaserScan](http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html)): Laser scan measurements obtained from connected device. The default topic name can be changed with &lt;remap&gt; tag.

### Services

`~query_serial` ([std_srvs/Empty](http://docs.ros.org/api/std_srvs/html/srv/Empty.html)): Returns a [std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html) about device's serial number.

`~query_firmware_version` ([std_srvs/Empty](http://docs.ros.org/api/std_srvs/html/srv/Empty.html)): Returns a [std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html) about device's firmware version. The version is encoded as a 4-digit string; the leftmost and rightmost 2 digits represent major and minor versions respectively. E.g., "0202" corresponds to version 2.2.

`~request_hibernation` ([std_srvs/Empty](http://docs.ros.org/api/std_srvs/html/srv/Empty.html)): Make the device to enter standby mode. During standby the device will turn off its motor and laser to prevent wearing and save power; no `LaserScan` messages will be published until it's brought out of standby.

`~request_wake_up` ([std_srvs/Empty](http://docs.ros.org/api/std_srvs/html/srv/Empty.html)): Exit standby mode and resume normal operation.

`~quit_driver` ([std_srvs/Empty](http://docs.ros.org/api/std_srvs/html/srv/Empty.html)): Instructs the driver to quit its process. This service can be handy when updating device firmware, as LDCP only allows one connection at a time, and the driver has to be terminated before running the updater.

### Parameters

`~device_model` (string) **[Required]**: Model name of the target device. Supported models are:
- LTME-02A

`~device_address` (string) **[Required]**: Address of the target device:
- For LTME-02A, this parameter should be an IP address with optional port number, e.g., `192.168.10.160` or `192.168.10.160:2105`

`~enforced_transport_mode` (string, default: "none") *[Optional]*: Enforce specific transport mode used by the device to stream measurement data. If set to "normal" or "oob", device settings will be changed to selected mode and saved in non-volatile memory in case of a mismatch. Available options are:
- `none`: don't enforce any mode and use device's current configuration
- `normal`: enforce normal (in-band) mode, i.e. connection for command interactions is reused for data streaming
- `oob`: enforce out-of-band mode, where data are streamed in binary form through a dedicated channel

*Note: LTME-02A supports OOB mode since firmware version 0201*

> **Some background**:
> <p>In order to support communication interfaces lacking multiplexed connections (serial interfaces in particular), LDCP was initially designed around a shared connection model. In this model, command interactions and measurement data are encapsulated in JSON-style messages and transmitted through the same underlying connection. For the receiver part, it has to parse received JSON messages and extract encoded binary data, which may have adverse impact on performance for systems under heavy load.
> <p>To solve this problem, later iteration of LDCP introduced a new out-of-band (OOB) transport mode. For supported interfaces, this mode creates a separate (out-of-band) communication channel and uses this channel to stream measurements in binary form, making it much easier for the receiver to process. For example, if OOB mode is enabled for Ethernet capable devices (such as LTME-02A), command interactions will be transmitted over a TCP connection, while measurement data are streamed through a dedicated UDP channel.

`~frame_id` (string, default: "laser") *[Optional]*: Frame ID of published LaserScan messages.

`~invert_frame` (bool, default: "false") *[Optional]*: If this option is enabled, published LaserScan messages will have their X and Z axes inverted. This is mostly useful when the device is mounted upside down, as it effectively undos the inversion created by the mounting, and makes it look like the scans are from a device installed in a normal, upward direction.

`~scan_frequency_override` (integer, default: 0) *[Optional]*: LTME series devices can be configured to have different scan frequencies, ranging from 10 Hz to 30 Hz with 5 Hz increment. `ltme_node` automatically queries device for its scan frequency upon connection and setup `LaserScan` parameters accordingly, so normally you don't need this parameter and should leave it commented out. If for some reason this doesn't work (e.g., a device with outdated firmware), this parameter can be used to override automatic detection and manually specify a correct frequency value.

`~angle_min` and `~angle_max` (float, default: -2.356 and 2.356): Start and end angle of published laser scans (in radians). As LTME series devices have an FOV of 270 degrees, the minimum allowed value for `angle_min` is -2.356 (about -3 * pi / 4), and the maximum allowed value for `angle_max` is 2.356 (about 3 * pi / 4).

`~angle_excluded_min` and `~angle_excluded_max` (float, default: -3.142 and -3.142): Range of angle (in radians) for which data should be excluded from published laser scans. Leave these two parameters commented out if a full 270-degree FOV is desired.

`~range_min` and `~range_max` (float, default: 0.05 and 30): Minimum and maximum range value of published laser scans. Range values out of these bounds should be ignored.

`~average_factor` (integer, default: 1): Number of neighboring measurements to be combined and averaged. Only integers &ge; 1 and &le; 8 are allowed. Averaging reduces jitter but angular resolution will also decrease by the same factor.

`~shadow_filter_strength` (integer, default: 50): Indicates how data post-processing stage will filter scan artifacts caused by veiling effect. This effect usually occurs when the edge of an object is scanned and manifests itself in the form of a series of points spreading out along the ray direction. Only integers &ge; 0 and &le; 100 are allowed. Larger value leads to more aggressive filtering.

# ~~Utilities~~

## ~~update_firmware~~

~~Connects to the device specified and updates its firmware with provided file. Command syntax is as follows:~~

```
update_firmware <device address> <firmware file>
```

~~For example, if the device is at `192.168.10.160`, and latest firmware is stored in file `firmware.bin`, then the following command will do:~~


```
update_firmware 192.168.10.160 firmware.bin
```

*~~Note 1: LDCP only allows one simultaneous connection to device. Before running the updater, please make sure no other clients are connected. If the driver is currently active, its `~quit_driver` service can be called to close the connection.~~*

*~~Note 2: When updating device's firmware, it must be rebooted to run bootloader. For LTME-02A, IP address in bootloader mode is fixed to be 192.168.10.161, regardless of how you've set its address in normal mode. For the updater to successfully connect to bootloader, host computer's network interface must be configured with an address in the 192.168.10.x range; otherwise, it will not be able to reach the bootloader, and you'll have to manually power cycle the device to recover from bootloader mode.~~*
