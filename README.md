ROS driver for LitraTech's latest generation of mechanical 2D LiDARs running LDCP (**L**iDAR **D**ata and **C**ontrol **P**rotocol). Supported models are:
* LTME-02A

# Build and Install

## Dependencies

* C++11 capable compiler: **[Required]**
* OpenSSL: *[Optional]* The firmware updater will not be built if OpenSSL development files are missing; other parts of the package are not affected.

## Build the Package

Clone or extract package source to your catkin workspace's `src` directory, then build the workspace:

```
cd ~/catkin_ws/src
git clone https://github.com/LitraTech/ltme_node.git
cd .. && catkin_make
```

Or if you only want the package itself to be built:

```
cd ~/catkin_ws
catkin_make --only-pkg-with-deps ltme_node
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

`~get_background_intensity_threshold` ([std_srvs/Empty](http://docs.ros.org/api/std_srvs/html/srv/Empty.html)): Returns currently active value of background intensity threshold. Before making a distance measurement, the sensor samples background light intensity and returns an invalid result if intensity value exceeds the threshold.

`~set_background_intensity_threshold` ([ltme_node/SetBackgroundIntensityThreshold]): Sets value for background intensity threshold. Valid values are between 0 and 4095 inclusive. Higher threshold value means toleration to more intense background light (at the cost of greater possibility of phantom echo), and a value of 4095 completely disables background light intensity check.

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

`~enforced_scan_frequency` (integer, default: 0) *[Optional]*: Enforce the device to scan at specified frequency. Valid values are: 0 (disable scan frequency enforcement), 10, 15, 20, 25 and 30. If this option is enabled (i.e., set to non-zero), and the device is running at a different frequency, then the enforced value will be set (and saved in non-volatile memory) as new scan frequency, and the device will be rebooted for once for the updated settings to take effect.

`~frame_id` (string, default: "laser") *[Optional]*: Frame ID of published LaserScan messages.

`~angle_min` and `~angle_max` (float, default: -2.356 and 2.356) *[Optional]*: Start and end angle of published laser scans (in radians). As LTME series devices have an FOV of 270 degrees, the minimum allowed value for `angle_min` is -2.356 (about -3 * pi / 4), and the maximum allowed value for `angle_max` is 2.356 (about 3 * pi / 4).

`~angle_excluded_min` and `~angle_excluded_max` (float, default: -3.142 and -3.142) *[Optional]*: Range of angle (in radians) for which data should be excluded from published laser scans. Leave these two parameters commented out if a full 270-degree FOV is desired.

`~range_min` and `~range_max` (float, default: 0.05 and 30) *[Optional]*: Minimum and maximum range value of published laser scans. Range values out of these bounds should be ignored.

`~average_factor` (integer, default: 1) *[Optional]*: Number of neighboring measurements to be combined and averaged. Only integers &ge; 1 and &le; 8 are allowed. Averaging reduces jitter but angular resolution will also decrease by the same factor.

`~background_intensity_threshold` (integer, default: 256) *[Optional]*: Threshold value of tolerated background light intensity. Only integers &ge; 0 and &le; 4095 are allowed.

# Utilities

## update_firmware

Connects to the device specified and updates its firmware with provided file. Command syntax is as follows:

```
update_firmware <device address> <firmware file>
```

For example, if the device is at `192.168.10.160`, and latest firmware is stored in file `firmware.bin`, then the following command will do:

```
update_firmware 192.168.10.160 firmware.bin
```

*Note 1: LDCP only allows one simultaneous connection to device. Before running the updater, please make sure no other clients are connected. If the driver is currently active, its `~quit_driver` service can be called to close the connection.*

*Note 2: When updating device's firmware, it must be rebooted to run bootloader. For LTME-02A, IP address in bootloader mode is fixed to be 192.168.10.161, regardless of how you've set its address in normal mode. For the updater to successfully connect to bootloader, host computer's network interface must be configured with an address in the 192.168.10.x range; otherwise, it will not be able to reach the bootloader, and you'll have to manually power cycle the device to recover from bootloader mode.*
