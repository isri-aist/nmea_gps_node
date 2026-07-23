# nmea_gps_node
A ROS2 node that reads NMEA GPS data from a serial port and publishes GPS fix messages.

## Features
- Parses NMEA GGA and GSA sentences from GPS receivers
- Publishes `sensor_msgs/NavSatFix` messages on the `fix` topic
- Publishes `gps_msgs/GPSFix` messages on the `fix_extended` topic
- Computes position covariance based on HDOP/VDOP and GPS quality
- Logs confidence radius and satellite count
- Configurable serial port and baud rate

## Parameters
- `port` (string): Serial port device (default: `/dev/ttyUSB0`)
- `baudrate` (int): Serial port baud rate (default: `115200`)
- `read_period` (float): Serial polling timer period in seconds (default: `0.02`)
- `serial_timeout` (float): Serial readline timeout in seconds (default: `0.01`)
- `max_nmea_lines_per_tick` (int): Maximum NMEA lines drained per timer tick (default: `50`)

## Published Topics
- `fix` (`sensor_msgs/NavSatFix`): GPS fix with position, altitude, and covariance
- `fix_extended` (`gps_msgs/GPSFix`): GPS fix with position, covariance, DOP values, satellite count, GPS quality status, and 95% error estimates

## Requirements
- `pynmea2` - NMEA sentence parsing
- `pyserial` - Serial communication
- `gps_msgs` - Extended ROS GPS messages, required for the `fix_extended` topic

## GPS Quality Levels
The node supports quality indicators from 0-8, including standard GPS fixes, DGPS, RTK, and INS/dead reckoning modes.

## Running the Node

```bash
ros2 run nmea_gps_node nmea_gps_node --ros-args -p port:=/dev/ttyUSB0 -p baudrate:=115200
```
