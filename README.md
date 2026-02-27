# nmea_gps_node
A ROS2 node that reads NMEA GPS data from a serial port and publishes `NavSatFix` messages.

## Features
- Parses NMEA GGA and GSA sentences from GPS receivers
- Publishes `sensor_msgs/NavSatFix` messages on the `fix` topic
- Computes position covariance based on HDOP/VDOP and GPS quality
- Logs confidence radius and satellite count
- Configurable serial port and baud rate

## Parameters
- `port` (string): Serial port device (default: `/dev/ttyUSB0`)
- `baudrate` (int): Serial port baud rate (default: `115200`)

## Published Topics
- `fix` (`sensor_msgs/NavSatFix`): GPS fix with position, altitude, and covariance

## Requirements
- `pynmea2` - NMEA sentence parsing
- `pyserial` - Serial communication

## GPS Quality Levels
The node supports quality indicators from 0-8, including standard GPS fixes, DGPS, RTK, and INS/dead reckoning modes.

## Running the Node

```bash
ros2 run nmea_gps_node nmea_gps_node --ros-args -p port:=/dev/ttyUSB0 -p baudrate:=115200
```
