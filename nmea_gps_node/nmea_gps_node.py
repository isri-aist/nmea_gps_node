import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
import numpy as np

import serial
import pynmea2

QUALITY = ["0 - Fix not valid", "1 - GPS Fix", "2 - DGPS Fix", "3 - N/A", "4 - RTK Fix", "5 - RTK Float", "6 - INS Dead reckoning", "7 - Manual Input mode", "8 - Simulation mode"]

class GPSNode(Node):
    def __init__(self):
        super().__init__('nmea_gps_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value

        self.ser = serial.Serial(port, baud, timeout=1.0)

        self.pub = self.create_publisher(NavSatFix, 'fix', 10)
        self.timer = self.create_timer(0.1, self.read_gps)

        self.get_logger().info(f"GPS connected on {port} @ {baud}")
        self.GGARX = False
        self.GSARX = False

    def read_gps(self):
        line = self.ser.readline().decode(errors='ignore').strip()
        if not line.startswith('$'):
            return

        try:
            msg = pynmea2.parse(line)
            # print(type(msg))
        except pynmea2.ParseError:
            return
        
        if type(msg) == pynmea2.talker.GGA:
            # print(repr(msg))
            # print(msg.__getattr__)
            self.lat = msg.latitude
            self.lon = msg.longitude
            self.alt = msg.altitude
            self.gps_qual = msg.gps_qual
            self.num_sats = msg.num_sats
            
            # print(f"lat: {self.lat} \tlon: {self.lon} \talt: {self.alt} \tqual: {self.gps_qual}")
            self.GGARX = True

        if type(msg) == pynmea2.talker.GSA:
            self.gps_mode = msg.mode
            self.fixType = msg.mode_fix_type
            self.pdop = float(msg.pdop)
            self.hdop = float(msg.hdop)
            self.vdop = float(msg.vdop)

            # print(repr(msg))
            # print(f"gps mode: {self.gps_mode} \tfix type: {self.fixType} \tPDOP: {self.pdop} \tVDOP: {self.vdop} \tHDOP: {self.hdop}")
            self.GSARX = True

        if self.GGARX == True and self.GSARX == True:
            self.GGARX = False
            self.GSARX = False

            fix = NavSatFix()
            fix.header.stamp = self.get_clock().now().to_msg()
            fix.header.frame_id = 'gps'

            fix.altitude = self.alt
            fix.longitude = self.lon
            fix.latitude = self.lat
            
            fix.status.service = NavSatStatus.SERVICE_GPS

            if self.gps_qual == 0:
                fix.status.status = NavSatStatus.STATUS_NO_FIX
            elif self.gps_qual in (1, 2, 5):
                fix.status.status = NavSatStatus.STATUS_FIX
            elif self.gps_qual == 4:
                fix.status.status = NavSatStatus.STATUS_GBAS_FIX
            else:
                fix.status.status = NavSatStatus.STATUS_FIX

            # --- choose sigma_rho from gps_qual ---
            if self.gps_qual == 1:
                sigma_rho = 5.0
            elif self.gps_qual == 2:
                sigma_rho = 1.5
            elif self.gps_qual == 5:
                sigma_rho = 0.3
            elif self.gps_qual == 4:
                sigma_rho = 0.02
            elif self.gps_qual == 6:
                sigma_rho = 10.0
            else:
                sigma_rho = 100.0  # invalid / unknown

            sigma_h = self.hdop * sigma_rho
            sigma_v = self.vdop * sigma_rho

            fix.position_covariance = [
                sigma_h**2, 0.0, 0.0,
                0.0, sigma_h**2, 0.0,
                0.0, 0.0, sigma_v**2
            ]

            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

            if self.pdop > 8.0:
                self.get_logger().warn("GNSS geometry poor (PDOP > 8)")

            r95conf = np.sqrt(5.99) * sigma_h
            r68conf = np.sqrt(3.53) * sigma_h
            self.get_logger().info(f"Radius (95% conf.): {r95conf:.2f}m - (68% conf.): {r68conf:.2f}m \twith {self.num_sats} sats & quality = {QUALITY[self.gps_qual]}")

            self.pub.publish(fix)



        # if hasattr(msg, 'latitude') and hasattr(msg, 'longitude'):
        #     fix = NavSatFix()
        #     fix.header.stamp = self.get_clock().now().to_msg()
        #     fix.header.frame_id = 'gps'

        #     fix.latitude = msg.latitude
        #     fix.longitude = msg.longitude
        #     fix.altitude = float(msg.altitude) if hasattr(msg, 'altitude') else 0.0

        #     fix.status.status = NavSatStatus.STATUS_FIX
        #     fix.status.service = NavSatStatus.SERVICE_GPS

        #     self.pub.publish(fix)

def main():
    rclpy.init()
    node = GPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
