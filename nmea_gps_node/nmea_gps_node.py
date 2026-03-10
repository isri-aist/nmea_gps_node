import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
import numpy as np

import serial
import serial.tools.list_ports
import pynmea2
import time
import socket
import base64

import threading

QUALITY = ["0 - Fix not valid", "1 - GPS Fix", "2 - DGPS Fix", "3 - N/A", "4 - RTK Fix", "5 - RTK Float", "6 - INS Dead reckoning", "7 - Manual Input mode", "8 - Simulation mode"]
COMMON_BAUDRATES = [4800, 9600, 115200]


class GPSNode(Node):
    def __init__(self):
        super().__init__('nmea_gps_node')

        self.declare_parameter('port', '')
        self.declare_parameter('baudrate', -1)
        self.declare_parameter('RTK', True)
        self.declare_parameter('ntrip_user', 'antoine.caillot-at-aist.go.jp')
        self.declare_parameter('ntrip_password', 'none')
        self.declare_parameter('ntrip_url', 'rtk2go.com')
        self.declare_parameter('ntrip_port', 2101)
        self.declare_parameter('ntrip_mountpoint', 'IBRK_RyGSK-TJ00')

        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        self.ntrip_user = self.get_parameter('ntrip_user').value
        self.ntrip_password = self.get_parameter('ntrip_password').value
        self.ntrip_url = self.get_parameter('ntrip_url').value
        self.ntrip_port = self.get_parameter('ntrip_port').value
        self.ntrip_mountpoint = self.get_parameter('ntrip_mountpoint').value

        self.useRTK = self.get_parameter("RTK").value
        
        self.RTCM_thread_running = False

        if port == '' or baud == -1:
            gpsReceivers = self.auto_detect_device()
            port, baud = gpsReceivers[0]

        self.ser = serial.Serial(port, baud, timeout=0.1)

        if self.useRTK:
            self.RTCM_thread_running = True
            self.rtcm_client_thread = threading.Thread(target=self.RTCM_client)
            self.rtcm_client_thread.daemon = True
            self.rtcm_client_thread.start()

        self.pub = self.create_publisher(NavSatFix, 'fix', 10)
        self.timer = self.create_timer(0.1, self.read_gps)

        self.get_logger().info(f"GPS connected on {port} @ {baud}")
        self.GGARX = False
        self.GSARX = False

    def RTCM_client(self):
        auth = base64.b64encode(f"{self.ntrip_user}:{self.ntrip_password}".encode()).decode()

        req = (
            f"GET /{self.ntrip_mountpoint} HTTP/1.0\r\n"
            f"User-Agent: NTRIP PythonClient\r\n"
            f"Authorization: Basic {auth}\r\n\r\n"
        )

        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((self.ntrip_url, self.ntrip_port))

        sock.send(req.encode())

        while self.RTCM_thread_running:
            data = sock.recv(4096)
            self.ser.write(data)


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
            try:
                self.lat = float(msg.latitude)
                self.lon = float(msg.longitude)
                self.alt = float(msg.altitude)
                self.gps_qual = msg.gps_qual
                self.num_sats = msg.num_sats
                self.GGARX = True
            except:
                # self.get_logger().info(f"{msg}")
                return
            
            # print(f"lat: {self.lat} \tlon: {self.lon} \talt: {self.alt} \tqual: {self.gps_qual}")

        if type(msg) == pynmea2.talker.GSA:
            try:
                self.gps_mode = msg.mode
                self.fixType = msg.mode_fix_type
                self.pdop = float(msg.pdop)
                self.hdop = float(msg.hdop)
                self.vdop = float(msg.vdop)
                self.GSARX = True
            except:
                # self.get_logger().info(f"{msg}")
                return

            # print(repr(msg))
            # print(f"gps mode: {self.gps_mode} \tfix type: {self.fixType} \tPDOP: {self.pdop} \tVDOP: {self.vdop} \tHDOP: {self.hdop}")
            
            # self.get_logger().info(f"{repr(msg)}")
            

        if self.GGARX == True and self.GSARX == True:
            self.GGARX = False
            self.GSARX = False

            # try:
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
            # except:
            #     self.get_logger().info(f"msg: {repr(msg)}, lat: {self.lat}, lon: {self.lon}, alt: {self.alt}")

    def auto_detect_device(self):
        gpsReceivers = []
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if port.vid == None or port.pid == None:
                continue
            self.get_logger().info(f"Device: {port.device}")
            self.get_logger().info(f"  Description: {port.description}")
            self.get_logger().info(f"  HWID: {port.hwid}")
            self.get_logger().info(f"  VID: {port.vid}")
            self.get_logger().info(f"  PID: {port.pid}")
            

            try:
                foundBaudRate = False
                for baud in COMMON_BAUDRATES:
                    with serial.Serial(port.device, baud, timeout=0) as ser:
                        time.sleep(1.5)
                        validLines = 0
                        for _ in range(10):
                            raw = ser.readline()
                            if not raw:
                                continue
                            
                            line = raw.decode(encoding='ascii', errors='ignore').strip()
                            self.get_logger().info(f"[{baud}]: {line}")

                            if line.startswith("$") and "*" in line:
                                validLines += 1

                            if validLines >= 3:
                                self.get_logger().info(f"{port.device} @ {baud} outputs valid NMEA frames")
                                gpsReceivers.append((port.device, baud))
                                foundBaudRate = True
                                break

                if not foundBaudRate:
                    self.get_logger().warn(f"No NMEA frames found for {port.device}")
                    

            except Exception as e:
                self.get_logger().error(f"Error {e}")
        return gpsReceivers


def main():
    rclpy.init()
    node = GPSNode()
    rclpy.spin(node)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()
