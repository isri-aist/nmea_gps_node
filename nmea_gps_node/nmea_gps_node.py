import base64
import math
import socket
import threading
import time

import numpy as np
import pynmea2
import rclpy
import serial
import serial.tools.list_ports
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus

try:
    from gps_msgs.msg import GPSFix, GPSStatus
except ModuleNotFoundError:
    GPSFix = None
    GPSStatus = None


QUALITY = [
    "0 - Fix not valid",
    "1 - GPS Fix",
    "2 - DGPS Fix",
    "3 - N/A",
    "4 - RTK Fix",
    "5 - RTK Float",
    "6 - INS Dead reckoning",
    "7 - Manual Input mode",
    "8 - Simulation mode",
]
COMMON_BAUDRATES = [4800, 9600, 115200]
RTCM_PREAMBLE = 0xD3


class GPSNode(Node):
    def __init__(self):
        super().__init__('nmea_gps_node')

        self.declare_parameter('port', '')
        self.declare_parameter('baudrate', -1)
        self.declare_parameter('RTK', True)
        self.declare_parameter('ntrip_user', 'a')
        self.declare_parameter('ntrip_password', 'a')
        self.declare_parameter('ntrip_url', 'ntrip1.bizstation.jp')
        self.declare_parameter('ntrip_port', 2101)
        self.declare_parameter('ntrip_mountpoint', '')
        self.declare_parameter('rtcm_feedback_interval', 5.0)
        self.declare_parameter('gps_feedback_interval', 5.0)
        self.declare_parameter('ntrip_gga_interval', 5.0)
        self.declare_parameter('ntrip_reconnect_initial_delay', 1.0)
        self.declare_parameter('ntrip_reconnect_max_delay', 30.0)
        self.declare_parameter('nmea_pair_max_age', 2.0)
        self.declare_parameter('gst_max_age', 2.0)
        self.declare_parameter('read_period', 0.02)
        self.declare_parameter('serial_timeout', 0.01)
        self.declare_parameter('max_nmea_lines_per_tick', 50)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        self.ntrip_user = self.get_parameter('ntrip_user').value
        self.ntrip_password = self.get_parameter('ntrip_password').value
        self.ntrip_url = self.get_parameter('ntrip_url').value
        self.ntrip_port = self.get_parameter('ntrip_port').value
        self.ntrip_mountpoint = self.get_parameter('ntrip_mountpoint').value
        self.rtcm_feedback_interval = self._positive_parameter('rtcm_feedback_interval', 5.0)
        self.gps_feedback_interval = self._positive_parameter('gps_feedback_interval', 5.0)
        self.ntrip_gga_interval = self._positive_parameter('ntrip_gga_interval', 5.0)
        self.ntrip_reconnect_initial_delay = self._positive_parameter(
            'ntrip_reconnect_initial_delay', 1.0
        )
        self.ntrip_reconnect_max_delay = self._positive_parameter(
            'ntrip_reconnect_max_delay', 30.0
        )
        self.nmea_pair_max_age = self._positive_parameter('nmea_pair_max_age', 2.0)
        self.gst_max_age = self._positive_parameter('gst_max_age', 2.0)
        self.read_period = self._positive_parameter('read_period', 0.02)
        self.serial_timeout = self._positive_parameter('serial_timeout', 0.01)
        self.max_nmea_lines_per_tick = self._positive_int_parameter(
            'max_nmea_lines_per_tick',
            50,
        )

        self.useRTK = self.get_parameter('RTK').value
        self.ntrip_reconnect_max_delay = max(
            self.ntrip_reconnect_initial_delay,
            self.ntrip_reconnect_max_delay,
        )

        self.RTCM_thread_running = False
        self.rtcm_client_thread = None
        self.rtcm_socket = None
        self.position_lock = threading.Lock()
        self.serial_lock = threading.Lock()
        self.state_lock = threading.Lock()

        self.latest_position = None
        self.latest_gga_line = None
        self.last_gga = None
        self.last_gsa = None
        self.last_gst = None
        self.last_published_gga_time = None
        self.last_gps_log_time = 0.0

        self.ntrip_connected = False
        self.ntrip_status = 'disabled'
        self.ntrip_selected_mountpoint = self.ntrip_mountpoint.strip()
        self.ntrip_reconnect_count = 0
        self.rtcm_total_bytes = 0
        self.rtcm_total_frames = 0
        self.rtcm_last_message_type = None
        self.rtcm_last_time = None
        self.last_gga_sent_time = None

        if port == '' or baud == -1:
            gpsReceivers = self.auto_detect_device()
            if not gpsReceivers:
                raise RuntimeError("No NMEA GPS receiver detected")
            port, baud = gpsReceivers[0]

        self.ser = serial.Serial(port, baud, timeout=self.serial_timeout)

        self.pub = self.create_publisher(NavSatFix, 'fix', 10)
        self.extended_pub = None
        if GPSFix is None:
            self.get_logger().warn(
                "gps_msgs is not installed; fix_extended will not be published"
            )
        else:
            self.extended_pub = self.create_publisher(GPSFix, 'fix_extended', 10)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
        self.timer = self.create_timer(self.read_period, self.read_gps)
        self.diagnostics_timer = self.create_timer(1.0, self.publish_diagnostics)

        if self.useRTK:
            self.RTCM_thread_running = True
            self.rtcm_client_thread = threading.Thread(target=self.RTCM_client)
            self.rtcm_client_thread.daemon = True
            self.rtcm_client_thread.start()

        self.get_logger().info(f"GPS connected on {port} @ {baud}")

    def _positive_parameter(self, name, default):
        value = float(self.get_parameter(name).value)
        if value <= 0.0:
            self.get_logger().warn(f"{name} must be > 0, using {default}")
            return default
        return value

    def _positive_int_parameter(self, name, default):
        value = int(self.get_parameter(name).value)
        if value <= 0:
            self.get_logger().warn(f"{name} must be > 0, using {default}")
            return default
        return value

    def RTCM_client(self):
        reconnect_delay = self.ntrip_reconnect_initial_delay

        while self.RTCM_thread_running:
            try:
                mountpoint = self.ntrip_mountpoint.strip()
                if not mountpoint:
                    mountpoint = self._select_closest_mountpoint()
                    self.ntrip_mountpoint = mountpoint

                self.ntrip_selected_mountpoint = mountpoint
                self._run_ntrip_session(mountpoint)
                reconnect_delay = self.ntrip_reconnect_initial_delay

            except Exception as e:
                if not self.RTCM_thread_running:
                    break

                with self.state_lock:
                    self.ntrip_connected = False
                    self.ntrip_status = f"error: {e}"
                    self.ntrip_reconnect_count += 1

                self.get_logger().warn(
                    f"NTRIP connection failed: {e}. Reconnecting in {reconnect_delay:.1f}s"
                )
                self._sleep_while_running(reconnect_delay)
                reconnect_delay = min(reconnect_delay * 2.0, self.ntrip_reconnect_max_delay)

        with self.state_lock:
            self.ntrip_connected = False
            if self.ntrip_status != 'disabled':
                self.ntrip_status = 'stopped'

    def _run_ntrip_session(self, mountpoint):
        rtcm_buffer = bytearray()
        bytes_since_log = 0
        frames_since_log = 0
        last_log_time = time.monotonic()
        last_gga_send_time = 0.0
        header_pending = True
        header_buffer = bytearray()
        sock = None

        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.rtcm_socket = sock
            sock.settimeout(5.0)
            sock.connect((self.ntrip_url, self.ntrip_port))
            sock.sendall(self._build_ntrip_request(mountpoint).encode())

            with self.state_lock:
                self.ntrip_connected = True
                self.ntrip_status = 'connected'

            self.get_logger().info(
                f"NTRIP connected to {self.ntrip_url}:{self.ntrip_port}/{mountpoint}"
            )

            while self.RTCM_thread_running:
                now = time.monotonic()
                if (
                    not header_pending
                    and now - last_gga_send_time >= self.ntrip_gga_interval
                ):
                    if self._send_latest_gga(sock):
                        last_gga_send_time = now

                try:
                    data = sock.recv(4096)
                except socket.timeout:
                    self._log_rtcm_feedback(bytes_since_log, frames_since_log, force=True)
                    last_log_time = time.monotonic()
                    bytes_since_log = 0
                    frames_since_log = 0
                    continue

                if not data:
                    raise RuntimeError("connection closed by server")

                if not self.RTCM_thread_running:
                    break

                if header_pending:
                    header_buffer.extend(data)
                    data = self._strip_ntrip_header(header_buffer)
                    if data is None:
                        continue
                    header_pending = False
                    header_buffer.clear()
                    if not data:
                        continue

                try:
                    with self.serial_lock:
                        self.ser.write(data)
                except serial.SerialException as e:
                    raise RuntimeError(f"serial write failed: {e}") from e

                bytes_since_log += len(data)
                frame_count, last_type = self._count_rtcm_frames(rtcm_buffer, data)
                frames_since_log += frame_count

                with self.state_lock:
                    self.rtcm_total_bytes += len(data)
                    self.rtcm_total_frames += frame_count
                    if last_type is not None:
                        self.rtcm_last_message_type = last_type
                    if frame_count > 0:
                        self.rtcm_last_time = self.get_clock().now()

                now = time.monotonic()
                if now - last_log_time >= self.rtcm_feedback_interval:
                    self._log_rtcm_feedback(bytes_since_log, frames_since_log)
                    last_log_time = now
                    bytes_since_log = 0
                    frames_since_log = 0

        finally:
            with self.state_lock:
                self.ntrip_connected = False
                if self.RTCM_thread_running:
                    self.ntrip_status = 'disconnected'

            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass
            self.rtcm_socket = None

    def _sleep_while_running(self, duration):
        deadline = time.monotonic() + duration
        while self.RTCM_thread_running and time.monotonic() < deadline:
            time.sleep(0.2)

    def _send_latest_gga(self, sock):
        with self.position_lock:
            line = self.latest_gga_line

        if line is None:
            return False

        try:
            sock.sendall((line + "\r\n").encode())
        except OSError as e:
            raise RuntimeError(f"failed to send GGA to NTRIP caster: {e}") from e

        with self.state_lock:
            self.last_gga_sent_time = self.get_clock().now()
        return True

    def _build_ntrip_request(self, mountpoint):
        auth = base64.b64encode(f"{self.ntrip_user}:{self.ntrip_password}".encode()).decode()
        path = "/" if not mountpoint else f"/{mountpoint}"

        return (
            f"GET {path} HTTP/1.0\r\n"
            f"Host: {self.ntrip_url}:{self.ntrip_port}\r\n"
            f"User-Agent: NTRIP PythonClient\r\n"
            f"Ntrip-Version: Ntrip/2.0\r\n"
            f"Authorization: Basic {auth}\r\n\r\n"
        )

    def _select_closest_mountpoint(self):
        position = self._wait_for_current_position(timeout=30.0)
        if position is None:
            raise RuntimeError("Cannot auto-select NTRIP mountpoint: no GPS position available")

        sourcetable = self._fetch_ntrip_sourcetable()
        bases = self._parse_sourcetable(sourcetable)
        if not bases:
            raise RuntimeError(
                "Cannot auto-select NTRIP mountpoint: no STR entries with coordinates found"
            )

        lat, lon = position
        closest = min(
            bases,
            key=lambda base: self._distance_meters(lat, lon, base['lat'], base['lon']),
        )
        distance = self._distance_meters(lat, lon, closest['lat'], closest['lon'])
        self.get_logger().info(
            f"Auto-selected NTRIP mountpoint '{closest['mountpoint']}' "
            f"({distance / 1000.0:.1f} km from current position)"
        )
        return closest['mountpoint']

    def _wait_for_current_position(self, timeout):
        deadline = time.monotonic() + timeout
        self.get_logger().info(
            "NTRIP mountpoint empty, waiting for GPS position to select closest base"
        )

        while self.RTCM_thread_running and time.monotonic() < deadline:
            with self.position_lock:
                if self.latest_position is not None:
                    return self.latest_position
            time.sleep(0.2)

        return None

    def _fetch_ntrip_sourcetable(self):
        req = self._build_ntrip_request("")
        chunks = []

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.settimeout(10.0)
            sock.connect((self.ntrip_url, self.ntrip_port))
            sock.sendall(req.encode())

            while True:
                data = sock.recv(4096)
                if not data:
                    break

                chunks.append(data)
                if b"ENDSOURCETABLE" in data:
                    break

        response = b"".join(chunks).decode(errors='ignore')
        first_line = response.splitlines()[0] if response.splitlines() else ""
        if first_line:
            self.get_logger().info(f"NTRIP sourcetable response: {first_line}")

        return response

    def _parse_sourcetable(self, sourcetable):
        bases = []

        for line in sourcetable.splitlines():
            if not line.startswith("STR;"):
                continue

            fields = line.split(";")
            if len(fields) < 11:
                continue

            try:
                lat = float(fields[9])
                lon = float(fields[10])
            except ValueError:
                continue

            bases.append({
                'mountpoint': fields[1],
                'lat': lat,
                'lon': lon,
            })

        return bases

    def _distance_meters(self, lat1, lon1, lat2, lon2):
        radius = 6371000.0
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)

        a = (
            math.sin(delta_phi / 2.0) ** 2
            + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2.0) ** 2
        )
        return radius * 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))

    def _strip_ntrip_header(self, header_buffer):
        if header_buffer and header_buffer[0] == RTCM_PREAMBLE:
            return bytes(header_buffer)

        header_end = header_buffer.find(b"\r\n\r\n")
        if header_end == -1:
            line_end = header_buffer.find(b"\r\n")
            if line_end == -1:
                return None

            payload = header_buffer[line_end + 2:]
            if payload and payload[0] != RTCM_PREAMBLE:
                return None

            header_end = line_end
            payload_start = line_end + 2
        else:
            payload_start = header_end + 4

        header = header_buffer[:header_end]
        status_line = header.splitlines()[0].decode(errors='ignore')
        self.get_logger().info(f"NTRIP response: {status_line}")
        if not self._ntrip_status_ok(status_line):
            raise RuntimeError(f"NTRIP server rejected request: {status_line}")

        return bytes(header_buffer[payload_start:])

    def _ntrip_status_ok(self, status_line):
        if status_line.startswith("ICY 200"):
            return True

        parts = status_line.split()
        return len(parts) >= 2 and parts[0].startswith("HTTP/") and parts[1] == "200"

    def _count_rtcm_frames(self, rtcm_buffer, data):
        rtcm_buffer.extend(data)
        frame_count = 0
        last_message_type = None

        while len(rtcm_buffer) >= 6:
            if rtcm_buffer[0] != RTCM_PREAMBLE:
                del rtcm_buffer[0]
                continue

            payload_length = ((rtcm_buffer[1] & 0x03) << 8) | rtcm_buffer[2]
            frame_length = 3 + payload_length + 3

            if len(rtcm_buffer) < frame_length:
                break

            if payload_length >= 2:
                last_message_type = (rtcm_buffer[3] << 4) | (rtcm_buffer[4] >> 4)

            frame_count += 1
            del rtcm_buffer[:frame_length]

        return frame_count, last_message_type

    def _log_rtcm_feedback(self, bytes_since_log, frames_since_log, force=False):
        if not force and bytes_since_log == 0 and frames_since_log == 0:
            return

        with self.state_lock:
            total_frames = self.rtcm_total_frames
            total_bytes = self.rtcm_total_bytes
            last_type = self.rtcm_last_message_type

        last_type = "unknown" if last_type is None else last_type
        self.get_logger().info(
            f"RTCM received: {frames_since_log} frames / {bytes_since_log} bytes "
            f"(total: {total_frames} frames / {total_bytes} bytes, last type: {last_type})"
        )

    def read_gps(self):
        lines_read = 0

        while lines_read < self.max_nmea_lines_per_tick:
            try:
                with self.serial_lock:
                    raw = self.ser.readline()
                    pending_bytes = self.ser.in_waiting
            except serial.SerialException as e:
                self.get_logger().error(f"Serial read failed: {e}")
                return

            if not raw:
                return

            lines_read += 1
            self._process_nmea_line(raw)

            if pending_bytes <= 0:
                return

    def _process_nmea_line(self, raw):
        try:
            line = raw.decode(errors='ignore').strip()
        except AttributeError:
            return

        if not line.startswith('$'):
            return

        try:
            msg = pynmea2.parse(line)
        except pynmea2.ParseError:
            return

        sentence_type = getattr(msg, 'sentence_type', '')

        if sentence_type == 'GGA':
            self._handle_gga(msg, line)
        elif sentence_type == 'GSA':
            self._handle_gsa(msg)
        elif sentence_type == 'GST':
            self._handle_gst(msg)

        self._publish_fix_if_ready()

    def _handle_gga(self, msg, line):
        try:
            gga = {
                'lat': float(msg.latitude),
                'lon': float(msg.longitude),
                'alt': float(msg.altitude),
                'gps_qual': int(msg.gps_qual),
                'num_sats': int(msg.num_sats),
                'stamp': self.get_clock().now(),
                'monotonic': time.monotonic(),
            }
        except (TypeError, ValueError):
            self.get_logger().warn(f"Invalid GGA frame ignored: {repr(msg)}")
            return

        self.last_gga = gga
        with self.position_lock:
            self.latest_position = (gga['lat'], gga['lon'])
            self.latest_gga_line = line

    def _handle_gsa(self, msg):
        try:
            self.last_gsa = {
                'gps_mode': msg.mode,
                'fix_type': msg.mode_fix_type,
                'pdop': float(msg.pdop),
                'hdop': float(msg.hdop),
                'vdop': float(msg.vdop),
                'stamp': self.get_clock().now(),
                'monotonic': time.monotonic(),
            }
        except (TypeError, ValueError):
            self.get_logger().warn(f"Invalid GSA frame ignored: {repr(msg)}")

    def _handle_gst(self, msg):
        try:
            self.last_gst = {
                'lat_std': float(msg.std_dev_latitude),
                'lon_std': float(msg.std_dev_longitude),
                'alt_std': float(msg.std_dev_altitude),
                'stamp': self.get_clock().now(),
                'monotonic': time.monotonic(),
            }
        except (AttributeError, TypeError, ValueError):
            self.get_logger().warn(f"Invalid GST frame ignored: {repr(msg)}")

    def _publish_fix_if_ready(self):
        if self.last_gga is None or self.last_gsa is None:
            return

        now = time.monotonic()
        if now - self.last_gga['monotonic'] > self.nmea_pair_max_age:
            return
        if now - self.last_gsa['monotonic'] > self.nmea_pair_max_age:
            return
        if self.last_published_gga_time == self.last_gga['monotonic']:
            return

        gga = self.last_gga
        gsa = self.last_gsa
        self.last_published_gga_time = gga['monotonic']

        fix = NavSatFix()
        fix.header.stamp = gga['stamp'].to_msg()
        fix.header.frame_id = 'gps'
        fix.altitude = gga['alt']
        fix.longitude = gga['lon']
        fix.latitude = gga['lat']
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.status.status = self._navsat_status_from_quality(gga['gps_qual'])
        fix.position_covariance, fix.position_covariance_type = self._compute_covariance(
            gga['gps_qual'],
            gsa,
            now,
        )

        if gsa['pdop'] > 8.0:
            self.get_logger().warn("GNSS geometry poor (PDOP > 8)")

        self._log_gps_feedback(gga, gsa, fix.position_covariance)
        self.pub.publish(fix)
        if self.extended_pub is not None:
            self.extended_pub.publish(self._build_extended_fix(gga, gsa, fix))

    def _navsat_status_from_quality(self, gps_qual):
        if gps_qual == 0:
            return NavSatStatus.STATUS_NO_FIX
        if gps_qual == 4:
            return NavSatStatus.STATUS_GBAS_FIX
        return NavSatStatus.STATUS_FIX

    def _gps_status_from_quality(self, gps_qual):
        if gps_qual == 0:
            return GPSStatus.STATUS_NO_FIX
        if gps_qual == 2:
            return getattr(GPSStatus, 'STATUS_DGPS_FIX', GPSStatus.STATUS_SBAS_FIX)
        if gps_qual == 4:
            return getattr(GPSStatus, 'STATUS_RTK_FIX', GPSStatus.STATUS_GBAS_FIX)
        if gps_qual == 5:
            return getattr(GPSStatus, 'STATUS_RTK_FLOAT', GPSStatus.STATUS_FIX)
        return GPSStatus.STATUS_FIX

    def _build_extended_fix(self, gga, gsa, navsat_fix):
        fix = GPSFix()
        fix.header = navsat_fix.header
        fix.status.header = navsat_fix.header
        fix.status.status = self._gps_status_from_quality(gga['gps_qual'])
        fix.status.satellites_used = gga['num_sats']
        fix.status.position_source = (
            GPSStatus.SOURCE_NONE
            if gga['gps_qual'] == 0
            else GPSStatus.SOURCE_GPS
        )
        fix.status.motion_source = GPSStatus.SOURCE_NONE
        fix.status.orientation_source = GPSStatus.SOURCE_NONE

        fix.latitude = gga['lat']
        fix.longitude = gga['lon']
        fix.altitude = gga['alt']
        fix.pdop = gsa['pdop']
        fix.hdop = gsa['hdop']
        fix.vdop = gsa['vdop']

        fix.position_covariance = navsat_fix.position_covariance
        fix.position_covariance_type = navsat_fix.position_covariance_type

        east_var = fix.position_covariance[0]
        north_var = fix.position_covariance[4]
        up_var = fix.position_covariance[8]
        sigma_h = math.sqrt(max(east_var, north_var))
        sigma_v = math.sqrt(up_var)
        fix.err_horz = math.sqrt(5.99) * sigma_h
        fix.err_vert = 1.96 * sigma_v
        fix.err = math.sqrt(fix.err_horz ** 2 + fix.err_vert ** 2)

        return fix

    def _compute_covariance(self, gps_qual, gsa, now):
        if self.last_gst is not None and now - self.last_gst['monotonic'] <= self.gst_max_age:
            east_std = self.last_gst['lon_std']
            north_std = self.last_gst['lat_std']
            up_std = self.last_gst['alt_std']
            return [
                east_std ** 2, 0.0, 0.0,
                0.0, north_std ** 2, 0.0,
                0.0, 0.0, up_std ** 2,
            ], NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        sigma_rho = self._sigma_rho_from_quality(gps_qual)
        sigma_h = gsa['hdop'] * sigma_rho
        sigma_v = gsa['vdop'] * sigma_rho
        return [
            sigma_h ** 2, 0.0, 0.0,
            0.0, sigma_h ** 2, 0.0,
            0.0, 0.0, sigma_v ** 2,
        ], NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

    def _sigma_rho_from_quality(self, gps_qual):
        if gps_qual == 1:
            return 5.0
        if gps_qual == 2:
            return 1.5
        if gps_qual == 5:
            return 0.3
        if gps_qual == 4:
            return 0.02
        if gps_qual == 6:
            return 10.0
        return 100.0

    def _log_gps_feedback(self, gga, gsa, covariance):
        now = time.monotonic()
        if now - self.last_gps_log_time < self.gps_feedback_interval:
            return
        self.last_gps_log_time = now

        sigma_h = math.sqrt(max(covariance[0], covariance[4]))
        r95conf = np.sqrt(5.99) * sigma_h
        r68conf = np.sqrt(3.53) * sigma_h
        gps_qual = gga['gps_qual']
        quality = QUALITY[gps_qual] if 0 <= gps_qual < len(QUALITY) else f"{gps_qual} - Unknown"
        self.get_logger().info(
            f"Radius (95% conf.): {r95conf:.2f}m - (68% conf.): {r68conf:.2f}m "
            f"with {gga['num_sats']} sats, PDOP={gsa['pdop']:.1f}, quality={quality}"
        )

    def publish_diagnostics(self):
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()

        gps_status = DiagnosticStatus()
        gps_status.name = 'nmea_gps_node/gps'
        gps_status.hardware_id = 'nmea_gps'

        if self.last_gga is None:
            gps_status.level = DiagnosticStatus.WARN
            gps_status.message = 'No GGA received'
        elif self.last_gga['gps_qual'] == 0:
            gps_status.level = DiagnosticStatus.WARN
            gps_status.message = 'No GPS fix'
        else:
            gps_status.level = DiagnosticStatus.OK
            gps_status.message = 'GPS fix available'

        gps_status.values = self._gps_diagnostic_values()
        msg.status.append(gps_status)

        ntrip_status = DiagnosticStatus()
        ntrip_status.name = 'nmea_gps_node/ntrip'
        ntrip_status.hardware_id = self.ntrip_url

        with self.state_lock:
            connected = self.ntrip_connected
            status = self.ntrip_status
            values = [
                KeyValue(key='enabled', value=str(self.useRTK)),
                KeyValue(key='connected', value=str(connected)),
                KeyValue(key='status', value=status),
                KeyValue(key='caster', value=f"{self.ntrip_url}:{self.ntrip_port}"),
                KeyValue(key='mountpoint', value=self.ntrip_selected_mountpoint),
                KeyValue(key='reconnect_count', value=str(self.ntrip_reconnect_count)),
                KeyValue(key='rtcm_total_bytes', value=str(self.rtcm_total_bytes)),
                KeyValue(key='rtcm_total_frames', value=str(self.rtcm_total_frames)),
                KeyValue(
                    key='rtcm_last_message_type',
                    value=str(self.rtcm_last_message_type),
                ),
                KeyValue(
                    key='rtcm_last_age_sec',
                    value=self._age_string_from_ros_time(self.rtcm_last_time),
                ),
                KeyValue(
                    key='last_gga_sent_age_sec',
                    value=self._age_string_from_ros_time(self.last_gga_sent_time),
                ),
            ]

        if not self.useRTK:
            ntrip_status.level = DiagnosticStatus.OK
            ntrip_status.message = 'RTK disabled'
        elif connected:
            ntrip_status.level = DiagnosticStatus.OK
            ntrip_status.message = 'NTRIP connected'
        else:
            ntrip_status.level = DiagnosticStatus.WARN
            ntrip_status.message = status

        ntrip_status.values = values
        msg.status.append(ntrip_status)
        self.diagnostics_pub.publish(msg)

    def _gps_diagnostic_values(self):
        if self.last_gga is None:
            return []

        values = [
            KeyValue(key='latitude', value=f"{self.last_gga['lat']:.9f}"),
            KeyValue(key='longitude', value=f"{self.last_gga['lon']:.9f}"),
            KeyValue(key='altitude', value=f"{self.last_gga['alt']:.3f}"),
            KeyValue(key='gps_quality', value=str(self.last_gga['gps_qual'])),
            KeyValue(key='num_sats', value=str(self.last_gga['num_sats'])),
            KeyValue(
                key='last_gga_age_sec',
                value=f"{time.monotonic() - self.last_gga['monotonic']:.1f}",
            ),
        ]

        if self.last_gsa is not None:
            values.extend([
                KeyValue(key='pdop', value=f"{self.last_gsa['pdop']:.2f}"),
                KeyValue(key='hdop', value=f"{self.last_gsa['hdop']:.2f}"),
                KeyValue(key='vdop', value=f"{self.last_gsa['vdop']:.2f}"),
                KeyValue(
                    key='last_gsa_age_sec',
                    value=f"{time.monotonic() - self.last_gsa['monotonic']:.1f}",
                ),
            ])

        if self.last_gst is not None:
            values.extend([
                KeyValue(key='gst_lat_std_m', value=f"{self.last_gst['lat_std']:.3f}"),
                KeyValue(key='gst_lon_std_m', value=f"{self.last_gst['lon_std']:.3f}"),
                KeyValue(key='gst_alt_std_m', value=f"{self.last_gst['alt_std']:.3f}"),
                KeyValue(
                    key='last_gst_age_sec',
                    value=f"{time.monotonic() - self.last_gst['monotonic']:.1f}",
                ),
            ])

        return values

    def _age_string_from_ros_time(self, stamp):
        if stamp is None:
            return 'never'
        age_ns = (self.get_clock().now() - stamp).nanoseconds
        return f"{age_ns / 1e9:.1f}"

    def destroy_node(self):
        self.RTCM_thread_running = False

        if self.rtcm_socket is not None:
            try:
                self.rtcm_socket.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                self.rtcm_socket.close()
            except OSError:
                pass

        if self.rtcm_client_thread is not None and self.rtcm_client_thread.is_alive():
            self.rtcm_client_thread.join(timeout=1.0)

        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()

        super().destroy_node()

    def auto_detect_device(self):
        gpsReceivers = []
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if port.vid is None or port.pid is None:
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
                                self.get_logger().info(
                                    f"{port.device} @ {baud} outputs valid NMEA frames"
                                )
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
