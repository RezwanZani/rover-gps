import time
import threading
import datetime
import serial
from pynmeagps import NMEAReader

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus


# Quality codes sent by LC29H (matches base station's LC29H output)
FIX_LABELS = {
    0: "NO FIX",
    1: "GPS",
    2: "DGPS",
    4: "RTK FIXED 🔥",
    5: "RTK FLOAT ⚡",
}


class RTKBridgeNode(Node):
    """
    Robust RTK Serial Bridge Node for Rover.

    Responsibilities:
      1. Connects to the XBee 900MHz module on static port /dev/rover_900mhz.
         Receives RTCM3 corrections from Base Station (API mode 1 or transparent)
         and safely injects them into the LC29H GPS module.
      2. Connects to the LC29H RTK GPS module on static port /dev/rover_rtk.
         Reads NMEA GGA messages, extracts coordinates and fix quality, and
         publishes standard sensor_msgs/NavSatFix messages to /fix.
      3. Fault-Tolerant & Auto-Reconnecting:
         - Neither device is required at startup; node keeps running and waits.
         - If either module disconnects mid-operation, the other continues running.
         - Automatic background reconnection as soon as the port becomes available.
    """

    def __init__(self):
        super().__init__('rtk_bridge_node')

        # ---------------- PARAMETERS ----------------
        self.declare_parameter('xbee_port', '/dev/rover_900mhz')
        self.declare_parameter('gps_port',  '/dev/rover_rtk')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('frame_id',  'gps_link')

        self.xbee_port = self.get_parameter('xbee_port').value
        self.gps_port  = self.get_parameter('gps_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.frame_id  = self.get_parameter('frame_id').value

        self.get_logger().info(f"Configured XBee Port : {self.xbee_port} @ {self.baud_rate}")
        self.get_logger().info(f"Configured GPS Port  : {self.gps_port} @ {self.baud_rate}")

        # ---------------- THREAD SYNCHRONIZATION & STATE ----------------
        self.stop_event = threading.Event()
        self.gps_lock   = threading.Lock()
        self.xbee_lock  = threading.Lock()
        self.state_lock = threading.Lock()

        # Serial handles
        self.xbee_ser = None
        self.gps_ser  = None

        # Device connection flags
        self.gps_connected  = False
        self.xbee_connected = False

        # Link & Navigation State
        self.rtcm_pkt_count     = 0
        self.current_quality    = 0
        self.current_lat        = None
        self.current_lon        = None
        self.current_alt        = float('nan')

        # XBee Health tracking
        self.xbee_buf           = bytearray()
        self.xbee_last_rx_time  = None
        self.xbee_total_bytes   = 0
        self.xbee_data_active   = False
        self._XBEE_TIMEOUT_SEC  = 5.0

        # Disconnection warning throttling timestamps
        self._last_gps_warn_time  = 0.0
        self._last_xbee_warn_time = 0.0

        # ---------------- PUBLISHER ----------------
        self.fix_pub = self.create_publisher(NavSatFix, '/fix', 10)

        # ---------------- INDEPENDENT WORKER THREADS ----------------
        # Thread 1: Manages XBee connection, reception & RTCM injection
        self.xbee_thread = threading.Thread(
            target=self._xbee_worker_loop, name="xbee_worker", daemon=True
        )

        # Thread 2: Manages GPS connection, NMEA parsing & /fix publishing
        self.gps_thread = threading.Thread(
            target=self._gps_worker_loop, name="gps_worker", daemon=True
        )

        self.xbee_thread.start()
        self.gps_thread.start()

        # 1 Hz Terminal Feedback Dashboard
        self.create_timer(1.0, self.display_status)
        self.get_logger().info("🚀 RTK Bridge Node Initialized (Auto-reconnect & Fault Tolerance active).")

    # =========================================================================
    #                       SAFE SERIAL HELPERS
    # =========================================================================

    def _close_xbee(self):
        with self.xbee_lock:
            if self.xbee_ser:
                try:
                    self.xbee_ser.close()
                except Exception:
                    pass
                self.xbee_ser = None
            self.xbee_connected = False
            self.xbee_data_active = False

    def _close_gps(self):
        with self.gps_lock:
            if self.gps_ser:
                try:
                    self.gps_ser.close()
                except Exception:
                    pass
                self.gps_ser = None
            self.gps_connected = False
            with self.state_lock:
                self.current_lat = None
                self.current_lon = None
                self.current_alt = float('nan')
                self.current_quality = 0

    def inject_to_gps(self, payload: bytes) -> bool:
        """Safely writes RTCM payload into LC29H. Returns False if GPS is not connected."""
        with self.gps_lock:
            if self.gps_ser and self.gps_ser.is_open:
                try:
                    self.gps_ser.write(payload)
                    return True
                except (serial.SerialException, OSError, IOError) as e:
                    self.get_logger().warn(f"Failed writing RTCM to GPS: {e}")
                    return False
            return False

    # =========================================================================
    #                       XBEE WORKER THREAD
    # =========================================================================

    def _xbee_worker_loop(self):
        """
        Continuously connects / reconnects to the XBee serial port and extracts
        RTCM packets from incoming frames. Runs independently of GPS status.
        """
        while not self.stop_event.is_set():
            # 1. Connect or Reconnect
            if not self.xbee_connected or self.xbee_ser is None:
                try:
                    ser = serial.Serial(self.xbee_port, self.baud_rate, timeout=1.0)
                    with self.xbee_lock:
                        self.xbee_ser = ser
                        self.xbee_connected = True
                    self.get_logger().info(f"🔌 XBee 900MHz CONNECTED on {self.xbee_port} @ {self.baud_rate}")
                except (serial.SerialException, FileNotFoundError, PermissionError, OSError) as e:
                    now = time.monotonic()
                    if now - self._last_xbee_warn_time >= 5.0:
                        self.get_logger().warn(f"⏳ Waiting for XBee at {self.xbee_port} ({e})")
                        self._last_xbee_warn_time = now
                    self._close_xbee()
                    time.sleep(1.5)
                    continue

            # 2. Read and parse incoming data
            try:
                waiting = self.xbee_ser.in_waiting
                if waiting > 0:
                    data = self.xbee_ser.read(waiting)
                    if data:
                        self._process_xbee_bytes(data)
                else:
                    time.sleep(0.01)

            except (serial.SerialException, OSError, IOError) as e:
                self.get_logger().error(f"❌ XBee 900MHz DISCONNECTED from {self.xbee_port}: {e}. Retrying in background...")
                self._close_xbee()
                time.sleep(1.0)
            except Exception as e:
                self.get_logger().debug(f"Unexpected XBee read warning: {e}")
                time.sleep(0.05)

    def _process_xbee_bytes(self, data: bytes):
        """Parses incoming XBee API mode 1 frames (0x7E) or transparent RTCM (0xD3)."""
        self.xbee_buf.extend(data)

        while len(self.xbee_buf) > 0:
            # Case 1: XBee API Frame (starts with 0x7E)
            if self.xbee_buf[0] == 0x7E:
                if len(self.xbee_buf) < 3:
                    break
                length = (self.xbee_buf[1] << 8) | self.xbee_buf[2]
                total_len = 3 + length + 1  # 0x7E + 2 bytes len + length bytes data + 1 byte checksum
                if len(self.xbee_buf) < total_len:
                    break  # Wait for remaining frame bytes

                frame = self.xbee_buf[:total_len]
                del self.xbee_buf[:total_len]

                # Checksum verification: sum(frame_data + checksum) & 0xFF == 0xFF
                if (sum(frame[3:]) & 0xFF) != 0xFF:
                    self.get_logger().debug("XBee API checksum mismatch, skipping frame")
                    continue

                frame_data = frame[3:-1]
                frame_type = frame_data[0]
                payload = None

                # Extract payload based on API Frame Type
                if frame_type == 0x90 and len(frame_data) > 12:  # Zigbee / DigiMesh / XBee 3 RX
                    payload = frame_data[12:]
                elif frame_type == 0x91 and len(frame_data) > 18:  # Explicit RX
                    payload = frame_data[18:]
                elif frame_type == 0x80 and len(frame_data) > 11:  # 802.15.4 64-bit RX
                    payload = frame_data[11:]
                elif frame_type == 0x81 and len(frame_data) > 5:   # 802.15.4 16-bit RX
                    payload = frame_data[5:]

                if payload:
                    self.inject_to_gps(payload)
                    with self.state_lock:
                        self.rtcm_pkt_count += 1
                        self.xbee_total_bytes += len(payload)
                        self.xbee_last_rx_time = time.monotonic()

                    if not self.xbee_data_active:
                        self.xbee_data_active = True
                        self.get_logger().info(f"✅ XBee RF link ACTIVE (Frame Type 0x{frame_type:02X}) — RTCM injecting!")

            # Case 2: Transparent / raw RTCM3 fallback (starts with 0xD3)
            elif self.xbee_buf[0] == 0xD3:
                if len(self.xbee_buf) < 3:
                    break
                rtcm_len = ((self.xbee_buf[1] & 0x03) << 8) | self.xbee_buf[2]
                total_rtcm = 3 + rtcm_len + 3
                if len(self.xbee_buf) < total_rtcm:
                    break

                rtcm_pkt = self.xbee_buf[:total_rtcm]
                del self.xbee_buf[:total_rtcm]

                self.inject_to_gps(rtcm_pkt)
                with self.state_lock:
                    self.rtcm_pkt_count += 1
                    self.xbee_total_bytes += len(rtcm_pkt)
                    self.xbee_last_rx_time = time.monotonic()

                if not self.xbee_data_active:
                    self.xbee_data_active = True
                    self.get_logger().info("✅ XBee Transparent link ACTIVE — RTCM injecting!")

            else:
                # Discard non-frame sync byte
                del self.xbee_buf[0]

    # =========================================================================
    #                       GPS WORKER THREAD
    # =========================================================================

    def _gps_worker_loop(self):
        """
        Continuously connects / reconnects to the LC29H GPS module, reads NMEA sentences,
        and publishes NavSatFix to ROS. Runs independently of XBee status.
        """
        while not self.stop_event.is_set():
            # 1. Connect or Reconnect
            if not self.gps_connected or self.gps_ser is None:
                try:
                    ser = serial.Serial(self.gps_port, self.baud_rate, timeout=1.0)
                    with self.gps_lock:
                        self.gps_ser = ser
                        self.gps_connected = True
                    self.get_logger().info(f"🔌 GPS RTK CONNECTED on {self.gps_port} @ {self.baud_rate}")
                except (serial.SerialException, FileNotFoundError, PermissionError, OSError) as e:
                    now = time.monotonic()
                    if now - self._last_gps_warn_time >= 5.0:
                        self.get_logger().warn(f"⏳ Waiting for GPS RTK at {self.gps_port} ({e})")
                        self._last_gps_warn_time = now
                    self._close_gps()
                    time.sleep(1.5)
                    continue

            # 2. Read NMEA Sentences
            try:
                nmr = NMEAReader(self.gps_ser)
                while not self.stop_event.is_set() and self.gps_connected:
                    try:
                        _, parsed = nmr.read()
                    except (serial.SerialException, OSError, IOError) as e:
                        raise e  # Device disconnected, bubble to reconnect loop
                    except Exception as e:
                        self.get_logger().debug(f"NMEA parse warning: {e}")
                        continue

                    if parsed and parsed.msgID == 'GGA':
                        self._handle_gga(parsed)

            except (serial.SerialException, OSError, IOError) as e:
                self.get_logger().error(f"❌ GPS RTK DISCONNECTED from {self.gps_port}: {e}. Retrying in background...")
                self._close_gps()
                time.sleep(1.0)
            except Exception as e:
                self.get_logger().error(f"Unexpected GPS error: {e}")
                self._close_gps()
                time.sleep(1.0)

    def _handle_gga(self, parsed):
        """Parses GGA sentence and publishes NavSatFix message."""
        try:
            quality = getattr(parsed, 'quality', 0)
            lat = float(parsed.lat) if getattr(parsed, 'lat', None) is not None else None
            lon = float(parsed.lon) if getattr(parsed, 'lon', None) is not None else None
            alt = float(getattr(parsed, 'alt', float('nan')))

            with self.state_lock:
                self.current_quality = quality
                self.current_lat = lat
                self.current_lon = lon
                self.current_alt = alt

            # Publish if we have coordinates
            if lat is not None and lon is not None:
                msg = NavSatFix()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.frame_id

                msg.latitude  = lat
                msg.longitude = lon
                msg.altitude  = alt

                if quality in (4, 5):
                    msg.status.status = NavSatStatus.STATUS_GBAS_FIX
                elif quality in (1, 2):
                    msg.status.status = NavSatStatus.STATUS_FIX
                else:
                    msg.status.status = NavSatStatus.STATUS_NO_FIX

                msg.status.service = NavSatStatus.SERVICE_GPS
                msg.position_covariance = [0.0] * 9
                msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

                self.fix_pub.publish(msg)

                if quality == 4:
                    self.get_logger().info(f"🔥 RTK FIXED! Google Maps → {lat:.8f}, {lon:.8f}")

        except Exception as e:
            self.get_logger().debug(f"Error handling GGA message: {e}")

    # =========================================================================
    #                       1 HZ TERMINAL DASHBOARD
    # =========================================================================

    def display_status(self):
        """Prints live operational feedback on terminal every second."""
        ts = datetime.datetime.now().strftime("%H:%M:%S")
        now = time.monotonic()

        with self.state_lock:
            lat = self.current_lat
            lon = self.current_lon
            qual_code = self.current_quality
            pkts = self.rtcm_pkt_count
            rx_time = self.xbee_last_rx_time
            bytes_recv = self.xbee_total_bytes

        # Format GPS feedback
        if not self.gps_connected:
            gps_status = f"❌ GPS RTK: DISCONNECTED ({self.gps_port})"
        elif lat is not None and lon is not None:
            qual_label = FIX_LABELS.get(qual_code, f"Q{qual_code}")
            gps_status = f"{qual_label} | {lat:.8f}, {lon:.8f}"
        else:
            gps_status = "searching for satellites..."

        # Format XBee feedback
        if not self.xbee_connected:
            xbee_status = f"❌ XBee: DISCONNECTED ({self.xbee_port})"
        elif rx_time is None:
            xbee_status = "⏳ XBee: WAITING for base RF..."
        elif (now - rx_time) > self._XBEE_TIMEOUT_SEC:
            silent_sec = int(now - rx_time)
            xbee_status = f"⚠️ XBee: RF SILENT ({silent_sec}s no data)"
            self.xbee_data_active = False
        else:
            xbee_status = f"✅ XBee: ACTIVE ({bytes_recv} B)"

        self.get_logger().info(
            f"[{ts}] {gps_status} | RTCM pkts: {pkts} | {xbee_status}"
        )

    # =========================================================================
    #                       CLEAN SHUTDOWN
    # =========================================================================

    def destroy_node(self):
        self.stop_event.set()
        self._close_xbee()
        self._close_gps()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RTKBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
