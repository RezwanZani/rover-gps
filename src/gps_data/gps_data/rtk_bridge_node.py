import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
import serial
import threading
import datetime
from pynmeagps import NMEAReader


# Quality codes sent by LC29H (matches base station's LC29H output)
FIX_LABELS = {
    0: "NO FIX",
    1: "GPS",
    2: "DGPS",
    4: "RTK FIXED 🔥",
    5: "RTK FLOAT ⚡",
}


class RTKBridgeNode(Node):
    def __init__(self):
        super().__init__('rtk_bridge_node')

        # ---------------- PARAMETERS ----------------
        # Use ROS parameters so ports can be set at launch without editing code.
        # Base station streams RTCM3 binary via XBee → /dev/ttyUSB_XBEE
        # Rover LC29H GPS module          → /dev/ttyUSB_GPS
        self.declare_parameter('xbee_port', '/dev/ttyUSB_XBEE')
        self.declare_parameter('gps_port',  '/dev/ttyUSB_GPS')
        self.declare_parameter('baud_rate', 115200)

        self.xbee_port = self.get_parameter('xbee_port').value
        self.gps_port  = self.get_parameter('gps_port').value
        self.baud_rate = self.get_parameter('baud_rate').value

        # ---------------- SERIAL ----------------
        self.xbee_ser = serial.Serial(self.xbee_port, self.baud_rate, timeout=1)
        self.gps_ser  = serial.Serial(self.gps_port,  self.baud_rate, timeout=1)

        self.get_logger().info(
            f"XBee (RTCM in) : {self.xbee_port} @ {self.baud_rate}"
        )
        self.get_logger().info(
            f"GPS  (NMEA out): {self.gps_port}  @ {self.baud_rate}"
        )

        # ---------------- STATE ----------------
        self.rtcm_pkt_count = 0
        self.current_quality = 0
        self.current_lat = None
        self.current_lon = None
        self.current_alt = float('nan')

        # XBee buffer & health tracking
        self.xbee_buf           = bytearray()
        self.xbee_last_rx_time  = None   # timestamp of last byte received from XBee
        self.xbee_total_bytes   = 0      # total payload bytes received
        self.xbee_connected     = False  # True once first RTCM data arrives
        self._XBEE_TIMEOUT_SEC  = 5.0   # seconds of silence before flagging as lost

        # ---------------- PUB ----------------
        self.fix_pub = self.create_publisher(NavSatFix, '/fix', 10)

        # ---------------- THREADS ----------------
        # Thread 1: XBee → GPS  (forward raw RTCM3 binary from base station)
        self.rtcm_thread = threading.Thread(
            target=self.inject_rtcm_loop, daemon=True
        )
        # Thread 2: GPS → ROS   (parse NMEA, publish NavSatFix)
        self.nmea_thread = threading.Thread(
            target=self.publish_nmea_loop, daemon=True
        )

        self.rtcm_thread.start()
        self.nmea_thread.start()

        # Status log every second (mirrors base station dashboard cadence)
        self.create_timer(1.0, self.display_status)

        self.get_logger().info("RTK Serial Bridge Started.")

    # ---------------- RTCM INJECTION ----------------
    def inject_rtcm_loop(self):
        """
        Reads from XBee and injects RTCM3 packets into rover LC29H.
        Supports BOTH:
          1. XBee API Mode (0x7E frames: Types 0x90, 0x91, 0x80, 0x81) - extracts payload
          2. Transparent / raw RTCM3 fallback (starts with 0xD3)
        """
        import time as _time
        while rclpy.ok():
            try:
                waiting = self.xbee_ser.in_waiting
                if waiting > 0:
                    data = self.xbee_ser.read(waiting)
                    self.xbee_buf.extend(data)

                while len(self.xbee_buf) > 0:
                    # Case 1: XBee API Frame (starts with 0x7E)
                    if self.xbee_buf[0] == 0x7E:
                        if len(self.xbee_buf) < 3:
                            break
                        length = (self.xbee_buf[1] << 8) | self.xbee_buf[2]
                        total_len = 3 + length + 1  # 0x7E + 2 bytes len + length bytes data + 1 byte checksum
                        if len(self.xbee_buf) < total_len:
                            break  # wait for remaining frame bytes

                        frame = self.xbee_buf[:total_len]
                        del self.xbee_buf[:total_len]

                        # Verify Checksum: (sum of frame_data + checksum) & 0xFF == 0xFF
                        if (sum(frame[3:]) & 0xFF) != 0xFF:
                            self.get_logger().debug("XBee API checksum mismatch, skipping frame")
                            continue

                        frame_data = frame[3:-1]
                        frame_type = frame_data[0]
                        payload = None

                        if frame_type == 0x90 and len(frame_data) > 12:  # Zigbee / DigiMesh / XBee 3 RX
                            payload = frame_data[12:]
                        elif frame_type == 0x91 and len(frame_data) > 18:  # Explicit RX
                            payload = frame_data[18:]
                        elif frame_type == 0x80 and len(frame_data) > 11:  # 802.15.4 64-bit RX
                            payload = frame_data[11:]
                        elif frame_type == 0x81 and len(frame_data) > 5:   # 802.15.4 16-bit RX
                            payload = frame_data[5:]

                        if payload:
                            self.gps_ser.write(payload)
                            self.rtcm_pkt_count += 1
                            self.xbee_total_bytes += len(payload)
                            self.xbee_last_rx_time = _time.monotonic()

                            if not self.xbee_connected:
                                self.xbee_connected = True
                                self.get_logger().info(
                                    f"✅ XBee API RF link ESTABLISHED (Type 0x{frame_type:02X}) — RTCM injecting!"
                                )

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

                        self.gps_ser.write(rtcm_pkt)
                        self.rtcm_pkt_count += 1
                        self.xbee_total_bytes += len(rtcm_pkt)
                        self.xbee_last_rx_time = _time.monotonic()

                        if not self.xbee_connected:
                            self.xbee_connected = True
                            self.get_logger().info("✅ XBee Transparent link ESTABLISHED — RTCM injecting!")

                    else:
                        # Discard stray non-framing bytes
                        del self.xbee_buf[0]

                _time.sleep(0.005)
            except Exception as e:
                self.get_logger().warn(f"RTCM inject error: {e}")
                _time.sleep(0.01)

    # ---------------- NMEA PARSING ----------------
    def publish_nmea_loop(self):
        """
        Reads NMEA sentences from the rover LC29H.
        The LC29H outputs multi-constellation GGA using the $GN talker prefix,
        so we match on msgID ('GGA') rather than the full identity string —
        this covers both $GNGGA and $GPGGA transparently.

        Quality codes (from LC29H datasheet, consistent with base station):
            0 = No fix      — skip
            1 = GPS         — publish
            2 = DGPS        — publish (base RTCM arriving, ambiguity not resolved)
            4 = RTK Fixed   — publish (cm-level accuracy 🔥)
            5 = RTK Float   — publish (sub-metre accuracy ⚡)
        """
        nmr = NMEAReader(self.gps_ser)
        while rclpy.ok():
            try:
                _, parsed = nmr.read()

                # msgID matches 'GGA' for both $GNGGA and $GPGGA
                if parsed and parsed.msgID == 'GGA':
                    quality = getattr(parsed, 'quality', 0)
                    self.current_quality = quality

                    if quality in (1, 2, 4, 5):  # any valid fix
                        lat = float(parsed.lat)
                        lon = float(parsed.lon)
                        alt = float(getattr(parsed, 'alt', float('nan')))

                        self.current_lat = lat
                        self.current_lon = lon
                        self.current_alt = alt

                        msg = NavSatFix()
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.header.frame_id = 'gps_link'

                        msg.latitude  = lat
                        msg.longitude = lon
                        msg.altitude  = alt

                        # STATUS_GBAS_FIX covers RTK; STATUS_FIX covers GPS/DGPS
                        if quality in (4, 5):
                            msg.status.status = NavSatStatus.STATUS_GBAS_FIX
                        else:
                            msg.status.status = NavSatStatus.STATUS_FIX
                        msg.status.service = NavSatStatus.SERVICE_GPS

                        msg.position_covariance = [0.0] * 9
                        msg.position_covariance_type = (
                            NavSatFix.COVARIANCE_TYPE_UNKNOWN
                        )

                        self.fix_pub.publish(msg)

                        if quality == 4:
                            gmaps = f"{lat:.8f}, {lon:.8f}"
                            self.get_logger().info(f"🔥 RTK FIXED! Google Maps → {gmaps}")

            except Exception as e:
                self.get_logger().debug(f"NMEA parse error: {e}")

    # ---------------- STATUS LOG ----------------
    def display_status(self):
        """
        Logs a one-liner every second with coordinates and XBee link health.

        XBee status:
          ⏳ WAITING  — port open, no data received yet (base not transmitting)
          ✅ ACTIVE   — data received within the last 5 seconds
          ❌ LOST     — was connected but silent for >5 s (RF link dropped)
        """
        import time as _time

        ts   = datetime.datetime.now().strftime("%H:%M:%S")
        qual = FIX_LABELS.get(self.current_quality, f"Q{self.current_quality}")
        pkts = self.rtcm_pkt_count
        now  = _time.monotonic()

        # Coordinate format for quick copy-paste into Google Maps
        if self.current_lat is not None and self.current_lon is not None:
            gmaps = f"{self.current_lat:.8f}, {self.current_lon:.8f}"
        else:
            gmaps = "waiting for fix..."

        # --- XBee link status ---
        if self.xbee_last_rx_time is None:
            xbee_status = "⏳ XBee: WAITING for base station..."
        elif (now - self.xbee_last_rx_time) > self._XBEE_TIMEOUT_SEC:
            silent_for = int(now - self.xbee_last_rx_time)
            xbee_status = f"❌ XBee: LINK LOST ({silent_for}s no data)"
            # Reset so reconnection is announced cleanly
            if self.xbee_connected:
                self.xbee_connected = False
                self.get_logger().warn(
                    "❌ XBee RF link LOST — no data from base station!"
                )
        else:
            xbee_status = f"✅ XBee: ACTIVE ({self.xbee_total_bytes} B recv)"

        self.get_logger().info(
            f"[{ts}] {qual} | {gmaps} | RTCM pkts: {pkts} | {xbee_status}"
        )

    # ---------------- CLEANUP ----------------
    def destroy_node(self):
        try:
            if self.xbee_ser and self.xbee_ser.is_open:
                self.xbee_ser.close()
        except Exception:
            pass
        try:
            if self.gps_ser and self.gps_ser.is_open:
                self.gps_ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RTKBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()