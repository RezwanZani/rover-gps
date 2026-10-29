import time
import threading
import datetime
import serial
from pynmeagps import NMEAReader

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from rtcm_msgs.msg import Message


class RoverGPSNode(Node):
    """
    Standalone ROS 2 GPS Node.

    Subscribes to ROS topic /rtcm (from ntrip_client or network), injects RTCM into
    the LC29H RTK GPS module on static port /dev/rover_rtk, and publishes /fix.
    Fault-tolerant: auto-connects and auto-reconnects if disconnected.
    """

    def __init__(self):
        super().__init__('rover_gps_node')

        # ---------------- PARAMETERS ----------------
        self.declare_parameter('gps_port', '/dev/rover_rtk')
        self.declare_parameter('gps_baud', 115200)
        self.declare_parameter('frame_id', 'gps_link')

        self.gps_port = self.get_parameter('gps_port').value
        self.gps_baud = self.get_parameter('gps_baud').value
        self.frame_id = self.get_parameter('frame_id').value

        # ---------------- PUB/SUB ----------------
        self.fix_pub = self.create_publisher(NavSatFix, '/fix', 10)

        self.rtcm_sub = self.create_subscription(
            Message,
            '/rtcm',
            self.rtcm_callback,
            10,
        )

        # ---------------- STATE & THREADING ----------------
        self.lock       = threading.Lock()
        self.stop_event = threading.Event()

        self.gps_ser        = None
        self.gps_connected  = False

        self.current_lat     = None
        self.current_lon     = None
        self.current_alt     = float('nan')
        self.current_quality = 0
        self.rtcm_pkt_count  = 0
        self._last_warn_time = 0.0

        self.fix_labels = {
            0: "NO FIX",
            1: "GPS",
            2: "DGPS",
            4: "RTK FIXED 🔥",
            5: "RTK FLOAT ⚡",
        }

        # ---------------- START BACKGROUND WORKER ----------------
        self.gps_thread = threading.Thread(
            target=self._gps_worker_loop, name="gps_worker", daemon=True
        )
        self.gps_thread.start()

        # 1 Hz Status Dashboard
        self.create_timer(1.0, self.display_status)
        self.get_logger().info(f"Rover GPS Node started for {self.gps_port} @ {self.gps_baud} (Auto-reconnect active)")

    # ---------------- RTCM INJECTION CALLBACK ----------------
    def rtcm_callback(self, msg):
        """Safely injects incoming ROS /rtcm message bytes into GPS serial."""
        with self.lock:
            if self.gps_ser and self.gps_ser.is_open:
                try:
                    self.gps_ser.write(bytes(msg.message))
                    self.rtcm_pkt_count += 1
                except (serial.SerialException, OSError, IOError) as e:
                    self.get_logger().debug(f"RTCM write error: {e}")

    # ---------------- SERIAL CLEANUP ----------------
    def _close_gps(self):
        with self.lock:
            if self.gps_ser:
                try:
                    self.gps_ser.close()
                except Exception:
                    pass
                self.gps_ser = None
            self.gps_connected = False
            self.current_lat = None
            self.current_lon = None
            self.current_alt = float('nan')
            self.current_quality = 0

    # ---------------- GPS WORKER LOOP ----------------
    def _gps_worker_loop(self):
        while not self.stop_event.is_set():
            # 1. Connect or Reconnect
            if not self.gps_connected or self.gps_ser is None:
                try:
                    ser = serial.Serial(self.gps_port, self.gps_baud, timeout=1.0)
                    with self.lock:
                        self.gps_ser = ser
                        self.gps_connected = True
                    self.get_logger().info(f"🔌 GPS RTK CONNECTED on {self.gps_port} @ {self.gps_baud}")
                except (serial.SerialException, FileNotFoundError, PermissionError, OSError) as e:
                    now = time.monotonic()
                    if now - self._last_warn_time >= 5.0:
                        self.get_logger().warn(f"⏳ Waiting for GPS RTK at {self.gps_port} ({e})")
                        self._last_warn_time = now
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
                        raise e  # Device disconnected, bubble up to reconnect loop
                    except Exception as e:
                        self.get_logger().debug(f"GPS NMEA parse warning: {e}")
                        continue

                    if parsed and parsed.msgID == 'GGA':
                        self._handle_parsed_gga(parsed)

            except (serial.SerialException, OSError, IOError) as e:
                self.get_logger().error(f"❌ GPS RTK DISCONNECTED from {self.gps_port}: {e}. Retrying in background...")
                self._close_gps()
                time.sleep(1.0)
            except Exception as e:
                self.get_logger().error(f"Unexpected GPS read error: {e}")
                self._close_gps()
                time.sleep(1.0)

    # ---------------- NMEA PARSING & FIX PUBLISHING ----------------
    def _handle_parsed_gga(self, parsed):
        try:
            quality = getattr(parsed, 'quality', 0)
            lat = float(parsed.lat) if getattr(parsed, 'lat', None) is not None else None
            lon = float(parsed.lon) if getattr(parsed, 'lon', None) is not None else None
            alt = float(getattr(parsed, 'alt', float('nan')))

            with self.lock:
                self.current_lat = lat
                self.current_lon = lon
                self.current_alt = alt
                self.current_quality = quality

            if lat is not None and lon is not None:
                msg = NavSatFix()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.frame_id

                if quality in (4, 5):
                    msg.status.status = NavSatStatus.STATUS_GBAS_FIX
                elif quality in (1, 2):
                    msg.status.status = NavSatStatus.STATUS_FIX
                else:
                    msg.status.status = NavSatStatus.STATUS_NO_FIX

                msg.status.service = NavSatStatus.SERVICE_GPS
                msg.latitude = lat
                msg.longitude = lon
                msg.altitude = alt

                msg.position_covariance = [0.0] * 9
                msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

                self.fix_pub.publish(msg)

                if quality == 4:
                    self.get_logger().info(f"🔥 RTK FIXED! Google Maps → {lat:.8f}, {lon:.8f}")

        except Exception as e:
            self.get_logger().debug(f"Error handling GGA message: {e}")

    # ---------------- STATUS PRINT ----------------
    def display_status(self):
        ts = datetime.datetime.now().strftime("%H:%M:%S")

        with self.lock:
            connected = self.gps_connected
            lat = self.current_lat
            lon = self.current_lon
            qual_code = self.current_quality
            pkts = self.rtcm_pkt_count

        if not connected:
            self.get_logger().info(f"[{ts}] ❌ GPS RTK: DISCONNECTED ({self.gps_port}) | RTCM pkts: {pkts}")
        elif lat is not None and lon is not None:
            qual_label = self.fix_labels.get(qual_code, f"Q{qual_code}")
            gmaps = f"{lat:.8f}, {lon:.8f}"
            self.get_logger().info(f"[{ts}] {qual_label} | {gmaps} | RTCM pkts: {pkts}")
        else:
            self.get_logger().info(f"[{ts}] searching for satellites... | RTCM pkts: {pkts}")

    # ---------------- CLEANUP ----------------
    def destroy_node(self):
        self.stop_event.set()
        self._close_gps()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RoverGPSNode()
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
