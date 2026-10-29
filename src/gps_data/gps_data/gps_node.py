import time
import threading
import datetime

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix, NavSatStatus
from rtcm_msgs.msg import Message  #Correct message type
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import serial
from pynmeagps import NMEAReader


class RoverGPSNode(Node):
    def __init__(self):
        super().__init__('rover_gps_node')

        # ---------------- PARAMETERS ----------------
        self.declare_parameter('gps_port', '/dev/ttyUSB_GPS')
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

        # ---------------- STATE ----------------
        self.lock = threading.Lock()
        self.stop_event = threading.Event()

        self.current_lat = None
        self.current_lon = None
        self.current_alt = float('nan')
        self.current_quality = 0
        self.rtcm_pkt_count = 0

        self.fix_labels = {
            0: "NO FIX",
            1: "GPS",
            2: "DGPS",
            4: "RTK FIXED 🔥",
            5: "RTK FLOAT ⚡"
        }

        self.gps_ser = None

        # ---------------- INIT ----------------
        self.setup_serial()
        self.start_gps_thread()

        self.create_timer(1.0, self.display_status)

    # ---------------- SERIAL SETUP ----------------
    def setup_serial(self):
        try:
            self.gps_ser = serial.Serial(
                self.gps_port,
                self.gps_baud,
                timeout=1
            )
            self.get_logger().info(
                f'GPS connected on {self.gps_port} @ {self.gps_baud}'
            )
        except Exception as e:
            self.get_logger().error(f'GPS serial failed: {e}')
            raise

    # ---------------- RTCM CALLBACK ----------------
    def rtcm_callback(self, msg):
        try:
            if self.gps_ser and self.gps_ser.is_open:
                # Inject RTCM into GPS
                self.gps_ser.write(bytes(msg.message))

                with self.lock:
                    self.rtcm_pkt_count += 1

        except Exception as e:
            self.get_logger().warn(f'RTCM write error: {e}')

    # ---------------- GPS THREAD ----------------
    def start_gps_thread(self):
        t = threading.Thread(target=self.gps_loop, daemon=True)
        t.start()

    def gps_loop(self):
        nmr = NMEAReader(self.gps_ser)

        while not self.stop_event.is_set():
            try:
                if self.gps_ser.in_waiting > 0:
                    _, parsed = nmr.read()

                    if parsed and parsed.identity in ("GNGGA", "GPGGA"):
                        with self.lock:
                            self.current_lat = parsed.lat
                            self.current_lon = parsed.lon
                            self.current_alt = getattr(parsed, 'alt', float('nan'))
                            self.current_quality = getattr(parsed, 'quality', 0)

                        self.publish_fix()

            except Exception as e:
                self.get_logger().debug(f'GPS read error: {e}')
                time.sleep(0.05)

    # ---------------- PUBLISH FIX ----------------
    def publish_fix(self):
        with self.lock:
            lat = self.current_lat
            lon = self.current_lon
            alt = self.current_alt
            quality = self.current_quality

        if lat is None or lon is None:
            return

        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # Status mapping
        if quality == 0:
            msg.status.status = NavSatStatus.STATUS_NO_FIX
        else:
            msg.status.status = NavSatStatus.STATUS_FIX

        msg.status.service = NavSatStatus.SERVICE_GPS

        msg.latitude = float(lat)
        msg.longitude = float(lon)
        msg.altitude = float(alt)

        msg.position_covariance = [0.0] * 9
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self.fix_pub.publish(msg)

    # ---------------- STATUS PRINT ----------------
    def display_status(self):
        with self.lock:
            lat = self.current_lat
            lon = self.current_lon
            qual = self.fix_labels.get(
                self.current_quality,
                f"Q{self.current_quality}"
            )
            pkts = self.rtcm_pkt_count

        ts = datetime.datetime.now().strftime("%H:%M:%S")

        if lat is not None and lon is not None:
            # Decimal-degree format accepted directly by Google Maps search bar
            gmaps = f"{float(lat):.8f}, {float(lon):.8f}"
            self.get_logger().info(
                f"[{ts}] {qual} | {gmaps} | RTCM pkts: {pkts}"
            )
        else:
            self.get_logger().info(
                f"[{ts}] {qual} | waiting for fix... | RTCM pkts: {pkts}"
            )

    # ---------------- CLEANUP ----------------
    def destroy_node(self):
        self.stop_event.set()

        try:
            if self.gps_ser and self.gps_ser.is_open:
                self.gps_ser.close()
        except:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = RoverGPSNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
