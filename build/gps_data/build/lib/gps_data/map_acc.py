#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class DriftPlot(Node):
    def __init__(self):
        super().__init__('drift_plot')

        self.ref_lat = None
        self.ref_lon = None

        self.east = []
        self.north = []

        self.create_subscription(
            NavSatFix,
            '/fix',
            self.callback,
            10
        )

        # plot setup
        self.fig, self.ax = plt.subplots(figsize=(6, 6))
        self.setup_plot()

        self.anim = FuncAnimation(self.fig, self.update_plot, interval=500)

        self.get_logger().info("Drift plot started. Waiting for GPS data...")

    def setup_plot(self):
        self.ax.set_title("RTK GPS Drift (cm-level)")
        self.ax.set_xlabel("East (m)")
        self.ax.set_ylabel("North (m)")
        self.ax.set_aspect('equal')
        self.ax.grid(True)

        # draw cm rings
        for r in [0.01, 0.02, 0.05, 0.1]:  # 1,2,5,10 cm
            circle = plt.Circle((0, 0), r, fill=False, linestyle='--')
            self.ax.add_patch(circle)

        # center lines
        self.ax.axhline(0)
        self.ax.axvline(0)

        self.ax.set_xlim(-0.2, 0.2)
        self.ax.set_ylim(-0.2, 0.2)

    def callback(self, msg):
        if msg.latitude == 0.0 or msg.longitude == 0.0:
            return

        if self.ref_lat is None:
            self.ref_lat = msg.latitude
            self.ref_lon = msg.longitude
            self.get_logger().info("Reference point set")
            return

        # simple conversion (no Earth radius, no complexity)
        dlat = msg.latitude - self.ref_lat
        dlon = msg.longitude - self.ref_lon

        north = dlat * 111000     # meters per degree latitude
        east  = dlon * 111000     # approx

        self.east.append(east)
        self.north.append(north)

        # keep last 1000 points
        if len(self.east) > 1000:
            self.east.pop(0)
            self.north.pop(0)

    def update_plot(self, _):
        self.ax.collections.clear()

        if self.east:
            self.ax.scatter(self.east, self.north, s=10)

        return []


def main():
    rclpy.init()
    node = DriftPlot()

    import threading
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    try:
        plt.show()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()