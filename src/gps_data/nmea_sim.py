#!/usr/bin/env python3
"""
NMEA Simulator for LC29H RTK testing.

Writes GNGGA sentences to the master side of a socat virtual serial pair,
simulating a rover GPS module. Fix quality cycles through GPS → DGPS → RTK FLOAT → RTK FIXED
to let you verify the full pipeline.

Usage:
    # 1. Create virtual serial pair first:
    #    socat -d -d pty,raw,echo=0,link=/tmp/gps_master pty,raw,echo=0,link=/tmp/gps_slave
    #
    # 2. Run this script:
    #    python3 nmea_sim.py /tmp/gps_master
    #
    # 3. Point gps_node at the slave port:
    #    ros2 run gps_data gps_node --ros-args -p gps_port:=/tmp/gps_slave -p gps_baud:=115200
"""

import serial
import time
import math
import sys

PORT = sys.argv[1] if len(sys.argv) > 1 else "/tmp/gps_master"
BAUD = 115200

# Base coordinates (Dhaka, Bangladesh — adjust to your location)
BASE_LAT = 23.8103
BASE_LON = 90.4125
BASE_ALT = 10.0


def nmea_checksum(sentence: str) -> str:
    chk = 0
    for c in sentence:
        chk ^= ord(c)
    return f"{chk:02X}"


def dd_to_nmea(deg: float, is_lat: bool):
    """Convert decimal degrees to NMEA ddmm.mmmmm format."""
    d = int(abs(deg))
    m = (abs(deg) - d) * 60
    if is_lat:
        return f"{d:02d}{m:08.5f}", "N" if deg >= 0 else "S"
    else:
        return f"{d:03d}{m:08.5f}", "E" if deg >= 0 else "W"


def make_gga(lat: float, lon: float, quality: int = 4,
             alt: float = BASE_ALT, sats: int = 12) -> str:
    """Build a complete GNGGA sentence with valid checksum."""
    lat_str, lat_dir = dd_to_nmea(lat, True)
    lon_str, lon_dir = dd_to_nmea(lon, False)
    utc = time.strftime("%H%M%S.00", time.gmtime())
    body = (
        f"GNGGA,{utc},"
        f"{lat_str},{lat_dir},"
        f"{lon_str},{lon_dir},"
        f"{quality},{sats:02d},0.8,"
        f"{alt:.1f},M,0.0,M,,"
    )
    return f"${body}*{nmea_checksum(body)}\r\n"


# Quality progression: simulates module achieving RTK fix over time
QUALITY_SEQUENCE = [
    (0,  5,  "NO FIX  "),
    (1,  10, "GPS     "),
    (2,  5,  "DGPS    "),
    (5,  10, "RTK FLOAT"),
    (4,  999, "RTK FIXED 🔥"),
]


def main():
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print(f"[nmea_sim] Writing NMEA to {PORT} @ {BAUD} baud")
    print(f"[nmea_sim] Base coords: {BASE_LAT:.5f}, {BASE_LON:.5f}")
    print("[nmea_sim] Press Ctrl+C to stop\n")

    t = 0
    seq_idx = 0
    seq_count = 0

    try:
        while True:
            quality, duration, label = QUALITY_SEQUENCE[seq_idx]

            # Small circular drift to make the scatter plot interesting
            drift_m = 0.001 if quality == 4 else 0.5  # cm-level for RTK, ~50cm otherwise
            lat = BASE_LAT + (drift_m / 111000) * math.sin(t * 0.3)
            lon = BASE_LON + (drift_m / 111000) * math.cos(t * 0.3)

            sentence = make_gga(lat, lon, quality=quality)
            ser.write(sentence.encode())
            print(f"  [{label}] {sentence.strip()}")

            t += 1
            seq_count += 1
            if seq_count >= duration and seq_idx < len(QUALITY_SEQUENCE) - 1:
                seq_idx += 1
                seq_count = 0
                print(f"\n  ── Advancing to: {QUALITY_SEQUENCE[seq_idx][2]} ──\n")

            time.sleep(1.0)

    except KeyboardInterrupt:
        print("\n[nmea_sim] Stopped.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
