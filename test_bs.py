import serial
import time
import sys

# Update this if your port changes
PORT = '/dev/ttyUSB0'
BAUD = 115200

try:
    # Open the serial port
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print(f"Connected to Base Station on {PORT} @ {BAUD} baud.")
    
    # 1. Enable Survey-In mode (300 seconds, 15 meters)
    ser.write(b'$PQTMCFGSVIN,W,1,300,15,0,0,0*15\r\n')
    time.sleep(0.1)
    
    # 2. Enable ASCII status output every 1 second
    ser.write(b'$PQTMCFGMSGRATE,W,PQTMSVINSTATUS,1,1*58\r\n')
    time.sleep(0.1)
    
    print("Commands sent successfully. Listening for Survey-In status...")
    print("Keep the antenna outside and stationary. Press Ctrl+C to stop.")
    print("-" * 60)
    
    while True:
        raw_line = ser.readline()
        if raw_line:
            # Decode ignoring the raw RTCM3 binary bytes
            line = raw_line.decode('ascii', errors='ignore').strip()
            
            # Print only the Survey-In configuration and status lines
            if line.startswith('$'):
                print(line)

except serial.SerialException as e:
    print(f"Serial Error: {e}")
    print("Check if the port is correct and you have dialout permissions.")
except KeyboardInterrupt:
    print("\nStopping...")
    # Optional cleanup: Turn off the text output so the port returns to pure RTCM3
    if 'ser' in locals() and ser.is_open:
        ser.write(b'$PQTMCFGMSGRATE,W,PQTMSVINSTATUS,0,1*59\r\n')
        ser.close()
    sys.exit(0)