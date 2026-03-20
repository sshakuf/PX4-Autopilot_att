#!/bin/bash

# Serial port config — adjust to match your device
SERIAL_PORT="${1:-/dev/tty.usbmodem01}"
BAUD_RATE="${2:-921600}"

echo "======================================"
echo "Starting MAVProxy on Mac"
echo "======================================"
echo "Connecting to PX4 via serial: $SERIAL_PORT @ $BAUD_RATE"
echo "Forwarding to:"
echo "  - UDP:14560 (for MAVSDK scripts)"
echo "  - UDP:14550 (for QGroundControl)"
echo "======================================"
echo "Usage: $0 [serial_port] [baud_rate]"
echo "  e.g. $0 /dev/tty.usbserial-0001 921600"
echo "======================================"
echo "run : ./Tools/mavlink_shell.py 0.0.0.0:14560"

# Run MAVProxy
# --master connects to the flight controller via serial port
# --out creates outputs for other applications
mavproxy.py \
    --master="$SERIAL_PORT,$BAUD_RATE" \
    --out=udp:127.0.0.1:14560 \
    --out=udp:127.0.0.1:14550 \
    --out=udp:127.0.0.1:14552 \
    --out=udp:100.79.96.45:14551

echo "MAVProxy stopped"

