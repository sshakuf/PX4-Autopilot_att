#!/usr/bin/env python3
"""
Minimal MAVSDK reader for the IR camera report.

`ir_camera_report` is a custom uORB topic, so no MAVLink client can subscribe to
it directly. The IR_CAM_DEBUG_MAVLINK block in IrCam.cpp mirrors every report
into `debug_array`, which PX4 streams as DEBUG_FLOAT_ARRAY named "IRCAM" at
50 Hz over USB (MAVLINK_MODE_CONFIG). This reads it via MAVSDK's mavlink_direct
plugin, which exposes arbitrary MAVLink messages as JSON.

Requires the IR_CAM_DEBUG_MAVLINK firmware to be flashed.

Note: mavsdk_server takes exclusive ownership of the serial port, so this cannot
run at the same time as ir_flow_monitor.py. For the actual bench work use
ir_flow_monitor.py -- it draws the picture and records CSV. This file exists to
show the MAVSDK path works.

Usage:
    python3 mavsdk_read_ircam.py
    python3 mavsdk_read_ircam.py --connect serial:///dev/cu.usbmodem01:57600
    python3 mavsdk_read_ircam.py --connect udp://:14540
"""

import argparse
import asyncio
import glob
import json
import sys

try:
    from mavsdk import System
except ImportError:
    sys.exit("mavsdk missing:  pip install mavsdk")

# must match IrCam::publishDebugArray()
ARRAY_NAME = "IRCAM"
LAYOUT = ["dx_px", "dy_px", "angle_x", "angle_y", "valid", "spot_id", "spot_score"]


def autodetect():
    for pat in ("/dev/cu.usbmodem*", "/dev/tty.usbmodem*",
                "/dev/serial/by-id/*PX4*", "/dev/ttyACM*"):
        hits = sorted(glob.glob(pat))
        if hits:
            return f"serial://{hits[0]}:57600"
    return None


async def run(address):
    drone = System()
    print(f"connecting to {address} ...")
    await drone.connect(system_address=address)

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("connected")
            break

    n = 0
    async for msg in drone.mavlink_direct.message("DEBUG_FLOAT_ARRAY"):
        fields = json.loads(msg.fields_json)
        # pymavlink/MAVSDK may or may not strip the trailing NULs
        if fields.get("name", "").rstrip("\x00") != ARRAY_NAME:
            continue
        data = fields["data"]
        rec = dict(zip(LAYOUT, data[:len(LAYOUT)]))
        n += 1
        print(f"[{n:5d}] dx={rec['dx_px']:+8.1f} dy={rec['dy_px']:+8.1f}  "
              f"ax={rec['angle_x']:+8.4f} ay={rec['angle_y']:+8.4f}  "
              f"valid={'Y' if rec['valid'] > 0.5 else 'n'} "
              f"id={int(rec['spot_id'])} score={int(rec['spot_score'])}")


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--connect", default=None,
                   help="MAVSDK address, e.g. serial:///dev/cu.usbmodem01:57600 "
                        "(default: auto-detect USB)")
    args = p.parse_args()
    address = args.connect or autodetect()
    if not address:
        sys.exit("no USB device found; pass --connect")
    try:
        asyncio.run(run(address))
    except KeyboardInterrupt:
        print("\nstopped")


if __name__ == "__main__":
    main()
