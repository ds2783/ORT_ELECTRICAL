# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

# Simple GPS module demonstration with antenna status output.

import time
import board
import busio
import adafruit_gps
import serial

# Create a serial connection for the GPS connection
uart = serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=5)
# For USB-based GPS:
# uart = serial.Serial("/dev/ttyUSB0", baudrate=9600, timeout=10)

# Create a GPS module instance
gps = adafruit_gps.GPS(uart, debug=False)

# Send antenna status command
gps.send_command(adafruit_gps.PGCMD_ANTENNA)

# Configure GPS to send basic info
gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
# Set update rate to 1 Hz
gps.send_command(b"PMTK220,1000")

# Main loop
last_print = time.monotonic()
while True:
    gps.update()
    current = time.monotonic()
    if current - last_print >= 1.0:
        last_print = current

        print("=" * 40)
        # Always print antenna status
        if gps.antenna is not None:
            if gps.antenna == 0:
                print("Antenna status: Internal")
            elif gps.antenna == 1:
                print("Antenna status: External")
            else:
                print(f"Antenna status: Unknown ({gps.antenna})")
        else:
            print("Antenna status: Not available yet")

        if not gps.has_fix:
            print("Waiting for fix...")
            continue

        # We have a fix, print details
        print(
            "Fix timestamp: {}/{}/{} {:02}:{:02}:{:02}".format(
                gps.timestamp_utc.tm_mon,
                gps.timestamp_utc.tm_mday,
                gps.timestamp_utc.tm_year,
                gps.timestamp_utc.tm_hour,
                gps.timestamp_utc.tm_min,
                gps.timestamp_utc.tm_sec,
            )
        )
        print(f"Latitude: {gps.latitude:.6f} degrees")
        print(f"Longitude: {gps.longitude:.6f} degrees")
        print(f"Precise Latitude: {gps.latitude_degrees} degs, {gps.latitude_minutes:2.4f} mins")
        print(f"Precise Longitude: {gps.longitude_degrees} degs, {gps.longitude_minutes:2.4f} mins")
        print(f"Fix quality: {gps.fix_quality}")
        if gps.satellites is not None:
            print(f"# satellites: {gps.satellites}")
        if gps.altitude_m is not None:
            print(f"Altitude: {gps.altitude_m} meters")
        if gps.speed_knots is not None:
            print(f"Speed: {gps.speed_knots} knots")
        if gps.speed_kmh is not None:
            print(f"Speed: {gps.speed_kmh} km/h")
        if gps.track_angle_deg is not None:
            print(f"Track angle: {gps.track_angle_deg} degrees")
        if gps.horizontal_dilution is not None:
            print(f"Horizontal dilution: {gps.horizontal_dilution}")
        if gps.height_geoid is not None:
            print(f"Height geoid: {gps.height_geoid} meters")
