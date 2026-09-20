#!/usr/bin/env python3
"""
ld06_check.py  --  proves the LD06 is sending data, with NO ROS involved.

    ros2 run hospital_robot_base ld06_check            (uses /dev/lidar)
    ros2 run hospital_robot_base ld06_check /dev/ttyAMA0

Each LD06 packet is 47 bytes and starts with 0x54 0x2C. We count packets,
read the spin speed, and print the distance of the closest point.
Expect roughly 370-470 packets/s and a speed near 3600 deg/s (10 Hz).
Ctrl+C to stop.
"""

import struct
import sys
import time

import serial

PACKET_LEN = 47


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/lidar'
    try:
        ser = serial.Serial(port, 230400, timeout=0.1)
    except serial.SerialException as err:
        print(f'Could not open {port}: {err}')
        print('Check: does the port exist (ls -l /dev/lidar /dev/ttyAMA0)? '
              'Are you in the dialout group (groups)?')
        return

    print(f'Listening on {port} at 230400 baud... (Ctrl+C to stop)')
    buf = b''
    packets = 0
    bad = 0
    total_bytes = 0
    speed = 0
    closest = None
    t0 = time.monotonic()
    try:
        while True:
            chunk = ser.read(512)
            total_bytes += len(chunk)
            buf += chunk
            while True:
                i = buf.find(b'\x54\x2c')
                if i < 0 or len(buf) - i < PACKET_LEN:
                    buf = buf[i:] if i >= 0 else buf[-1:]
                    break
                pkt = buf[i:i + PACKET_LEN]
                buf = buf[i + PACKET_LEN:]
                speed = struct.unpack_from('<H', pkt, 2)[0]
                if speed > 10000:          # nonsense = corrupted packet
                    bad += 1
                    continue
                packets += 1
                for n in range(12):
                    dist = struct.unpack_from('<H', pkt, 6 + 3 * n)[0]
                    if 20 < dist < 12000 and (closest is None or dist < closest):
                        closest = dist
            if time.monotonic() - t0 >= 1.0:
                print(f'{packets:4d} packets/s  | {bad:3d} bad | spin {speed:5d} deg/s '
                      f'(~{speed / 360:4.1f} Hz) | closest point '
                      f'{closest if closest else "-"} mm | {total_bytes} bytes/s')
                if total_bytes == 0:
                    print('   -> NO BYTES AT ALL: check Tx wire to pin 10, 5 V power, '
                          'GND, and that the serial console is disabled.')
                elif packets == 0:
                    print('   -> Bytes but no packets: wrong baud rate, or you are on the '
                          'mini-UART (dtoverlay=disable-bt missing).')
                packets = bad = total_bytes = 0
                closest = None
                t0 = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()


if __name__ == '__main__':
    main()
