#!/usr/bin/env python3
"""
lift_console.py - bench-test the lift on the Pico WITHOUT ROS (Phase 8, Stage B).

Your Pico prints 'e' encoder lines 50 times a second, which buries everything
in miniterm. This console hides them and shows only:
  * '#' messages (lift, arming, faults)
  * the arm state when it changes
  * the lift status when it changes

  python3 lift_console.py            (uses /dev/pico)
  python3 lift_console.py /dev/ttyACM0

Then type commands and press Enter, e.g.:
  a 1        arm (closes the relay -> the lift gets power)
  l ?        lift settings + switch reading
  l h        home          l p   park          l g 80   go to 80 mm
  l s        stop          a 0   disarm        quit     leave (stops + disarms)

Stop pico_bridge first: only one program can use /dev/pico at a time.
Needs only python3-serial (installed in Phase 5).
"""
import sys
import threading
import time

import serial

LIFT_STATES = {0: 'IDLE', 1: 'MOVING', 2: 'HOMING', 3: 'PARKING', 4: 'FAULT'}
LIFT_FAULTS = {0: '', 1: '  FAULT: switch never found', 2: '  FAULT: switch stuck pressed',
               3: '  FAULT: switch not found on slow approach'}
ARM = {0: 'DISARMED', 1: 'ARMING', 2: 'ARMED', 3: 'DISARMING'}


def reader(ser, stop):
    buf = b''
    last_lift = None
    last_arm = None
    last_print = 0.0
    while not stop.is_set():
        try:
            data = ser.read(256)
        except (serial.SerialException, OSError) as err:
            print(f'\n!! lost the Pico: {err}')
            stop.set()
            return
        buf += data
        while b'\n' in buf:
            raw, buf = buf.split(b'\n', 1)
            line = raw.decode('ascii', errors='ignore').strip()
            if not line or line.startswith('e '):
                continue
            if line.startswith('#'):
                print(f'  {line}')
            elif line.startswith('s '):
                parts = line.split()
                if len(parts) >= 5:
                    arm = ARM.get(int(parts[2]), parts[2])
                    if arm != last_arm:
                        print(f'  [arm] {arm}   (flags {parts[1]}, battery {parts[3]} V, motor bus {parts[4]} V)')
                        last_arm = arm
            elif line.startswith('l '):
                p = line.split()
                if len(p) < 7:
                    continue
                key = (p[1], p[2], p[5], p[6])
                now = time.monotonic()
                moving = p[1] in ('1', '2', '3')
                if key != last_lift or (moving and now - last_print > 1.0):
                    print(f'  [lift] {LIFT_STATES.get(int(p[1]), p[1]):<8} homed={p[2]} '
                          f'pos={p[3]:>7} mm  target={p[4]:>7} mm  '
                          f'switch={"PRESSED" if p[5] == "1" else "free"}'
                          f'{LIFT_FAULTS.get(int(p[6]), "")}')
                    last_lift = key
                    last_print = now


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/pico'
    try:
        ser = serial.Serial(port, 115200, timeout=0.1)
    except (serial.SerialException, OSError) as err:
        print(f'Cannot open {port}: {err}\n(Is pico_bridge or miniterm still running?)')
        sys.exit(1)
    print(f'Connected to {port}. Type commands (a 1, l ?, l h, l g 80, l p, l s, a 0), "quit" to leave.')
    stop = threading.Event()
    threading.Thread(target=reader, args=(ser, stop), daemon=True).start()
    try:
        while not stop.is_set():
            cmd = input().strip()
            if cmd in ('quit', 'exit', 'q'):
                break
            if cmd:
                ser.write((cmd + '\n').encode('ascii'))
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        print('Stopping the lift and disarming...')
        try:
            ser.write(b'l s\na 0\n')
            time.sleep(0.2)
        except Exception:
            pass
        stop.set()
        ser.close()


if __name__ == '__main__':
    main()
