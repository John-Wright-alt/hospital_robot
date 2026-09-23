#!/usr/bin/env python3
"""
esp_test.py - talk to the head-column ESP32 WITHOUT ROS.

Use it first, to prove the Pi (or your laptop) can reach the ESP32.
Only uses the Python standard library - nothing to install.

Examples
  python3 esp_test.py 192.168.1.50            # show live status (Ctrl+C to quit)
  python3 esp_test.py headcolumn.local park   # send "park", then watch
  python3 esp_test.py 192.168.1.50 up 20      # move up 20 mm, then watch

Commands: search, park, stop, home, wait, up <mm>, down <mm>, goto <mm>
"""
import json
import sys
import time
import urllib.parse
import urllib.request

COMMANDS = {"search", "park", "stop", "home", "wait", "up", "down", "goto"}


def get_json(host, path, timeout=2.0):
    with urllib.request.urlopen(f"http://{host}{path}", timeout=timeout) as r:
        return json.loads(r.read().decode("utf-8"))


def send(host, cmd, value=None):
    query = {"c": cmd}
    if value is not None:
        query["v"] = value
    return get_json(host, "/cmd?" + urllib.parse.urlencode(query))


def show(st):
    face = f"YES (error {st['error_mm']:+.0f} mm)" if st.get("face") else "no"
    line = (f"{st['state']:<11} height {st['height_mm']:6.1f} mm  "
            f"homed={'Y' if st['homed'] else 'N'}  parked={'Y' if st['parked'] else 'N'}  "
            f"switch={'PRESSED' if st['bottom_switch'] else 'free   '}  face={face:<18} "
            f"{st['fps']:4.1f} fps")
    if st.get("fault"):
        line += f"  FAULT: {st['fault']}"
    print(line)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    host = sys.argv[1]

    if len(sys.argv) >= 3:
        cmd = sys.argv[2].lower()
        if cmd not in COMMANDS:
            print(f"Unknown command '{cmd}'. Use one of: {', '.join(sorted(COMMANDS))}")
            sys.exit(1)
        value = sys.argv[3] if len(sys.argv) >= 4 else None
        try:
            print("Sent:", send(host, cmd, value))
        except Exception as e:
            print(f"Could not reach http://{host}/ : {e}")
            sys.exit(2)

    print("Watching status - press Ctrl+C to stop")
    while True:
        try:
            show(get_json(host, "/status"))
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"(no answer from {host}: {e})")
        try:
            time.sleep(0.5)
        except KeyboardInterrupt:
            break


if __name__ == "__main__":
    main()
