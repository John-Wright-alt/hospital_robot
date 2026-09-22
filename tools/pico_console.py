#!/usr/bin/env python3
"""
pico_console.py — talk to the Pico motor firmware from your Ubuntu PC.

WHY NOT JUST THE ARDUINO SERIAL MONITOR?
  The firmware stops the wheels if it does not receive a 'v' command at least
  every 0.5 s (that is the safety watchdog doing its job). You cannot type that
  fast, so this script re-sends your last 'v' command 20 times per second.
  (Close the Arduino Serial Monitor first — only one program can own the port.)

RUN IT
  python3 pico_console.py                  # finds the Pico automatically
  python3 pico_console.py --port /dev/ttyACM0
  python3 pico_console.py --log run1.csv   # also record every 'e' line

TYPE AT THE PROMPT
  a 1        arm (motor power on)            a 0    disarm
  v 1 1      both wheels 1 rad/s forward     (re-sent automatically)
  v 1 -1     spin on the spot
  x          stop: sends v 0 0 (and keeps sending zeros)
  pause      stop sending ANYTHING -> the firmware watchdog must stop the wheels
  show       toggle printing of every 'e' line (50 per second!)
  i  h  c  k <P> <I> <D>     passed straight to the Pico
  quit       stops the wheels, disarms, exits (Ctrl-C does the same)
"""
import argparse
import sys
import threading
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    sys.exit("pyserial is missing. Install it with:  sudo apt install python3-serial")

FLAG_NAMES = {
    0x0001: "ESTOP", 0x0002: "NO_VMOT", 0x0004: "CMD_TIMEOUT", 0x0008: "BATT_WARN",
    0x0010: "NO_BATT", 0x0020: "LOOP_SLOW", 0x0100: "BATT_CUTOFF*", 0x0200: "STALL_L*",
    0x0400: "STALL_R*", 0x0800: "ENCODER_L*", 0x1000: "ENCODER_R*", 0x2000: "RELAY*",
}
ARM_NAMES = {0: "DISARMED", 1: "ARMING", 2: "ARMED", 3: "DISARMING"}


def decode_flags(flags):
    names = [n for bit, n in FLAG_NAMES.items() if flags & bit]
    return " ".join(names) if names else "none"


def find_pico_port():
    ports = list(serial.tools.list_ports.comports())
    picos = [p for p in ports if p.vid == 0x2E8A]          # Raspberry Pi's USB vendor ID
    if picos:
        return picos[0].device
    listing = "\n".join(f"   {p.device}  {p.description}" for p in ports) or "   (none)"
    sys.exit("Could not find a Pico. Is it plugged in (with a DATA cable)?\n"
             "Serial ports I can see:\n" + listing + "\nTry:  --port /dev/ttyACM0")


class Console:
    def __init__(self, port, log_path):
        self.ser = serial.Serial(port, 115200, timeout=0.1)
        self.lock = threading.Lock()
        self.running = True
        self.held = None            # the 'v' line being repeated, or None
        self.paused = False
        self.show_e = False
        self.last_e = None
        self.last_status = None
        self.log = open(log_path, "w") if log_path else None
        if self.log:
            self.log.write("host_time,ticks_l,ticks_r,vel_l,vel_r,pico_micros\n")

    def send(self, text):
        with self.lock:
            self.ser.write((text.strip() + "\n").encode())

    # ------------------------------------------------------------ threads
    def reader(self):
        buf = b""
        while self.running:
            try:
                buf += self.ser.read(256)
            except serial.SerialException:
                print("\n!! Serial port lost (USB unplugged?). Exiting.")
                self.running = False
                return
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                self.handle(raw.decode(errors="replace").strip())

    def handle(self, line):
        if not line:
            return
        parts = line.split()
        if parts[0] == "e" and len(parts) >= 6:
            self.last_e = parts
            if self.log:
                self.log.write(f"{time.time():.4f}," + ",".join(parts[1:6]) + "\n")
            if self.show_e:
                print(line)
        elif parts[0] == "s" and len(parts) >= 5:
            flags, state = int(parts[1], 16), int(parts[2])
            key = (flags, state)
            if key != self.last_status:
                self.last_status = key
                print(f"[status] {ARM_NAMES.get(state, state)}  flags={flags:04X} ({decode_flags(flags)})"
                      f"  vbat={parts[3]} V  vmot={parts[4]} V")
        else:
            print(line)

    def sender(self):
        last_summary = 0.0
        while self.running:
            if self.held and not self.paused:
                try:
                    self.send(self.held)
                except serial.SerialException:
                    return
            now = time.time()
            if self.held and self.held != "v 0 0" and not self.paused and now - last_summary > 1.0 \
                    and self.last_e and not self.show_e:
                last_summary = now
                e = self.last_e
                print(f"   wheels: L={float(e[3]):+6.2f}  R={float(e[4]):+6.2f} rad/s   ticks L={e[1]} R={e[2]}")
            time.sleep(0.05)                                   # 20 Hz

    # ------------------------------------------------------------ main loop
    def run(self):
        threading.Thread(target=self.reader, daemon=True).start()
        threading.Thread(target=self.sender, daemon=True).start()
        print("Connected. Type h for Pico help, quit to exit. Remember: a 1 to arm.\n")
        time.sleep(0.3)
        self.send("i")
        try:
            while self.running:
                cmd = input().strip()
                if not cmd:
                    continue
                low = cmd.lower()
                if low in ("quit", "exit", "q"):
                    break
                if low == "x":
                    self.held, self.paused = "v 0 0", False
                    print("   -> sending v 0 0 repeatedly")
                elif low == "pause":
                    self.paused = True
                    print("   -> NOT sending anything now. Wheels must stop within ~1 s (watchdog).")
                    print("      Type a new 'v' command or 'x' to resume.")
                elif low == "show":
                    self.show_e = not self.show_e
                elif low.startswith("v"):
                    self.held, self.paused = cmd, False
                    print(f"   -> repeating '{cmd}' at 20 Hz  (x = stop, pause = test watchdog)")
                else:
                    self.send(cmd)
        except (KeyboardInterrupt, EOFError):
            pass
        self.shutdown()

    def shutdown(self):
        print("\nStopping wheels and disarming...")
        self.held = None
        try:
            for _ in range(5):
                self.send("v 0 0")
                time.sleep(0.05)
            self.send("a 0")
            time.sleep(0.3)
        except serial.SerialException:
            pass
        self.running = False
        if self.log:
            self.log.close()
        self.ser.close()


def main():
    ap = argparse.ArgumentParser(description="Console for pico_motor_firmware")
    ap.add_argument("--port", help="serial port, e.g. /dev/ttyACM0 (default: auto-detect)")
    ap.add_argument("--log", help="write every 'e' line to this CSV file")
    args = ap.parse_args()
    port = args.port or find_pico_port()
    print(f"Opening {port} ...")
    try:
        Console(port, args.log).run()
    except serial.SerialException as exc:
        sys.exit(f"Could not open {port}: {exc}\n"
                 "Is the Arduino Serial Monitor still open? Are you in the 'dialout' group?")


if __name__ == "__main__":
    main()
