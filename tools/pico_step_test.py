#!/usr/bin/env python3
"""
pico_step_test.py — automatic velocity step test for PID tuning (roadmap 4.5, step 6).

WHAT IT DOES
  1. Arms the robot.
  2. Commands both wheels:  0  ->  +AMP  ->  0  ->  -AMP  ->  0   (rad/s at the wheel)
     sending 'v' at 50 Hz and recording every 'e' line from the Pico.
  3. Stops and disarms.
  4. Prints rise time, overshoot, settling time, steady-state error and a
     DIRECTION CHECK (do the ticks count up when the wheel is told to go forward?).
  5. Saves  <out>.csv  and  <out>.png  (the plot is good report material).

ROBOT ON BLOCKS. Close the Arduino Serial Monitor first.

EXAMPLES
  python3 pico_step_test.py                          # 2 rad/s step, both wheels forward
  python3 pico_step_test.py --amp 4                  # bigger step
  python3 pico_step_test.py --spin                   # wheels in opposite directions
  python3 pico_step_test.py --pid 0.08 0.8 0 --out p008_i08
Needs:  sudo apt install python3-serial python3-matplotlib
"""
import argparse
import csv
import sys
import threading
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    sys.exit("pyserial is missing:  sudo apt install python3-serial")

FLAG_NAMES = {
    0x0001: "ESTOP", 0x0002: "NO_VMOT", 0x0004: "CMD_TIMEOUT", 0x0008: "BATT_WARN",
    0x0010: "NO_BATT", 0x0020: "LOOP_SLOW", 0x0100: "BATT_CUTOFF", 0x0200: "STALL_L",
    0x0400: "STALL_R", 0x0800: "ENCODER_L", 0x1000: "ENCODER_R", 0x2000: "RELAY",
}


def decode_flags(flags):
    return " ".join(n for bit, n in FLAG_NAMES.items() if flags & bit) or "none"


def find_pico_port():
    for p in serial.tools.list_ports.comports():
        if p.vid == 0x2E8A:
            return p.device
    sys.exit("No Pico found. Plug it in or use --port /dev/ttyACM0")


class Link:
    """Background reader that keeps the latest status and a log of 'e' samples."""

    def __init__(self, port):
        self.ser = serial.Serial(port, 115200, timeout=0.05)
        self.lock = threading.Lock()
        self.running = True
        self.state = None
        self.flags = 0
        self.samples = []            # (host_t, cmd_l, cmd_r, ticks_l, ticks_r, vel_l, vel_r)
        self.cmd = (0.0, 0.0)
        self.recording = False
        self.t0 = time.time()
        threading.Thread(target=self._reader, daemon=True).start()

    def send(self, text):
        with self.lock:
            self.ser.write((text + "\n").encode())

    def _reader(self):
        buf = b""
        while self.running:
            try:
                buf += self.ser.read(512)
            except serial.SerialException:
                self.running = False
                return
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                p = raw.decode(errors="replace").split()
                if not p:
                    continue
                if p[0] == "e" and len(p) >= 6 and self.recording:
                    self.samples.append((time.time() - self.t0, self.cmd[0], self.cmd[1],
                                         int(p[1]), int(p[2]), float(p[3]), float(p[4])))
                elif p[0] == "s" and len(p) >= 5:
                    self.flags, self.state = int(p[1], 16), int(p[2])
                elif p[0] == "#":
                    print("  pico:", " ".join(p[1:]))

    def close(self):
        self.running = False
        time.sleep(0.1)
        self.ser.close()


def analyse(t, vel, amp, t_start, t_end, label):
    """Metrics for one step that starts at t_start and ends at t_end."""
    seg = [(ti - t_start, v) for ti, v in zip(t, vel) if t_start <= ti < t_end]
    if len(seg) < 20:
        return None, f"{label}: not enough samples"
    sign = 1.0 if amp > 0 else -1.0
    a = abs(amp)
    y = [(ts, sign * v) for ts, v in seg]            # make the step positive for maths
    t90 = next((ts for ts, v in y if v >= 0.9 * a), None)
    peak = max(v for _, v in y)
    overshoot = max(0.0, (peak - a) / a * 100)
    tail = [v for ts, v in y if ts >= (t_end - t_start) - 1.0]
    mean_tail = sum(tail) / len(tail)
    ss_err = (mean_tail - a) / a * 100
    std_tail = (sum((v - mean_tail) ** 2 for v in tail) / len(tail)) ** 0.5 / a * 100
    outside = [ts for ts, v in y if abs(v - a) > 0.05 * a]
    settle = outside[-1] if outside else 0.0
    settled = settle < (t_end - t_start) - 0.5
    ok = settled and settle < 1.0 and overshoot < 15 and abs(ss_err) < 5 and std_tail < 5
    text = (f"{label}: rise(90%)={'%.2f s' % t90 if t90 is not None else 'never'}  "
            f"overshoot={overshoot:4.1f}%  settle(5%)={'%.2f s' % settle if settled else 'NOT SETTLED'}  "
            f"ss_error={ss_err:+4.1f}%  ripple={std_tail:3.1f}%   -> {'PASS' if ok else 'CHECK'}")
    return ok, text


def main():
    ap = argparse.ArgumentParser(description="Velocity step test for pico_motor_firmware")
    ap.add_argument("--port")
    ap.add_argument("--amp", type=float, default=2.0, help="step size, wheel rad/s (default 2)")
    ap.add_argument("--hold", type=float, default=3.0, help="seconds at each step (default 3)")
    ap.add_argument("--spin", action="store_true", help="right wheel opposite to left")
    ap.add_argument("--pid", type=float, nargs=3, metavar=("P", "I", "D"), help="send 'k P I D' first")
    ap.add_argument("--out", default="step_test", help="output file name (no extension)")
    ap.add_argument("--no-show", action="store_true", help="do not open the plot window")
    args = ap.parse_args()

    port = args.port or find_pico_port()
    print(f"Opening {port} ...")
    link = Link(port)
    time.sleep(1.0)

    def abort(msg):
        print("\n!! " + msg)
        for _ in range(3):
            link.send("v 0 0")
            time.sleep(0.05)
        link.send("a 0")
        time.sleep(0.3)
        link.close()
        sys.exit(1)

    if args.pid:
        link.send("k %g %g %g" % tuple(args.pid))
        time.sleep(0.2)

    print("Arming...")
    link.send("a 1")
    deadline = time.time() + 6
    while link.state != 2 and time.time() < deadline:
        link.send("v 0 0")
        time.sleep(0.05)
    if link.state != 2:
        abort(f"Could not arm. State={link.state}, flags={link.flags:04X} ({decode_flags(link.flags)})")

    r_sign = -1.0 if args.spin else 1.0
    profile = [(1.0, 0.0), (args.hold, args.amp), (1.5, 0.0), (args.hold, -args.amp), (1.5, 0.0)]
    edges, t_acc = [], 1.0
    for dur, _ in profile[1:]:
        edges.append(t_acc)
        t_acc += dur
    print(f"Running step test: {args.amp} rad/s, {args.hold} s holds"
          f"{' (spin: right wheel reversed)' if args.spin else ''} ... E-stop in reach!")

    link.t0 = time.time()
    link.recording = True
    try:
        t_seg = 0.0
        for dur, val in profile:
            link.cmd = (val, r_sign * val)
            seg_end = link.t0 + t_seg + dur
            while time.time() < seg_end:
                link.send(f"v {link.cmd[0]:.3f} {link.cmd[1]:.3f}")
                if link.state is not None and link.state != 2:
                    abort(f"Robot left ARMED state mid-test: flags={link.flags:04X} ({decode_flags(link.flags)})")
                time.sleep(0.02)                     # 50 Hz
            t_seg += dur
    except KeyboardInterrupt:
        abort("Stopped by you (Ctrl-C).")
    link.recording = False
    for _ in range(10):
        link.send("v 0 0")
        time.sleep(0.05)
    link.send("a 0")
    time.sleep(0.5)
    link.close()

    s = link.samples
    if len(s) < 100:
        sys.exit(f"Only {len(s)} samples recorded — is the firmware sending 'e' lines?")
    with open(args.out + ".csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "cmd_l", "cmd_r", "ticks_l", "ticks_r", "vel_l", "vel_r"])
        w.writerows(s)
    t = [x[0] for x in s]
    rate = len(s) / (t[-1] - t[0])
    print(f"\nRecorded {len(s)} samples ({rate:.0f} Hz). Saved {args.out}.csv")

    # ---- Direction check: during the FORWARD step ticks must increase ----------
    print("\nDIRECTION CHECK (roadmap bench test step 4)")
    fwd = [x for x in s if edges[0] + 0.5 <= x[0] < edges[1]]
    all_ok = True
    for name, idx_ticks, idx_vel, want in (("LEFT ", 3, 5, 1.0), ("RIGHT", 4, 6, r_sign)):
        dticks = fwd[-1][idx_ticks] - fwd[0][idx_ticks]
        mean_v = sum(x[idx_vel] for x in fwd) / len(fwd)
        good = dticks * want > 0 and mean_v * want > 0
        all_ok &= good
        print(f"  {name}: ticks changed by {dticks:+d}, mean speed {mean_v:+.2f} rad/s "
              f"(expected {'+' if want > 0 else '-'})  -> {'PASS' if good else 'FAIL'}")
    print("  Also LOOK at the wheels: during the first step both must turn the robot's FORWARD way.")

    print("\nSTEP RESPONSE (includes the firmware accel ramp)")
    for name, idx_vel, sgn in (("LEFT ", 5, 1.0), ("RIGHT", 6, r_sign)):
        vel = [x[idx_vel] for x in s]
        for label, t0_, t1_, amp in ((f"{name} +step", edges[0], edges[1], args.amp * sgn),
                                     (f"{name} -step", edges[2], edges[3], -args.amp * sgn)):
            ok, text = analyse(t, vel, amp, t0_, t1_, label)
            print("  " + text)
            all_ok &= bool(ok)
    print("\nOVERALL:", "PASS" if all_ok else "CHECK the lines marked CHECK/FAIL (see guide Part 9)")

    # ---- Plot ---------------------------------------------------------------------
    try:
        import matplotlib
        if args.no_show:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed (sudo apt install python3-matplotlib) — skipping plot")
        return
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    for ax, name, ic, iv in ((axes[0], "Left wheel", 1, 5), (axes[1], "Right wheel", 2, 6)):
        ax.plot(t, [x[ic] for x in s], "k--", label="commanded")
        ax.plot(t, [x[iv] for x in s], "b", label="measured")
        ax.set_ylabel("wheel speed (rad/s)")
        ax.set_title(name)
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right")
    axes[1].set_xlabel("time (s)")
    fig.suptitle(f"Velocity step test — {args.amp} rad/s" + (f", PID {args.pid}" if args.pid else ""))
    fig.tight_layout()
    fig.savefig(args.out + ".png", dpi=120)
    print(f"Saved {args.out}.png")
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
