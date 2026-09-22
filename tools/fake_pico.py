#!/usr/bin/env python3
"""
fake_pico.py — a pretend Pico, so you can practise with the tools before
touching real hardware. It speaks the same protocol as pico_motor_firmware
(arming, 500 ms watchdog, speed clamp, accel ramp, 'e' and 's' lines) and
simulates two wheels with a simple first-order response.

  Terminal 1:   python3 fake_pico.py
                -> prints e.g.  "Fake Pico on /dev/pts/5"
  Terminal 2:   python3 pico_console.py --port /dev/pts/5
       or:      python3 pico_step_test.py --port /dev/pts/5

Linux only (uses a pseudo-terminal). Nothing here touches your robot.
"""
import math
import os
import pty
import random
import select
import time
import tty

GEAR, CPR = 3.75, 4096
MAX_SPEED, ACCEL, DECEL, TIMEOUT = 9.0, 7.0, 14.0, 0.5
TAU = 0.08                      # simulated wheel time constant (s)


def main():
    master, slave = pty.openpty()
    tty.setraw(slave)
    print(f"Fake Pico on {os.ttyname(slave)}   (Ctrl-C to quit)")
    state, flags_latched = 0, 0          # 0 DISARMED 1 ARMING 2 ARMED 3 DISARMING
    arm_t = 0.0
    cmd = [0.0, 0.0]
    ramp = [0.0, 0.0]
    vel = [0.0, 0.0]
    ang = [0.0, 0.0]                     # wheel angle, rad
    last_cmd = -1.0
    t_last = time.time()
    next_e = next_s = 0.0
    last_flags = None
    buf = b""

    def out(text):
        try:
            os.write(master, (text + "\n").encode())
        except OSError:
            pass

    out("# fake_pico ready. Send 'h' for help.")
    while True:
        r, _, _ = select.select([master], [], [], 0.002)
        now = time.time()
        if r:
            try:
                buf += os.read(master, 1024)
            except OSError:
                buf = b""
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                p = raw.decode(errors="replace").split()
                if not p:
                    continue
                if p[0] == "v" and len(p) >= 3:
                    l, rr = float(p[1]), float(p[2])
                    big = max(abs(l), abs(rr))
                    if big > MAX_SPEED:
                        l, rr = l * MAX_SPEED / big, rr * MAX_SPEED / big
                    last_cmd = now
                    if state == 2:
                        cmd = [l, rr]
                elif p[0] == "a" and len(p) >= 2:
                    if p[1] == "1" and state == 0:
                        if flags_latched:
                            out(f"# cannot arm: latched fault 0x{flags_latched:04X}")
                        else:
                            state, arm_t = 1, now
                            out("# arming...")
                    elif p[1] == "0" and state in (1, 2):
                        state, arm_t = 3, now
                        cmd = ramp = [0.0, 0.0]
                        out("# DISARMED: requested by host")
                elif p[0] == "c":
                    flags_latched = 0
                    out("# faults cleared")
                elif p[0] == "k":
                    out("# PID set (fake Pico ignores the values)")
                elif p[0] == "i":
                    out(f"# fake_pico  state={state}  loop: 2400 Hz (simulated)")
                elif p[0] in ("h", "?"):
                    out("# commands: v <left> <right> | a 1 | a 0 | c | k <P> <I> <D> | i | h")
                else:
                    out(f"# unknown command '{p[0]}'")

        dt, t_last = now - t_last, now
        if state == 1 and now - arm_t > 0.25:
            state = 2
            out("# ARMED - motor power ON")
        if state == 3 and now - arm_t > 0.8:
            state = 0
            out("# motor power OFF")

        stale = last_cmd < 0 or now - last_cmd > TIMEOUT
        want = cmd if (state == 2 and not stale) else [0.0, 0.0]
        if state != 2:
            ramp = [0.0, 0.0]
        else:
            d = [want[i] - ramp[i] for i in range(2)]
            scale = 1.0
            for i in range(2):
                up = abs(want[i]) > abs(ramp[i]) and want[i] * ramp[i] >= 0
                lim = (ACCEL if up else DECEL) * dt
                if abs(d[i]) > lim:
                    scale = min(scale, lim / abs(d[i]))
            ramp = [ramp[i] + d[i] * scale for i in range(2)]
        for i in range(2):
            vel[i] += (ramp[i] - vel[i]) * min(1.0, dt / TAU)
            ang[i] += vel[i] * dt

        if now >= next_e:
            next_e = now + 0.02
            n = [v + random.gauss(0, 0.02) for v in vel]
            ticks = [int(a * GEAR * CPR / (2 * math.pi)) for a in ang]
            out(f"e {ticks[0]} {ticks[1]} {n[0]:.4f} {n[1]:.4f} {int(now * 1e6) & 0xFFFFFFFF}")
        flags = (0x0004 if stale else 0) | (0x0002 if state != 2 else 0) | flags_latched
        if flags != last_flags or now >= next_s:
            last_flags, next_s = flags, now + 0.5
            out(f"s {flags:04X} {state} 11.90 {11.85 if state == 2 else 0.0:.2f}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
