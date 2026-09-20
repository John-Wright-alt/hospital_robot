#!/usr/bin/env python3
"""
fake_pico.py  --  a pretend Pico W, running on the Raspberry Pi.

It creates a virtual serial port at /tmp/pico and behaves like YOUR
pico_motor_firmware (1.0.0 / 1.0.1): same text protocol, same arm state machine,
same relay timing, same flags, same watchdog, same speed clamp and ramps.
The "motors" are a simple model, so /odom moves as if wheels were turning.

Use it to test the bridge node, launch files, RViz and pico_console.py while
the real Pico is somewhere else.

    ros2 run hospital_robot_base fake_pico                 # behaves like firmware 1.0.1
    ros2 run hospital_robot_base fake_pico --fw 1.0.0 --noise
          ^ reproduces your "have to arm twice" bug, so you can see why it happened

Then point the bridge (or pico_console.py) at /tmp/pico instead of /dev/pico.

Type these into THIS terminal to simulate hardware events:
    estop      press the E-stop            release   release it
    stall L    left wheel stall fault      stall R   right wheel stall fault
    batt 10.2  set the battery voltage     unplug    pull the USB cable for 2 s
    noise      toggle the relay-inrush noise spike on GP14
    status     print the internal state    help      this list      quit
"""

import argparse
import math
import os
import pty
import select
import sys
import time
import tty

LINK = '/tmp/pico'

# ---- numbers copied from your config.h ------------------------------------------
GEAR_RATIO = 3.75
ENCODER_CPR = 4096
MAX_WHEEL_SPEED = 9.0
MAX_WHEEL_ACCEL = 7.0
MAX_WHEEL_DECEL = 14.0
CMD_TIMEOUT_MS = 500
VBAT_PRESENT_V, VBAT_WARN_V, VBAT_CUTOFF_V, VBAT_CUTOFF_MS = 5.0, 11.1, 10.5, 2000
VMOT_ON_V, VMOT_OFF_V = 8.0, 6.0
RELAY_PULSE_MS, RELAY_SETTLE_MS, RELAY_OPEN_TIMEOUT_MS = 100, 250, 3000
REPORT_PERIOD_MS, STATUS_PERIOD_MS, CONTROL_PERIOD_MS = 20, 500, 5
ESTOP_CONFIRM_MS, VMOT_LOST_CONFIRM_MS = 3, 20            # firmware 1.0.1 only

F_ESTOP, F_NO_VMOT, F_CMD_TIMEOUT, F_BATT_WARN, F_NO_BATT = 0x1, 0x2, 0x4, 0x8, 0x10
F_BATT_CUTOFF, F_STALL_L, F_STALL_R, F_RELAY = 0x100, 0x200, 0x400, 0x2000
DISARMED, ARMING, ARMED, DISARMING = 0, 1, 2, 3
ARM_NAMES = ['DISARMED', 'ARMING', 'ARMED', 'DISARMING']


class FakePico:

    def __init__(self, fw, noise, vbat):
        self.fw = fw
        self.noise = noise
        self.t0 = time.monotonic()
        self.vbat_true = vbat
        self.master = None
        self.plug_in()

    # ------------------------------------------------------------------ USB
    def ms(self):
        return int((time.monotonic() - self.t0) * 1000)

    def plug_in(self):
        """New virtual port + a freshly booted 'Pico' (like re-plugging USB)."""
        self.master, slave = pty.openpty()
        tty.setraw(slave)
        name = os.ttyname(slave)
        self.slave = slave
        try:
            os.remove(LINK)
        except FileNotFoundError:
            pass
        os.symlink(name, LINK)
        self.reset_state()
        print(f'[fake_pico] "Pico" running firmware {self.fw} on {LINK} -> {name}')
        print('[fake_pico] relay-inrush noise spike: ' + ('ON' if self.noise else 'off'))
        self.say(f'# pico_motor_firmware {self.fw} (FAKE) ready. Send \'h\' for help.')

    def unplug(self):
        print('[fake_pico] USB unplugged for 2 s ...')
        os.close(self.master)
        os.close(self.slave)
        self.master = None
        try:
            os.remove(LINK)
        except FileNotFoundError:
            pass
        time.sleep(2.0)
        self.plug_in()

    def reset_state(self):
        # physical world
        self.relay_closed = False
        self.estop_pressed = False
        self.vmot = 0.0
        self.pos_l = self.pos_r = 0.0          # wheel angle, rad
        self.vel_l = self.vel_r = 0.0          # wheel rad/s
        self.glitch_ms = None                  # a GP14 noise spike is happening now
        # firmware state (names follow pico_motor_firmware.ino)
        self.arm = DISARMED
        self.latched = 0
        self.flags = 0
        self.last_flags = 0xFFFF
        self.cmd_l = self.cmd_r = 0.0
        self.ramp_l = self.ramp_r = 0.0
        self.last_cmd = 0
        self.ever_cmd = False
        self.vmot_live = False
        self.estop = False
        self.estop_low_since = 0
        self.estop_high_seen = False
        self.estop_high_since = 0
        self.glitches = 0
        self.glitches_reported = 0
        self.last_glitch_msg = 0
        self.vmot_low_seen = False
        self.vmot_low_since = 0
        self.batt_low = False
        self.batt_low_since = 0
        self.relay_pending = 0
        self.relay_pulsing = False
        self.relay_pulse_end = 0
        self.relay_idle_since = 0
        self.relay_last_pulse = -100000
        self.last_not_armed_msg = -100000
        self.enabled = False
        self.inbuf = b''
        now = self.ms()
        self.t_control = self.t_report = self.t_status = now
        self.micros_offset = 0xFFFFFFFF - 5_000_000    # wraps after 5 s: tests the bridge

    def say(self, line):
        if self.master is None:
            return
        try:
            os.write(self.master, (line + '\n').encode())
        except OSError:
            pass                                        # nobody listening: drop it

    # -------------------------------------------------------------- relay
    def relay_service(self, now):
        if self.relay_pulsing:
            if now >= self.relay_pulse_end:
                self.relay_pulsing = False
                self.relay_idle_since = now
        elif self.relay_pending > 0 and now - self.relay_idle_since >= 200:
            self.relay_pulsing = True
            self.relay_pulse_end = now + RELAY_PULSE_MS
            self.relay_last_pulse = now
            self.relay_pending -= 1
            was_empty = self.vmot < 1.0
            self.relay_closed = not self.relay_closed            # latching: TOGGLE
            if self.relay_closed and was_empty and self.noise:
                self.glitch_ms = now                             # inrush spike hits GP14
                print('[fake_pico] relay closed into empty capacitors -> '
                      'noise spike on GP14 (E-stop sense)')

    def relay_busy(self):
        return self.relay_pulsing or self.relay_pending > 0

    # ------------------------------------------------------ firmware logic
    def drivers_off(self):
        self.enabled = False
        self.cmd_l = self.cmd_r = self.ramp_l = self.ramp_r = 0.0

    def latch(self, flag, why):
        if self.latched & flag:
            return
        self.latched |= flag
        self.say(f'# FAULT: {why}')

    def disarm(self, why):
        self.drivers_off()
        if self.arm == ARMED:
            self.relay_pending += 1
        elif self.arm == ARMING:
            if self.relay_pending > 0:
                self.relay_pending -= 1
            else:
                self.relay_pending += 1          # <- the second pulse in the bug
        else:
            return
        self.arm = DISARMING
        self.say(f'# DISARMED: {why}')

    def request_arm(self):
        if self.arm == ARMED:
            return self.say('# already armed')
        if self.arm != DISARMED:
            return self.say('# relay busy, try again in a second')
        if self.latched:
            return self.say(f"# cannot arm: latched fault 0x{self.latched:04X} "
                            "- fix it, then send 'c'")
        if self.estop:
            return self.say('# cannot arm: E-stop is pressed')
        if self.flags & F_NO_BATT:
            return self.say('# cannot arm: no battery voltage (green switch / battery?)')
        self.cmd_l = self.cmd_r = self.ramp_l = self.ramp_r = 0.0
        self.vmot_low_seen = False
        if self.vmot_live:
            self.arm = ARMED
            return self.say('# ARMED (motor power was already on)')
        self.relay_pending += 1
        self.arm = ARMING
        self.say('# arming...')

    def clear_faults(self):
        keep = 0
        vb = self.vbat_true
        if (self.latched & F_BATT_CUTOFF) and VBAT_PRESENT_V < vb < VBAT_CUTOFF_V + 0.3:
            keep |= F_BATT_CUTOFF
        self.latched = keep
        self.batt_low = False
        self.say(f'# some faults could not be cleared (cause still present): 0x{keep:04X}'
                 if keep else '# faults cleared')

    def estop_input(self, now):
        """What GP14 reads right now (button OR a 1 ms noise spike)."""
        high = self.estop_pressed
        if self.glitch_ms is not None:
            if now - self.glitch_ms <= 1:              # ~1-2 ms spike
                high = True
            else:
                self.glitch_ms = None
        return high

    def estop_logic(self, now):
        high = self.estop_input(now)
        if self.fw == '1.0.0':
            if high:
                if not self.estop:
                    self.estop = True
                    self.drivers_off()
                    self.disarm('E-STOP PRESSED')
                    self.say('# E-STOP PRESSED')
                self.estop_low_since = now
            elif self.estop and now - self.estop_low_since >= 50:
                self.estop = False
                self.say("# E-stop released. Motors stay OFF until you send 'a 1'.")
            return
        # firmware 1.0.1: needs ESTOP_CONFIRM_MS of continuous HIGH
        if high:
            if not self.estop_high_seen:
                self.estop_high_seen = True
                self.estop_high_since = now
            if not self.estop and now - self.estop_high_since >= ESTOP_CONFIRM_MS:
                self.estop = True
                self.drivers_off()
                self.disarm('E-STOP PRESSED')
                self.say('# E-STOP PRESSED')
            self.estop_low_since = now
        else:
            if self.estop_high_seen and not self.estop:
                self.glitches += 1
            self.estop_high_seen = False
            if self.estop and now - self.estop_low_since >= 50:
                self.estop = False
                self.say("# E-stop released. Motors stay OFF until you send 'a 1'.")
        if self.glitches != self.glitches_reported and now - self.last_glitch_msg >= 1000:
            self.last_glitch_msg = now
            self.glitches_reported = self.glitches
            self.say(f'# E-stop input: ignored {self.glitches} short glitch(es) so far '
                     '(noise on GP14)')

    @staticmethod
    def step_allowed(cur, want, dt):
        speeding_up = abs(want) > abs(cur) and want * cur >= 0.0
        return (MAX_WHEEL_ACCEL if speeding_up else MAX_WHEEL_DECEL) * dt

    def ramp_pair(self, want_l, want_r, dt):
        dl, dr, scale = want_l - self.ramp_l, want_r - self.ramp_r, 1.0
        ml = self.step_allowed(self.ramp_l, want_l, dt)
        mr = self.step_allowed(self.ramp_r, want_r, dt)
        if abs(dl) > ml:
            scale = min(scale, ml / abs(dl))
        if abs(dr) > mr:
            scale = min(scale, mr / abs(dr))
        self.ramp_l += dl * scale
        self.ramp_r += dr * scale

    def control_tick(self, now, dt):
        vbat, vmot = self.vbat_true, self.vmot
        if self.vmot_live and vmot < VMOT_OFF_V:
            self.vmot_live = False
        elif not self.vmot_live and vmot > VMOT_ON_V:
            self.vmot_live = True

        batt_present = vbat > VBAT_PRESENT_V
        if batt_present and vbat < VBAT_CUTOFF_V:
            if not self.batt_low:
                self.batt_low, self.batt_low_since = True, now
            elif now - self.batt_low_since > VBAT_CUTOFF_MS:
                self.latch(F_BATT_CUTOFF, 'battery below cutoff - charge it')
        else:
            self.batt_low = False

        if self.arm == ARMING:
            if not self.relay_busy() and now - self.relay_last_pulse >= RELAY_SETTLE_MS:
                if self.vmot_live:
                    self.arm = ARMED
                    self.say('# ARMED - motor power ON')
                else:
                    self.arm = DISARMED
                    self.say('# arm failed: motor bus did not come on. '
                             'Is the blue switch on? Relay wired?')
        elif self.arm == DISARMING:
            if not self.relay_busy():
                if not self.vmot_live:
                    self.arm = DISARMED
                    self.say('# motor power OFF')
                elif now - self.relay_last_pulse > RELAY_OPEN_TIMEOUT_MS:
                    self.arm = DISARMED
                    self.latch(F_RELAY, 'relay did NOT open. Drivers are off; '
                                        'use the E-stop or blue switch')
        elif self.arm == ARMED:
            lost = vmot < VMOT_ON_V
            if self.fw != '1.0.0':
                if lost:
                    if not self.vmot_low_seen:
                        self.vmot_low_seen, self.vmot_low_since = True, now
                        lost = False
                    else:
                        lost = now - self.vmot_low_since >= VMOT_LOST_CONFIRM_MS
                else:
                    self.vmot_low_seen = False
            if lost:
                self.disarm('motor power lost (E-stop, blue switch or flat battery)')
            if self.arm == ARMED and self.latched:
                self.disarm('latched fault')
        elif self.arm == DISARMED:
            if (self.vmot_live and not self.estop and not (self.latched & F_RELAY)
                    and not self.relay_busy() and now - self.relay_last_pulse > 1000):
                self.relay_pending += 1
                self.arm = DISARMING
                self.say('# motor bus is live while disarmed - opening relay')

        can_drive = self.arm == ARMED and not self.estop and not self.latched
        stale = (not self.ever_cmd) or (now - self.last_cmd > CMD_TIMEOUT_MS)
        if can_drive:
            self.ramp_pair(0.0 if stale else self.cmd_l, 0.0 if stale else self.cmd_r, dt)
            self.enabled = True
        else:
            self.drivers_off()

        live = 0
        if self.estop:
            live |= F_ESTOP
        if not self.vmot_live:
            live |= F_NO_VMOT
        if stale:
            live |= F_CMD_TIMEOUT
        if not batt_present:
            live |= F_NO_BATT
        elif vbat < VBAT_WARN_V:
            live |= F_BATT_WARN
        self.flags = live | self.latched

    # ------------------------------------------------------------ commands
    def handle_line(self, line, now):
        line = line.strip()
        if not line:
            return
        cmd, args = line[0], line[1:].split()
        try:
            if cmd == 'v':
                left, right = float(args[0]), float(args[1])
                if not (math.isfinite(left) and math.isfinite(right)):
                    return self.say('# rejected: not a number')
                biggest = max(abs(left), abs(right))
                if biggest > MAX_WHEEL_SPEED:
                    left *= MAX_WHEEL_SPEED / biggest
                    right *= MAX_WHEEL_SPEED / biggest
                self.last_cmd, self.ever_cmd = now, True
                if self.arm == ARMED:
                    self.cmd_l, self.cmd_r = left, right
                elif now - self.last_not_armed_msg > 2000:
                    self.last_not_armed_msg = now
                    self.say("# ignoring 'v': not armed (send 'a 1')")
            elif cmd == 'a':
                self.request_arm() if int(args[0]) else self.disarm('requested by host')
            elif cmd == 'c':
                self.clear_faults()
            elif cmd == 'i':
                self.say(f'# pico_motor_firmware {self.fw} (FAKE)')
                self.say(f'# state={ARM_NAMES[self.arm]} flags=0x{self.flags:04X}  '
                         f'vbat={self.vbat_true:.2f} V  vmot={self.vmot:.2f} V  '
                         f'estop={"PRESSED" if self.estop else "released"}')
                self.say('# FOC loop: 2500 Hz (each motor)   encoder errors: L=0 R=0')
                self.say(f'# gear={GEAR_RATIO:.3f}  ticks per wheel rev='
                         f'{ENCODER_CPR * GEAR_RATIO:.0f}')
                self.say(f'# limits: speed={MAX_WHEEL_SPEED:.2f} rad/s accel='
                         f'{MAX_WHEEL_ACCEL:.1f} decel={MAX_WHEEL_DECEL:.1f} rad/s^2 '
                         f'timeout={CMD_TIMEOUT_MS} ms')
            elif cmd == 'k':
                self.say(f'# PID set: P={float(args[0]):.4f} I={float(args[1]):.4f} '
                         f'D={float(args[2]) if len(args) > 2 else 0:.4f} (fake: no effect)')
            elif cmd in 'h?':
                self.say('# commands: v <left> <right> | a 1 | a 0 | c | k <P> <I> <D> | i | h')
            else:
                self.say(f"# unknown command '{cmd}' (send h for help)")
        except (IndexError, ValueError):
            self.say(f'# usage error: {line}')

    def keyboard(self, text):
        words = text.strip().split()
        if not words:
            return
        w = words[0].lower()
        if w == 'estop':
            self.estop_pressed = True
            print('[fake_pico] E-stop PRESSED (motor power cut in "hardware")')
        elif w == 'release':
            self.estop_pressed = False
            print('[fake_pico] E-stop released')
        elif w == 'stall' and len(words) > 1:
            flag = F_STALL_L if words[1].upper().startswith('L') else F_STALL_R
            side = 'LEFT' if flag == F_STALL_L else 'RIGHT'
            self.latch(flag, f'{side} wheel not following command (stall, or wrong direction)')
        elif w == 'batt' and len(words) > 1:
            self.vbat_true = float(words[1])
            print(f'[fake_pico] battery now {self.vbat_true:.2f} V')
        elif w == 'noise':
            self.noise = not self.noise
            print('[fake_pico] relay-inrush noise spike: ' + ('ON' if self.noise else 'off'))
        elif w == 'unplug':
            self.unplug()
        elif w == 'status':
            print(f'[fake_pico] fw={self.fw} arm={ARM_NAMES[self.arm]} flags=0x{self.flags:04X} '
                  f'relay={"CLOSED" if self.relay_closed else "open"} vmot={self.vmot:.2f} V '
                  f'wheels L={self.vel_l:+.2f} R={self.vel_r:+.2f} rad/s')
        elif w in ('help', 'h'):
            print(__doc__.split('Type these')[1])
        elif w in ('quit', 'exit', 'q'):
            raise KeyboardInterrupt
        else:
            print('[fake_pico] unknown - type help')

    # ------------------------------------------------------------ main loop
    def physics(self, dt):
        powered = self.relay_closed and not self.estop_pressed and self.vbat_true > 0
        if powered:                                       # capacitors charge fast
            self.vmot += (self.vbat_true - self.vmot) * min(1.0, dt / 0.005)
        else:                                             # and drain slowly
            self.vmot -= self.vmot * min(1.0, dt / 1.5)   # 3 x 470 uF: seconds
        driving = self.enabled and self.vmot > VMOT_ON_V
        tau = 0.06 if driving else 0.3                    # closed loop vs. coasting
        tgt_l = self.ramp_l if driving else 0.0
        tgt_r = self.ramp_r if driving else 0.0
        self.vel_l += (tgt_l - self.vel_l) * min(1.0, dt / tau)
        self.vel_r += (tgt_r - self.vel_r) * min(1.0, dt / tau)
        self.pos_l += self.vel_l * dt
        self.pos_r += self.vel_r * dt

    def run(self):
        last = time.monotonic()
        print("[fake_pico] type 'help' for hardware events. Ctrl+C to quit.")
        while True:
            fds = [sys.stdin] + ([self.master] if self.master is not None else [])
            readable, _, _ = select.select(fds, [], [], 0.001)
            now = self.ms()
            if self.master in readable:
                try:
                    self.inbuf += os.read(self.master, 1024)
                except OSError:
                    pass
                while b'\n' in self.inbuf:
                    raw, self.inbuf = self.inbuf.split(b'\n', 1)
                    self.handle_line(raw.decode(errors='ignore').replace('\r', ''), now)
            if sys.stdin in readable:
                self.keyboard(sys.stdin.readline())
                now = self.ms()

            t = time.monotonic()
            self.physics(t - last)
            last = t

            self.estop_logic(now)
            self.relay_service(now)
            if now - self.t_control >= CONTROL_PERIOD_MS:
                dt = min((now - self.t_control) / 1000.0, 0.05)
                self.t_control = now
                self.control_tick(now, dt)
            if now - self.t_report >= REPORT_PERIOD_MS:
                self.t_report = now
                k = ENCODER_CPR * GEAR_RATIO / (2 * math.pi)
                micros = (now * 1000 + self.micros_offset) & 0xFFFFFFFF
                self.say(f'e {int(self.pos_l * k)} {int(self.pos_r * k)} '
                         f'{self.vel_l:.4f} {self.vel_r:.4f} {micros}')
            if self.flags != self.last_flags or now - self.t_status >= STATUS_PERIOD_MS:
                self.last_flags = self.flags
                self.t_status = now
                self.say(f's {self.flags:04X} {self.arm} {self.vbat_true:.2f} {self.vmot:.2f}')


def main():
    ap = argparse.ArgumentParser(description='Pretend Pico W on /tmp/pico')
    ap.add_argument('--fw', choices=['1.0.0', '1.0.1'], default='1.0.1',
                    help='which firmware behaviour to copy (default 1.0.1)')
    ap.add_argument('--noise', action='store_true',
                    help='simulate the GP14 noise spike when the relay closes into '
                         'empty capacitors (your double-arm bug)')
    ap.add_argument('--vbat', type=float, default=12.2, help='battery volts (default 12.2)')
    args, _ = ap.parse_known_args()
    pico = FakePico(args.fw, args.noise, args.vbat)
    try:
        pico.run()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            os.remove(LINK)
        except FileNotFoundError:
            pass
        print('\n[fake_pico] stopped')


if __name__ == '__main__':
    main()
