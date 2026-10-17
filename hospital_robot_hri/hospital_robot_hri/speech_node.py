#!/usr/bin/env python3
"""
speech_node.py  --  ROS 2 Jazzy node "hri_speech": the robot's voice.

    /odom              (nav_msgs/Odometry)   ─┐  is the robot really moving?
    /robot_state       (std_msgs/String)     ─┤  ARMED / ESTOP / FAULT / lost contact
    /pico/status       (std_msgs/String)     ─┤  BATT_WARN
    /hri/expression    (std_msgs/String)     ─┼─► [hri_speech] ──► speaker (aplay)
    /hri/gaze/status   (std_msgs/String JSON)─┤                └─► /hri/speech/status (JSON)
    /hri/speech/event  (std_msgs/String)     ─┤  an event key from phrases.yaml
    /hri/speech/say    (std_msgs/String)     ─┘  free text (testing, staff)

    Service /hri_speech/stop (std_srvs/Trigger): stop talking now, forget queued
    chatter/info (warnings and critical messages are kept).

SAFETY: this node only LISTENS and TALKS. It publishes nothing that can move
the robot, and nothing else depends on it. If it crashes, or the speaker is
unplugged, the Pico watchdog, the E-stop and (later) Nav2 behave exactly as
before. The screen still shows the message (caption) if the audio fails.

Try it:
    ros2 topic pub --once /hri/speech/event std_msgs/msg/String "{data: speaker_test}"
    ros2 topic pub --once /hri/speech/say   std_msgs/msg/String "{data: 'Hello, testing.'}"
"""

import csv
import json
import os
import threading
import time
from datetime import datetime

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from hospital_robot_hri.audio_player import (
    SOUNDS, AlsaPlayer, AudioError, Tts, concat, duration_s, make_sound, prepare)
from hospital_robot_hri.speech_logic import (
    PRIORITIES, FlagWatcher, GreetingDetector, GreetParams, MotionAnnounceParams,
    MotionAnnouncer, MotionDetector, MotionParams, Repeater, RobotStateWatcher,
    SpeechPlanner, SpeechQueue, Utterance, all_jobs, in_quiet_hours, load_library,
    motors_can_drive)

# events that tell people we are about to move: no extra "I'm moving" afterwards
JOURNEY_EVENTS = ('journey_start',)


class SpeechNode(Node):

    def __init__(self):
        super().__init__('hri_speech')
        dp = lambda name, default: self.declare_parameter(name, default).value  # noqa: E731

        # ------------------------------------------------------------ settings
        self.device = dp('alsa_device', 'plughw:CARD=Headphones,DEV=0')
        self.volume = float(dp('volume', 0.8))
        self.out_channels = int(dp('output_channels', 2))
        self.lead_ms = float(dp('lead_silence_ms', 60.0))
        self.tts = Tts(cache_dir=dp('cache_dir', '~/.local/share/hospital_robot_hri/speech_cache'),
                       voice=dp('voice', 'en_GB-cori-medium'),
                       voice_dir=dp('voice_dir', '~/piper-voices'),
                       piper_python=dp('piper_python', '~/piper-venv/bin/python3'),
                       length_scale=float(dp('length_scale', 1.1)),
                       sentence_silence=float(dp('sentence_silence', 0.25)),
                       espeak_voice=dp('espeak_voice', 'en-gb+f3'),
                       espeak_speed=int(dp('espeak_speed', 150)))
        prerender = bool(dp('prerender_on_start', True))
        phrases_file = dp('phrases_file', '') or os.path.join(
            get_package_share_directory('hospital_robot_hri'), 'config', 'phrases.yaml')
        humour = float(dp('humour_chance', 0.25))
        self.quiet = (int(dp('quiet_start_hour', -1)), int(dp('quiet_end_hour', -1)))
        self.quiet_scale = float(dp('quiet_volume_scale', 0.6))
        say_startup = bool(dp('say_startup', True))
        self.say_shutdown = bool(dp('say_shutdown', True))
        self.simulated = bool(dp('simulated', False))

        mp = MotionParams(v_on=float(dp('v_on', 0.03)), v_off=float(dp('v_off', 0.015)),
                          w_on=float(dp('w_on', 0.15)), w_off=float(dp('w_off', 0.08)),
                          start_s=float(dp('start_s', 0.25)), stop_s=float(dp('stop_s', 1.0)),
                          stale_s=float(dp('odom_stale_s', 0.5)))
        ap = MotionAnnounceParams(restart_quiet_s=float(dp('restart_quiet_s', 10.0)),
                                  cue_interval_s=float(dp('cue_interval_s', 6.0)),
                                  reminder_interval_s=float(dp('reminder_interval_s', 45.0)))
        self.mp = mp
        self.journey_quiet_s = float(dp('journey_quiet_s', 15.0))
        gp = GreetParams(enabled=bool(dp('greet_enabled', False)),
                         range_m=float(dp('greet_range_m', 1.2)),
                         dwell_s=float(dp('greet_dwell_s', 2.0)),
                         cooldown_s=float(dp('greet_cooldown_s', 90.0)))
        self.fault_rep = Repeater(float(dp('fault_repeat_s', 60.0)))
        self.lost_rep = Repeater(float(self.get_parameter('fault_repeat_s').value))
        self.assist_rep = Repeater(float(dp('assistance_repeat_s', 30.0)))
        self.batt_rep = Repeater(float(dp('battery_repeat_s', 300.0)))
        log_csv = os.path.expanduser(dp('log_csv', '~/.local/share/hospital_robot_hri/speech_log.csv'))
        status_rate = float(dp('status_rate', 5.0))

        # ------------------------------------------------------------ logic
        with open(phrases_file) as f:
            self.library = load_library(yaml.safe_load(f), known_sounds=SOUNDS)
        self.planner = SpeechPlanner(self.library, humour_chance=humour)
        self.queue = SpeechQueue()
        self.motion = MotionDetector(mp)
        self.announcer = MotionAnnouncer(ap)
        self.state_watch = RobotStateWatcher(float(dp('state_timeout_s', 3.0)))
        self.batt_watch = FlagWatcher('BATT_WARN')
        self.greeter = GreetingDetector(gp)
        self.expression = 'idle'
        self.sounds = {name: make_sound(name) for name in SOUNDS}

        # ------------------------------------------------------------ audio
        self.player = AlsaPlayer(self.device)
        ok, why = self.player.available()
        self.audio_ok = ok
        self.audio_error = '' if ok else why
        piper_ok, piper_why = self.tts.piper_ok()
        espeak_ok, _ = self.tts.espeak_ok()
        jobs = all_jobs(self.library)
        n_texts = len(jobs)
        n_cached = sum(1 for j in jobs if self.tts.cached(*j))
        self.get_logger().info(
            f'hri_speech: device "{self.device}", volume {self.volume:.2f}, '
            f'voice {self.tts.voice} ({n_cached}/{n_texts} sentences pre-rendered), '
            f'Piper {"OK" if piper_ok else "NOT available: " + piper_why}, '
            f'espeak-ng {"OK" if espeak_ok else "not installed"}.')
        if not ok:
            self.get_logger().error(f'No audio player: {why}. Captions only.')
        if self.simulated:
            self.get_logger().warn('SIMULATED robot inputs (simulate_robot). Not real sensor data.')

        # ------------------------------------------------------------ worker state
        self.lock = threading.Condition()
        self.playing = None              # Utterance being played now
        self.engine = ''
        self.interrupted_for = None
        self.flushing = False
        self.stopping = False
        self.spoken = 0
        self.done_count = 0          # every message that ended, however it ended
        self.last_key = ''
        self.status_dirty = True
        self._csv = None
        if log_csv:
            try:
                os.makedirs(os.path.dirname(log_csv), exist_ok=True)
                new = not os.path.exists(log_csv)
                self._csv_file = open(log_csv, 'a', newline='')
                self._csv = csv.writer(self._csv_file)
                if new:
                    self._csv.writerow(['wall_time', 'event', 'priority', 'result', 'engine',
                                        'latency_ms', 'duration_s', 'simulated', 'detail'])
                self.get_logger().info(f'Logging every message to {log_csv}')
            except OSError as err:
                self.get_logger().warn(f'Cannot write {log_csv}: {err}. Not logging.')

        self.worker = threading.Thread(target=self._work, daemon=True)
        self.worker.start()
        if prerender:
            threading.Thread(target=self._prerender, daemon=True).start()

        # ------------------------------------------------------------ ROS interfaces
        self.status_pub = self.create_publisher(String, 'hri/speech/status', 10)
        self.create_subscription(Odometry, dp('odom_topic', 'odom'), self.on_odom, 20)
        self.create_subscription(String, dp('robot_state_topic', 'robot_state'), self.on_state, 10)
        self.create_subscription(String, dp('pico_status_topic', 'pico/status'), self.on_status, 10)
        self.create_subscription(String, dp('gaze_topic', 'hri/gaze/status'), self.on_gaze, 10)
        self.create_subscription(String, dp('expression_topic', 'hri/expression'),
                                 self.on_expression, 10)
        self.create_subscription(String, 'hri/speech/event', self.on_event, 10)
        self.create_subscription(String, 'hri/speech/say', self.on_say, 10)
        self.create_service(Trigger, '~/stop', self.on_stop)
        self.add_on_set_parameters_callback(self.on_params)
        self.create_timer(0.1, self.tick)
        self.create_timer(1.0 / max(0.5, status_rate), self.publish_status)
        if say_startup:
            self._startup_timer = self.create_timer(1.5, self._say_startup)

    # ================================================================ requests
    def request(self, key: str, force: bool = False, why: str = ''):
        """Ask for an event from the library. Thread-safe."""
        now = time.monotonic()
        allow_humour = not self.in_quiet_hours()
        u = self.planner.make(key, now, force=force, allow_humour=allow_humour)
        if u is None:
            if key not in self.library:
                self.get_logger().warn(f'Unknown speech event "{key}". '
                                       f'Known: {" ".join(sorted(self.library))}')
            return
        if key in JOURNEY_EVENTS:
            self.announcer.suppress_start_speech(now, self.journey_quiet_s)
        self._offer(u, why)

    def _offer(self, u: Utterance, why: str = ''):
        with self.lock:
            result = self.queue.offer(u, self.playing)
            if result == 'interrupt':
                self.interrupted_for = u
                self.player.stop()
            if result == 'dropped':
                self._log(u, 'dropped', '', None, None, why)
            elif u.priority > PRIORITIES['cue']:
                self.get_logger().info(f'[{u.priority_name}] {u.key}'
                                       + (f' ({why})' if why else '')
                                       + (' -> interrupts current message'
                                          if result == 'interrupt' else ''))
            self.lock.notify_all()
            self.status_dirty = True

    # ================================================================ callbacks
    def _say_startup(self):
        self._startup_timer.cancel()
        self.request('startup')

    def on_odom(self, msg: Odometry):
        t = time.monotonic()
        v, w = msg.twist.twist.linear.x, msg.twist.twist.angular.z
        state = self.state_watch.state
        if not motors_can_drive(state):
            # The motors have no power, so this speed cannot be real driving
            # (e.g. noisy Pico speed readings while disarmed). Don't announce it,
            # but keep the problem visible in the log.
            if abs(v) >= self.mp.v_on or abs(w) >= self.mp.w_on:
                self.get_logger().warn(
                    f'/odom says v={v:+.3f} m/s, w={w:+.2f} rad/s while the robot is '
                    f'{state} (no motor power): ignored. Check pico_bridge odometry.',
                    throttle_duration_sec=30.0)
            v, w = 0.0, 0.0
        old = self.motion.state
        new = self.motion.update(t, v, w)
        if new:
            self._motion_changed(t, old, new)

    def _motion_changed(self, t, old, new):
        self.status_dirty = True
        self.get_logger().info(f'motion: {old} -> {new}')
        for key in self.announcer.on_change(t, old, new):
            self.request(key, why=f'motion {old} -> {new}')

    def on_state(self, msg: String):
        t = time.monotonic()
        for key in self.state_watch.on_state(t, msg.data):
            self.request(key, force=key in ('estop', 'fault', 'motors_armed'),
                         why=f'/robot_state {msg.data}')
            if key == 'controller_restored':
                self.lost_rep.stop()
        if self.state_watch.state == 'FAULT':
            self.fault_rep.start(t)
        else:
            self.fault_rep.stop()

    def on_status(self, msg: String):
        edge = self.batt_watch.on_text(msg.data)
        if edge is True:
            self.batt_rep.start(time.monotonic())
            self.request('battery_low', force=True, why='BATT_WARN')
        elif edge is False:
            self.batt_rep.stop()

    def on_gaze(self, msg: String):
        try:
            status = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        allowed = (self.motion.state in ('stopped', 'unknown')
                   and self.expression in ('idle', 'waiting', 'curious')
                   and self.playing is None)
        if self.greeter.on_gaze(time.monotonic(), status, allowed):
            self.request('greeting', why='someone close to the parked robot')

    def on_expression(self, msg: String):
        name = (msg.data or '').strip().lower()
        prev, self.expression = self.expression, name
        if name == 'assistance' and prev != 'assistance':
            self.assist_rep.start(time.monotonic())
            self.request('assistance', force=True, why='/hri/expression assistance')
        elif prev == 'assistance' and name != 'assistance':
            self.assist_rep.stop()
            self.request('thank_you', why='assistance finished')

    def on_event(self, msg: String):
        self.request((msg.data or '').strip(), why='/hri/speech/event')

    def on_say(self, msg: String):
        u = self.planner.make_text(msg.data, time.monotonic())
        if u:
            self._offer(u, 'free text')

    def on_stop(self, request, response):
        with self.lock:
            keep = [u for u in self.queue.items if u.priority >= PRIORITIES['warning']]
            dropped = len(self.queue.items) - len(keep)
            for u in self.queue.items:
                if u not in keep:
                    self._log(u, 'flushed', '', None, None, 'stop service')
            self.queue.items = keep
            if self.playing is not None and self.playing.priority < PRIORITIES['critical']:
                self.flushing = True
                self.player.stop()
        response.success = True
        response.message = (f'Stopped. Forgot {dropped} queued message(s); kept '
                            f'{len(keep)} warning/critical.')
        return response

    def on_params(self, params):
        for p in params:
            if p.name in ('volume', 'humour_chance', 'quiet_volume_scale'):
                if not 0.0 <= float(p.value) <= 1.0:
                    return SetParametersResult(successful=False, reason='must be 0.0 .. 1.0')
        for p in params:
            if p.name == 'volume':
                self.volume = float(p.value)
                self.get_logger().info(f'volume -> {self.volume:.2f}')
            elif p.name == 'humour_chance':
                self.planner.humour_chance = float(p.value)
            elif p.name == 'quiet_volume_scale':
                self.quiet_scale = float(p.value)
        self.status_dirty = True
        return SetParametersResult(successful=True)

    # ================================================================ periodic
    def tick(self):
        t = time.monotonic()
        old = self.motion.state
        new = self.motion.tick(t)
        if new:
            self._motion_changed(t, old, new)
        for key in self.state_watch.tick(t):
            self.request(key, force=True, why='/robot_state silent')
            self.lost_rep.start(t)
            self.fault_rep.stop()               # "lost contact" replaces the fault repeats
        for key in self.announcer.tick(t):
            self.request(key)
        for rep, key in ((self.fault_rep, 'fault'), (self.lost_rep, 'controller_lost'),
                         (self.assist_rep, 'assistance'), (self.batt_rep, 'battery_low')):
            if rep.due(t):
                self.request(key, force=True, why='repeat')

    def in_quiet_hours(self):
        return in_quiet_hours(datetime.now().hour, *self.quiet)

    def effective_volume(self):
        return self.volume * (self.quiet_scale if self.in_quiet_hours() else 1.0)

    def publish_status(self):
        with self.lock:
            u = self.playing
            status = {
                'speaking': u is not None,
                'key': u.key if u else '',
                'text': u.text if u else '',
                'priority': u.priority_name if u else '',
                'engine': self.engine if u else '',
                'queue': len(self.queue),
                'motion': self.motion.state,
                'turn': round(self.motion.turn, 2),
                'robot_state': self.state_watch.state or '',
                'audio_ok': self.audio_ok and not self.audio_error,
                'audio_error': self.audio_error,
                'volume': round(self.effective_volume(), 2),
                'quiet_hours': self.in_quiet_hours(),
                'spoken': self.spoken,
                'last_key': self.last_key,
                'done_count': self.done_count,
                'simulated': self.simulated,
            }
            self.status_dirty = False
        self.status_pub.publish(String(data=json.dumps(status)))

    # ================================================================ worker thread
    def _work(self):
        while True:
            with self.lock:
                self.lock.wait_for(lambda: len(self.queue) > 0 or self.stopping, timeout=0.5)
                if self.stopping:
                    return
                u = self.queue.pop(time.monotonic())
                for old in self.queue.expired + self.queue.evicted:
                    self._log(old, 'expired' if old in self.queue.expired else 'evicted',
                              '', None, None, '')
                self.queue.expired.clear()
                self.queue.evicted.clear()
                if u is None:
                    continue
                self.playing, self.engine = u, ''
                self.interrupted_for = None
                self.flushing = False
                self.status_dirty = True
            self._play(u)
            with self.lock:
                self.playing = None
                self.last_key = u.key
                self.done_count += 1
                self.status_dirty = True
            self.publish_status_safe()

    def publish_status_safe(self):
        try:
            self.publish_status()
        except Exception:                       # node shutting down
            pass

    def _build(self, u: Utterance):
        """-> (wav or None, engine, error)."""
        parts, engine, error = [], '', ''
        if u.sound:
            parts.append(self.sounds[u.sound])
            engine = 'chime'
        if u.text:
            try:
                wav, engine = self.tts.get(u.text, prefer_fast=u.priority >= PRIORITIES['warning'],
                                           length_scale=u.length_scale,
                                           sentence_silence=u.sentence_silence)
                parts.append(wav)
            except AudioError as err:
                error = str(err)
        if not parts:
            return None, engine, error
        try:
            joined = concat(parts)
        except AudioError:
            joined = parts[-1]                  # formats differ: the words matter most
        return prepare(joined, gain=self.effective_volume(), out_channels=self.out_channels,
                       lead_ms=self.lead_ms), engine, error

    def _play(self, u: Utterance):
        t_start = time.monotonic()
        try:
            wav, engine, error = self._build(u)
        except Exception as err:                # never let a bad file kill the worker
            wav, engine, error = None, '', f'{type(err).__name__}: {err}'
        with self.lock:
            self.engine = engine
        latency_ms = (time.monotonic() - u.created) * 1000.0
        if error:
            self.get_logger().warn(f'speech for "{u.key}": {error}', throttle_duration_sec=10.0)

        if wav is None or not self.audio_ok:
            # No sound possible: keep the caption on screen for a reading time.
            show = self._hold_caption(u)
            self._log(u, 'caption_only', engine, latency_ms, show, error or self.audio_error)
            return

        # Something more important arrived while this one was being prepared?
        result = 'stopped' if self._cut_off() else self.player.play(wav)
        dur = duration_s(wav)
        if result == 'done':
            with self.lock:
                self.spoken += 1
                if self.audio_error and not error:
                    self.get_logger().info('Audio output works again.')
                    self.audio_error = ''
            self._log(u, 'played', engine, latency_ms, dur, error)
        elif result == 'stopped':
            with self.lock:
                by = self.interrupted_for
            self._log(u, 'interrupted', engine, latency_ms, time.monotonic() - t_start,
                      f'by {by.key}' if by else ('shutdown' if self.stopping else 'stop service'))
        else:
            with self.lock:
                self.audio_error = result
            self.get_logger().error(f'Playback failed: {result}. Check alsa_device '
                                    f'("{self.device}") with `aplay -l`.',
                                    throttle_duration_sec=15.0)
            self._log(u, 'failed', engine, latency_ms, None, result)
            self._hold_caption(u)               # the screen still shows the message

    def _hold_caption(self, u: Utterance):
        """Keep the message 'playing' (so the caption stays on screen) for a
        normal reading time, unless something more important arrives."""
        show = max(2.5, len(u.text) / 14.0) if u.text else 0.5
        end = time.monotonic() + show
        while time.monotonic() < end and not self._cut_off():
            time.sleep(0.05)
        return show

    def _cut_off(self):
        with self.lock:
            return self.interrupted_for is not None or self.flushing or self.stopping

    def _log(self, u, result, engine, latency_ms, dur, detail):
        if self._csv is None:
            return
        try:
            self._csv.writerow([
                datetime.now().isoformat(timespec='milliseconds'), u.key, u.priority_name,
                result, engine, '' if latency_ms is None else f'{latency_ms:.0f}',
                '' if dur is None else f'{dur:.2f}', int(self.simulated),
                (detail or '')[:120]])
            self._csv_file.flush()
        except (OSError, ValueError):
            pass

    def _prerender(self):
        """Render, in the background and at low CPU priority, every library
        sentence that has no WAV yet. Only if Piper is installed."""
        ok, why = self.tts.piper_ok()
        if not ok:
            return
        missing = [j for j in all_jobs(self.library) if not self.tts.cached(*j)]
        if not missing:
            return
        self.get_logger().info(f'Pre-rendering {len(missing)} sentence(s) with Piper '
                               'in the background...')
        t0 = time.monotonic()
        done = 0
        for i in range(0, len(missing), 5):
            if self.stopping:
                return
            batch = missing[i:i + 5]
            try:
                self.tts.render_piper(batch)
                done += len(batch)
            except AudioError as err:
                self.get_logger().warn(f'Pre-render stopped: {err}')
                return
        self.get_logger().info(f'Pre-rendered {done} sentence(s) in '
                               f'{time.monotonic() - t0:.0f} s.')

    # ================================================================ shutdown
    def shutdown(self):
        with self.lock:
            self.stopping = True
            self.lock.notify_all()
        self.player.stop()
        self.worker.join(2.0)
        if self.say_shutdown and self.audio_ok:
            u = self.planner.make('shutdown', time.monotonic(), force=True,
                                  allow_humour=not self.in_quiet_hours())
            if u is not None:
                try:
                    wav, _, _ = self._build(u)
                    if wav is not None:
                        self.player.play(wav, timeout_s=6.0)
                except Exception:
                    pass
        if self._csv is not None:
            try:
                self._csv_file.close()
            except OSError:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SpeechNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
