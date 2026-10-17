#!/usr/bin/env python3
"""
audio_tool.py  --  speaker checks WITHOUT ROS running (no nodes, no robot motion).
Reads the same settings as hri_speech (config/speech.yaml, config/phrases.yaml).

    ros2 run hospital_robot_hri audio_tool check            # what works, what is missing
    ros2 run hospital_robot_hri audio_tool chimes           # play every built-in sound
    ros2 run hospital_robot_hri audio_tool say "Hello there"   # speak a sentence (Piper)
    ros2 run hospital_robot_hri audio_tool say "Hello" --engine espeak
    ros2 run hospital_robot_hri audio_tool event moving_start  # chime + primary phrase
    ros2 run hospital_robot_hri audio_tool render           # pre-render the whole library
    ros2 run hospital_robot_hri audio_tool tone --seconds 5 --level 0.5   # steady 1 kHz tone

Options for all:  --device plughw:CARD=Headphones,DEV=0   --volume 0.8

Do not run it while hri_speech is talking: two programs on one ALSA device
either queue or fail with "Device or resource busy".
"""

import argparse
import array
import math
import os
import shutil
import subprocess
import sys
import time

import yaml

from hospital_robot_hri.audio_player import (
    SOUNDS, AlsaPlayer, AudioError, Tts, concat, duration_s, make_sound, prepare,
    write_wav, _pcm)
from hospital_robot_hri.speech_logic import all_jobs, load_library


def config_dir():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('hospital_robot_hri'), 'config')
    except Exception:                              # running from the source folder
        return os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config')


def load_settings():
    with open(os.path.join(config_dir(), 'speech.yaml')) as f:
        return yaml.safe_load(f)['hri_speech']['ros__parameters']


def load_lib(settings):
    path = settings.get('phrases_file') or os.path.join(config_dir(), 'phrases.yaml')
    with open(path) as f:
        return load_library(yaml.safe_load(f), known_sounds=SOUNDS)


def make_tts(s):
    return Tts(cache_dir=s['cache_dir'], voice=s['voice'], voice_dir=s['voice_dir'],
               piper_python=s['piper_python'], length_scale=s['length_scale'],
               sentence_silence=s.get('sentence_silence', 0.25),
               espeak_voice=s['espeak_voice'], espeak_speed=s['espeak_speed'])


def play(player, wav, volume, channels=2, label=''):
    out = prepare(wav, gain=volume, out_channels=channels)
    t0 = time.monotonic()
    r = player.play(out)
    print(f'  {label:<12} {duration_s(out):5.2f} s  -> {r}  ({time.monotonic() - t0:.2f} s)')
    return r


def cmd_check(a, s, player, tts):
    ok = True
    print('Audio player:')
    av, why = player.available()
    print(f'  aplay              {"OK " + why if av else "MISSING: " + why}')
    ok &= av
    if shutil.which('aplay'):
        cards = subprocess.run(['aplay', '-l'], capture_output=True, text=True).stdout
        print('  cards (aplay -l):')
        for line in cards.splitlines():
            if line.startswith('card'):
                print('    ' + line)
    print(f'  device             {player.device}')
    if av:
        silence = write_wav(22050, 2, bytes(4 * 2205))
        r = player.play(silence)
        print(f'  open device        {"OK" if r == "done" else r}')
        ok &= r == 'done'
    print('Speech engines:')
    pok, pwhy = tts.piper_ok()
    print(f'  Piper              {"OK" if pok else "not available: " + pwhy}')
    eok, ewhy = tts.espeak_ok()
    print(f'  espeak-ng          {"OK" if eok else "not available: " + ewhy}')
    ok &= pok or eok
    lib = load_lib(s)
    texts = all_jobs(lib)
    n = sum(1 for j in texts if tts.cached(*j))
    print(f'Library: {len(lib)} events, {len(texts)} sentences, {n} pre-rendered '
          f'in {os.path.join(os.path.expanduser(s["cache_dir"]), "")}')
    print('\nRESULT:', 'OK' if ok else 'PROBLEMS - see above')
    return 0 if ok else 1


def cmd_chimes(a, s, player, tts):
    for name in SOUNDS:
        play(player, make_sound(name), a.volume, label=name)
        time.sleep(0.6)
    return 0


def speak(player, tts, text, engine, volume, sound=None, ls=None, ss=None):
    t0 = time.monotonic()
    try:
        if engine == 'espeak':
            wav, src = tts.render_espeak(text, ls), 'espeak'
        elif engine == 'piper':
            if not tts.cached(text, ls, ss):
                tts.render_piper([(text, ls, ss)])
            with open(tts.path_for(text, ls, ss), 'rb') as f:
                wav, src = f.read(), 'piper'
        else:
            wav, src = tts.get(text, length_scale=ls, sentence_silence=ss)
    except AudioError as err:
        print(f'  FAILED: {err}')
        return 1
    synth = time.monotonic() - t0
    if sound:
        try:
            wav = concat([make_sound(sound), wav])
        except AudioError:
            pass
    print(f'  engine {src}, ready after {synth:.2f} s')
    return 0 if play(player, wav, volume, label=src) == 'done' else 1


def cmd_say(a, s, player, tts):
    return speak(player, tts, a.text, a.engine, a.volume)


def cmd_event(a, s, player, tts):
    lib = load_lib(s)
    e = lib.get(a.key)
    if e is None:
        print(f'Unknown event. Known: {" ".join(sorted(lib))}')
        return 1
    print(f'{a.key} [{e.priority}]: {e.phrases[0] if e.phrases else "(sound only)"}')
    if not e.phrases:
        return 0 if play(player, make_sound(e.sound), a.volume, label=e.sound) == 'done' else 1
    return speak(player, tts, e.phrases[0], a.engine, a.volume, sound=e.sound,
                 ls=e.length_scale, ss=e.sentence_silence)


def cmd_render(a, s, player, tts):
    ok, why = tts.piper_ok()
    if not ok:
        print(f'Piper is not available: {why}')
        return 1
    texts = all_jobs(load_lib(s))
    todo = texts if a.force else [j for j in texts if not tts.cached(*j)]
    print(f'{len(texts)} sentences, {len(todo)} to render with {tts.voice} '
          f'(length_scale {tts.length_scale}) ...')
    t0 = time.monotonic()
    for i in range(0, len(todo), 10):
        batch = todo[i:i + 10]
        if a.force:
            for j in batch:
                p = tts.path_for(*j)
                if os.path.exists(p):
                    os.remove(p)
        try:
            tts.render_piper(batch, timeout_s=600)
        except AudioError as err:
            print(f'FAILED: {err}')
            return 1
        print(f'  {min(i + 10, len(todo))}/{len(todo)}')
    dt = time.monotonic() - t0
    print(f'Done in {dt:.1f} s ({dt / max(1, len(todo)):.2f} s per sentence). '
          f'Files: {os.path.join(tts.cache_dir, "")}')
    return 0


def cmd_tone(a, s, player, tts):
    rate, n = 22050, int(22050 * a.seconds)
    level = max(0.0, min(1.0, a.level))
    ramp = int(0.02 * rate)
    samples = array.array('h', (
        int(32767 * level * min(1.0, k / ramp, (n - k) / ramp)
            * math.sin(2 * math.pi * a.freq * k / rate)) for k in range(n)))
    print(f'{a.freq:.0f} Hz tone, level {level:.2f}, {a.seconds:.1f} s, volume {a.volume:.2f}')
    return 0 if play(player, write_wav(rate, 1, _pcm(samples)), a.volume,
                     label='tone') == 'done' else 1


def main():
    s = load_settings()
    ap = argparse.ArgumentParser(description='Speaker checks without ROS')
    ap.add_argument('--device', default=s['alsa_device'])
    ap.add_argument('--volume', type=float, default=float(s['volume']))
    sub = ap.add_subparsers(dest='cmd', required=True)
    sub.add_parser('check')
    sub.add_parser('chimes')
    p = sub.add_parser('say')
    p.add_argument('text')
    p.add_argument('--engine', choices=['auto', 'piper', 'espeak'], default='auto')
    p = sub.add_parser('event')
    p.add_argument('key')
    p.add_argument('--engine', choices=['auto', 'piper', 'espeak'], default='auto')
    p = sub.add_parser('render')
    p.add_argument('--force', action='store_true', help='render again even if the WAV exists')
    p = sub.add_parser('tone')
    p.add_argument('--freq', type=float, default=1000.0)
    p.add_argument('--seconds', type=float, default=5.0)
    p.add_argument('--level', type=float, default=0.5)
    a = ap.parse_args()

    player = AlsaPlayer(a.device)
    tts = make_tts(s)
    fn = {'check': cmd_check, 'chimes': cmd_chimes, 'say': cmd_say, 'event': cmd_event,
          'render': cmd_render, 'tone': cmd_tone}[a.cmd]
    sys.exit(fn(a, s, player, tts))


if __name__ == '__main__':
    main()
