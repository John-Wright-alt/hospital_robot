"""Unit tests for audio_player.py. No speaker, no ROS: `aplay` is replaced by a
tiny stand-in script, so these run on any Linux PC.

    python3 -m pytest test/ -v        (from inside the hospital_robot_hri folder)
"""

import os
import stat
import threading
import time

import pytest

from hospital_robot_hri.audio_player import (
    SOUNDS, AlsaPlayer, AudioError, Tts, cache_file, concat, duration_s, make_sound,
    prepare, read_wav, write_wav, _fix_stream_wav)


def fake_cmd(tmp_path, body):
    p = tmp_path / 'fake_aplay'
    p.write_text('#!/bin/sh\n' + body + '\n')
    p.chmod(p.stat().st_mode | stat.S_IEXEC)
    return str(p)


def test_every_builtin_sound_is_valid_and_short():
    for name in SOUNDS:
        w = make_sound(name)
        rate, ch, width, pcm = read_wav(w)
        assert (rate, ch, width) == (22050, 1, 2)
        assert 0.1 < duration_s(w) < (2.5 if name == 'wake' else 1.5), name


def test_unknown_sound():
    with pytest.raises(AudioError):
        make_sound('kazoo')


def test_prepare_makes_stereo_scales_and_pads():
    w = write_wav(22050, 1, (1000).to_bytes(2, 'little', signed=True) * 2205)   # 0.1 s
    out = prepare(w, gain=0.5, out_channels=2, lead_ms=60, tail_ms=40)
    rate, ch, width, pcm = read_wav(out)
    assert ch == 2
    assert abs(duration_s(out) - 0.2) < 0.002                # 0.1 s + 60 ms + 40 ms
    lead = int(22050 * 0.06) * 4
    first = int.from_bytes(pcm[lead:lead + 2], 'little', signed=True)
    second = int.from_bytes(pcm[lead + 2:lead + 4], 'little', signed=True)
    assert first == second == 500                            # same on L and R, half volume


def test_gain_never_amplifies():
    w = write_wav(22050, 1, (1000).to_bytes(2, 'little', signed=True) * 100)
    out = prepare(w, gain=5.0, out_channels=1, lead_ms=0, tail_ms=0)
    assert int.from_bytes(read_wav(out)[3][:2], 'little', signed=True) == 1000


def test_concat_and_mismatch():
    a, b = make_sound('tick'), make_sound('ok')
    assert duration_s(concat([a, b])) > duration_s(a) + duration_s(b)
    with pytest.raises(AudioError):
        concat([a, write_wav(16000, 1, b'\0\0' * 100)])


def test_cache_name_is_stable_and_whitespace_insensitive():
    a = cache_file('/c', 'en_GB-cori-medium', 1.1, 'Hello  there.')
    b = cache_file('/c', 'en_GB-cori-medium', 1.1, 'Hello there.')
    c = cache_file('/c', 'en_GB-cori-medium', 1.0, 'Hello there.')
    assert a == b and a != c and a.endswith('.wav')


def test_espeak_stream_header_is_repaired():
    good = write_wav(22050, 1, b'\x10\x00' * 500)
    broken = bytearray(good)
    broken[4:8] = b'\xff\xff\xff\xff'
    broken[40:44] = b'\xff\xff\xff\xff'
    fixed = _fix_stream_wav(bytes(broken))
    assert read_wav(fixed)[3] == b'\x10\x00' * 500


def test_style_changes_the_cache_file(tmp_path):
    tts = Tts(cache_dir=str(tmp_path), voice='v', length_scale=1.1, sentence_silence=0.25)
    assert tts.path_for('Hi') == tts.path_for('Hi', 1.1, 0.25)       # None = defaults
    assert tts.path_for('Hi', 1.3, 0.6) != tts.path_for('Hi')        # calm intro = own file


def test_tts_uses_cache_first(tmp_path):
    tts = Tts(cache_dir=str(tmp_path), voice='v', piper_python='/nonexistent')
    path = tts.path_for('Hi')
    os.makedirs(os.path.dirname(path))
    with open(path, 'wb') as f:
        f.write(make_sound('tick'))
    wav, src = tts.get('Hi')
    assert src == 'cache' and read_wav(wav)


def test_tts_reports_clearly_when_nothing_works(tmp_path, monkeypatch):
    monkeypatch.setenv('PATH', str(tmp_path))                 # no espeak-ng on PATH
    tts = Tts(cache_dir=str(tmp_path), piper_python='/nonexistent')
    with pytest.raises(AudioError, match='no speech engine'):
        tts.get('Hello')


def test_player_done_error_and_stop(tmp_path):
    ok = AlsaPlayer(command=fake_cmd(tmp_path, 'cat > /dev/null'))
    assert ok.play(make_sound('tick')) == 'done'

    bad = AlsaPlayer(command=fake_cmd(tmp_path, 'echo "no such device" >&2; exit 1'))
    assert bad.play(make_sound('tick')).startswith('error')

    missing = AlsaPlayer(command=str(tmp_path / 'nope'))
    assert missing.play(make_sound('tick')).startswith('error')
    assert missing.available()[0] is False

    slow = AlsaPlayer(command=fake_cmd(tmp_path, 'cat > /dev/null; sleep 5'))
    result = {}
    th = threading.Thread(target=lambda: result.setdefault('r', slow.play(make_sound('tick'))))
    t0 = time.monotonic()
    th.start()
    time.sleep(0.3)
    slow.stop()
    th.join(3.0)
    assert result.get('r') == 'stopped'
    assert time.monotonic() - t0 < 2.0                        # cut off, not waited out
