"""
audio_player.py  --  makes sound come out of the speaker. NO ROS code
(tested in test/test_audio_player.py).

    Tts           text -> WAV:  1. pre-rendered cache file (instant)
                                2. Piper neural voice (offline, ~1-3 s on the Pi)
                                3. espeak-ng (offline, instant, robotic) - fallback
    make_sound()  the built-in chimes, generated mathematically (no sound files,
                  no licences to worry about)
    prepare()     volume, mono -> stereo, a little lead-in silence
    AlsaPlayer    plays a WAV with `aplay` on one ALSA device; stop() cuts it off

Hardware (wiring sheet): Pi 4 3.5 mm jack -> PAM8403 -> 3 W 4 ohm speaker on
the LEFT channel only (L+/L-). That is why everything is played as STEREO with
the same signal on both channels: the speaker always gets it.
"""

import array
import hashlib
import io
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import threading
import wave

RATE = 22050                      # Hz, the built-in sounds (Piper medium voices use 22050 too)


class AudioError(Exception):
    pass


# ---------------------------------------------------------------------------
# WAV helpers
# ---------------------------------------------------------------------------
def read_wav(data) -> tuple:
    """data = file path or bytes. Returns (rate, channels, sampwidth, pcm_bytes)."""
    try:
        f = wave.open(io.BytesIO(data) if isinstance(data, (bytes, bytearray)) else data, 'rb')
    except (wave.Error, EOFError, OSError) as err:
        raise AudioError(f'not a readable WAV file: {err}') from err
    with f:
        return f.getframerate(), f.getnchannels(), f.getsampwidth(), f.readframes(f.getnframes())


def write_wav(rate: int, channels: int, pcm: bytes) -> bytes:
    buf = io.BytesIO()
    with wave.open(buf, 'wb') as f:
        f.setnchannels(channels)
        f.setsampwidth(2)
        f.setframerate(rate)
        f.writeframes(pcm)
    return buf.getvalue()


def _samples(pcm: bytes) -> array.array:
    a = array.array('h')
    a.frombytes(pcm)
    if sys.byteorder == 'big':                 # WAV is little-endian
        a.byteswap()
    return a


def _pcm(a: array.array) -> bytes:
    if sys.byteorder == 'big':
        a = array.array('h', a)
        a.byteswap()
    return a.tobytes()


def prepare(wav: bytes, gain: float = 1.0, out_channels: int = 2,
            lead_ms: float = 60.0, tail_ms: float = 40.0) -> bytes:
    """Scale the volume (0..1, never louder than the file), convert mono to
    stereo (same signal both sides) and add a little silence at the start and
    end. The lead-in stops the first syllable being clipped while the Pi's
    audio output and the amplifier wake up."""
    rate, ch, width, pcm = read_wav(wav)
    if width != 2:
        raise AudioError(f'only 16-bit WAV is supported (this one is {8 * width}-bit)')
    s = _samples(pcm)
    if ch not in (1, 2):
        raise AudioError(f'only mono or stereo WAV is supported ({ch} channels)')
    gain = max(0.0, min(1.0, float(gain)))
    if gain < 0.999:
        s = array.array('h', (int(x * gain) for x in s))
    if ch == 1 and out_channels == 2:
        st = array.array('h', bytes(4 * len(s)))
        st[0::2] = s
        st[1::2] = s
        s, ch = st, 2
    elif ch == 2 and out_channels == 1:
        s = array.array('h', ((s[i] + s[i + 1]) // 2 for i in range(0, len(s), 2)))
        ch = 1
    lead = bytes(2 * ch * int(rate * lead_ms / 1000.0))
    tail = bytes(2 * ch * int(rate * tail_ms / 1000.0))
    return write_wav(rate, ch, lead + _pcm(s) + tail)


def concat(wavs) -> bytes:
    """Join WAVs that share rate/channels (used for chime + speech), with a
    short gap between them. Different formats raise AudioError."""
    parts, fmt = [], None
    for w in wavs:
        rate, ch, width, pcm = read_wav(w)
        if fmt is None:
            fmt = (rate, ch, width)
        elif (rate, ch, width) != fmt:
            raise AudioError('cannot join WAVs with different formats')
        if parts:
            parts.append(bytes(2 * ch * int(rate * 0.12)))
        parts.append(pcm)
    if fmt is None:
        raise AudioError('nothing to join')
    return write_wav(fmt[0], fmt[1], b''.join(parts))


def duration_s(wav: bytes) -> float:
    rate, ch, width, pcm = read_wav(wav)
    return len(pcm) / float(rate * ch * width)


# ---------------------------------------------------------------------------
# Built-in sounds: soft, low-pitched-enough chimes made from decaying sines.
# (start s, frequency Hz, length s, loudness 0..1)
# ---------------------------------------------------------------------------
SOUNDS = {
    # gentle "hello" rising three notes (C5 E5 G5)
    'start': [(0.00, 523.25, 0.45, 0.5), (0.14, 659.25, 0.45, 0.5), (0.28, 783.99, 0.7, 0.55)],
    # "I'm about to move": two-note rising (E5 -> A5), friendly but noticeable
    'move': [(0.00, 659.25, 0.35, 0.6), (0.18, 880.00, 0.55, 0.6)],
    # quiet single tick every few seconds while moving
    'tick': [(0.00, 987.77, 0.18, 0.35)],
    # attention: three quick notes, for warnings
    'attention': [(0.00, 880.00, 0.22, 0.6), (0.16, 880.00, 0.22, 0.6), (0.32, 1174.66, 0.4, 0.65)],
    # alert: two-tone falling, repeated, for critical messages
    'alert': [(0.00, 1046.50, 0.25, 0.7), (0.25, 783.99, 0.25, 0.7),
              (0.55, 1046.50, 0.25, 0.7), (0.80, 783.99, 0.35, 0.7)],
    # Anna waking up: slow, soft, low rising notes (C5 G5 C6), long fade
    'wake': [(0.00, 523.25, 0.9, 0.35), (0.35, 783.99, 0.9, 0.4), (0.75, 1046.50, 1.3, 0.45)],
    # arrival / success: descending-then-up "ta-da" (G5 C6)
    'ok': [(0.00, 783.99, 0.3, 0.5), (0.15, 1046.50, 0.6, 0.55)],
}


def make_sound(name: str, rate: int = RATE) -> bytes:
    """Mono 16-bit WAV of a built-in sound."""
    notes = SOUNDS.get(name)
    if notes is None:
        raise AudioError(f'unknown sound "{name}". Known: {sorted(SOUNDS)}')
    total = max(start + length for start, _, length, _ in notes) + 0.05
    n = int(total * rate)
    buf = [0.0] * n
    for start, freq, length, amp in notes:
        i0 = int(start * rate)
        m = int(length * rate)
        attack = max(1, int(0.008 * rate))           # 8 ms: no click
        for k in range(m):
            if i0 + k >= n:
                break
            t = k / rate
            env = min(1.0, k / attack) * math.exp(-t * 6.0 / length)
            x = (math.sin(2 * math.pi * freq * t)
                 + 0.25 * math.sin(2 * math.pi * 2 * freq * t)    # a little brightness
                 + 0.08 * math.sin(2 * math.pi * 3 * freq * t))
            buf[i0 + k] += amp * env * x / 1.33
    peak = max(1e-9, max(abs(v) for v in buf))
    scale = 0.8 * 32767 / max(peak, 1.0)                     # never clip
    a = array.array('h', (int(max(-32767, min(32767, v * scale))) for v in buf))
    return write_wav(rate, 1, _pcm(a))


# ---------------------------------------------------------------------------
# Text to speech
# ---------------------------------------------------------------------------
def voice_tag(voice: str, length_scale: float, sentence_silence: float = 0.25) -> str:
    safe = ''.join(c if c.isalnum() or c in '-_.' else '_' for c in voice)
    return f'{safe}_ls{length_scale:.2f}_ss{sentence_silence:.2f}'


def cache_file(cache_dir: str, voice: str, length_scale: float, text: str,
               sentence_silence: float = 0.25) -> str:
    """Where the WAV for this exact sentence (and speaking style) lives. The
    name is a hash of the text, so the same sentence always maps to the same
    file (on the PC and the Pi alike), and changing a sentence or its speed
    automatically means a new file."""
    h = hashlib.sha1(' '.join(text.split()).encode('utf-8')).hexdigest()[:16]
    return os.path.join(os.path.expanduser(cache_dir),
                        voice_tag(voice, length_scale, sentence_silence), h + '.wav')


PIPER_BATCH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'piper_batch.py')


class Tts:
    """length_scale / sentence_silence arguments of None mean "use the default
    from speech.yaml". A calm intro (Anna's wake-up line) uses its own slower
    values from phrases.yaml."""

    def __init__(self, cache_dir='~/.local/share/hospital_robot_hri/speech_cache',
                 voice='en_GB-cori-medium', voice_dir='~/piper-voices',
                 piper_python='~/piper-venv/bin/python3', length_scale=1.1,
                 sentence_silence=0.25, espeak_voice='en-gb+f3', espeak_speed=150,
                 piper_timeout_s=60.0):
        self.cache_dir = os.path.expanduser(cache_dir)
        self.voice = voice
        self.voice_dir = os.path.expanduser(voice_dir)
        self.piper_python = os.path.expanduser(piper_python)
        self.length_scale = float(length_scale)
        self.sentence_silence = float(sentence_silence)
        self.espeak_voice = espeak_voice
        self.espeak_speed = int(espeak_speed)
        self.piper_timeout_s = piper_timeout_s
        self._lock = threading.Lock()            # one Piper process at a time

    def _style(self, length_scale=None, sentence_silence=None):
        return (self.length_scale if length_scale is None else float(length_scale),
                self.sentence_silence if sentence_silence is None else float(sentence_silence))

    # -------------------------------------------------------- what is available
    def path_for(self, text: str, length_scale=None, sentence_silence=None) -> str:
        ls, ss = self._style(length_scale, sentence_silence)
        return cache_file(self.cache_dir, self.voice, ls, text, ss)

    def cached(self, text: str, length_scale=None, sentence_silence=None):
        p = self.path_for(text, length_scale, sentence_silence)
        return p if os.path.isfile(p) and os.path.getsize(p) > 44 else None

    def piper_ok(self):
        model = os.path.join(self.voice_dir, self.voice + '.onnx')
        if not os.path.isfile(self.piper_python):
            return False, f'Piper python not found: {self.piper_python}'
        if not os.path.isfile(model):
            return False, f'voice model not found: {model}'
        if not os.path.isfile(model + '.json'):
            return False, f'voice config not found: {model}.json'
        return True, 'ok'

    @staticmethod
    def espeak_ok():
        exe = shutil.which('espeak-ng')
        return (True, exe) if exe else (False, 'espeak-ng not installed')

    # -------------------------------------------------------- engines
    def render_piper(self, items, timeout_s=None):
        """items = list of (text, length_scale, sentence_silence); None = default.
        Renders them all with ONE Piper process (the voice model is loaded
        once) into the cache. Raises AudioError."""
        ok, why = self.piper_ok()
        if not ok:
            raise AudioError(why)
        jobs = []
        for item in items:
            text, ls, ss = (tuple(item) + (None, None))[:3]
            ls, ss = self._style(ls, ss)
            path = self.path_for(text, ls, ss)
            os.makedirs(os.path.dirname(path), exist_ok=True)
            jobs.append({'text': text, 'path': path, 'length_scale': ls,
                         'sentence_silence': ss})
        # `nice` = low CPU priority, so rendering never slows the LiDAR or Nav2
        nice = ['nice', '-n', '10'] if shutil.which('nice') else []
        cmd = nice + [self.piper_python, PIPER_BATCH,
                      '--model', os.path.join(self.voice_dir, self.voice + '.onnx'),
                      '--length-scale', str(self.length_scale),
                      '--sentence-silence', str(self.sentence_silence)]
        with self._lock:
            try:
                r = subprocess.run(cmd, input=json.dumps(jobs).encode(), capture_output=True,
                                   timeout=timeout_s or self.piper_timeout_s * max(1, len(jobs)))
            except subprocess.TimeoutExpired as err:
                raise AudioError('Piper took too long') from err
            except OSError as err:
                raise AudioError(f'could not start Piper: {err}') from err
        if r.returncode != 0:
            tail = r.stderr.decode(errors='replace').strip().splitlines()[-3:]
            raise AudioError('Piper failed: ' + ' | '.join(tail))
        missing = [j['path'] for j in jobs if not os.path.isfile(j['path'])]
        if missing:
            raise AudioError(f'Piper wrote no file for {len(missing)} sentence(s)')

    def render_espeak(self, text: str, length_scale=None) -> bytes:
        ok, why = self.espeak_ok()
        if not ok:
            raise AudioError(why)
        ls, _ = self._style(length_scale, None)
        speed = max(80, int(self.espeak_speed * self.length_scale / ls))   # slower style = slower
        try:
            r = subprocess.run(['espeak-ng', '-v', self.espeak_voice, '-s', str(speed),
                                '--stdout', text], capture_output=True, timeout=15)
        except (OSError, subprocess.TimeoutExpired) as err:
            raise AudioError(f'espeak-ng failed: {err}') from err
        if r.returncode != 0 or len(r.stdout) < 100:
            raise AudioError('espeak-ng failed: ' + r.stderr.decode(errors='replace').strip())
        return _fix_stream_wav(r.stdout)

    def get(self, text: str, prefer_fast: bool = False, length_scale=None,
            sentence_silence=None):
        """Returns (wav_bytes, source) with source 'cache' | 'piper' | 'espeak'.
        prefer_fast = True (used for warnings/critical): if the sentence is not
        pre-rendered, use espeak-ng now rather than wait seconds for Piper."""
        p = self.cached(text, length_scale, sentence_silence)
        if p:
            with open(p, 'rb') as f:
                return f.read(), 'cache'
        errors = []
        order = ('espeak', 'piper') if prefer_fast else ('piper', 'espeak')
        for engine in order:
            try:
                if engine == 'piper':
                    self.render_piper([(text, length_scale, sentence_silence)])
                    with open(self.path_for(text, length_scale, sentence_silence), 'rb') as f:
                        return f.read(), 'piper'
                return self.render_espeak(text, length_scale), 'espeak'
            except AudioError as err:
                errors.append(f'{engine}: {err}')
        raise AudioError('no speech engine worked (' + '; '.join(errors) + ')')


def _fix_stream_wav(data: bytes) -> bytes:
    """espeak-ng --stdout writes a WAV whose size fields are 'unknown'
    (0xFFFFFFFF). Rebuild a proper header so the wave module accepts it."""
    if data[:4] != b'RIFF' or b'data' not in data[:200]:
        raise AudioError('espeak-ng did not produce a WAV')
    fmt_at = data.find(b'fmt ')
    ch = int.from_bytes(data[fmt_at + 10:fmt_at + 12], 'little')
    rate = int.from_bytes(data[fmt_at + 12:fmt_at + 16], 'little')
    width = int.from_bytes(data[fmt_at + 22:fmt_at + 24], 'little') // 8
    pcm = data[data.find(b'data') + 8:]
    pcm = pcm[:len(pcm) - len(pcm) % (ch * width)]
    if width != 2:
        raise AudioError('espeak-ng output is not 16-bit')
    return write_wav(rate, ch, pcm)


# ---------------------------------------------------------------------------
# Playback
# ---------------------------------------------------------------------------
class AlsaPlayer:
    """Plays one WAV at a time through `aplay`. play() blocks until the sound
    has finished; stop() (from another thread) cuts it off immediately."""

    def __init__(self, device: str = 'default', command: str = 'aplay'):
        self.device = device
        self.command = command
        self._proc = None
        self._stopped = False
        self._lock = threading.Lock()

    def available(self):
        exe = shutil.which(self.command)
        return (True, exe) if exe else (False, f'"{self.command}" not found (sudo apt install alsa-utils)')

    def play(self, wav: bytes, timeout_s: float = 60.0) -> str:
        """Returns 'done', 'stopped', or 'error: ...'. Never raises."""
        cmd = [self.command, '-q', '-D', self.device, '-']
        with self._lock:
            self._stopped = False
            try:
                # own process group, so stop() also ends any helper processes
                self._proc = subprocess.Popen(cmd, stdin=subprocess.PIPE,
                                              stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                                              start_new_session=True)
            except OSError as err:
                self._proc = None
                return f'error: could not start {self.command}: {err}'
            proc = self._proc
        try:
            _, err = proc.communicate(input=wav, timeout=timeout_s)
        except subprocess.TimeoutExpired:
            _kill(proc)
            proc.communicate()
            return 'error: playback timed out'
        except (BrokenPipeError, OSError, ValueError):
            _kill(proc)
            proc.wait()
            err = b''
        finally:
            with self._lock:
                self._proc = None
        if self._stopped:
            return 'stopped'
        if proc.returncode != 0:
            msg = (err or b'').decode(errors='replace').strip().splitlines()
            return f'error: {self.command} exit {proc.returncode}: ' + (msg[-1] if msg else '')
        return 'done'

    def stop(self):
        with self._lock:
            self._stopped = True
            if self._proc is not None and self._proc.poll() is None:
                _kill(self._proc)


def _kill(proc):
    try:
        os.killpg(proc.pid, signal.SIGKILL)
    except (ProcessLookupError, PermissionError, OSError):
        try:
            proc.kill()
        except OSError:
            pass
