#!/usr/bin/env python3
"""
piper_batch.py  --  renders several sentences to WAV files with Piper, loading
the voice model only ONCE. Runs with the Python inside the Piper virtual
environment (NOT the ROS Python), so it must not import anything from ROS or
from the rest of this package.

    echo '[{"text": "Hello", "path": "/tmp/hello.wav"}]' | \
        ~/piper-venv/bin/python3 piper_batch.py --model ~/piper-voices/en_GB-cori-medium.onnx

Each WAV is written to a temporary name first and then renamed, so a half-written
file can never be played.
"""

import argparse
import json
import os
import sys
import wave


def main():
    ap = argparse.ArgumentParser(description='Render sentences with Piper (one model load)')
    ap.add_argument('--model', required=True)
    ap.add_argument('--length-scale', type=float, default=1.0,
                    help='> 1 speaks more slowly (1.1 = 10%% slower)')
    ap.add_argument('--sentence-silence', type=float, default=0.25)
    args = ap.parse_args()

    from piper import PiperVoice, SynthesisConfig      # inside the venv only

    jobs = json.load(sys.stdin)
    voice = PiperVoice.load(args.model)
    for job in jobs:
        text, path = job['text'], job['path']
        # each sentence may have its own speed and pause (e.g. Anna's calm intro)
        cfg = SynthesisConfig(length_scale=float(job.get('length_scale', args.length_scale)))
        pause = float(job.get('sentence_silence', args.sentence_silence))
        os.makedirs(os.path.dirname(path), exist_ok=True)
        tmp = path + '.part'
        with wave.open(tmp, 'wb') as wav:
            first = True
            for i, chunk in enumerate(voice.synthesize(text, cfg)):
                if first:
                    wav.setframerate(chunk.sample_rate)
                    wav.setsampwidth(chunk.sample_width)
                    wav.setnchannels(chunk.sample_channels)
                    silence = bytes(int(chunk.sample_rate * pause) * 2)
                    first = False
                if i > 0:
                    wav.writeframes(silence)
                wav.writeframes(chunk.audio_int16_bytes)
        os.replace(tmp, path)
        print(f'ok {path}', flush=True)


if __name__ == '__main__':
    main()
