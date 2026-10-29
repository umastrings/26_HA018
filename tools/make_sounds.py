#!/usr/bin/env python3
"""Generate care_voice mp3 files once on a laptop with internet.

  pip install edge-tts lameenc
  python tools/make_sounds.py --aed-location "간호사실 옆"

Speech uses edge-tts; alarm.mp3 is generated locally (nurse-call style ding-dong chime).
Copy the output folder to the Jetson; playback there is fully offline.
Without --aed-location, fall_aed.mp3 is skipped (location not decided yet).
"""
import argparse
import asyncio
import math
from pathlib import Path
import struct

import edge_tts
import lameenc

VOICE = 'ko-KR-SunHiNeural'
RATE = '-10%'
SENTENCES = {
    'fall_1': '낙상이 감지되었습니다.',
    'fall_119': '가까운 직원을 불러주세요.',  # replaces the original 119 request
    'estop': '비상정지되었습니다.',
    'clear': '상황이 해제되었습니다. 대기 위치로 돌아갑니다.',
    'patrol': '병실 순회를 시작합니다.',
    'bedsore': '환자분의 체위 변경이 필요합니다.',  # pressure-sore risk, played once
}
AED_TEMPLATE = '자동심장충격기는 {}에 있습니다.'


def alarm_mp3(path, tones=(784.0, 659.0), beat=0.5, repeats=2, harmonics=(1.0, 0.5, 0.25, 0.1),
              decay=3.0, amplitude=0.6, rate=44100):
    """2 s nurse-call chime: ding (G5) - dong (E5) with bell-like decay, repeated."""
    fade = int(0.005 * rate)
    n = int(beat * rate)
    norm = sum(harmonics)
    samples = []
    for _ in range(repeats):
        for freq in tones:
            for i in range(n):
                envelope = min(1.0, i / fade, (n - 1 - i) / fade) * math.exp(-decay * i / rate)
                t = 2 * math.pi * freq * i / rate
                value = sum(h * math.sin((k + 1) * t) for k, h in enumerate(harmonics)) / norm
                samples.append(int(32767 * amplitude * envelope * value))
    encoder = lameenc.Encoder()
    encoder.set_bit_rate(128)
    encoder.set_in_sample_rate(rate)
    encoder.set_channels(1)
    encoder.set_quality(2)
    pcm = struct.pack(f'<{len(samples)}h', *samples)
    path.write_bytes(encoder.encode(pcm) + encoder.flush())


async def speak(text, path):
    await edge_tts.Communicate(text, VOICE, rate=RATE).save(str(path))


async def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', default=str(root/'sounds'))
    parser.add_argument('--aed-location', default='')
    args = parser.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    sentences = dict(SENTENCES)
    if args.aed_location:
        sentences['fall_aed'] = AED_TEMPLATE.format(args.aed_location)
    else:
        print('fall_aed.mp3 skipped: pass --aed-location once the AED location is decided')

    alarm_mp3(out/'alarm.mp3')
    print('alarm.mp3')
    for name, text in sentences.items():
        await speak(text, out/f'{name}.mp3')
        print(f'{name}.mp3  {text}')


if __name__ == '__main__':
    asyncio.run(main())
