#!/usr/bin/env python3
import argparse
import math
import time

import numpy as np

try:
    import speech_recognition as sr
except Exception as exc:  # pragma: no cover
    raise SystemExit(f"speech_recognition import 실패: {exc}")


def _rms_level(chunk: bytes) -> float:
    if not chunk:
        return 0.0
    samples = np.frombuffer(chunk, dtype=np.int16)
    if samples.size <= 0:
        return 0.0
    return float(np.sqrt(np.mean(np.square(samples.astype(np.float32)))))


def _to_percent(rms: float, ref: float = 3000.0) -> int:
    if rms <= 0.0:
        return 0
    normalized = min(1.0, float(rms) / float(ref))
    return int(max(0, min(100, round((normalized ** 0.65) * 100.0))))


def _to_dbfs(rms: float) -> float:
    if rms <= 0.0:
        return -120.0
    return 20.0 * math.log10(float(rms) / 32768.0)


def main():
    parser = argparse.ArgumentParser(description="마이크 입력 레벨 임시 테스트")
    parser.add_argument("--device-index", type=int, default=None, help="speech_recognition Microphone device_index")
    parser.add_argument("--seconds", type=float, default=8.0, help="테스트 시간(초)")
    parser.add_argument("--gate", type=int, default=2, help="UI와 동일한 gate(이 값 미만은 0)")
    args = parser.parse_args()

    rec = sr.Recognizer()
    print(f"[mic-test] start device_index={args.device_index} seconds={args.seconds:.1f} gate={args.gate}")

    start = time.time()
    with sr.Microphone(device_index=args.device_index) as source:
        rec.adjust_for_ambient_noise(source, duration=0.2)
        while (time.time() - start) < float(args.seconds):
            chunk = source.stream.read(source.CHUNK)
            rms = _rms_level(chunk)
            level = _to_percent(rms)
            if level < int(args.gate):
                level = 0
            dbfs = _to_dbfs(rms)
            bar = "#" * int(level / 4)
            print(f"level={level:3d}%  rms={rms:8.2f}  dbfs={dbfs:7.2f}  {bar}")
            time.sleep(0.08)

    print("[mic-test] done")


if __name__ == "__main__":
    main()
