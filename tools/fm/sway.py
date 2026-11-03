#!/usr/bin/env python3
"""Side-to-side sway on straight roads while openpilot steers: how much the wheel, the model's request, lane centering's
addition and the final request wander (drift slower than 3 s removed), and the dominant period.

usage: sway.py rlog.zst [--min-mph 20]
"""
import argparse

import numpy as np

from fmlog import MPH, model_name, read


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("rlog", nargs="+")
  ap.add_argument("--min-mph", type=float, default=20.0)
  a = ap.parse_args()
  for path in a.rlog:
    evs = read(path)
    t0 = next(e.logMonoTime for e in evs if e.which() == "carState")
    T = lambda e: (e.logMonoTime - t0) / 1e9  # noqa: E731
    cs = np.array([(T(e), e.carState.vEgo, e.carState.steeringAngleDeg, float(e.carState.steeringPressed)) for e in evs if e.which() == "carState"])
    ctl = np.array([(T(e), e.controlsState.desiredCurvature) for e in evs if e.which() == "controlsState"])
    cc = np.array([(T(e), float(e.carControl.latActive)) for e in evs if e.which() == "carControl"])
    mv = np.array([(T(e), e.modelV2.action.desiredCurvature) for e in evs if e.which() == "modelV2"])
    t = mv[:, 0]
    I = lambda arr, c: np.interp(t, arr[:, 0], arr[:, c])  # noqa: E731
    v, ang, pressed, lat, des = I(cs, 1), I(cs, 2), I(cs, 3) > 0.5, I(cc, 1) > 0.5, I(ctl, 1)
    straight = (v * MPH > a.min_mph) & lat & ~pressed & (np.abs(mv[:, 1]) < 0.002)
    idx = np.where(straight)[0]
    runs = [r for r in np.split(idx, np.where(np.diff(idx) > 1)[0] + 1) if len(r) > 160] if idx.size else []
    if not runs:
      print(f"{path}: model {model_name(evs)}, no straight engaged stretch of 8 s or more above {a.min_mph:.0f} mph")
      continue

    def wander(x):
      parts = []
      for r in runs:
        y = x[r] - np.convolve(x[r], np.ones(60) / 60, "same")
        parts.append(y[30:-30])
      return np.std(np.concatenate(parts))

    r = max(runs, key=len)
    y = ang[r] - np.convolve(ang[r], np.ones(60) / 60, "same")
    F = np.abs(np.fft.rfft(y * np.hanning(len(y))))
    fr = np.fft.rfftfreq(len(y), 0.05)
    k = np.argmax(F * (fr > 0.15))
    n = sum(len(r) for r in runs) / 20
    print(f"{path}: model {model_name(evs)}, {n:.0f} s straight at {np.mean(v[np.concatenate(runs)]) * MPH:.0f} mph | wheel sway "
          f"{wander(ang):.2f} deg | model {wander(mv[:, 1]) * 1e4:.2f}e-4, lane centering adds {wander(des - mv[:, 1]) * 1e4:.2f}e-4, "
          f"final {wander(des) * 1e4:.2f}e-4 | period {1 / fr[k]:.1f} s")


if __name__ == "__main__":
  main()
