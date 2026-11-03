#!/usr/bin/env python3
"""Side-to-side sway on straight roads while openpilot steers: how much the wheel, the model's request, lane centering's
addition and the final request wander (drift slower than 3 s removed), and the dominant period.

usage: sway.py rlog.zst [rlog.zst ...] [--min-mph 20] [--each]
  Segments of one drive are joined, so a straight stretch across a segment boundary counts as one.
"""
import numpy as np

from fmlog import MPH, load_drives, log_args, model_name, split_runs

MIN_RUN = 160  # model frames (20 Hz): 8 s
DETREND = 60   # model frames: 3 s


def wander(x, runs):
  """Standard deviation of x after removing a 3 s moving average, over every run (edges dropped)."""
  parts = []
  for r in runs:
    y = x[r] - np.convolve(x[r], np.ones(DETREND) / DETREND, "same")
    parts.append(y[DETREND // 2:-DETREND // 2])
  return float(np.std(np.concatenate(parts)))


def main(drive, min_mph):
  evs, T = drive.evs, drive.t
  cs = np.array([(T(e), e.carState.vEgo, e.carState.steeringAngleDeg, float(e.carState.steeringPressed)) for e in evs if e.which() == "carState"])
  ctl = np.array([(T(e), e.controlsState.desiredCurvature) for e in evs if e.which() == "controlsState"])
  cc = np.array([(T(e), float(e.carControl.latActive)) for e in evs if e.which() == "carControl"])
  mv = np.array([(T(e), e.modelV2.action.desiredCurvature) for e in evs if e.which() == "modelV2"])
  if not (len(cs) and len(ctl) and len(cc) and len(mv)):
    print(f"{drive.label()}: missing carState / controlsState / carControl / modelV2")
    return
  t = mv[:, 0]
  I = lambda arr, c: np.interp(t, arr[:, 0], arr[:, c])  # noqa: E731
  v, ang, pressed, lat, des = I(cs, 1), I(cs, 2), I(cs, 3) > 0.5, I(cc, 1) > 0.5, I(ctl, 1)
  straight = (v * MPH > min_mph) & lat & ~pressed & (np.abs(mv[:, 1]) < 0.002)
  runs = [r for r in split_runs(t, straight) if len(r) > MIN_RUN]
  if not runs:
    print(f"{drive.label()}: model {model_name(evs)}, no straight engaged stretch of 8 s or more above {min_mph:.0f} mph")
    return
  r = max(runs, key=len)
  y = ang[r] - np.convolve(ang[r], np.ones(DETREND) / DETREND, "same")
  F = np.abs(np.fft.rfft(y * np.hanning(len(y))))
  fr = np.fft.rfftfreq(len(y), 0.05)
  k = np.argmax(F * (fr > 0.15))
  n = sum(len(r) for r in runs) / 20
  print(f"{drive.label()}: model {model_name(evs)}, {n:.0f} s straight in {len(runs)} stretches at "
        f"{np.mean(v[np.concatenate(runs)]) * MPH:.0f} mph | wheel sway {wander(ang, runs):.2f} deg | model "
        f"{wander(mv[:, 1], runs) * 1e4:.2f}e-4, lane centering adds {wander(des - mv[:, 1], runs) * 1e4:.2f}e-4, "
        f"final {wander(des, runs) * 1e4:.2f}e-4 | period {1 / fr[k]:.1f} s (longest stretch, from {drive.where(t[r[0]]).strip()})")


if __name__ == "__main__":
  args = log_args(__doc__, **{"--min-mph": {"type": float, "default": 20.0}}).parse_args()
  for d in load_drives(args.rlog, args.each):
    main(d, args.min_mph)
