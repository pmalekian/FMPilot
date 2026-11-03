#!/usr/bin/env python3
"""Every sharp turn (radius under 40 m, moving) in one or more segments: what the model asked for, what the truck did,
how long torque sat at its limit, and how hard and which way the driver's hands pushed.

usage: turns.py rlog.zst [rlog.zst ...]
"""
import sys

import numpy as np

from fmlog import MPH, model_name, read


def series(evs):
  t0 = next(e.logMonoTime for e in evs if e.which() == "carState")
  cs, ctl, cc, md = [], [], [], []
  for e in evs:
    t = (e.logMonoTime - t0) / 1e9
    w = e.which()
    if w == "carState":
      c = e.carState
      cs.append((t, c.vEgo, c.steeringAngleDeg, float(c.steeringPressed), c.steeringTorque))
    elif w == "controlsState":
      lat = e.controlsState.lateralControlState
      ts = getattr(lat, lat.which())
      ctl.append((t, e.controlsState.curvature, e.controlsState.desiredCurvature, getattr(ts, "output", np.nan)))
    elif w == "carControl":
      cc.append((t, float(e.carControl.latActive)))
    elif w == "modelV2":
      md.append((t, e.modelV2.action.desiredCurvature))
  return [np.array(x, float) for x in (cs, ctl, cc, md)]


def main(path):
  evs = read(path)
  cs, ctl, cc, md = series(evs)
  at = lambda arr, t, c: np.interp(t, arr[:, 0], arr[:, c])  # noqa: E731
  t, curv = ctl[:, 0], ctl[:, 1]
  v = at(cs, t, 1)
  turning = (np.abs(curv) > 1 / 40) & (v > 1.5)
  segs, s = [], None
  for i, on in enumerate(turning):
    if on and s is None:
      s = i
    if (not on or i == len(turning) - 1) and s is not None:
      if t[i] - t[s] > 1:
        segs.append((max(0, s - 60), min(len(t) - 1, i + 20)))
      s = None
  print(f"{path}: model {model_name(evs)}, {len(segs)} sharp turns")
  for a, b in segs:
    tt = t[a:b]
    la = at(cc, tt, 1) > 0.5
    ang, hand = at(cs, tt, 2), at(cs, tt, 4)
    mk, ak, out = np.abs(at(md, tt, 1)), np.abs(curv[a:b]), np.abs(ctl[a:b, 3])
    turn_sign = np.sign(np.mean(ang[np.abs(ang) > 30])) if (np.abs(ang) > 30).any() else np.sign(np.mean(ang))
    pushing = np.abs(hand) > 0.5
    into = np.mean(np.sign(hand[pushing]) == turn_sign) * 100 if pushing.any() else float("nan")
    print(f"  t={tt[0]:5.1f}s {np.mean(v[a:b]) * MPH:4.1f} mph {'R' if turn_sign < 0 else 'L'} | openpilot steering {la.mean() * 100:3.0f}% | "
          f"wheel max {np.abs(ang).max():4.0f} deg | tightest asked r {1 / max(mk.max(), 1e-4):5.1f} m, driven r {1 / max(ak.max(), 1e-4):5.1f} m | "
          f"torque at limit {np.mean(out >= 0.95) * 100:3.0f}% | hands {np.abs(hand).mean():.2f} Nm avg, {into:3.0f}% into the turn")


if __name__ == "__main__":
  for p in sys.argv[1:]:
    main(p)
