#!/usr/bin/env python3
"""Every sharp turn (radius under 40 m, moving) in one or more segments: what the model asked for, what the truck did,
how long torque sat at its limit, and how hard and which way the driver's hands pushed.

Hand torque is the column's torsion bar, which also twists when openpilot's torque turns a loosely held wheel. That
reaction always reads AGAINST openpilot's torque, so while openpilot steers the hands are split by openpilot's sent
torque: "with" it is certainly the driver helping; "against" it is the driver resisting OR the wheel's reaction, which
this sensor can't tell apart. While the driver steers alone, "into the turn" is the driver's own effort.

usage: turns.py rlog.zst [rlog.zst ...]
"""
import sys

import numpy as np

HANDS_NM = 0.5     # column torque that counts as pushing
SENT_MIN = 0.05    # openpilot torque (fraction of max) that counts as steering one way
AMBIGUOUS_NOTE = ("* against openpilot = you resisting OR the loose wheel's reaction to openpilot's torque; the column "
                  "sensor can't tell them apart")

from fmlog import MPH, model_name, read


def hand_split(hand, sent, lat_active, turn_lp):
  """Where the hands pushed during one turn. All torques left-positive (Nm for hands, fraction of max for sent);
  turn_lp is +1 for a left turn, -1 for a right one. Returns percentages of the openpilot-steered time pushing
  with / against openpilot's torque, and of the driver-steered time pushing into the turn (nan when there was none)."""
  hand, sent, lat_active = np.asarray(hand, float), np.asarray(sent, float), np.asarray(lat_active, bool)
  pushing = np.abs(hand) > HANDS_NM
  op = lat_active & (np.abs(sent) > SENT_MIN)
  pct = lambda m, n: float(np.sum(m) / np.sum(n) * 100) if np.any(n) else float("nan")  # noqa: E731
  return {
    "with_op": pct(op & pushing & (np.sign(hand) == np.sign(sent)), op),
    "against_op": pct(op & pushing & (np.sign(hand) != np.sign(sent)), op),
    "driver_into": pct(~lat_active & pushing & (np.sign(hand) == turn_lp), ~lat_active),
  }


def series(evs):
  t0 = next(e.logMonoTime for e in evs if e.which() == "carState")
  cs, ctl, cc, md, co = [], [], [], [], []
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
    elif w == "carOutput":
      co.append((t, e.carOutput.actuatorsOutput.torque))
  co = co or [(0.0, 0.0)]
  return [np.array(x, float) for x in (cs, ctl, cc, md, co)]


def main(path):
  evs = read(path)
  cs, ctl, cc, md, co = series(evs)
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
  if segs:
    print(f"  {AMBIGUOUS_NOTE}")
  for a, b in segs:
    tt = t[a:b]
    la = at(cc, tt, 1) > 0.5
    ang, hand = at(cs, tt, 2), at(cs, tt, 4)
    mk, ak, out = np.abs(at(md, tt, 1)), np.abs(curv[a:b]), np.abs(ctl[a:b, 3])
    turn_sign = np.sign(np.mean(ang[np.abs(ang) > 30])) if (np.abs(ang) > 30).any() else np.sign(np.mean(ang))
    h = hand_split(hand, at(co, tt, 1), la, turn_sign)
    print(f"  t={tt[0]:5.1f}s {np.mean(v[a:b]) * MPH:4.1f} mph {'R' if turn_sign < 0 else 'L'} | openpilot steering {la.mean() * 100:3.0f}% | "
          f"wheel max {np.abs(ang).max():4.0f} deg | tightest asked r {1 / max(mk.max(), 1e-4):5.1f} m, driven r {1 / max(ak.max(), 1e-4):5.1f} m | "
          f"torque at limit {np.mean(out >= 0.95) * 100:3.0f}% | hands {np.abs(hand).mean():.2f} Nm avg; openpilot steering: "
          f"with it {h['with_op']:3.0f}%, against it* {h['against_op']:3.0f}%; you steering: into the turn {h['driver_into']:3.0f}%")


if __name__ == "__main__":
  for p in sys.argv[1:]:
    main(p)
