#!/usr/bin/env python3
"""Every sharp turn (radius under 40 m, moving) in a drive: what the model asked for, what the truck did, how long
torque sat at its limit, and how hard and which way the driver's hands pushed.

usage: turns.py rlog.zst [rlog.zst ...] [--each]
  Segments of one drive are joined, so a turn across a segment boundary is one turn. Times read "--3 +12.4s":
  segment 3, 12.4 s in.

Hand torque is the column's torsion bar, which also twists when openpilot's torque turns a loosely held wheel. That
reaction always reads AGAINST openpilot's torque, so while openpilot steers the hands are split by openpilot's sent
torque: "with" it is certainly the driver helping; "against" it is the driver resisting OR the wheel's reaction, which
this sensor can't tell apart. While the driver steers alone, "into the turn" is the driver's own effort.
"""
import numpy as np

from fmlog import MPH, load_drives, log_args, model_name, model_requests, pct, split_runs

TURN_RADIUS_M = 40.0
HANDS_NM = 0.5     # column torque that counts as pushing
SENT_MIN = 0.05    # openpilot torque (fraction of max) that counts as steering one way
AMBIGUOUS_NOTE = ("* against openpilot = you resisting OR the loose wheel's reaction to openpilot's torque; the column "
                  "sensor can't tell them apart")


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


def series(drive):
  cs, ctl, cc, md, co = [], [], [], [], []
  for e in drive.evs:
    w = e.which()
    if w not in ("carState", "controlsState", "carControl", "carOutput"):
      continue
    t = drive.t(e)
    if w == "carState":
      c = e.carState
      cs.append((t, c.vEgo, c.steeringAngleDeg, float(c.steeringPressed), c.steeringTorque))
    elif w == "controlsState":
      lat = e.controlsState.lateralControlState
      ts = getattr(lat, lat.which())
      ctl.append((t, e.controlsState.curvature, e.controlsState.desiredCurvature, getattr(ts, "output", np.nan)))
    elif w == "carControl":
      cc.append((t, float(e.carControl.latActive)))
    else:
      co.append((t, e.carOutput.actuatorsOutput.torque))
  co = co or [(0.0, 0.0)]
  md = model_requests(drive)
  return [np.array(x, float) for x in (cs, ctl, cc)] + [md, np.array(co, float)]


def find_turns(t, curv, v):
  """(start, end) index windows of turns tighter than TURN_RADIUS_M lasting over 1 s, padded 0.6 s before and 0.2 s
  after, never across a gap in the log."""
  turning = (np.abs(curv) > 1 / TURN_RADIUS_M) & (v > 1.5)
  out = []
  for r in split_runs(t, turning):
    if t[r[-1]] - t[r[0]] > 1:
      a, b = r[0], r[-1] + 1
      while a > 0 and r[0] - a < 60 and t[a] - t[a - 1] < 1:
        a -= 1
      while b < len(t) - 1 and b - r[-1] < 20 and t[b + 1] - t[b] < 1:
        b += 1
      out.append((a, b))
  return out


def main(drive):
  cs, ctl, cc, md, co = series(drive)
  if not len(cs) or not len(ctl):
    print(f"{drive.label()}: no carState / controlsState")
    return
  at = lambda arr, t, c: np.interp(t, arr[:, 0], arr[:, c])  # noqa: E731
  t, curv = ctl[:, 0], ctl[:, 1]
  v = at(cs, t, 1)
  turns = find_turns(t, curv, v)
  print(f"{drive.label()}: model {model_name(drive.evs)}, {len(turns)} sharp turns")
  if turns:
    print(f"  {AMBIGUOUS_NOTE}")
  for a, b in turns:
    tt = t[a:b]
    la = at(cc, tt, 1) > 0.5
    ang, hand = at(cs, tt, 2), at(cs, tt, 4)
    mk, ak, out = np.abs(at(md, tt, 1)), np.abs(curv[a:b]), np.abs(ctl[a:b, 3])
    big = np.abs(ang) > 30
    turn_lp = np.sign(np.mean(ang[big])) if big.any() else np.sign(np.mean(ang))  # angle is left-positive
    h = hand_split(hand, at(co, tt, 1), la, turn_lp)
    print(f"  {drive.where(tt[0])} {np.mean(v[a:b]) * MPH:4.1f} mph {'L' if turn_lp > 0 else 'R'} | openpilot steering "
          f"{la.mean() * 100:3.0f}% | wheel max {np.abs(ang).max():4.0f} deg | tightest asked r {1 / max(mk.max(), 1e-4):5.1f} m, "
          f"driven r {1 / max(ak.max(), 1e-4):5.1f} m | torque at limit {np.mean(out >= 0.95) * 100:3.0f}% | hands "
          f"{np.abs(hand).mean():.2f} Nm avg; openpilot steering: with it {pct(h['with_op'])}, against it* {pct(h['against_op'])}; "
          f"you steering: into the turn {pct(h['driver_into'])}")


if __name__ == "__main__":
  args = log_args(__doc__).parse_args()
  for d in load_drives(args.rlog, args.each):
    main(d)
