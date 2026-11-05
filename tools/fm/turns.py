#!/usr/bin/env python3
"""Every sharp turn (radius under 40 m, moving) in a drive: what the model asked for, what the truck did, how long
torque sat at its limit, and how hard and which way the driver's hands pushed.

Each turn also gets a timing line: when the model's planned path started curving, when its own steering request
(modelV2.action, before Turn Assist or lane centering touch it) did, and when the truck did. "request lags path" is
the late turn-in Turn Lead covers. The path's curve is read the way Turn Lead reads it: a circle from the truck through
the path point 1.3 s ahead (4 to 14 m). A curve "starts" at its last rise above 30% of its own peak (so a driver-tightened
turn doesn't make the request look late), searched from 4 s before the turn to 1 s after the truck's tightest point.

usage: turns.py rlog.zst [rlog.zst ...] [--each]
  Segments of one drive are joined, so a turn across a segment boundary is one turn. Times read "--3 +12.4s":
  segment 3, 12.4 s in.

Hand torque is the column's torsion bar, which also twists when openpilot's torque turns a loosely held wheel. That
reaction always reads AGAINST openpilot's torque, so while openpilot steers the hands are split by openpilot's sent
torque: "with" it is certainly the driver helping; "against" it is the driver resisting OR the wheel's reaction, which
this sensor can't tell apart. While the driver steers alone, "into the turn" is the driver's own effort.
"""
import numpy as np

from fmlog import MPH, load_drives, log_args, model_name, pct, split_runs

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


LEAD_S, LEAD_MIN_M, LEAD_MAX_M = 1.3, 4.0, 14.0  # Turn Lead's look-ahead (fm_turn_assist.py)
ONSET_FRAC = 0.3   # a curve has started at this fraction of its own peak
ONSET_LOOKBACK_S = 4.0


def path_curvature(x, y, v):
  """Curvature (right-positive, like modelV2's y) of the circle from the truck, tangent to its heading, through the
  planned path point LEAD_S ahead (by distance, clipped to 4..14 m). nan without a usable path."""
  x, y = np.asarray(x, float), np.asarray(y, float)
  d = float(np.clip(LEAD_S * v, LEAD_MIN_M, LEAD_MAX_M))
  if len(x) < 2 or x[-1] < d or np.any(np.diff(x) <= 0):
    return float("nan")
  py = float(np.interp(d, x, y))
  return 2 * py / (d * d + py * py)


def onset(t, sig, t_from, t_to):
  """When sig (signed toward the turn) last rose above ONSET_FRAC of its own peak in [t_from, t_to], before that
  peak. nan without a peak toward the turn, or if it was already above at t_from."""
  m = (t >= t_from) & (t <= t_to) & ~np.isnan(sig)
  tt, ss = t[m], sig[m]
  if not len(ss) or ss.max() <= 0:
    return float("nan")
  i = int(np.argmax(ss))
  below = np.flatnonzero(ss[:i + 1] < ONSET_FRAC * ss[i])
  return float(tt[below[-1] + 1]) if len(below) else float("nan")


def series(drive):
  cs, ctl, cc, md, co = [], [], [], [], []
  for e in drive.evs:
    w = e.which()
    if w not in ("carState", "controlsState", "carControl", "modelV2", "carOutput"):
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
    elif w == "modelV2":
      p = e.modelV2.position
      md.append((t, e.modelV2.action.desiredCurvature, list(p.x), list(p.y)))
    else:
      co.append((t, e.carOutput.actuatorsOutput.torque))
  co = co or [(0.0, 0.0)]
  paths = [(m[2], m[3]) for m in md]
  md = [(m[0], m[1]) for m in md] or [(0.0, 0.0)]
  return [np.array(x, float) for x in (cs, ctl, cc, md, co)] + [paths]


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
  cs, ctl, cc, md, co, paths = series(drive)
  if not len(cs) or not len(ctl):
    print(f"{drive.label()}: no carState / controlsState")
    return
  at = lambda arr, t, c: np.interp(t, arr[:, 0], arr[:, c])  # noqa: E731
  t, curv = ctl[:, 0], ctl[:, 1]
  v = at(cs, t, 1)
  turns = find_turns(t, curv, v)
  mt = md[:, 0]
  kpath = np.array([path_curvature(x, y, np.interp(tm, cs[:, 0], cs[:, 1])) for tm, (x, y) in zip(mt, paths, strict=True)]) \
    if paths else np.zeros(len(mt))
  gaps = []
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
    s = -turn_lp  # +1 right: curvatures are right-positive
    pk = a + int(np.argmax(s * curv[a:b]))
    t_from, t_to = t[a] - ONSET_LOOKBACK_S, t[pk] + 1.0
    o_path, o_req, o_drv = onset(mt, s * kpath, t_from, t_to), onset(mt, s * md[:, 1], t_from, t_to), onset(t, s * curv, t_from, t_to)
    f = lambda x: "  n/a" if np.isnan(x) else f"{x:+5.1f}"  # noqa: E731
    rel = lambda x: f(x - o_path) if not np.isnan(o_path) else "  n/a"  # noqa: E731
    print(f"      timing (s, 0 = path starts curving, {la.mean() * 100:.0f}% openpilot): request {rel(o_req)}, "
          f"truck {rel(o_drv)}")
    if la.mean() > 0.8 and not np.isnan(o_path) and not np.isnan(o_req):
      gaps.append(o_req - o_path)
  if gaps:
    print(f"  request lags path: median {np.median(gaps):+.1f} s over {len(gaps)} openpilot-steered turns "
          f"(range {min(gaps):+.1f} to {max(gaps):+.1f})")


if __name__ == "__main__":
  args = log_args(__doc__).parse_args()
  for d in load_drives(args.rlog, args.each):
    main(d)
