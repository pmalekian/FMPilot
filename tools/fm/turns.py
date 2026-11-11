#!/usr/bin/env python3
"""Every sharp turn (radius under 40 m, moving) in a drive: what the model asked for, what the truck did, where torque
was held back (at the limit, or still climbing / unwinding at the ramp rate), and how hard and which way the driver's
hands pushed.

usage: turns.py rlog.zst [rlog.zst ...] [--each]
  Segments of one drive are joined, so a turn across a segment boundary is one turn. Times read "--3 +12.4s":
  segment 3, 12.4 s in.

Torque line (counts, left-positive; shares are of the time openpilot was steering in the turn):
  peak sent / limit  most torque sent in the turn, and the limit in force at that moment
  at limit           sent torque within LIMIT_MARGIN of the limit while openpilot asked for at least that much
  climbing           asked for more than was sent, torque still rising: the ramp (3 or 5 per message) is the limit
  held               asked for more than was sent and torque not rising: something else holds it back (hands on the
                     wheel, the driver-torque limiter)
  unwinding          more torque is still being sent than asked for, or the opposite way: the same ramp, on the way down
The limit is the logged `carStateSP.fmSteerCap` frame by frame (so Max Torque, the 95% cap past 90 degrees and the
speed curve all count). Logs from before it existed fall back to the base speed curve, and logs without
`carOutput.torqueOutputCan` use the logged torque times the same curve.

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
LIMIT_MARGIN = 5   # counts: sent torque this close to the limit is at it
RAMP_GAP = 20      # counts: asked and sent further apart than this means the ramp (or something else) holds it back
# Rivian base speed curve (m/s -> counts), the scale of carControl.actuators.torque. Mirrors
# CarControllerParams.STEER_MAX_LOOKUP in opendbc/car/rivian/values.py; a test keeps the two equal.
STEER_MAX_LOOKUP = ([9, 13, 25, 27], [385, 350, 295, 275])
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


def torque_limits(asked, sent, limit, lat_active):
  """Where openpilot's torque was held back during one turn. asked, sent and limit are in counts, left-positive for
  asked and sent, one value per frame at a steady rate; limit is the most that could be sent at that moment.
  Returns percentages of the openpilot-steered frames (nan when openpilot never steered) for the torque sitting at the
  limit, still climbing toward what was asked, held below it (not rising), and unwinding (still more sent than asked
  for, or the opposite way), plus the peak sent (absolute) and the limit in force at that peak."""
  asked, sent, limit = (np.asarray(x, float) for x in (asked, sent, limit))
  la = np.asarray(lat_active, bool)
  if not la.any():
    return {"at_limit": np.nan, "climbing": np.nan, "held": np.nan, "unwinding": np.nan, "peak": 0.0, "limit": 0.0}
  a, s = np.abs(asked), np.abs(sent)
  same = asked * sent >= 0
  rising = np.diff(s, prepend=s[0]) > 0
  if len(s) > 1:
    rising[0] = rising[1]  # the first frame has nothing before it to compare with
  at_limit = la & same & (s >= limit - LIMIT_MARGIN) & (a >= limit - LIMIT_MARGIN)
  below = la & same & ~at_limit & (a - s > RAMP_GAP)
  unwinding = la & ~at_limit & ((~same & (s > RAMP_GAP)) | (same & (s - a > RAMP_GAP)))
  share = lambda m: float(m.sum() / la.sum() * 100)  # noqa: E731
  pk = int(np.argmax(np.where(la, s, -1.0)))
  return {"at_limit": share(at_limit), "climbing": share(below & rising), "held": share(below & ~rising),
          "unwinding": share(unwinding), "peak": float(s[pk]), "limit": float(limit[pk])}


def series(drive):
  cs, ctl, cc, md, co, sp = [], [], [], [], [], []
  for e in drive.evs:
    w = e.which()
    if w not in ("carState", "controlsState", "carControl", "carOutput", "carStateSP"):
      continue
    t = drive.t(e)
    if w == "carState":
      c = e.carState
      cs.append((t, c.vEgo, c.steeringAngleDeg, float(c.steeringPressed), c.steeringTorque, c.vEgoRaw))
    elif w == "controlsState":
      lat = e.controlsState.lateralControlState
      ts = getattr(lat, lat.which())
      ctl.append((t, e.controlsState.curvature, e.controlsState.desiredCurvature, getattr(ts, "output", np.nan)))
    elif w == "carControl":
      cc.append((t, float(e.carControl.latActive), e.carControl.actuators.torque))
    elif w == "carStateSP":
      sp.append((t, e.carStateSP.fmSteerCap))
    else:
      out = e.carOutput.actuatorsOutput
      co.append((t, out.torque, out.torqueOutputCan))
  co = co or [(0.0, 0.0, 0.0)]
  sp = sp or [(0.0, 0.0)]
  md = model_requests(drive)
  return [np.array(x, float) for x in (cs, ctl, cc)] + [md, np.array(co, float), np.array(sp, float)]


def torque_counts(tt, cs, cc, co, sp):
  """(asked, sent, limit) in counts over the times tt, and a note when something had to be worked out from the speed
  curve because the log lacks it (logs from before carStateSP.fmSteerCap / carOutput.torqueOutputCan existed)."""
  at = lambda arr, c: np.interp(tt, arr[:, 0], arr[:, c])  # noqa: E731
  scale = np.interp(at(cs, 5), *STEER_MAX_LOOKUP)
  asked = at(cc, 2) * scale
  can = at(co, 2)
  sent = can if np.any(can != 0) else at(co, 1) * scale
  cap = at(sp, 1)
  notes = []
  if not np.any(cap > 0):
    cap = scale
    notes.append("limit from the speed curve")
  if not np.any(can != 0):
    notes.append("sent from the logged torque")
  return asked, sent, cap, notes


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
  cs, ctl, cc, md, co, sp = series(drive)
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
    mk, ak = np.abs(at(md, tt, 1)), np.abs(curv[a:b])
    big = np.abs(ang) > 30
    turn_lp = np.sign(np.mean(ang[big])) if big.any() else np.sign(np.mean(ang))  # angle is left-positive
    h = hand_split(hand, at(co, tt, 1), la, turn_lp)
    print(f"  {drive.where(tt[0])} {np.mean(v[a:b]) * MPH:4.1f} mph {'L' if turn_lp > 0 else 'R'} | openpilot steering "
          f"{la.mean() * 100:3.0f}% | wheel max {np.abs(ang).max():4.0f} deg | tightest asked r {1 / max(mk.max(), 1e-4):5.1f} m, "
          f"driven r {1 / max(ak.max(), 1e-4):5.1f} m | hands "
          f"{np.abs(hand).mean():.2f} Nm avg; openpilot steering: with it {pct(h['with_op'])}, against it* {pct(h['against_op'])}; "
          f"you steering: into the turn {pct(h['driver_into'])}")
    asked, sent, cap, notes = torque_counts(tt, cs, cc, co, sp)
    q = torque_limits(asked, sent, cap, la)
    if np.isnan(q["at_limit"]):
      print("      torque: openpilot not steering")
      continue
    print(f"      torque: peak sent {q['peak']:3.0f} of {q['limit']:3.0f} limit | at limit {pct(q['at_limit'])}, climbing "
          f"{pct(q['climbing'])}, held {pct(q['held'])}, unwinding {pct(q['unwinding'])}"
          + (f"  ({'; '.join(notes)})" if notes else ""))


if __name__ == "__main__":
  args = log_args(__doc__).parse_args()
  for d in load_drives(args.rlog, args.each):
    main(d)
