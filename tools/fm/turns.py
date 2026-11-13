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
Timing line: when the model (modelV2 action, or the raw model curve while openpilot isn't steering), openpilot (its
desired curvature, after Turn Lead / Hold / Boost) and the truck each first reached ONSET_FRAC of the turn's tightest
driven curve, searched back up to ONSET_LOOKBACK_S before the turn. Times are relative to the truck. The verdict:
  model late   the model asked less than MODEL_LATE_S before the truck turned in (or never asked that much): the model
               is late, and only Turn Lead / Hold (or you) can start the turn on time
  wheel late   the model asked more than WHEEL_LATE_S before the truck turned in: the request was there, the steering
               followed slowly (torque ramp, controller)
  on time      in between
These thresholds are working guesses, to sort turns, not limits of anything.
Release line (only for turns where you pressed the wheel into the turn while openpilot steered): how long you pressed
into it, the most and average torque openpilot sent against you meanwhile, and when you let go: what was still being
sent out of the turn, how far the wheel then went past center (within RELEASE_LOOK_S) and how long openpilot's torque
kept pushing it that way after it crossed center. With Release Guard on (logged as carControlSP.releaseGuard) also the
share of the pressing time it limited the push and the most it cut; "guard off" when it never ran in this log.

Assist line (logs with carControlSP.turnAssist, from Nov 13, 2026): which Turn Assist part acted in the turn, from up to
ONSET_LOOKBACK_S before it to RELEASE_LOOK_S after: where the turn came from (blinker, Carry-Through), each part that
added more than ASSIST_MIN 1/m (Turn Lead, Hold Adds Your Steering, Hold Path Source, Turn Hold's floor, Turn Boost,
Turn Nudge, a fading hold) with when it first and last acted and the most it added, Turn Hold's release (time, reason,
degrees turned), early corrections, what was asked at the truck's turn-in (model alone or which part added the most),
and "kept past the model": the time after the truck's tightest point that the request stayed more than ASSIST_MIN
tighter than the model.

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
ONSET_FRAC = 0.3        # of the turn's tightest driven curve
ONSET_LOOKBACK_S = 4.0  # s before the turn window to look for the start of the request
MODEL_LATE_S = 0.3
WHEEL_LATE_S = 0.8
PRESS_CURVATURE = 0.003  # 1/m: the truck already turning that way (as Release Guard and Hold Adds Your Steering)
RELEASE_LOOK_S = 2.0    # s after letting go to look for the wheel passing center
PRESS_MIN_S = 0.3       # s: shorter presses into the turn get no release line
PRESS_GAP_S = 0.2       # s: breaks shorter than this are one press
ASSIST_MIN = 0.003  # 1/m: an assist part adding less than this doesn't count as acting
ASSIST_PARTS = (("lead", "Turn Lead"), ("driverAdds", "your steering (Hold Adds)"), ("holdPath", "Hold Path Source"),
                ("holdFloor", "Turn Hold floor"), ("boost", "Turn Boost"), ("nudge", "Turn Nudge"),
                ("exitFade", "hold fading out"))
RELEASE_NAMES = {"driverAgainst": "you steered against the turn", "maxTurned": "75 deg limit", "maxTime": "8 s limit"}
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


def onset_time(t, toward, threshold, end):
  """Time the signal (toward the turn, positive) last rose to the threshold before index end, or nan if it was below
  it at end."""
  above = np.asarray(toward[:end + 1], float) >= threshold
  if not len(above) or not above[-1]:
    return float("nan")
  below = np.flatnonzero(~above)
  return float(t[below[-1] + 1] if below.size else t[0])


def turn_timing(t, model, openpilot, truck, peak):
  """Onsets of the model's request, openpilot's request and the truck's curve (all toward the turn, 1/m) at ONSET_FRAC
  of the truck's curve at index peak, relative to the truck, and a verdict ('model late', 'wheel late', 'on time')."""
  thr = ONSET_FRAC * truck[peak]
  truck_on = onset_time(t, truck, thr, peak)
  model_on, op_on = onset_time(t, model, thr, peak), onset_time(t, openpilot, thr, peak)
  model_lead = truck_on - model_on if np.isfinite(model_on) else -np.inf
  verdict = "model late" if model_lead < MODEL_LATE_S else ("wheel late" if model_lead > WHEEL_LATE_S else "on time")
  return {"truck": truck_on, "model": model_on - truck_on, "openpilot": op_on - truck_on, "verdict": verdict}


def series(drive):
  cs, ctl, cc, md, co, sp, rg = [], [], [], [], [], [], []
  for e in drive.evs:
    w = e.which()
    if w not in ("carState", "controlsState", "carControl", "carOutput", "carStateSP", "carControlSP"):
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
    elif w == "carControlSP":
      g = e.carControlSP.releaseGuard
      rg.append((t, float(g.active), abs(g.cut), g.limit))
    else:
      out = e.carOutput.actuatorsOutput
      co.append((t, out.torque, out.torqueOutputCan))
  co = co or [(0.0, 0.0, 0.0)]
  sp = sp or [(0.0, 0.0)]
  rg = rg or [(0.0, 0.0, 0.0, 0.0)]
  md = model_requests(drive)
  return [np.array(x, float) for x in (cs, ctl, cc)] + [md, np.array(co, float), np.array(sp, float), np.array(rg, float)]


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


def release_check(t, ang, hand, pressed, curv, sent, lat_active, turn_lp, guard_active=None, guard_cut=None,
                  guard_logged=False):
  """The release line's numbers, or None when you didn't press into the turn for PRESS_MIN_S while openpilot steered.
  Angle, hand and sent are left-positive, curv right-positive, turn_lp the turn's side (+1 left). The longest press into
  the turn is the one checked (not a short catch afterwards); 'released' is False when the log ends (or the window does)
  before you let go."""
  toward = -np.asarray(curv) * turn_lp  # the truck's curve toward the turn
  into_all = pressed & lat_active & (np.asarray(hand) * turn_lp > 0) & (toward > PRESS_CURVATURE)
  if not into_all.any():
    return None
  dt = float(np.median(np.diff(t))) if len(t) > 1 else 0.01
  runs = []
  for r in split_runs(t, into_all):
    if runs and t[r[0]] - t[runs[-1][-1]] < PRESS_GAP_S:
      runs[-1] = [runs[-1][0], int(r[-1])]
    else:
      runs.append([int(r[0]), int(r[-1])])
  run = max(runs, key=lambda r: t[r[-1]] - t[r[0]])
  into = np.zeros(len(t), bool)
  into[run[0]:run[-1] + 1] = into_all[run[0]:run[-1] + 1]
  if into.sum() * dt < PRESS_MIN_S:
    return None
  against = np.clip(-np.asarray(sent) * turn_lp, 0, None)  # sent out of the turn
  out = {"press_s": into.sum() * dt, "against_max": against[into].max(), "against_avg": against[into].mean(),
         "released": False}
  if guard_logged and guard_active is not None:
    out["guard_share"] = float(np.mean(guard_active[into] > 0.5))
    out["guard_cut"] = float(np.max(guard_cut[into])) if len(guard_cut[into]) else 0.0
  last = int(np.where(into)[0][-1])
  after = np.where(~pressed[last + 1:])[0]
  if not len(after):
    return out
  rel = last + 1 + int(after[0])
  end = int(np.searchsorted(t, t[rel] + RELEASE_LOOK_S))
  side = np.asarray(ang[rel:end]) * turn_lp  # positive while still on the turn's side
  out.update(released=True, t=t[rel], at_release=against[rel], crossed=False, past_deg=0.0, push_past_s=0.0)
  cross = np.where(side <= 0)[0]
  if len(cross):
    c = rel + int(cross[0])
    out.update(crossed=True, past_deg=float(max(-side.min(), 0.0)), at_center=against[c])
    pushing = against[c:end] > 0
    out["push_past_s"] = (int(np.argmin(pushing)) if not pushing.all() else len(pushing)) * dt
  return out


def assist_series(drive):
  """carControlSP.turnAssist as {field: array} with 't', or None when this log doesn't have it."""
  rows = []
  for e in drive.evs:
    if e.which() != "carControlSP":
      continue
    ta = e.carControlSP.turnAssist
    rows.append((drive.t(e), ta.sign, str(ta.source), ta.requestIn, ta.requestOut, ta.lead, ta.driverAdds, ta.holdPath,
                 ta.holdFloor, ta.boost, ta.nudge, ta.exitFade, ta.holdActive, str(ta.releaseReason), ta.turned,
                 ta.earlyCorrection))
  if not rows or not any(r[3] != 0.0 or r[1] != 0 for r in rows):
    return None
  names = ("t", "sign", "source", "requestIn", "requestOut", "lead", "driverAdds", "holdPath", "holdFloor", "boost", "nudge",
           "exitFade", "holdActive", "releaseReason", "turned", "earlyCorrection")
  cols = list(zip(*rows, strict=True))
  return {n: np.array(c, dtype=object if n in ("source", "releaseReason") else float) for n, c in zip(names, cols, strict=True)}


def assist_summary(ta, lo, hi, turn_sign, truck_in_t, peak_t):
  """What Turn Assist did between drive times lo and hi in a turn whose curvature sign is turn_sign (+1 right)."""
  m = (ta["t"] >= lo) & (ta["t"] <= hi)
  if not m.any():
    return None
  q = {k: v[m] for k, v in ta.items()}
  t = q["t"]
  mine = q["sign"] == turn_sign
  out = {"sources": sorted({s for s, x in zip(q["source"], mine, strict=True) if x and s != "none"}), "parts": []}
  for key, name in ASSIST_PARTS:
    act = mine & (q[key] > ASSIST_MIN)
    if act.any():
      out["parts"].append((name, t[act][0], t[act][-1], float(q[key][act].max())))
  carry = mine & (q["source"] == "carry")
  out["carry"] = (t[carry][0], t[carry][-1]) if carry.any() else None
  rel = np.flatnonzero(mine & (q["releaseReason"] != "none"))
  out["release"] = None
  if len(rel):
    i = rel[0]
    out["release"] = (t[i], str(q["releaseReason"][i]), float(q["turned"][i]))
  out["early_s"] = float(np.sum(mine & (q["earlyCorrection"] > 0.5)) * np.median(np.diff(t))) if len(t) > 1 else 0.0
  # at the truck's turn-in: model alone, or which part added the most
  i = int(np.clip(np.searchsorted(t, truck_in_t), 0, len(t) - 1))
  adds = [(float(q[k][i]), name) for k, name in ASSIST_PARTS if mine[i]]
  best = max(adds, default=(0.0, ""))
  out["at_turn_in"] = best[1] if best[0] > ASSIST_MIN else "model"
  # kept past the model: after the truck's tightest point, the request still tighter than the model
  toward_in, toward_out = turn_sign * q["requestIn"], turn_sign * q["requestOut"]
  past = mine & (t >= peak_t) & (toward_out - toward_in > ASSIST_MIN)
  out["kept_past_s"] = float(np.sum(past) * np.median(np.diff(t))) if len(t) > 1 and past.any() else 0.0
  return out


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
  cs, ctl, cc, md, co, sp, rg = series(drive)
  if not len(cs) or not len(ctl):
    print(f"{drive.label()}: no carState / controlsState")
    return
  at = lambda arr, t, c: np.interp(t, arr[:, 0], arr[:, c])  # noqa: E731
  t, curv = ctl[:, 0], ctl[:, 1]
  v = at(cs, t, 1)
  turns = find_turns(t, curv, v)
  guard_logged = bool(np.any(rg[:, 3] > 0))
  ta = assist_series(drive)
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
    onset = print_timing(drive, t, ctl, md, curv, a, b, turn_lp)
    if ta is not None and onset is not None:
      print_assist(drive, ta, t, a, b, turn_lp, *onset)
    asked, sent, cap, notes = torque_counts(tt, cs, cc, co, sp)
    q = torque_limits(asked, sent, cap, la)
    if np.isnan(q["at_limit"]):
      print("      torque: openpilot not steering")
      continue
    print(f"      torque: peak sent {q['peak']:3.0f} of {q['limit']:3.0f} limit | at limit {pct(q['at_limit'])}, climbing "
          f"{pct(q['climbing'])}, held {pct(q['held'])}, unwinding {pct(q['unwinding'])}"
          + (f"  ({'; '.join(notes)})" if notes else ""))
    print_release(drive, t, cs, cc, co, sp, rg, curv, a, b, turn_lp, guard_logged)


def print_release(drive, t, cs, cc, co, sp, rg, curv, a, b, turn_lp, guard_logged):
  """The release line for the turn window [a, b), carried on RELEASE_LOOK_S past it to see the wheel cross center."""
  end = min(int(np.searchsorted(t, t[b - 1] + RELEASE_LOOK_S)), len(t))
  while end > b and t[end - 1] - t[b - 1] > RELEASE_LOOK_S + 1:
    end -= 1
  tt = t[a:end]
  at = lambda arr, c: np.interp(tt, arr[:, 0], arr[:, c])  # noqa: E731
  _, sent, _, _ = torque_counts(tt, cs, cc, co, sp)
  r = release_check(tt, at(cs, 2), at(cs, 4), at(cs, 3) > 0.5, curv[a:end], sent, at(cc, 1) > 0.5, turn_lp,
                    at(rg, 1), at(rg, 2), guard_logged)
  if r is None:
    return
  guard = "guard off"
  if "guard_share" in r:
    guard = f"guard on {pct(r['guard_share'])} of it, cut up to {r['guard_cut']:.1f} m/s^2"
  line = (f"      release: you pressed into the turn {r['press_s']:.1f} s, openpilot against you max {r['against_max']:3.0f} "
          f"avg {r['against_avg']:3.0f} ({guard})")
  if not r["released"]:
    line += " | still pressing at the end"
  elif not r["crossed"]:
    line += f" | let go at {drive.where(r['t'])}: {r['at_release']:3.0f} still pushing out, wheel stayed on the turn's side"
  else:
    line += (f" | let go at {drive.where(r['t'])}: {r['at_release']:3.0f} still pushing out, {r['at_center']:3.0f} at center, "
             f"wheel {r['past_deg']:3.0f} deg past center, pushed past center {r['push_past_s']:.2f} s")
  print(line)


def print_assist(drive, ta, t, a, b, turn_lp, truck_in_t, peak_t):
  """The assist line for the turn window [a, b): ONSET_LOOKBACK_S before it to RELEASE_LOOK_S after."""
  turn_sign = -int(turn_lp)  # curvature is right-positive, turn_lp left-positive
  q = assist_summary(ta, t[a] - ONSET_LOOKBACK_S, t[b - 1] + RELEASE_LOOK_S, turn_sign, truck_in_t, peak_t)
  if q is None:
    return
  if not q["sources"] and not q["parts"]:
    print("      assist: none (no turn desire)")
    return
  src = " + ".join({"desire": "blinker", "carry": "Carry-Through"}.get(x, x) for x in q["sources"]) or "none"
  parts = "; ".join(f"{name} {drive.where(t0)}..{t1 - t0:+.1f} s max {mx:.3f}" for name, t0, t1, mx in q["parts"]) or "nothing added"
  line = f"      assist: from {src} | {parts} | at turn-in: {q['at_turn_in']}"
  if q["carry"]:
    line += f" | carry {drive.where(q['carry'][0])}..{q['carry'][1] - q['carry'][0]:+.1f} s"
  if q["release"]:
    rt, why, deg = q["release"]
    line += f" | hold released {drive.where(rt)} ({RELEASE_NAMES.get(why, why)}, {deg:.0f} deg turned)"
  if q["early_s"] > 0:
    line += f" | early correction {q['early_s']:.1f} s"
  line += f" | kept past the model {q['kept_past_s']:.1f} s"
  print(line)


def print_timing(drive, t, ctl, md, curv, a, b, turn_lp):
  """The timing line for the turn window [a, b), looking back up to ONSET_LOOKBACK_S without crossing a gap in the log.
  Returns (truck turn-in time, truck's tightest time), or None."""
  start = a
  while start > 0 and t[a] - t[start - 1] <= ONSET_LOOKBACK_S and t[start] - t[start - 1] < 1:
    start -= 1
  toward = lambda x: -turn_lp * np.asarray(x, float)  # noqa: E731  curvature is right-positive, turn_lp left-positive
  tw = t[start:b]
  truck = toward(curv[start:b])
  model = toward(np.interp(tw, md[:, 0], md[:, 1])) if len(md) else np.zeros_like(tw)
  op = toward(ctl[start:b, 2])
  peak = int(np.argmax(truck))
  if truck[peak] <= 0:
    return None
  q = turn_timing(tw, model, op, truck, peak)
  rel = lambda x: "never" if np.isnan(x) else f"{x:+.1f} s"  # noqa: E731
  print(f"      timing: truck turned in at {drive.where(q['truck'])} | model asked {rel(q['model'])}, openpilot asked "
        f"{rel(q['openpilot'])} (before is -) | {q['verdict']}")
  return q["truck"], tw[peak]


if __name__ == "__main__":
  args = log_args(__doc__).parse_args()
  for d in load_drives(args.rlog, args.each):
    main(d)
