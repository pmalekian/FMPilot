#!/usr/bin/env python3
"""Nav Assist per blinker: which street the map picked, whether that is where the truck went, and what Map Turn Desire
did or would have done.

usage: nav.py rlog.zst [rlog.zst ...] [--map file.pmtiles] [--replay] [--each]
  Segments of one drive are joined; times read "--3 +12.4s".
  Logs recorded with Nav Assist on carry its fmNavTurn message; that is what is reported. Older logs (or --replay) are
  run through the same code off the device from the logged GPS, speed and yaw rate; that needs --map (or FM_NAV_MAP).
  "went into" always needs the map: without it the street the truck took can't be named.

Each blinker gets one line:
  street        the truck turned onto a street             pick ok / WRONG / none
  service road  it turned onto a parking or access road    (the service pick is shown; these never start a turn)
  off the map   it turned where the map has no road
  no turn       blinker without a turn (lane change, pulling over)
and what Map Turn Desire does with the logged speed, steering state, lane changes and these picks (the real class, run
at 5 Hz, blind spots not replayed): "would start" with the time before turn-in, for every log; "STARTED" when the log
says it did.
"""
import math
import os
from types import SimpleNamespace

import numpy as np

from fmlog import MPH, load_drives, log_args

from openpilot.sunnypilot.fm_nav.core import NavAssist
from openpilot.sunnypilot.fm_nav.roads import RoadMap
from openpilot.sunnypilot.selfdrive.controls.lib.fm_nav_turn_desire import FmNavTurnDesire

KEEP = {"gpsLocationExternal", "carState", "liveLocationKalman", "fmNavTurn", "modelDataV2SP", "carControl", "drivingModelData", "initData"}
TICK = 0.2
TURN_MIN_DEG, TURN_MAX_DEG, TURN_MAX_SPEED = 45.0, 150.0, 16.0
YAW_ON = 6.0  # deg/s


def _pick(turns):
  if not turns:
    return SimpleNamespace(valid=False, distance=0.0, angle=0.0, name="", kind="", count=0, nextGap=0.0)
  t = turns[0]
  return SimpleNamespace(valid=True, distance=t.distance, angle=t.angle, name=t.name, kind=t.kind, count=len(turns),
                         nextGap=turns[1].distance - t.distance if len(turns) > 1 else 0.0)


class _On:
  @staticmethod
  def get_bool(key):
    return True


def track(drive, road_map, replay):
  """Per-tick records (t, status, heading, lon, lat, left, right, leftService, rightService) and the car's signals."""
  T = drive.t
  nav = NavAssist(road_map)
  logged = [e for e in drive.evs if e.which() == "fmNavTurn"]
  use_logged = bool(logged) and not replay
  ticks, cs, yaw, armed = [], [], [], []
  next_tick = None
  desire, car, lat_active, lane_changing = FmNavTurnDesire(_On()), None, False, False

  def add(t, status, heading, picks):
    fix = nav.fix or (0, float("nan"), float("nan"), 0)
    would = False
    if car is not None:
      desire.update(car, lat_active, SimpleNamespace(status=status, left=picks[0], right=picks[1]), lane_changing)
      would = desire.active
    ticks.append((t, status, heading, fix[1], fix[2], *picks, would))
  for e in drive.evs:
    w, t = e.which(), T(e)
    if w == "liveLocationKalman":
      a = e.liveLocationKalman.angularVelocityCalibrated
      if a.valid and len(a.value) == 3:
        nav.on_pose(t, a.value[2])
        yaw.append((t, a.value[2]))
    elif w == "carState":
      c = e.carState
      nav.on_car(c.vEgo)
      cs.append((t, c.vEgo, c.leftBlinker, c.rightBlinker))
      car = SimpleNamespace(vEgo=c.vEgo, leftBlinker=c.leftBlinker, rightBlinker=c.rightBlinker, leftBlindspot=False, rightBlindspot=False)
    elif w == "carControl":
      lat_active = e.carControl.latActive
    elif w == "drivingModelData":
      lane_changing = str(e.drivingModelData.meta.laneChangeState) in ("laneChangeStarting", "laneChangeFinishing")
    elif w == "gpsLocationExternal":
      g = e.gpsLocationExternal
      if g.hasFix:
        nav.on_gps(t, g.longitude, g.latitude, g.speed, g.bearingDeg, g.horizontalAccuracy)
    elif w == "modelDataV2SP":
      m = e.modelDataV2SP
      armed.append((t, m.fmNavTurnArmed, m.fmNavTurnActive))
    elif w == "fmNavTurn" and use_logged:
      n = e.fmNavTurn
      add(t, str(n.status), n.heading, (n.left, n.right, n.leftService, n.rightService))
    if not use_logged and w in ("carState", "liveLocationKalman") and (next_tick is None or t >= next_tick):
      next_tick = t + TICK
      s = nav.tick(t)
      add(t, s.status, s.heading, (_pick(s.left.streets), _pick(s.right.streets), _pick(s.left.services), _pick(s.right.services)))
  return ticks, np.array(cs, dtype=float).reshape(-1, 4), np.array(yaw, dtype=float).reshape(-1, 2), armed, use_logged


def find_turns(yaw, cs):
  """Turns of 45-150 deg under 36 mph from the yaw rate: (turn-in time, end time, degrees + right)."""
  if len(yaw) < 20:
    return []
  t, rate = yaw[:, 0], np.degrees(yaw[:, 1])
  hd = np.cumsum(rate * np.r_[0, np.clip(np.diff(t), 0, 0.2)])
  v = np.interp(t, cs[:, 0], cs[:, 1])
  on = np.abs(np.convolve(rate, np.ones(9) / 9, "same")) > YAW_ON
  out, i, n = [], 0, len(t)
  while i < n:
    if on[i]:
      j = i
      while j < n and (on[j] or on[j:j + 10].any()):
        j += 1
      j = min(j, n - 1)
      dh = hd[j] - hd[i]
      if TURN_MIN_DEG < abs(dh) < TURN_MAX_DEG and v[i:j + 1].max() < TURN_MAX_SPEED:
        k = i + int(np.argmax(np.abs(hd[i:j + 1] - hd[i]) >= 0.1 * abs(dh)))
        out.append((t[k], t[j], dh, float(np.interp(t[(i + j) // 2], cs[:, 0], cs[:, 1]))))
      i = j + 1
    else:
      i += 1
  return out


def blinks(cs):
  out = []
  for side, col in ((-1, 2), (1, 3)):
    on = cs[:, col] > 0.5
    edges = list(np.flatnonzero(np.diff(on.astype(int))) + 1)
    if len(on) and on[0]:
      edges = [0] + edges
    if len(on) and on[-1]:
      edges = edges + [len(on) - 1]
    merged = []
    for a, z in zip(cs[edges[::2], 0], cs[edges[1::2], 0], strict=False):
      if merged and a - merged[-1][1] < 1.0:
        merged[-1][1] = z
      else:
        merged.append([a, z])
    out += [(a, z, side) for a, z in merged]
  return sorted(out)


def went_into(road_map, ticks, t):
  """(kind of place, road name) the truck is on at time t."""
  if road_map is None:
    return "?", ""
  i = min(range(len(ticks)), key=lambda k: abs(ticks[k][0] - t))
  _, _, heading, lon, lat = ticks[i][:5]
  if not math.isfinite(lon):
    return "no GPS", ""
  road_map.update(lon, lat)
  cur = road_map.current_road(road_map.local(lon, lat)[0], heading)
  if cur is None:
    return "off the map", ""
  r = cur[0]
  return ("service road" if road_map.service[r] else "street"), (road_map.names[r] or road_map.keys[r])


def main(drive, road_map, replay):
  ticks, cs, yaw, armed, used_logged = track(drive, road_map, replay)
  W = drive.where
  print(f"{drive.label()}: {'logged fmNavTurn' if used_logged else 'replayed from GPS, speed and yaw rate'}, {len(ticks)} samples")
  if not ticks or not len(cs):
    print("  nothing to report")
    return {}
  status = {}
  for k in ticks:
    status[k[1]] = status.get(k[1], 0) + 1
  print("  status: " + ", ".join(f"{s} {100 * n / len(ticks):.0f}%" for s, n in sorted(status.items(), key=lambda x: -x[1])))
  turns = find_turns(yaw, cs)
  tt = np.array([k[0] for k in ticks])
  tally = {}
  for a, z, side in blinks(cs):
    turn = next((x for x in turns if np.sign(x[2]) == side and a - 0.5 <= x[0] <= z + 3.0), None)
    stop = turn[0] if turn else z
    first = last = start = None
    for k in ticks[int(np.searchsorted(tt, a)):int(np.searchsorted(tt, stop, side="right"))]:
      pick, service = (k[5], k[7]) if side < 0 else (k[6], k[8])
      v = float(np.interp(k[0], cs[:, 0], cs[:, 1]))
      if k[1] == "ok" and pick.valid:
        first = first or (k[0], pick.name or pick.kind, pick.distance)
        last = (k[0], pick.name or pick.kind, pick.distance, service)
        if start is None and k[9]:
          start = (k[0], pick.name or pick.kind, v)
      elif k[1] == "ok" and service.valid and last is None:
        last = (k[0], None, 0.0, service)
    v_on = float(np.interp(a, cs[:, 0], cs[:, 1]))
    did = [x for x in armed if a <= x[0] <= stop + 0.5 and x[2]]
    desire = f"would start {'-' if not turn else f'{turn[0] - start[0]:.1f} s before turn-in'} toward {start[1]} at {start[2] * MPH:.0f} mph" if start else ""
    if did:
      desire += f"{'; ' if desire else ''}STARTED {W(did[0][0]).strip()}"
    if turn is None:
      kind, result = "no turn", (f"picked {first[1]}" if first else "no pick")
      extra = ""
    else:
      kind, name = went_into(road_map, ticks, turn[1] + 1.5)
      extra = f"{turn[2]:+4.0f} deg at {turn[3] * MPH:4.1f} mph into {name or kind}"
      if kind == "street":
        result = "none" if first is None else ("ok" if last[1] == name else f"WRONG ({last[1]})")
        if first:
          extra += f", first picked {turn[0] - first[0]:.1f} s before turn-in"
      elif kind == "service road":
        s = last[3] if last else None
        result = ("street pick " + first[1] if first else "no street pick") + (f", service pick {s.distance:.0f} m" if s is not None and s.valid else "")
      else:
        result = f"picked {first[1]}" if first else "no pick"
    tally.setdefault(kind, []).append(result.split(" (")[0].split(",")[0] + (" + would start" if start else ""))
    print(f"  {W(a)} {'R' if side > 0 else 'L'} {z - a:5.1f} s at {v_on * MPH:4.1f} mph  {kind:12s} {result:34s} {extra}{' | ' + desire if desire else ''}")
  for kind, results in tally.items():
    print(f"  {kind}: " + ", ".join(f"{r} x{results.count(r)}" for r in sorted(set(results))))
  return tally


if __name__ == "__main__":
  ap = log_args(__doc__, **{"--map": {"default": os.environ.get("FM_NAV_MAP"), "help": "road map (.pmtiles) for replay and for naming streets"},
                            "--replay": {"action": "store_true", "help": "ignore a logged fmNavTurn and replay from GPS"}})
  args = ap.parse_args()
  rm = RoadMap(args.map) if args.map else None
  for d in load_drives(args.rlog, each=args.each, keep=KEEP):
    main(d, rm, args.replay)
