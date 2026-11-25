#!/usr/bin/env python3
"""Settle After Turn > Early Right Push: one line per settle window from modelDataV2SP (fmTurnSettlePhase,
fmTurnSettlePush, fmTurnSettleLimit, fmTurnSettleGuard, fmTurnSettle), and how often each limit stopped a push.

  python settle.py <route>--3--rlog.zst <route>--4--rlog.zst

Logs from before the option (or with it off) have no windows: all phases read idle."""
from collections import Counter
from dataclasses import dataclass, field

import fmlog

GAP = 0.5  # s: idle frames shorter than this don't split a window


@dataclass
class Window:
  start: float
  end: float = 0.0
  phases: list = field(default_factory=list)  # in order, without repeats
  push: float = 0.0  # m, the most logged
  limit: str = 'none'  # the last limit logged
  limits: list = field(default_factory=list)  # every limit seen, in order, without repeats
  peak: float = 0.0  # 1/m, the most right
  least: float = 0.0  # 1/m, the most left (curb guard or centering)
  peak_lat: float = 0.0  # m/s^2, the most |curvature| * v^2
  guard: float = 0.0  # 1/m, the strongest curb push-back


def windows(events, t0):
  """Settle windows from a drive's events, as Window records (times in s from t0)."""
  out, cur, v, last_busy = [], None, 0.0, None
  for e in events:
    w = e.which()
    if w == 'carState':
      v = e.carState.vEgo
      continue
    if w != 'modelDataV2SP':
      continue
    t = (e.logMonoTime - t0) / 1e9
    m = e.modelDataV2SP
    phase = str(m.fmTurnSettlePhase)
    k = m.fmTurnSettle
    busy = phase != 'idle' or k != 0.0
    if not busy:
      if cur is not None and last_busy is not None and t - last_busy > GAP:
        out.append(cur)
        cur = None
      continue
    if cur is None:
      cur = Window(start=t)
    last_busy = cur.end = t
    if phase != 'idle' and (not cur.phases or cur.phases[-1] != phase):
      cur.phases.append(phase)
    limit = str(m.fmTurnSettleLimit)
    cur.limit = limit
    if limit != 'none' and (not cur.limits or cur.limits[-1] != limit):
      cur.limits.append(limit)
    cur.push = max(cur.push, m.fmTurnSettlePush)
    cur.peak, cur.least = max(cur.peak, k), min(cur.least, k)
    cur.peak_lat = max(cur.peak_lat, abs(k) * v * v)
    cur.guard = min(cur.guard, m.fmTurnSettleGuard)
  if cur is not None:
    out.append(cur)
  return out


def report(drive):
  ws = windows(drive.evs, drive.t0)
  print(drive.label())
  if not ws:
    print('  no Settle After Turn windows (option off, or a log from before it)')
    return ws
  for w in ws:
    print(f"  {drive.where(w.start)}  {w.end - w.start:4.1f}s  {' > '.join(w.phases) or '-':28s} push {w.push:4.2f} m  "
          f"stopped by {' > '.join(w.limits) or 'nothing'}  right {w.peak * 1e3:5.2f} left {-w.least * 1e3:5.2f} e-3/m  "
          f"lat {w.peak_lat:4.2f} m/s2" + (f"  curb push-back {-w.guard * 1e3:4.2f} e-3/m" if w.guard < 0 else ''))
  counts = Counter(limit for w in ws for limit in w.limits)
  print('  limits fired (windows): ' + (', '.join(f'{k} {n}' for k, n in counts.most_common()) or 'none'))
  return ws


def main():
  args = fmlog.log_args(__doc__).parse_args()
  for drive in fmlog.load_drives(args.rlog, each=args.each, keep={'carState', 'modelDataV2SP'}):
    report(drive)


if __name__ == '__main__':
  main()
