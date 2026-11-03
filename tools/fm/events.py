#!/usr/bin/env python3
"""Alerts, onroad events, fault flags and noteworthy log messages of one segment, plus FM Pilot settings.

usage: events.py rlog.zst
"""
import sys

from fmlog import log_msg, model_name, params, read, version

KEYS = ("FM_", "LaneCentering", "LaneGuardEnabled", "RivianOfftrack", "LaneTurn", "EnforceTorqueControl", "LagdToggle",
        "NeuralNetworkLateralControl", "Blendful", "AccelerationControl")
WORDS = ("fault", "fm custom model", "loading combined pkl", "rivian", "lockout", "refus", "rack")


def main(path):
  evs = read(path)
  t0 = next(e.logMonoTime for e in evs if e.which() == "carState")
  T = lambda e: (e.logMonoTime - t0) / 1e9  # noqa: E731
  print(f"{path}: version {version(evs)}, model {model_name(evs)}")
  for k, v in sorted(params(evs).items()):
    if k.startswith(KEYS):
      print(f"  setting {k} = {v[:40]}")
  prev = None
  for e in evs:
    if e.which() == "selfdriveState":
      s = e.selfdriveState
      key = (s.alertType, s.alertText1, s.alertText2)
      if key != prev and s.alertType:
        print(f"  {T(e):5.1f}s ALERT {s.alertType} | {s.alertText1} | {s.alertText2}")
      prev = key
  seen = {}
  for e in evs:
    if e.which() in ("onroadEvents", "onroadEventsSP"):
      lst = getattr(e, e.which())
      try:
        items = list(lst)
      except TypeError:
        items = list(lst.events)
      for x in items:
        seen.setdefault(str(x.name), []).append(T(e))
  for name, ts in sorted(seen.items()):
    print(f"  event {name}: {ts[0]:.1f}..{ts[-1]:.1f}s ({len(ts)}x)")
  for e in evs:
    if e.which() == "logMessage" and any(w in e.logMessage.lower() for w in WORDS):
      print(f"  {T(e):5.1f}s LOG {str(log_msg(e))[:240]}")


if __name__ == "__main__":
  for p in sys.argv[1:]:
    main(p)
