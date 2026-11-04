#!/usr/bin/env python3
"""Alerts, onroad events, fault flags and noteworthy log messages of a drive, plus FM Pilot settings.

usage: events.py rlog.zst [rlog.zst ...] [--each]
  Segments of one drive are joined; times read "--3 +12.4s" (segment 3, 12.4 s in).
"""
from fmlog import load_drives, log_args, log_msg, model_name, params, version

KEYS = ("FM_", "LaneCentering", "LaneGuardEnabled", "RivianOfftrack", "LaneTurn", "EnforceTorqueControl", "LagdToggle",
        "NeuralNetworkLateralControl", "Blendful", "AccelerationControl")
WORDS = ("fault", "fm custom model", "loading combined pkl", "rivian", "lockout", "refus", "rack")


def main(drive):
  evs, T, W = drive.evs, drive.t, lambda e: drive.where(drive.t(e))
  print(f"{drive.label()}: version {version(evs)}, model {model_name(evs)}")
  for k, v in sorted(params(evs).items()):
    if k.startswith(KEYS):
      print(f"  setting {k} = {v[:40]}")
  prev = None
  for e in evs:
    if e.which() == "selfdriveState":
      s = e.selfdriveState
      key = (s.alertType, s.alertText1, s.alertText2)
      if key != prev and s.alertType:
        print(f"  {W(e)} ALERT {s.alertType} | {s.alertText1} | {s.alertText2}")
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
    print(f"  event {name}: {drive.where(ts[0]).strip()} .. {drive.where(ts[-1]).strip()} ({len(ts)}x)")
  for e in evs:
    if e.which() == "logMessage" and any(w in e.logMessage.lower() for w in WORDS):
      print(f"  {W(e)} LOG {str(log_msg(e))[:240]}")


if __name__ == "__main__":
  args = log_args(__doc__).parse_args()
  # only what this report reads: keeps an hour of logs in memory
  keep = {"initData", "carState", "selfdriveState", "onroadEvents", "onroadEventsSP", "logMessage"}
  for d in load_drives(args.rlog, args.each, keep):
    main(d)
