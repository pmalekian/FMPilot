#!/usr/bin/env python3
"""Which Jetlink big model drove, and when it changed: the Jetlink link's own record of a drive (fmJetlinkStatus), the
big-model frames and their timing, and optionally the Mac app's server.log lined up with it.

usage: jetlink.py rlog.zst [rlog.zst ...] [--each] [--mac-log server.log] [--mac-tz America/Chicago]
  --mac-log  the Jetlink Mac app's ~/Library/Logs/Jetlink/server.log (optional: everything else here comes from the rlog).
             Lines from two minutes before the drive to its end are shown on the drive's time scale. The Mac writes local
             time without a zone: --mac-tz names the Mac's zone when the PC running this is elsewhere (on Windows,
             `pip install tzdata` first).
  Logs from before Nov 19, 2026 have no fmJetlinkStatus: only the big-frame counts are shown for them.
  Note the settings an rlog records at its top (initData) are a copy from when the drive started; they can't show a model
  picked in the Jetlink app afterwards, so the model named here is the one that actually drove.
"""
import numpy as np
import maclog
from fmlog import big_frames, big_model_label, jetlink_status, load_drives, log_args, params

KEEP = {"initData", "carState", "fmJetlinkStatus", "modelV2", "clocks", "logMessage"}
DETAIL = ("serverPickName", "slotName", "builtName", "builtReady", "linkKind", "linkSpeed", "hostPresent", "device", "backend")


def driving_runs(st, T):
  """[(start t, end t, bigDriving, label)] of unbroken stretches of one driving model, from the status messages
  (T: logMonoTime ns -> drive seconds)."""
  runs = []
  for mono, d in st:
    key = (d["bigDriving"], big_model_label(d) if d["bigDriving"] else d["drivingName"] or "small model")
    t = T(mono)
    if runs and tuple(runs[-1][2:]) == key:
      runs[-1][1] = t
    else:
      runs.append([t, t, *key])
  return runs


def changes(st, T):
  """[(t, text)] of what changed in the link between one status message and the next, other than the driving model."""
  out, prev = [], None
  for mono, d in st:
    if prev is not None:
      for f in DETAIL:
        if d[f] != prev[f]:
          out.append((T(mono), f"{f}: {prev[f]!r} -> {d[f]!r}"))
    prev = d
  return out


def frame_ms(evs, kind):
  return np.array([getattr(e, kind).modelExecutionTime for e in evs if e.which() == kind and getattr(e, kind).big]) * 1000


def comma_loaded_walls(evs, off):
  """Wall times the comma logged loading the Mac's engine (for the clock check against the Mac's hello lines)."""
  out = []
  for e in evs:
    if e.which() == "logMessage" and maclog.COMMA_LOADED.search(str(e.logMessage)):
      out.append(e.logMonoTime / 1e9 + off)
  return out


def main(drive, mac_lines=None, tz=None):
  evs = drive.evs
  def Tn(mono):
    return (mono - drive.t0) / 1e9

  st = jetlink_status(evs)
  n_big, n = big_frames(evs)
  bundle = params(evs, {"ModelManager_ActiveBundle"}).get("ModelManager_ActiveBundle", "?")[:60]
  print(f"{drive.label()}: {n_big}/{n} model frames from a big model" if n else f"{drive.label()}: no model frames read")
  print(f"  model per the log's start-of-drive settings (stale for a Jetlink pick made later): {bundle}")
  if not st:
    print("  no fmJetlinkStatus in this log (a build from before Nov 19, 2026, or the Jetlink link was off)")
  else:
    print("  driving model timeline:")
    for a, b, big, label in driving_runs(st, Tn):
      print(f"    {drive.where(a).strip()} .. {drive.where(b).strip()}  {'BIG   ' if big else 'small '} {label}")
    for t, text in changes(st, Tn):
      print(f"    {drive.where(t)} {text}")
    last = st[-1][1]
    print(f"  at the end: state {last['state']!r}, handovers {last['handovers']}, drops {last['drops']}, lags {last['lags']}, held {last['held']}")
  ms = frame_ms(evs, "modelV2") if any(e.which() == "modelV2" for e in evs) else frame_ms(evs, "drivingModelData")
  if len(ms):
    print(f"  big-model modelExecutionTime: median {np.median(ms):.1f} ms, p90 {np.percentile(ms, 90):.1f} ms ({len(ms)} frames)")
  if mac_lines is not None:
    off = maclog.wall_offset(evs)
    if off is None:
      print("  --mac-log: this log has no clocks messages, so the Mac log can't be placed on the drive")
      return
    start, end = drive.t0 / 1e9 + off, evs[-1].logMonoTime / 1e9 + off
    sk = maclog.skew(comma_loaded_walls(evs, off), mac_lines)
    if sk is None:
      print("  clock check: no pair of comma 'loaded' and Mac 'hello' lines to compare")
    elif abs(sk) > maclog.SKEW_WARN_S:
      print(f"  WARNING Mac clock reads {sk:+.1f} s from the comma's (hello vs loaded): check --mac-tz; times below may be off")
    else:
      print(f"  clock check: Mac vs comma {sk:+.1f} s")
    print("  Mac log (seconds into the drive; negative = before it):")
    for t, lvl, msg in maclog.window(mac_lines, start, end):
      if maclog.keep_line(lvl, msg):
        print(f"    {t - start:+8.1f}s {lvl:7s} {msg[:200]}")


if __name__ == "__main__":
  args = log_args(__doc__, **{"--mac-log": {"help": "server.log from the Jetlink Mac app"},
                              "--mac-tz": {"help": "the Mac's timezone name, default this computer's"}}).parse_args()
  lines = maclog.load(args.mac_log, maclog.zone(args.mac_tz)) if args.mac_log else None
  for d in load_drives(args.rlog, args.each, KEEP):
    main(d, lines)
