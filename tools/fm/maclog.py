"""The Jetlink Mac app's own log (~/Library/Logs/Jetlink/server.log, or the app's Logs view saved as text), lined up with
an rlog by the wall clock.

The Mac log has lines the comma never sees: the Mac's engine builds and unloads, "picked here" (a pick made in the app),
the session hand-offs and why a link closed. Its lines carry the Mac's local time with no zone; the rlog carries the
comma's wall clock (the `clocks` message), so the two are put on one clock here. The comma's own clock comes from GPS
or the network and the Mac's from the network, so expect them to agree to about a second: jetlink.py checks that against
the one event both ends log (the comma's "loaded" line and the Mac's "hello from modeld") and says when they don't.
"""
import re
import statistics
from datetime import datetime

LINE = re.compile(r"^(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}),(\d{3}) +([A-Z]+) +(.*)$")
HELLO = re.compile(r"hello from modeld/")
COMMA_LOADED = re.compile(r"jetlink: .*, engine .*, loaded ")
SKEW_WARN_S = 5.0


def zone(name):
  """A tzinfo for the Mac's zone name (America/Chicago; needs `pip install tzdata` on Windows), or None for this
  computer's own zone, which is right when the Mac and the PC that runs this are in the same place."""
  if not name:
    return None
  from zoneinfo import ZoneInfo
  return ZoneInfo(name)


def parse(text, tz=None):
  """[(epoch seconds, level, message)] in file order. A line without a timestamp (a traceback, a wrapped message)
  belongs to the line before it. tz: the Mac's tzinfo, or None for this computer's."""
  out = []
  for raw in text.splitlines():
    m = LINE.match(raw)
    if m:
      naive = datetime.strptime(m.group(1), "%Y-%m-%d %H:%M:%S")
      stamp = naive.replace(tzinfo=tz) if tz is not None else naive.astimezone()
      out.append((stamp.timestamp() + int(m.group(2)) / 1000.0, m.group(3), m.group(4).strip()))
    elif out and raw.strip():
      epoch, level, msg = out[-1]
      out[-1] = (epoch, level, f"{msg} {raw.strip()}")
  return out


def load(path, tz=None):
  with open(path, encoding="utf-8", errors="replace") as f:
    return parse(f.read(), tz)


def wall_offset(evs):
  """Seconds to add to an rlog logMonoTime (in seconds) to get the comma's wall clock, from the `clocks` messages (the
  median, so a stray one can't move it); None when the log has none."""
  offs = [(e.clocks.wallTimeNanos - e.logMonoTime) / 1e9 for e in evs if e.which() == "clocks" and e.clocks.wallTimeNanos > 0]
  return statistics.median(offs) if offs else None


def stamp(epoch, tz=None):
  """'09:00:53.000' for an epoch, in tz (default: this computer's zone)."""
  when = datetime.fromtimestamp(epoch, tz) if tz is not None else datetime.fromtimestamp(epoch)
  return when.strftime("%H:%M:%S.%f")[:-3]


def skew(comma_loaded, mac_lines):
  """Seconds the Mac's clock reads ahead of the comma's (+) or behind (-), from the nearest hello on the Mac for each
  time the comma logged that it loaded the Mac's engine (both ends log it within a moment of each other). comma_loaded:
  [wall epoch seconds]. None when there is nothing to pair."""
  hellos = [t for t, _, msg in mac_lines if HELLO.search(msg)]
  if not hellos or not comma_loaded:
    return None
  diffs = [min(hellos, key=lambda h: abs(h - c)) - c for c in comma_loaded]
  return statistics.median(diffs)


def window(lines, start, end, before=120.0, after=10.0):
  """The Mac lines from `before` seconds ahead of the drive (its picks and builds happen first) to just after it."""
  return [(t, lvl, msg) for t, lvl, msg in lines if start - before <= t <= end + after]


def keep_line(level, msg):
  """The lines worth reading: the Mac's decisions and its link; not the per-frame session noise."""
  if level != "INFO":
    return True
  return any(w in msg for w in ("picked here", "engine ", "hello from", "handed from", "client connected", "client disconnected",
                                "link closed", "gadget", "the comma's pick", "served", "benchmark"))


__all__ = ["COMMA_LOADED", "SKEW_WARN_S", "keep_line", "load", "parse", "skew", "stamp", "wall_offset", "window", "zone"]
