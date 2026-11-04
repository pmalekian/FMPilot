"""Shared helpers for the FM Pilot log tools: read an rlog/qlog (.zst or plain) with the fork's own cereal schema."""
import argparse
import glob
import json
import os
import re
import sys

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
for p in (REPO, os.path.join(REPO, "opendbc_repo")):
  if p not in sys.path:
    sys.path.insert(0, p)

import numpy as np

from openpilot.cereal import log

MPH = 2.23694


# route names: on the device 000001a3--c20ba54385, on Comma Connect 2026-11-03--09-12-33; then --<segment>
SEG_RE = re.compile(r"([0-9a-f]{8}--[0-9a-f]{10}|\d{4}-\d{2}-\d{2}--\d{2}-\d{2}-\d{2})--(\d+)(?![0-9])")
GAP_S = 2.0  # a jump in time longer than this is a missing piece of the drive, never bridged


def read(path):
  with open(path, "rb") as f:
    data = f.read()
  if data[:4] == b"\x28\xb5\x2f\xfd":
    import zstandard
    data = zstandard.ZstdDecompressor().stream_reader(data).read()
  elif data[:3] == b"BZh":
    import bz2
    data = bz2.decompress(data)
  return list(log.Event.read_multiple_bytes(data))


def log_msg(e):
  try:
    return json.loads(e.logMessage)["msg"]
  except Exception:
    return e.logMessage


def _bundle(raw):
  try:
    return json.loads(raw)
  except Exception:
    return None


def _pkl_name(path):
  """/data/.../fm_opm16d_rlm_splice_tinygrad.pkl -> fm_opm16d_rlm_splice (as modeld's fm_custom_label names it)."""
  name = os.path.basename(str(path).strip())
  for suffix in (".pkl", "_tinygrad"):
    if name.endswith(suffix):
      name = name[:-len(suffix)]
  return name


def model_info(evs):
  """(name, confirmed) of the driving model this drive ran.

  Every segment carries initData, a copy of the params taken when the drive started (the same in every segment), so
  the model is worked out from those the way modeld picks it: the selected bundle, replaced by the FM custom file when
  FM_CustomModelEnabled is on and FM_CustomModelBaseRef is the selected bundle. modeld's own "loading combined pkl"
  line is only in the first segment (--0); when it's there it confirms (or overrides) the answer."""
  p = raw_params(evs)
  bundle = _bundle(p.get("ModelManager_ActiveBundle", b""))
  name = base = None
  if bundle:
    name = base = bundle.get("displayName") or bundle.get("internalName")
    path, base_ref = p.get("FM_CustomModelPath", b"").decode(errors="replace"), p.get("FM_CustomModelBaseRef", b"").decode(errors="replace")
    if path and base_ref and bundle.get("ref") == base_ref and p.get("FM_CustomModelEnabled") == b"1":
      name = _pkl_name(path)
  for e in evs:
    if e.which() == "logMessage" and "loading combined pkl" in e.logMessage:
      loaded = _pkl_name(str(log_msg(e)).split(":", 1)[-1])
      files = [_pkl_name(m.get("artifact", {}).get("fileName", "")) for m in (bundle or {}).get("models", [])]
      if loaded == name:
        return name, True
      return (base if base and loaded in files else loaded), True  # e.g. custom file missing: modeld fell back
  return (name or "?"), False


def model_name(evs):
  """The driving model, marked "(from settings)" when the segment has no load line to confirm it (any but --0)."""
  name, confirmed = model_info(evs)
  return name if confirmed or name == "?" else f"{name} (from settings)"


def raw_params(evs):
  for e in evs:
    if e.which() == "initData":
      return {p.key: bytes(p.value) for p in e.initData.params.entries}
  return {}


def params(evs, keys=None):
  for e in evs:
    if e.which() == "initData":
      return {p.key: bytes(p.value).decode(errors="replace") for p in e.initData.params.entries if keys is None or p.key in keys}
  return {}


def version(evs):
  for e in evs:
    if e.which() == "initData":
      return e.initData.gitCommit[:8]
  return "?"


def route_segment(path):
  """(route, segment number) from a path like .../000001a3--c20ba54385--3/rlog.zst, else (None, None)."""
  found = list(SEG_RE.finditer(str(path).replace("\\", "/")))
  m = found[-1] if found else None
  return (m.group(1), int(m.group(2))) if m else (None, None)


def expand(args):
  """Paths from the command line: expands * and ? (PowerShell passes them through) and folders (every rlog/qlog in them)."""
  out = []
  for a in args:
    if os.path.isdir(a):
      found = sorted(p for p in glob.glob(os.path.join(a, "**", "*"), recursive=True)
                     if os.path.isfile(p) and re.search(r"[rq]log(\.zst|\.bz2)?$", os.path.basename(p)))
    elif any(c in a for c in "*?["):
      found = sorted(glob.glob(a))
    else:
      found = [a] if os.path.isfile(a) else []
    if not found:
      sys.exit(f"no log file at {a}")
    out += [p for p in found if p not in out]
  return out


class Drive:
  """One or more segments of one drive, joined on one clock. t is seconds since the first carState."""

  def __init__(self, route, segs):
    self.route = route
    self.segs = segs  # [(segment number or None, path, events)], in drive order
    self.evs = [e for _, _, evs in segs for e in evs]
    self.t0 = next((e.logMonoTime for e in self.evs if e.which() == "carState"), self.evs[0].logMonoTime if self.evs else 0)
    # a segment's start = its first carState: initData carries the drive's start time in every segment
    self.starts = [((self._first(evs) - self.t0) / 1e9 if evs else 0.0) for _, _, evs in segs]
    self.ends = [((evs[-1].logMonoTime - self.t0) / 1e9 if evs else 0.0) for _, _, evs in segs]

  @staticmethod
  def _first(evs):
    return next((e.logMonoTime for e in evs if e.which() == "carState"),
                min((e.logMonoTime for e in evs if e.which() != "initData"), default=evs[0].logMonoTime))

  def t(self, e):
    return (e.logMonoTime - self.t0) / 1e9

  @property
  def nums(self):
    return [n for n, _, _ in self.segs]

  def label(self):
    if len(self.segs) == 1:
      return self.segs[0][1]
    nums = self.nums
    if None in nums:
      return f"{len(nums)} files from {self.route or 'one drive'}"
    missing = sorted(set(range(nums[0], nums[-1] + 1)) - set(nums))
    gap = f" (missing --{', --'.join(map(str, missing))})" if missing else ""
    return f"{self.route} --{nums[0]}..--{nums[-1]}{gap}"

  def where(self, t):
    """'--3 +12.4s' for a drive time (segment and seconds into it); the drive time when segments aren't known or t is
    in a gap between the files given."""
    i = max(0, sum(1 for s in self.starts if s <= t) - 1)
    n = self.segs[i][0]
    if n is None or t > self.ends[i] + 1.0:
      return f"{t:6.1f}s"
    return f"--{n} +{t - self.starts[i]:4.1f}s"


def load_drives(paths, each=False):
  """Read logs and join the segments of each drive. Files are grouped by the route in their path and put in segment
  order; files whose path names no route are taken as one drive in time order. each=True keeps every file separate."""
  loaded = [(*route_segment(p), p, read(p)) for p in expand(paths)]
  if each:
    return [Drive(r, [(n, p, evs)]) for r, n, p, evs in loaded]
  groups = {}
  for r, n, p, evs in loaded:
    groups.setdefault(r, []).append((n, p, evs))
  drives = []
  for r, segs in groups.items():
    segs.sort(key=lambda s: (s[0] if s[0] is not None else -1, s[2][0].logMonoTime if s[2] else 0))
    drives.append(Drive(r, segs))
  return drives


def split_runs(t, mask, gap=GAP_S):
  """Index arrays of consecutive True samples in mask, also broken wherever t jumps by more than gap seconds."""
  idx = np.flatnonzero(mask)
  if not idx.size:
    return []
  breaks = np.flatnonzero((np.diff(idx) > 1) | (np.diff(t[idx]) > gap)) + 1
  return np.split(idx, breaks)


def pct(x):
  """'42%', or 'n/a' when there was nothing to measure."""
  return "n/a" if np.isnan(x) else f"{x:3.0f}%"


def log_args(doc, **extra):
  """Common command line: one or more rlogs / folders / wildcards, joined per drive unless --each."""
  ap = argparse.ArgumentParser(description=doc, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("rlog", nargs="+", help="rlog.zst files, folders, or wildcards; segments of one drive are joined")
  ap.add_argument("--each", action="store_true", help="analyze every file on its own instead of joining a drive")
  for name, kw in extra.items():
    ap.add_argument(name, **kw)
  return ap
