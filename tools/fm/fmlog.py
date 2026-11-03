"""Shared helpers for the FM Pilot log tools: read an rlog/qlog (.zst or plain) with the fork's own cereal schema."""
import json
import os
import sys

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
for p in (REPO, os.path.join(REPO, "opendbc_repo")):
  if p not in sys.path:
    sys.path.insert(0, p)

from openpilot.cereal import log

MPH = 2.23694


def read(path):
  with open(path, "rb") as f:
    data = f.read()
  if data[:4] == b"\x28\xb5\x2f\xfd":
    import zstandard
    data = zstandard.ZstdDecompressor().stream_reader(data).read()
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
