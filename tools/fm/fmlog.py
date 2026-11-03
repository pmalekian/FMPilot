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


def model_name(evs):
  """The driving model file modeld loaded (only logged in a drive's first segment), else the active bundle from initData."""
  for e in evs:
    if e.which() == "logMessage" and "loading combined pkl" in e.logMessage:
      return os.path.basename(str(log_msg(e)).split(":", 1)[-1].strip()).replace("_tinygrad.pkl", "")
  for e in evs:
    if e.which() == "initData":
      for p in e.initData.params.entries:
        if p.key == "ModelManager_ActiveBundle":
          try:
            return json.loads(bytes(p.value))["internalName"]
          except Exception:
            pass
  return "?"


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
