"""
FM Pilot Jetlink Status: what the Jetlink link is doing and which model is driving, as plain data.

The rlog's initData copies the params once, when the drive starts, so with Jetlink it names whatever was picked
before the drive and can't show a model picked later or a swap mid-drive. This reads the live records Jetlink keeps on
the comma (/dev/shm/jetlink/*.json: the owner's status, the Mac's last hello, modeld's runtime record) and the comma's
own params, so the log can say which model drove at every moment.

Read only. It never imports Jetlink (a Jetlink of another API can't blank the log; a field it doesn't write reads as
"") and never writes a param or a record. Nothing here touches modeld, the swap rules, driver monitoring or the car.

No cereal or messaging import: statusd.py owns the process; this is what it publishes and logs, and the tests run it
without a device.
"""
import json
import math
import os
import time
from collections.abc import Callable
from pathlib import Path

ROOT = Path("/dev/shm/jetlink")
NO_RECORD = -1.0

# Jetlink's own tests (jetlink.openpilot.status.runtime): a big model drives while modeld's record is this recent and
# its last answer was within OUTPUT_FRESH of the record's time. Two writer periods (the writer runs at background
# priority, so a late write must not read as a hand-back).
RECORD_PERIOD = 1.0
ACTIVE_FRESH = 2 * RECORD_PERIOD
OUTPUT_FRESH = 0.25
# and a record older than this is no live modeld at all (gadget.HEARTBEAT_TIMEOUT, which status.runtime also applies):
# a modeld that froze or stopped writing reads as none rather than keeping its last state
HEARTBEAT_TIMEOUT = 3.0
STAND_IN = 16  # Jetlink writes a model's name as its hash's first 16 characters when the comma's index lists none

TEXT_LIMIT = 160  # a log message with a runaway string must not grow the rlog

# The fields statusd.py fills into the message, in the schema's order (custom.capnp FmJetlinkStatus).
TEXT = ("state", "drivingName", "drivingSha256", "drivingCheckpoint", "serverPickName", "serverPickSha256", "slotName",
        "builtName", "builtSha256", "progressStage", "progressMsg", "linkKind", "linkSpeed", "device", "backend")
BOOL = ("bigDriving", "builtReady", "hostPresent", "held")
COUNT = ("handovers", "drops", "lags")


def read_record(path: Path) -> dict | None:
  """A JSON object Jetlink wrote, or None: missing, half-written or not an object."""
  try:
    record = json.loads(Path(path).read_text(encoding="utf-8"))
  except (OSError, ValueError):
    return None
  return record if isinstance(record, dict) else None


def as_dict(value) -> dict:
  """A param as a dict, whether the Params wrapper already parsed it or handed back its JSON text."""
  if isinstance(value, (bytes, bytearray)):
    value = bytes(value).decode(errors="replace")
  if isinstance(value, str):
    try:
      value = json.loads(value)
    except ValueError:
      return {}
  return value if isinstance(value, dict) else {}


def text(value) -> str:
  return value[:TEXT_LIMIT] if isinstance(value, str) else ""


def count(value) -> int:
  if isinstance(value, bool) or not isinstance(value, (int, float)):
    return 0
  if isinstance(value, float) and not math.isfinite(value):
    return 0
  return int(min(max(value, 0), 65535))


def real_name(name: str, sha: str) -> str:
  """name, or "" when it is Jetlink's stand-in for one (the hash's start, Models.name_for)."""
  return "" if not name or (sha and name == sha[:STAND_IN]) else name


def pid_alive(pid) -> bool:
  if not isinstance(pid, int) or isinstance(pid, bool) or pid <= 0:
    return False
  try:
    os.kill(pid, 0)
  except ProcessLookupError:
    return False
  except OSError:  # EPERM and the like: it exists
    return True
  return True


class Names:
  """Model names by the weights' hash, from the comma's copy of Jetlink's model index (read again only when the file
  changes). Models with identical weights share a hash, so a hash can have several names; they are joined."""

  def __init__(self, index_path: Path | None):
    self.path = index_path
    self._seen = None
    self._by_sha: dict[str, str] = {}

  def of(self, sha: str) -> str:
    if not sha or self.path is None:
      return ""
    try:
      st = Path(self.path).stat()
      seen = (st.st_mtime_ns, st.st_size)
    except OSError:
      return ""
    if seen != self._seen:
      self._seen = seen
      self._by_sha = {}
      index = read_record(self.path) or {}
      for m in index.get("models") if isinstance(index.get("models"), list) else []:
        if isinstance(m, dict) and isinstance(m.get("oid"), str) and isinstance(m.get("name"), str):
          known = self._by_sha.get(m["oid"])
          if known is None:
            self._by_sha[m["oid"]] = m["name"]
          elif m["name"] not in known.split(" / "):
            self._by_sha[m["oid"]] = f"{known} / {m['name']}"
    return text(self._by_sha.get(sha, ""))


class Reader:
  """read() -> {field: value} for every field of FmJetlinkStatus. Never raises: a record that is missing or can't be
  read gives that part's defaults."""

  def __init__(self, params, root: Path = ROOT, index_path: Path | None = None, mono: Callable[[], float] = time.monotonic,
               alive: Callable[[object], bool] = pid_alive):
    self.params = params
    self.root = Path(root)
    self.names = Names(index_path)
    self.mono = mono
    self.alive = alive

  def _param(self, key: str) -> dict:
    try:
      return as_dict(self.params.get(key))
    except Exception:
      return {}

  def read(self) -> dict:
    now = self.mono()
    out: dict = dict.fromkeys(TEXT, "") | dict.fromkeys(BOOL, False) | dict.fromkeys(COUNT, 0) | {"recordAge": NO_RECORD}

    modeld = read_record(self.root / "modeld.json") or {}
    at = modeld.get("at") if isinstance(modeld.get("at"), (int, float)) and not isinstance(modeld.get("at"), bool) else None
    if at is not None:
      out["recordAge"] = round(max(now - at, 0.0), 2)
    live = at is not None and now - at < HEARTBEAT_TIMEOUT and not modeld.get("stopping") and self.alive(modeld.get("pid"))
    if live:
      last = modeld.get("last_output_at")
      answered = isinstance(last, (int, float)) and at - last < OUTPUT_FRESH
      out["bigDriving"] = bool(modeld.get("active")) and answered and now - at < ACTIVE_FRESH
      out["state"] = text(modeld.get("state"))
      out["held"] = bool(modeld.get("held"))
      for k in COUNT:
        out[k] = count(modeld.get(k))
      if out["bigDriving"]:
        sha = text(modeld.get("sha256"))
        out["drivingSha256"] = sha
        out["drivingCheckpoint"] = text(modeld.get("checkpoint"))
        out["drivingName"] = real_name(text(modeld.get("name")), sha) or self.names.of(sha)

    server = read_record(self.root / "server.json") or {}
    out["device"], out["backend"] = text(server.get("device")), text(server.get("backend"))
    pick = server.get("pick") if isinstance(server.get("pick"), dict) else {}
    out["serverPickSha256"] = text(pick.get("sha256"))
    out["serverPickName"] = real_name(text(pick.get("name")), out["serverPickSha256"]) or self.names.of(out["serverPickSha256"])

    owner = read_record(self.root / "status.json") or {}
    out["linkKind"] = text(owner.get("link"))
    out["linkSpeed"] = text(owner.get("speed"))
    out["hostPresent"] = bool(owner.get("present"))

    spec = self._param("JetlinkSpec")
    out["builtSha256"] = text(spec.get("sha256"))
    out["builtReady"] = spec.get("ready") is True
    out["builtName"] = self.names.of(out["builtSha256"])

    # a model the comma's index copy doesn't list yet (one released after its last refresh) is still named by the Mac's
    # pick when it is the same weights
    pick_sha, pick_name = out["serverPickSha256"], out["serverPickName"]
    for name_key, sha_key in (("drivingName", "drivingSha256"), ("builtName", "builtSha256")):
      if not out[name_key] and out[sha_key] and out[sha_key] == pick_sha:
        out[name_key] = pick_name

    slot = self._param("ModelManager_ActiveBundleChestnut")
    out["slotName"] = text(slot.get("displayName") or slot.get("internalName"))

    progress = self._param("JetlinkProgress")
    out["progressStage"], out["progressMsg"] = text(progress.get("stage")), text(progress.get("msg"))
    return out


def fill_message(m, d: dict) -> None:
  """d (Reader.read()) into an FmJetlinkStatus builder."""
  m.recordAge = float(d["recordAge"])
  for k in TEXT + BOOL + COUNT:
    setattr(m, k, d[k])


def short(sha: str) -> str:
  return sha[:8] if sha else "?"


class ChangeLog:
  """One log line for each change worth reading in a log, not one per second. A change is the model driving (or the
  small model taking over), or the picks and engine that decide what drives next. State flapping (joining, retrying)
  stays in the message, not the log."""

  def __init__(self):
    self.driving = None
    self.picks = None

  def lines(self, d: dict) -> list[str]:
    out = []
    driving = (d["bigDriving"], d["drivingSha256"] if d["bigDriving"] else "")
    if driving != self.driving:
      if d["bigDriving"]:
        name = d["drivingName"] or "(not named)"
        out.append(f"FM Pilot Jetlink: big model driving: {name} ({short(d['drivingSha256'])}), {d['device'] or 'host ?'}")
      elif self.driving is None:
        out.append(f"FM Pilot Jetlink: small model driving (Jetlink state {d['state'] or 'none'})")
      else:
        was = short(self.driving[1]) if self.driving[1] else "?"
        out.append(f"FM Pilot Jetlink: small model driving again (Jetlink state {d['state'] or 'none'}), was {was}")
      self.driving = driving
    picks = (d["serverPickSha256"], d["slotName"], d["builtSha256"], d["builtReady"])
    if picks != self.picks:
      built = f"{d['builtName'] or '?'} ({short(d['builtSha256'])}{'' if d['builtReady'] else ', not ready'})"
      out.append(f"FM Pilot Jetlink: Mac pick {d['serverPickName'] or '?'}, comma slot {d['slotName'] or '?'}, last engine built {built}")
      self.picks = picks
    return out
