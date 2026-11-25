"""FM Pilot: the one-time settings migration onto fm-dev-sync.

fm-dev-sync reshapes some FM settings (new keys, new value ranges; see FM_FOLDED_DEFAULTS.md and FM_PILOT.md). On the
first boot each migration copies the value of the old settings (fm-dev-nav-settle and earlier) into the new ones, so
the truck drives as it did; a fresh install has none of the old keys and keeps the stock/off defaults.

Rules:
- The old keys stay declared and are never written, so switching back to fm-dev-nav-settle restores the old setup.
- A migration only writes a new key that is still unset: a value chosen on fm-dev-sync is never overwritten.
- Each migration runs once. FM_SyncMigration lists the ones that ran (comma-separated names); one that raises is
  logged and not marked, so it is tried again on the next boot. A migration added in a later build runs once on the
  first boot of that build.
- run() is called from params_migration.run_migration on manager start, before the defaults of unset params are
  written, so "unset" still means "not chosen".

Each feature PR that reshapes a setting adds its migration here with @migration("name").
"""
from collections.abc import Callable

from opendbc.sunnypilot.car.rivian.values import MAX_TORQUE_MIN, max_torque_grid
from openpilot.common.swaglog import cloudlog

FLAG = "FM_SyncMigration"

MIGRATIONS: list[tuple[str, Callable]] = []


def migration(name: str):
  """Register fn(params) -> str | None (a note for the log) as the one-time migration called name."""
  assert "," not in name and name not in (n for n, _ in MIGRATIONS), name

  def register(fn):
    MIGRATIONS.append((name, fn))
    return fn
  return register


def done(params) -> set[str]:
  raw = params.get(FLAG)
  if isinstance(raw, bytes):
    raw = raw.decode()
  return {n for n in (raw or "").split(",") if n}


def put_if_unset(params, key: str, value) -> bool:
  """Write value to key unless key already holds a value. True when written."""
  if params.get(key) is not None:
    return False
  params.put(key, value, block=True)
  return True


def run(params, migrations=None) -> list[str]:
  """Run every migration not yet marked done; returns the names that ran this time."""
  finished = done(params)
  ran = []
  for name, fn in (MIGRATIONS if migrations is None else migrations):
    if name in finished:
      continue
    try:
      note = fn(params)
    except Exception as e:
      cloudlog.exception(f"fm_migration: {name} failed, retrying next boot: {e}")
      continue
    cloudlog.info(f"fm_migration: {name}" + (f": {note}" if note else ""))
    finished.add(name)
    ran.append(name)
  if ran:
    params.put(FLAG, ",".join(sorted(finished)), block=True)
  return ran


# --- mappings, one per reshaped setting (registered at import, run in this order) ---

OLD_MAX_TORQUE_DEFAULT = 480  # fm-dev-nav-settle's FM_MaxTorque default


@migration("max_torque")
def _max_torque(params):
  """Max Torque: Panda Allows Higher Torque off -> 385; on -> Max Torque rounded to the 15 grid (580 -> 580). Faster
  Torque Ramp, Full Torque Past 90 and Higher Torque mode are folded into it, so nothing else is copied."""
  panda = params.get("FM_PandaHighTorque")
  if panda is None:
    return "no old setting"
  old = params.get("FM_MaxTorque")
  value = max_torque_grid(OLD_MAX_TORQUE_DEFAULT if old is None else old) if panda else MAX_TORQUE_MIN
  written = put_if_unset(params, "FM_CustomMaxTorque", value)
  folded = {k: params.get(k) for k in ("FM_TorqueFastRamp", "FM_TorqueFullCapHighAngle", "FM_TorqueMode")}
  return (f"FM_CustomMaxTorque {value} (panda {panda}, FM_MaxTorque {old}; folded, now on above 385: {folded})" +
          ("" if written else ", already set: kept"))
