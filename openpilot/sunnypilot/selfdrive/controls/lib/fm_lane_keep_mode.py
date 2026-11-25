"""
FM Pilot: lane keeping mode. One of Off, Lane Centering (AP's), Lane Guard (from gerrylum's rx-wb) or Blendful
(sunnypilot), chosen with a single setting so two of them can never run together.

FM_LaneKeepMode holds the choice (default Off). Before it is first set, the mode comes from the older separate toggles
(AP's LaneCenteringEnabled, then LaneGuardEnabled, then sunnypilot's own Blendful toggle, which is hidden on fm-dev-sync),
and the one-time migration (sunnypilot/system/fm_migration.py) writes that value before the default is, so a device
with lane centering (or sunnypilot's Blendful) on comes out on it. AP's Lane Centering and sunnypilot's Blendful read
this mode instead of their own toggles.
"""

PARAM_MODE = "FM_LaneKeepMode"
LEGACY_CENTERING = "LaneCenteringEnabled"
LEGACY_GUARD = "LaneGuardEnabled"
LEGACY_BLENDFUL = "Blendful"  # sunnypilot's own toggle (hidden from the Models page on fm-dev-sync)

MODE_OFF, MODE_CENTERING, MODE_GUARD, MODE_BLENDFUL = 0, 1, 2, 3
MODES = (MODE_OFF, MODE_CENTERING, MODE_GUARD, MODE_BLENDFUL)


def legacy_mode(params) -> int:
  # Lane centering won over Lane Guard when both were on (fm_lateral.py), so it wins here too.
  if params.get_bool(LEGACY_CENTERING):
    return MODE_CENTERING
  if params.get_bool(LEGACY_GUARD):
    return MODE_GUARD
  if params.get_bool(LEGACY_BLENDFUL):
    return MODE_BLENDFUL
  return MODE_OFF


def parse_mode(value) -> int | None:
  try:
    mode = int(value)
  except (TypeError, ValueError):
    return None
  return mode if mode in MODES else None


def get_lane_keep_mode(params) -> int:
  mode = parse_mode(params.get(PARAM_MODE))
  return legacy_mode(params) if mode is None else mode


def migrate_lane_keep_mode(params) -> bool:
  """Write FM_LaneKeepMode from the older toggles if it isn't set yet and they pick something (Off is the default
  anyway, so a fresh install writes nothing). Returns True if it wrote."""
  if parse_mode(params.get(PARAM_MODE)) is not None:
    return False
  mode = legacy_mode(params)
  if mode == MODE_OFF:
    return False
  params.put(PARAM_MODE, mode, block=True)
  return True


def set_lane_centering(params, enabled: bool) -> None:
  """For the device's own Lane Centering toggle: turning it on picks Lane Centering, turning it off picks Off
  (only if Lane Centering was the active mode, so it never switches off Lane Guard or Blendful)."""
  current = get_lane_keep_mode(params)
  if enabled:
    params.put(PARAM_MODE, MODE_CENTERING)
  elif current == MODE_CENTERING:
    params.put(PARAM_MODE, MODE_OFF)
  params.put_bool(LEGACY_CENTERING, enabled)
