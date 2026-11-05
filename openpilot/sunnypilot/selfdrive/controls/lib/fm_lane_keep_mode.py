"""
FM Pilot: lane keeping mode. One of Off, Lane Centering (AP's), Lane Guard (from gerrylum's rx-wb) or Blendful
(sunnypilot), chosen with a single setting so two of them can never run together.

FM_LaneKeepMode holds the choice. It has no default on purpose: until it is first written, the mode comes from the
older separate toggles (LaneCenteringEnabled, then LaneGuardEnabled), and params_migration writes that same value once
at boot so Sunnylink shows it. A device with lane centering on comes out on Lane Centering.
"""

PARAM_MODE = "FM_LaneKeepMode"
LEGACY_CENTERING = "LaneCenteringEnabled"
LEGACY_GUARD = "LaneGuardEnabled"

MODE_OFF, MODE_CENTERING, MODE_GUARD, MODE_BLENDFUL = 0, 1, 2, 3
MODES = (MODE_OFF, MODE_CENTERING, MODE_GUARD, MODE_BLENDFUL)


def legacy_mode(params) -> int:
  # Lane centering won over Lane Guard when both were on (fm_lateral.py), so it wins here too.
  if params.get_bool(LEGACY_CENTERING):
    return MODE_CENTERING
  if params.get_bool(LEGACY_GUARD):
    return MODE_GUARD
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
  """Write FM_LaneKeepMode from the older toggles if it isn't set yet. Returns True if it wrote."""
  if parse_mode(params.get(PARAM_MODE)) is not None:
    return False
  params.put(PARAM_MODE, legacy_mode(params), block=True)
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
