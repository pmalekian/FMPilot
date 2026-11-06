"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from enum import IntFlag


class RivianFlagsSP(IntFlag):
  LONGITUDINAL_HARNESS_UPGRADE = 1
  ENHANCED_MADS = 2
  HIGH_TORQUE_PANDA = 4  # FM Pilot: panda allows the higher low-speed torque limit this drive
  FAST_RAMP = 8  # FM Pilot: panda allows torque to rise 5 per frame at low speed this drive
  FULL_CAP_HIGH_ANGLE = 16  # FM Pilot: no 95% torque cap past 90 degrees of wheel this drive


class RivianSafetyFlagsSP:
  ENHANCED_MADS = 1
  HIGH_TORQUE = 2  # FM Pilot
  # FM Pilot "Max Torque": the low-speed limit chosen for this drive, as a level in bits 2-5 (see max_torque_level)
  TORQUE_LEVEL_SHIFT = 2
  TORQUE_LEVEL_MASK = 0xF
  FAST_RAMP = 64  # FM Pilot "Faster Torque Ramp", only with HIGH_TORQUE


# FM Pilot "Max Torque" slider: 405 to 580 in steps of 25. Level n (1-8) is 380 + 25 n; the panda decodes the same
# levels (rivian.h). Level 0 with HIGH_TORQUE set is the earlier fixed 481 (AP 5270cbba).
MAX_TORQUE_MIN = 405
MAX_TORQUE_MAX = 580
MAX_TORQUE_STEP = 25
MAX_TORQUE_DEFAULT = 480
_TORQUE_LEVEL_BASE = 380
_LEGACY_HIGH_TORQUE = 481


def max_torque_level(value) -> int:
  """Slider value -> level 1-8, rounded to the nearest step and clamped to 405-580. Unreadable values -> the default."""
  try:
    v = float(value)
  except (TypeError, ValueError):
    v = MAX_TORQUE_DEFAULT
  if v != v:  # NaN
    v = MAX_TORQUE_DEFAULT
  v = min(max(v, MAX_TORQUE_MIN), MAX_TORQUE_MAX)
  return int(round((v - _TORQUE_LEVEL_BASE) / MAX_TORQUE_STEP))


def high_torque_cap(safety_param: int) -> int:
  """The low-speed torque limit the panda enforces for this drive's safety param (385 when the higher limit is off)."""
  if not safety_param & RivianSafetyFlagsSP.HIGH_TORQUE:
    return 385
  level = (safety_param >> RivianSafetyFlagsSP.TORQUE_LEVEL_SHIFT) & RivianSafetyFlagsSP.TORQUE_LEVEL_MASK
  if 1 <= level <= 8:
    return _TORQUE_LEVEL_BASE + MAX_TORQUE_STEP * level
  return _LEGACY_HIGH_TORQUE
