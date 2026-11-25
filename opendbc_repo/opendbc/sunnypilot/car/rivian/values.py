"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from enum import IntFlag


class RivianFlagsSP(IntFlag):
  LONGITUDINAL_HARNESS_UPGRADE = 1
  ENHANCED_MADS = 2
  # FM Pilot: Max Torque is above 385 this drive. The panda allows the higher low-speed limit, and the car code uses it
  # with the faster ramp and without AP's 95% cap past 90 degrees (both folded into Max Torque)
  HIGH_TORQUE_PANDA = 4


class RivianSafetyFlagsSP:
  ENHANCED_MADS = 1
  HIGH_TORQUE = 2  # FM Pilot: Max Torque above 385 this drive
  # FM Pilot "Max Torque": the drive's low-speed limit as a level in bits 2-5 (see max_torque_level); the panda decodes
  # the same levels (safety/modes/rivian.h) and uses them only with HIGH_TORQUE
  TORQUE_LEVEL_SHIFT = 2
  TORQUE_LEVEL_MASK = 0xF


# FM Pilot "Max Torque" slider: 385 to 580 in steps of 15. 385 is AP's stock limit (level 0, nothing changes); level n
# (1-13) is 385 + 15 n. The high curve's shape and the panda envelope's shape are from AP 5270cbba (Uncle-Tony, MIT).
MAX_TORQUE_MIN = 385
MAX_TORQUE_MAX = 580
MAX_TORQUE_STEP = 15
MAX_TORQUE_DEFAULT = 385
MAX_TORQUE_LEVELS = (MAX_TORQUE_MAX - MAX_TORQUE_MIN) // MAX_TORQUE_STEP  # 13


def max_torque_grid(value) -> int:
  """Any value -> the nearest slider step, clamped to 385-580. Unreadable values -> 385 (stock)."""
  try:
    v = float(value)
  except (TypeError, ValueError):
    v = MAX_TORQUE_DEFAULT
  if v != v:  # NaN
    v = MAX_TORQUE_DEFAULT
  v = min(max(v, MAX_TORQUE_MIN), MAX_TORQUE_MAX)
  return MAX_TORQUE_MIN + MAX_TORQUE_STEP * int(round((v - MAX_TORQUE_MIN) / MAX_TORQUE_STEP))


def max_torque_level(value) -> int:
  """Slider value -> level 0-13 (0 = 385, stock)."""
  return (max_torque_grid(value) - MAX_TORQUE_MIN) // MAX_TORQUE_STEP


def high_torque_cap(safety_param: int) -> int:
  """The low-speed torque limit the panda enforces for this drive's safety param: 385 unless HIGH_TORQUE is set with a
  level 1-13 (the panda reads it the same way)."""
  if not safety_param & RivianSafetyFlagsSP.HIGH_TORQUE:
    return MAX_TORQUE_MIN
  level = (safety_param >> RivianSafetyFlagsSP.TORQUE_LEVEL_SHIFT) & RivianSafetyFlagsSP.TORQUE_LEVEL_MASK
  if 1 <= level <= MAX_TORQUE_LEVELS:
    return MAX_TORQUE_MIN + MAX_TORQUE_STEP * level
  return MAX_TORQUE_MIN
