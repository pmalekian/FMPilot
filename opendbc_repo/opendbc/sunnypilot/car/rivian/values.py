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


class RivianSafetyFlagsSP:
  ENHANCED_MADS = 1
  HIGH_TORQUE = 2  # FM Pilot
