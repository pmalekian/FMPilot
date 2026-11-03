"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

FM Pilot: Predictive Accel Boost, ported from sunnypilot branch `booooost` (James Vecellio-Grant / Discountchubbs,
commits 367ff603d..048727fb9, Nov 3 2026). The numbers and update() are unchanged. FM Pilot change: the AccelBoostSP
on/off param read is removed; FM_AccelBoostMode chooses between this and Gas Override Boost
(openpilot/selfdrive/controls/lib/fm_boost_mode.py). When sunnypilot ships this upstream, this copy gives way to theirs.
"""
import numpy as np

from openpilot.cereal import log
from openpilot.common.realtime import DT_MDL

ACCEL_BOOST_MIN_SPEED = 1.0
ACCEL_BOOST_MAX = (0.4, 0.25, 0.1)  # aggressive, standard, relaxed, m/s^2
ACCEL_BOOST_BUILD, ACCEL_BOOST_RELEASE = 0.2, 1.0  # m/s^3
BRAKE_PROB = (0.1, 0.3)  # the model's brake press in 2 s: it wants to slow
GAS_PROB = (0.6, 0.8)  # predicts a human gas press at this hypothesis at about 23%
CURVE_LAT_ACCEL = (1.5, 2.5)


def get_boost_max(personality=log.LongitudinalPersonality.standard):
  if personality == log.LongitudinalPersonality.relaxed:
    return ACCEL_BOOST_MAX[2]
  elif personality == log.LongitudinalPersonality.standard:
    return ACCEL_BOOST_MAX[1]
  elif personality == log.LongitudinalPersonality.aggressive:
    return ACCEL_BOOST_MAX[0]
  else:
    raise NotImplementedError("Longitudinal personality not supported")


class AccelBoostSP:
  def __init__(self):
    self.boost = 0.0
    self.total_boost = 0.0

  def reset(self):
    self.boost = self.total_boost = 0.0

  def update(self, sm, output_a_target_e2e, output_a_target_mpc, a_cruise):
    model = sm['modelV2']
    brake_probs = model.meta.disengagePredictions.brakePressProbs
    gas_probs = model.meta.disengagePredictions.gasPressProbs
    if not sm['selfdriveState'].enabled or len(brake_probs) < 2 or len(gas_probs) < 2:
      self.boost = self.total_boost = 0.0
      return output_a_target_e2e

    want = max(float(np.interp(brake_probs[1], BRAKE_PROB, [1.0, 0.0])), float(np.interp(gas_probs[1], GAS_PROB, [0.0, 1.0])))
    want = 0.0 if model.action.shouldStop else want
    want *= float(np.interp(abs(model.action.desiredCurvature) * sm['carState'].vEgo ** 2, CURVE_LAT_ACCEL, [1.0, 0.0]))
    target = want * get_boost_max(sm['selfdriveState'].personality)
    self.boost = float(np.clip(target, self.boost - ACCEL_BOOST_RELEASE * DT_MDL, self.boost + ACCEL_BOOST_BUILD * DT_MDL))
    speed_scale = np.interp(sm['carState'].vEgo, [ACCEL_BOOST_MIN_SPEED, 2 * ACCEL_BOOST_MIN_SPEED], [0.0, 1.0])
    self.total_boost = self.boost * speed_scale * np.interp(output_a_target_e2e, [-0.7, -0.2, 5.0], [0.0, 1.0, 1.0], right=0.0)
    return output_a_target_e2e + self.total_boost
