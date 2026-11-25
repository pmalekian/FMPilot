"""
Copyright (c) 2021-, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

FM Pilot: the AccelBoostSP / AccelerationControl interlock that LongitudinalPlannerSP.update ran is the Accel Boost
picker's job now (fm_accel_boost_mode.py; its toggle tests are in test_fm_accel_boost_mode.py). These check that the
SP update no longer picks or swaps a boost, whatever the toggles say, and that the planner builds the picker.
"""
from types import SimpleNamespace

from opendbc.car import structs
from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase

from openpilot.sunnypilot.selfdrive.controls.lib.fm_accel_boost_mode import MODE_OFF, MODE_SUNNY, PARAM_MODE, READ_FRAMES, FmAccelBoostMode
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerSP


class TestAccelBoostInterlock(OpenpilotTestCase):
  def _planner(self):
    boost = FmAccelBoostMode(structs.CarParams(openpilotLongitudinalControl=True))
    return SimpleNamespace(accel_boost=boost, openpilot_longitudinal=True, events_sp=SimpleNamespace(clear=lambda: None),
                           e2e_alerts_helper=SimpleNamespace(update=lambda sm, events: None))

  def test_sp_update_never_swaps_the_boost(self):
    params = Params()
    for sp in (False, True):
      for ac in (False, True):
        params.put_bool("AccelBoostSP", sp, block=True)
        params.put_bool("AccelerationControl", ac, block=True)
        planner = self._planner()
        kept = planner.accel_boost
        for _ in range(3 * READ_FRAMES):
          LongitudinalPlannerSP.update(planner, {})
        assert planner.accel_boost is kept
        assert kept.mode == MODE_OFF  # the toggles don't choose a mode

  def test_the_picker_mode_is_what_runs(self):
    Params().put(PARAM_MODE, MODE_SUNNY, block=True)
    Params().put_bool("AccelBoostSP", False, block=True)
    assert self._planner().accel_boost.mode == MODE_SUNNY
