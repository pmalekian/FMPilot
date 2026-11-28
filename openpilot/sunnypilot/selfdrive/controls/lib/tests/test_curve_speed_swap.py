"""FM uses the curve offset only to gate OP Personality, even while SCC vision is disabled."""
from unittest.mock import patch

import pytest

from openpilot.cereal import log
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.test.longitudinal_maneuvers.maneuver import Maneuver
from openpilot.sunnypilot.selfdrive.controls.lib.fm_accel_boost_mode import MODE_OP_PERSONALITY
from openpilot.sunnypilot.selfdrive.controls.lib.tests.test_fm_personality_settings import _planner


@pytest.mark.parametrize("vision_enabled", [False, True])
def test_curve_offset_only_gates_the_selected_boost(vision_enabled):
  with OpenpilotPrefix():
    Params().put("FM_AccelBoostMode", MODE_OP_PERSONALITY, block=True)
    planner = _planner()
    sm = Maneuver("curve", 1.0, initial_speed=10.0)._make_messages(0)
    model = sm["modelV2"]
    model.position.t, model.position.x = [0., 2.], [0., 30.]
    model.velocity.x, model.orientationRate.z = [10., 10.], [0., 0.4]
    sm["selfdriveState"].personality = log.LongitudinalPersonality.aggressive
    planner.scc.vision.enabled = vision_enabled
    with patch.object(planner.accel_boost, "update", wraps=planner.accel_boost.update) as boost:
      planner.update(sm)
    assert boost.call_args.args[1] == model.action.desiredAcceleration
    assert planner.accel_boost.op_personality.curve_offset < 0
    assert planner.accel_boost.applied == 0
