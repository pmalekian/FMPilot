"""
Copyright (c) 2021-, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
FLOOR = re.compile(r"desired_curvature = model_output\['action'\]\[0, ?0\] / \(max\(([\d.]+), v_ego\)\)\*\*2")


def test_action_curvature_floor_is_held_at_one():
  """Upstream moved this floor to 2.5 (comma #39182, sunnypilot #2091), with no rationale and no Rivian evidence.

  We hold the default floor at 1.0 on purpose, in both modeld paths (FM Pilot's FM_LowSpeedCurvatureFloor and
  FM_StraightPullAwayFloor opt into 2.5 in modeld_v2 when enabled). A sync can bring upstream's value back without a
  conflict, so taking 2.5 as the default has to be a choice: change this test with the evidence for it.
  """
  stock = (ROOT / 'openpilot/selfdrive/modeld/modeld.py').read_text()
  assert FLOOR.findall(stock) == ['1.0']
  v2 = (ROOT / 'openpilot/sunnypilot/modeld_v2/modeld.py').read_text()
  assert re.findall(r"^ACTION_SPEED_FLOOR = ([\d.]+)", v2, re.M) == ['1.0']
  assert "desired_curvature = model_output['action'][0, 0] / (max(speed_floor, v_ego))**2" in v2
