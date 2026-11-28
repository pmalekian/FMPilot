"""FM Pilot: the settings that follow the driving personality keep working with every Accel Boost mode.

The Accel Boost picker only lifts the model's (e2e) target. What changes with the personality elsewhere must not move:
- the lead MPC's follow time (1.45 / 1.45 / 1.25 s relaxed / standard / aggressive, openpilot #39440: Relaxed follows at
  Standard's distance, with gentler acceleration) and jerk factor,
- AP's / sunnypilot's Following Time Offset (SPFollowingTimeOffset, % on that follow time),
- the reduced stop distance and Link Stop Distance to Personality (SPStopDistance, SPStopDistancePersonality),
- the steady-state following distance behind a lead in every personality, for every mode (full planner simulation, as
  openpilot's test_following_distance.py, with the mode set).
"""
import itertools

import pytest

from opendbc.car.honda.interface import CarInterface
from opendbc.car.honda.values import CAR
from openpilot.cereal import log
from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import STOP_DISTANCE, get_jerk_factor, get_T_FOLLOW
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
from openpilot.selfdrive.controls.tests.test_following_distance import desired_follow_distance, run_following_distance_simulation
from openpilot.sunnypilot.selfdrive.controls.lib.fm_accel_boost_mode import MODES, PARAM_MODE

P = log.LongitudinalPersonality


def test_follow_time_and_jerk_by_personality():
  assert (get_T_FOLLOW(P.relaxed), get_T_FOLLOW(P.standard), get_T_FOLLOW(P.aggressive)) == (1.45, 1.45, 1.25)
  assert (get_jerk_factor(P.relaxed), get_jerk_factor(P.standard), get_jerk_factor(P.aggressive)) == (1.0, 1.0, 0.5)


def _planner():
  CP = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)
  CP_SP = CarInterface.get_non_essential_params_sp(CP, CAR.HONDA_CIVIC)
  return LongitudinalPlanner(CP, CP_SP)


class TestPersonalitySettingsWithEveryMode(OpenpilotTestCase):
  def test_planner_reads_follow_offset_and_stop_distance(self):
    params = Params()
    for mode in MODES:
      params.put(PARAM_MODE, mode, block=True)
      params.put("SPFollowingTimeOffset", 10, block=True)
      params.put("SPStopDistance", 45, block=True)
      params.put_bool("SPStopDistancePersonality", True, block=True)
      planner = _planner()
      assert planner.accel_boost.mode == mode
      assert planner.mpc.t_follow_offset_pct == 10
      assert planner.mpc.stop_distance == pytest.approx(4.5)
      assert planner.mpc.personality_linked
      lead = log.RadarState.new_message(leadOne={'present': True, 'dRel': 40.0, 'vLead': 20.0, 'aLeadK': 0.0, 'aLeadTau': 0.3})
      for personality in (P.aggressive, P.standard, P.relaxed):
        planner.mpc.update(lead, personality=personality)
        assert planner.mpc.params[0, 4] == pytest.approx(get_T_FOLLOW(personality) * 1.1)  # +10 % follow time

  def test_linked_stop_distance_by_personality(self):
    params = Params()
    params.put("SPFollowingTimeOffset", 0, block=True)
    params.put("SPStopDistance", 45, block=True)
    params.put_bool("SPStopDistancePersonality", True, block=True)
    for mode in MODES:
      params.put(PARAM_MODE, mode, block=True)
      gaps = {}
      for personality in (P.aggressive, P.standard, P.relaxed):
        mpc = _planner().mpc
        mpc.v_cruise_for_stop_gate = 10.0  # slow cruise: the reduced stop distance applies
        lead = log.RadarState.new_message(leadOne={'present': True, 'dRel': 20.0, 'vLead': 0.0, 'aLeadK': 0.0, 'aLeadTau': 0.3})
        mpc.update(lead, personality=personality)
        gaps[personality] = float(mpc.params[0, 2])  # the obstacle the MPC sees for the stopped lead
        assert mpc.params[0, 4] == pytest.approx(get_T_FOLLOW(personality))  # follow time for this personality
      # aggressive gets all of the 1.5 m reduction, standard half, relaxed none: the obstacle moves out by that much
      assert gaps[P.aggressive] - gaps[P.relaxed] == pytest.approx(STOP_DISTANCE - 4.5, abs=1e-3), mode
      assert gaps[P.standard] - gaps[P.relaxed] == pytest.approx((STOP_DISTANCE - 4.5) / 2, abs=1e-3), mode


@pytest.mark.parametrize("mode,personality,speed", list(itertools.product(MODES, (P.relaxed, P.standard, P.aggressive), (10, 35))))
def test_following_distance_with_every_mode(mode, personality, speed):
  with _prefix():
    Params().put(PARAM_MODE, mode, block=True)
    steady = run_following_distance_simulation(float(speed), personality=personality)
    expected = desired_follow_distance(float(speed), float(speed), get_T_FOLLOW(personality))
    assert steady == pytest.approx(expected, abs=0.2 * expected + 0.5)  # openpilot's tolerance


def _prefix():
  from openpilot.common.prefix import OpenpilotPrefix
  return OpenpilotPrefix(shared_download_cache=True)
