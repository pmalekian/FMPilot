# FM Pilot: tests ported from EVPilot (evpilotgo/openpilot dev, c5943813, Gerry / gerrylum), with the SetSpeedAssist param
# toggles removed (FM_AccelBoostMode picks it; off and mode switching are tested in test_fm_boost_mode.py), plus FM tests at the end.
from types import SimpleNamespace

import pytest

from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.selfdrive.controls.lib.set_speed_assist import (GRACE, GRACE_JERK, MAX_BOOST, RAMP_DOWN, RAMP_UP, SetSpeedAssist,
                                                                           personality_index)


def run(ssa, frames, a_e2e=0.1, a_cruise=1.2, v_mph=40., should_stop=False, curvature=0., enabled=True, personality=0, a_ego=0., lead=False):
  sm = {'carState': SimpleNamespace(vEgo=v_mph * CV.MPH_TO_MS, aEgo=a_ego),
        'selfdriveState': SimpleNamespace(enabled=enabled, personality=SimpleNamespace(raw=personality)),
        'modelV2': SimpleNamespace(action=SimpleNamespace(shouldStop=should_stop, desiredCurvature=curvature)),
        'radarState': SimpleNamespace(leadOne=SimpleNamespace(present=lead))}
  return [ssa.update(sm, a_e2e, a_cruise) for _ in range(frames)][-1]


@pytest.mark.parametrize("personality", [0, 1, 2])
def test_ramps_to_the_personality_cap_under_cruise(personality):
  ssa = SetSpeedAssist()
  assert run(ssa, 1, personality=personality) == pytest.approx(0.1 + RAMP_UP * DT_MDL)
  assert run(ssa, 100, personality=personality) == pytest.approx(0.1 + MAX_BOOST[personality])
  assert run(ssa, 100, a_cruise=0.35, personality=personality) == pytest.approx(0.35)  # never above the cruise ramp


@pytest.mark.parametrize("kw, at_once", [({"a_e2e": -0.6}, True), ({"a_e2e": -0.5}, False), ({"should_stop": True}, True), ({"v_mph": 15.}, False),
                                         ({"a_cruise": -0.3}, False), ({"curvature": 0.01}, False), ({"enabled": False}, False)])
def test_backs_off(kw, at_once):
  ssa = SetSpeedAssist()
  run(ssa, 100)
  a_e2e = kw.get("a_e2e", 0.1)
  assert (run(ssa, 1, **kw) == pytest.approx(a_e2e)) == at_once
  assert run(ssa, 100, **kw) == pytest.approx(a_e2e)


@pytest.mark.parametrize("a_e2e, share", [(0.1, 1.), (-0.1, 1.), (-0.2, 0.75), (-0.3, 0.5), (-0.4, 0.25)])
def test_a_model_slow_down_shrinks_the_cap_in_proportion(a_e2e, share):
  ssa = SetSpeedAssist()
  assert run(ssa, 100, a_e2e=a_e2e, a_cruise=2.0) == pytest.approx(a_e2e + MAX_BOOST[0] * share)


def test_the_model_drifting_past_minus_0_1_trims_the_boost_without_a_drop():
  ssa = SetSpeedAssist()
  run(ssa, 100, a_e2e=0.1, a_cruise=2.0)
  boosts = []
  for a_e2e in [0.1 - 0.01 * i for i in range(31)]:  # the model eases from +0.1 to -0.2 over 1.5 s, as on route 86a69c7afc58c683/28
    run(ssa, 1, a_e2e=a_e2e, a_cruise=2.0)
    boosts.append(ssa.boost)
  assert boosts[-1] == pytest.approx(MAX_BOOST[0] * 0.75)  # the old gate had it at 0 here
  assert all(b - a > -RAMP_DOWN * DT_MDL / 2 for b, a in zip(boosts[1:], boosts, strict=False))  # never drains at the full rate


@pytest.mark.parametrize("lat_accel, share", [(0.3, 1.), (0.5, 1.), (0.75, 0.5), (1.0, 0.)])
def test_a_turn_shrinks_the_cap_in_proportion(lat_accel, share):
  ssa = SetSpeedAssist()
  v = 40. * CV.MPH_TO_MS
  assert run(ssa, 100, a_cruise=2.0, curvature=lat_accel / v ** 2) == pytest.approx(0.1 + MAX_BOOST[0] * share)


def test_a_lead_leaving_mid_boost_does_not_step_the_accel_down():
  ssa = SetSpeedAssist()
  run(ssa, 100, a_e2e=-0.12, a_cruise=2.0, a_ego=-0.2, lead=True)  # boost near the cap, a lead far ahead, the truck easing off
  before = run(ssa, 1, a_e2e=-0.12, a_cruise=2.0, a_ego=-0.2, lead=True)
  assert run(ssa, 1, a_e2e=-0.12, a_cruise=2.0, a_ego=-0.2) >= before  # the lead leaves: grace starts from where we were


SLOWING = {"a_e2e": -0.8, "a_cruise": -0.5, "a_ego": -1.0}  # the model carries the truck's slowdown past the engage


def engaged_ssa():
  ssa = SetSpeedAssist()
  run(ssa, 1, enabled=False)
  return ssa


@pytest.mark.parametrize("after", [{"a_e2e": -1.6}, {"should_stop": True}, {"v_mph": 15.}])
def test_engage_grace_hands_back_at_once(after):
  ssa = engaged_ssa()
  assert run(ssa, 10, **SLOWING) == -0.5  # a ramp from the truck's accel up to the cruise ramp leads
  assert run(ssa, 10, **(SLOWING | {"a_e2e": -1.4})) == -0.5  # also while the model deepens the slowdown a little
  assert run(ssa, 1, **(SLOWING | after)) == after.get("a_e2e", -0.8)  # harder braking, a stop or low speed
  assert run(ssa, 1, **SLOWING) == -0.8  # and it never re-arms


def test_engage_grace_ends_after_the_window_and_fades_out_smoothly():
  ssa = engaged_ssa()
  assert run(ssa, int(GRACE / DT_MDL) - 1, **SLOWING) == -0.5
  assert run(ssa, 2, **SLOWING) == -0.8
  ssa = engaged_ssa()
  run(ssa, 10, **SLOWING)
  assert run(ssa, 1, a_e2e=-0.05, a_cruise=0.5) > -0.5  # braking faded out: no step down


def test_engage_grace_holds_a_slowdown_the_model_starts_after_the_engage():
  ssa = engaged_ssa()
  assert run(ssa, 1, a_e2e=-0.05, a_cruise=0.5) == pytest.approx(-0.05 + RAMP_UP * DT_MDL)  # not slowing yet: the usual ramp
  assert run(ssa, 10, a_e2e=-0.4, a_cruise=0.5, a_ego=-0.5) == pytest.approx(0.5)


@pytest.mark.parametrize("a_ego, v_mph, grace", [(-1.0, 15., True), (-1.0, 8., False), (0., 15., False)])
def test_grace_when_a_lead_we_braked_for_leaves(a_ego, v_mph, grace):
  ssa = engaged_ssa()
  run(ssa, 10, a_e2e=-0.9, a_cruise=2.0, a_ego=a_ego, v_mph=v_mph, lead=True)
  start = max(a_ego, -0.6 + ssa.boost)  # the truck's accel, or the model plus any boost we already had if higher
  out = run(ssa, 20, a_e2e=-0.6, a_cruise=2.0, a_ego=a_ego, v_mph=v_mph)  # the cruise ramp is far up: ramps from there instead
  assert out == (pytest.approx(start + 20 * GRACE_JERK * DT_MDL) if grace else -0.6)  # below LEAD_MIN_SPEED or not braking: no grace


# FM Pilot

def test_a_fresh_instance_while_engaged_is_not_an_engage():
  ssa = SetSpeedAssist()  # Set Speed picked in Sunnylink while driving, the truck slowing
  assert run(ssa, 10, **SLOWING) == -0.8  # no engage grace: the model leads


@pytest.mark.parametrize("personality, idx", [(0, 0), (1, 1), (2, 2), (SimpleNamespace(raw=2), 2), (None, 1), (9, 1)])
def test_personality_index(personality, idx):
  assert personality_index(personality) == idx
