"""FM Pilot: OP Personality boost (fm_op_personality_boost.py), against openpilot #39430 + #39440.

The reference below is the three lines openpilot master runs in longitudinal_planner.py after #39440 (6daa874b, Nov 23
2026), with its table, copied as they are there.
"""
import random
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.cereal import log
from openpilot.sunnypilot.selfdrive.controls.lib.fm_op_personality_boost import ACCEL_BOOST, OpPersonalityBoost, get_max_boost

P = log.LongitudinalPersonality

OPENPILOT_ACCEL_BOOST = {  # openpilot #39440
  log.LongitudinalPersonality.relaxed: 0.0,
  log.LongitudinalPersonality.standard: 0.05,
  log.LongitudinalPersonality.aggressive: 0.15,
}


def openpilot_reference(v_ego, output_a_target_e2e, personality_raw):
  speed_scale = np.interp(v_ego, [5.0, 10.0], [0.0, 1.0])
  accel_scale = np.interp(output_a_target_e2e, [-1.0, -0.5], [0.0, 1.0])
  output_a_target_e2e += OPENPILOT_ACCEL_BOOST[personality_raw] * speed_scale * accel_scale
  return output_a_target_e2e


def sm(v_ego, personality):
  return {'carState': SimpleNamespace(vEgo=v_ego), 'selfdriveState': SimpleNamespace(personality=personality)}


def test_table_matches_openpilot():
  assert ACCEL_BOOST == OPENPILOT_ACCEL_BOOST
  assert (get_max_boost(P.aggressive), get_max_boost(P.standard), get_max_boost(P.relaxed)) == (0.15, 0.05, 0.0)


@pytest.mark.parametrize("v_ego,share", [(0.0, 0.0), (5.0, 0.0), (7.5, 0.5), (10.0, 1.0), (30.0, 1.0)])
def test_speed_fade(v_ego, share):
  boost = OpPersonalityBoost()
  assert boost.update(sm(v_ego, P.aggressive), 0.3) == pytest.approx(0.3 + 0.15 * share)
  assert boost.total_boost == pytest.approx(0.15 * share)


@pytest.mark.parametrize("a_e2e,share", [(-2.0, 0.0), (-1.0, 0.0), (-0.75, 0.5), (-0.5, 1.0), (0.0, 1.0), (1.5, 1.0)])
def test_braking_fade(a_e2e, share):
  boost = OpPersonalityBoost()
  assert boost.update(sm(20.0, P.standard), a_e2e) == pytest.approx(a_e2e + 0.05 * share)


def test_relaxed_adds_nothing():
  boost = OpPersonalityBoost()
  assert boost.update(sm(20.0, P.relaxed), 0.4) == 0.4
  assert boost.total_boost == 0.0


@pytest.mark.parametrize("seed", range(3))
def test_same_as_openpilot_on_random_inputs(seed):
  rng = random.Random(seed)
  boost = OpPersonalityBoost()
  for _ in range(2000):
    v, a, p = rng.uniform(0, 40), rng.uniform(-3, 2), rng.choice((P.aggressive, P.standard, P.relaxed))
    capnp_personality = log.SelfdriveState.new_message(personality=p).personality  # the real enum, with .raw
    assert boost.update(sm(v, capnp_personality), a) == openpilot_reference(v, a, capnp_personality.raw)


@pytest.mark.parametrize("personality,expected", [(SimpleNamespace(raw=0), 0.15), (1, 0.05), (2, 0.0), (None, 0.05), (9, 0.05)])
def test_personality_forms(personality, expected):
  assert get_max_boost(personality) == expected
