"""FM Pilot: Accel Boost bar on the comma 4 side panel (accel_boost_indicator.py, from sunnypilot #2090).

Ported from fm-dev-nav-settle ca08aac9: reads longitudinalPlan.accelBoost now, and covers all five picker modes.
"""
from types import SimpleNamespace

import pytest

pytest.importorskip("raylib_nb")

from openpilot.cereal import log
from openpilot.selfdrive.ui.sunnypilot.mici.onroad.accel_boost_indicator import (CAP_BLEND_START, COLOR_FILL, AccelBoostIndicator,
                                                                                get_boost_color, get_boost_fraction)
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.sunnypilot.selfdrive.controls.lib.fm_accel_boost_mode import (MODE_AP_ACCEL_CONTROL, MODE_EVP_SET_SPEED, MODE_OFF,
                                                                            MODE_OP_PERSONALITY, MODE_SUNNY, boost_max)

E2E = log.LongitudinalPlan.LongitudinalPlanSource.e2e
LEAD = log.LongitudinalPlan.LongitudinalPlanSource.lead0
CRUISE = log.LongitudinalPlan.LongitudinalPlanSource.cruise
P = log.LongitudinalPersonality
BOOST_MODES = (MODE_SUNNY, MODE_EVP_SET_SPEED, MODE_AP_ACCEL_CONTROL, MODE_OP_PERSONALITY)


class FakeSM(dict):
  def __init__(self, boost=0.2, source=E2E, enabled=True, personality=P.aggressive):
    super().__init__(selfdriveState=SimpleNamespace(enabled=enabled, personality=personality),
                     longitudinalPlan=SimpleNamespace(longitudinalPlanSource=source, accelBoost=boost))
    self.alive = dict.fromkeys(self, True)


@pytest.fixture
def state(monkeypatch):
  monkeypatch.setattr(ui_state, "fm_accel_boost_indicator", True)
  monkeypatch.setattr(ui_state, "fm_accel_boost_mode", MODE_SUNNY)
  monkeypatch.setattr(type(ui_state), "has_longitudinal_control", property(lambda self: True), raising=False)
  return monkeypatch


def _settle(ind, sm, monkeypatch, frames=200):
  monkeypatch.setattr(ui_state, "sm", sm, raising=False)
  for _ in range(frames):
    ind._update_state()


def test_fresh_ui_state_sp_renders_before_update_params(monkeypatch):
  import raylib_nb as rl
  from openpilot.common.prefix import OpenpilotPrefix
  from openpilot.selfdrive.ui.sunnypilot.mici.onroad import accel_boost_indicator as mod
  from openpilot.selfdrive.ui.sunnypilot.ui_state import UIStateSP

  with OpenpilotPrefix():
    fresh = UIStateSP()
    assert fresh.fm_accel_boost_indicator is False
    assert fresh.fm_accel_boost_mode == MODE_OFF
    fresh.sm = FakeSM()
    fresh.has_longitudinal_control = True
    monkeypatch.setattr(mod, "ui_state", fresh)
    ind = AccelBoostIndicator()
    ind.render(rl.Rectangle(0, 0, 536, 240))
    assert not ind.holding_panel


def test_fraction_and_color():
  assert get_boost_fraction(0.2, 0.4) == 0.5
  assert get_boost_fraction(0.6, 0.4) == 1.0
  assert get_boost_fraction(-0.1, 0.4) == 0.0
  assert get_boost_fraction(0.1, 0.0) == 0.0
  base = get_boost_color(0.0)
  assert (base.r, base.g, base.b) == (COLOR_FILL.r, COLOR_FILL.g, COLOR_FILL.b)
  assert get_boost_color(CAP_BLEND_START).r == COLOR_FILL.r
  assert get_boost_color(1.0).r > base.r


@pytest.mark.parametrize("mode", BOOST_MODES)
@pytest.mark.parametrize("personality", [P.aggressive, P.standard])
def test_every_mode_fills_with_its_published_boost(state, mode, personality):
  state.setattr(ui_state, "fm_accel_boost_mode", mode)
  top = boost_max(mode, personality)
  ind = AccelBoostIndicator()
  _settle(ind, FakeSM(boost=top / 2, personality=personality), state)
  assert ind.holding_panel
  assert ind._boost_filter.x == pytest.approx(top / 2, abs=1e-3)
  assert ind._max_boost_filter.x == pytest.approx(top, abs=1e-3)
  assert get_boost_fraction(ind._boost_filter.x, ind._max_boost_filter.x) == pytest.approx(0.5, abs=0.01)


def test_op_personality_relaxed_stays_empty(state):
  state.setattr(ui_state, "fm_accel_boost_mode", MODE_OP_PERSONALITY)
  ind = AccelBoostIndicator()
  _settle(ind, FakeSM(boost=0.0, personality=P.relaxed), state)
  assert get_boost_fraction(ind._boost_filter.x, ind._max_boost_filter.x) == pytest.approx(0.0, abs=1e-3)


def test_empty_when_lead_limits(state):
  ind = AccelBoostIndicator()
  _settle(ind, FakeSM(boost=0.2, source=LEAD), state)
  assert ind.holding_panel  # bar still shown, ...
  assert ind._boost_filter.x == pytest.approx(0.0, abs=1e-3)  # ... but empty


@pytest.mark.parametrize("setting,mode,enabled", [(False, MODE_SUNNY, True), (True, MODE_OFF, True), (True, MODE_SUNNY, False)])
def test_hidden_keeps_confidence_ball(state, setting, mode, enabled):
  state.setattr(ui_state, "fm_accel_boost_indicator", setting)
  state.setattr(ui_state, "fm_accel_boost_mode", mode)
  ind = AccelBoostIndicator()
  _settle(ind, FakeSM(enabled=enabled), state)
  assert not ind.holding_panel


def test_evp_set_speed_fills_when_tied_with_cruise(state):
  state.setattr(ui_state, "fm_accel_boost_mode", MODE_EVP_SET_SPEED)
  ind = AccelBoostIndicator()
  _settle(ind, FakeSM(boost=0.3, source=CRUISE), state)
  assert ind._boost_filter.x == pytest.approx(0.3, abs=1e-3)


@pytest.mark.parametrize("mode", [MODE_SUNNY, MODE_AP_ACCEL_CONTROL, MODE_OP_PERSONALITY])
def test_other_modes_empty_when_cruise_limits(state, mode):
  state.setattr(ui_state, "fm_accel_boost_mode", mode)
  ind = AccelBoostIndicator()
  _settle(ind, FakeSM(boost=0.1, source=CRUISE), state)
  assert ind._boost_filter.x == pytest.approx(0.0, abs=1e-3)
