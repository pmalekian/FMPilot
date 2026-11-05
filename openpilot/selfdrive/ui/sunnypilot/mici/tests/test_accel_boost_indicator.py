"""FM Pilot: Accel Boost bar on the comma 4 side panel (accel_boost_indicator.py, from sunnypilot #2090)."""
from types import SimpleNamespace

import pytest

pytest.importorskip("pyray")

from openpilot.cereal import log
from openpilot.selfdrive.controls.lib.fm_boost_mode import MODE_GAS, MODE_OFF, MODE_PREDICTIVE, boost_max
from openpilot.selfdrive.ui.sunnypilot.mici.onroad.accel_boost_indicator import (CAP_BLEND_START, COLOR_FILL, AccelBoostIndicator,
                                                                                get_boost_color, get_boost_fraction)
from openpilot.selfdrive.ui.ui_state import ui_state

E2E = log.LongitudinalPlan.LongitudinalPlanSource.e2e
LEAD = log.LongitudinalPlan.LongitudinalPlanSource.lead0
P = log.LongitudinalPersonality


class FakeSM(dict):
  def __init__(self, boost=0.2, source=E2E, enabled=True, personality=P.aggressive):
    super().__init__(selfdriveState=SimpleNamespace(enabled=enabled, personality=personality),
                     longitudinalPlan=SimpleNamespace(longitudinalPlanSource=source),
                     longitudinalPlanSP=SimpleNamespace(fmAccelBoost=boost))
    self.alive = dict.fromkeys(self, True)


@pytest.fixture
def state(monkeypatch):
  monkeypatch.setattr(ui_state, "fm_accel_boost_indicator", True, raising=False)
  monkeypatch.setattr(ui_state, "fm_accel_boost_mode", MODE_PREDICTIVE, raising=False)
  monkeypatch.setattr(type(ui_state), "has_longitudinal_control", property(lambda self: True), raising=False)
  return monkeypatch


def _settle(ind, sm, monkeypatch, frames=200):
  monkeypatch.setattr(ui_state, "sm", sm, raising=False)
  for _ in range(frames):
    ind._update_state()


def test_fraction_and_color():
  assert get_boost_fraction(0.2, 0.4) == 0.5
  assert get_boost_fraction(0.6, 0.4) == 1.0
  assert get_boost_fraction(-0.1, 0.4) == 0.0
  assert get_boost_fraction(0.1, 0.0) == 0.0
  base = get_boost_color(0.0)
  assert (base.r, base.g, base.b) == (COLOR_FILL.r, COLOR_FILL.g, COLOR_FILL.b)
  assert get_boost_color(CAP_BLEND_START).r == COLOR_FILL.r
  assert get_boost_color(1.0).r > base.r


def test_shows_and_fills_while_boost_sets_the_speed(state):
  ind = AccelBoostIndicator()
  _settle(ind, FakeSM(boost=0.2), state)
  assert ind.holding_panel
  assert ind._boost_filter.x == pytest.approx(0.2, abs=1e-3)
  assert ind._max_boost_filter.x == pytest.approx(boost_max(MODE_PREDICTIVE, P.aggressive), abs=1e-3)


def test_empty_when_lead_or_cruise_limits(state):
  ind = AccelBoostIndicator()
  _settle(ind, FakeSM(boost=0.2, source=LEAD), state)
  assert ind.holding_panel  # bar still shown, ...
  assert ind._boost_filter.x == pytest.approx(0.0, abs=1e-3)  # ... but empty


def test_gas_mode_scales_to_gas_max(state):
  state.setattr(ui_state, "fm_accel_boost_mode", MODE_GAS, raising=False)
  ind = AccelBoostIndicator()
  _settle(ind, FakeSM(boost=0.1), state)
  assert ind._max_boost_filter.x == pytest.approx(boost_max(MODE_GAS, P.aggressive), abs=1e-3)


@pytest.mark.parametrize("setting,mode,enabled", [(False, MODE_PREDICTIVE, True), (True, MODE_OFF, True),
                                                  (True, MODE_PREDICTIVE, False)])
def test_hidden_keeps_confidence_ball(state, setting, mode, enabled):
  state.setattr(ui_state, "fm_accel_boost_indicator", setting, raising=False)
  state.setattr(ui_state, "fm_accel_boost_mode", mode, raising=False)
  ind = AccelBoostIndicator()
  _settle(ind, FakeSM(enabled=enabled), state)
  assert not ind.holding_panel
