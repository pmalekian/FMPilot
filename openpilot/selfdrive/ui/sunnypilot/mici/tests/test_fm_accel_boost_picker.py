"""FM Pilot: the comma 4's accel boost picker (toggles.py, FmAccelBoostPicker) stores FM_AccelBoostMode's mode numbers.

Mode 1 (Gas) was removed, so the options are 0, 2, 3, 4, 5: the button must store those numbers, not its option index, and
a stored 1 shows as off. The widget's drawing base is stubbed (no window here); the param mapping is the real code.
"""
import time

import pytest

pytest.importorskip("raylib_nb")

from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.ui.mici.widgets import button
from openpilot.selfdrive.ui.mici.layouts.settings.driving import ACCEL_BOOST_OPTIONS, FmAccelBoostPicker
from openpilot.sunnypilot.selfdrive.controls.lib.fm_accel_boost_mode import MODES, parse_mode

KEY = "FM_AccelBoostMode"
OPTIONS = [o for o, _ in ACCEL_BOOST_OPTIONS]
VALUES = [v for _, v in ACCEL_BOOST_OPTIONS]


def stored_after_tap(params, widget, previous):
  """The button writes without blocking (as the base widget does); wait for the write to land."""
  widget._handle_mouse_release(None)
  deadline = time.monotonic() + 2.0
  while params.get(KEY) == previous and time.monotonic() < deadline:
    time.sleep(0.01)
  return params.get(KEY)


@pytest.fixture(autouse=True)
def own_params():
  with OpenpilotPrefix(shared_download_cache=True):
    yield


@pytest.fixture
def picker(monkeypatch):
  def fake_init(self, text, value="", *args, **kwargs):
    self.value = value
  monkeypatch.setattr(button.BigToggle, "__init__", fake_init)
  monkeypatch.setattr(button.BigToggle, "_handle_mouse_release", lambda self, mouse_pos: None)
  monkeypatch.setattr(button.BigButton, "set_value", lambda self, value: setattr(self, "value", value))
  return lambda: FmAccelBoostPicker("accel boost", KEY, OPTIONS, values=VALUES)


def test_options_are_the_picker_modes():
  assert VALUES == list(MODES)
  assert 1 not in VALUES


@pytest.mark.parametrize("stored,shown", [(None, "off"), (0, "off"), (1, "off"), (2, "sunny boost"), (3, "evp set speed"),
                                          (4, "ap accel control"), (5, "op personality"), (9, "off")])
def test_shows_the_stored_mode(picker, stored, shown):
  params = Params()
  if stored is None:
    params.remove(KEY)
  else:
    params.put(KEY, stored, block=True)
  assert picker().value == shown
  # what it shows is what the planner runs
  assert VALUES[OPTIONS.index(shown)] == parse_mode(stored)


def test_tapping_cycles_through_mode_numbers(picker):
  params = Params()
  params.put(KEY, 0, block=True)
  widget = picker()
  stored = [0]
  for _ in range(len(VALUES)):
    stored.append(stored_after_tap(params, widget, stored[-1]))
  assert stored[1:] == [2, 3, 4, 5, 0]


def test_a_stored_gas_steps_to_sunny_boost(picker):
  params = Params()
  params.put(KEY, 1, block=True)
  widget = picker()
  assert stored_after_tap(params, widget, 1) == 2
