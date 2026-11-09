from types import SimpleNamespace

from openpilot.selfdrive.ui.sunnypilot.onroad.fm_jetlink_icon import ACTIVE, LOADING, jetlink_icon_state


def status(enabled=True, progress=None):
  return SimpleNamespace(enabled=enabled, progress=progress)


def test_green_while_the_big_model_drives():
  assert jetlink_icon_state(False, status(), True, True) == ACTIVE
  assert jetlink_icon_state(False, status(progress={"stage": "join", "frac": 0.5}), True, True) == ACTIVE


def test_pulsing_while_preparing_or_joining():
  assert jetlink_icon_state(False, status(progress={"stage": "build", "frac": 0.2}), True, False) == LOADING
  assert jetlink_icon_state(False, status(progress={"stage": "join"}), False, False) == LOADING


def test_nothing_otherwise():
  assert jetlink_icon_state(False, status(), True, False) is None
  assert jetlink_icon_state(False, status(progress={"stage": "ready"}), True, False) is None
  assert jetlink_icon_state(False, status(enabled=False), True, True) is None
  assert jetlink_icon_state(False, None, True, True) is None
  assert jetlink_icon_state(False, status(), False, True) is None  # stale modelV2


def test_a_chestnut_keeps_its_own_icon():
  assert jetlink_icon_state(True, status(), True, True) is None
