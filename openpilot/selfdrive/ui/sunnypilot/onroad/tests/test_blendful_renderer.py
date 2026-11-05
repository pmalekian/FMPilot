"""FM Pilot: Blendful's on-screen lane center line (selfdrive/ui/sunnypilot/onroad/blendful.py)."""
import numpy as np
import pytest

pytest.importorskip("pyray")

from openpilot.selfdrive.ui.sunnypilot.onroad.blendful import BlendfulRenderer, center_line
from openpilot.selfdrive.ui.ui_state import ui_state


def _lines(left_y=-1.8, right_y=1.8, n=33):
  x = np.linspace(0, 100, n)
  left = np.stack([x, np.full(n, left_y), np.zeros(n)], axis=1).astype(np.float32)
  right = np.stack([x, np.full(n, right_y), np.zeros(n)], axis=1).astype(np.float32)
  return left, right


def test_center_is_midway_with_equal_lines():
  left, right = _lines(-1.5, 2.1)
  c = center_line(left, right, [1.0, 0.1, 0.1, 1.0], 0.9, 0.9, 3.6)
  np.testing.assert_allclose(c[:, 1], 0.3, atol=1e-5)
  np.testing.assert_allclose(c[:, 0], left[:, 0])


def test_center_leans_on_the_surer_line():
  # left line sure, right line vague: center sits half a lane from the left line
  left, right = _lines(-1.5, 3.0)
  c = center_line(left, right, [1.0, 0.05, 0.6, 1.0], 0.9, 0.2, 3.6)
  assert abs(c[0, 1] - (-1.5 + 1.8)) < 0.05


def test_nothing_drawn_unless_blendful_selected():
  r = BlendfulRenderer(rect_relative=True)
  r._center_points = np.ones((4, 2), dtype=np.float32)
  old = ui_state.blendful_enabled
  try:
    ui_state.blendful_enabled = False
    r.update(None, 10)  # returns before touching the renderer
    assert r._center_points.size == 0
  finally:
    ui_state.blendful_enabled = old
