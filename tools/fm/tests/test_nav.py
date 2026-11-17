import numpy as np

import nav


def car(rows):
  return np.array(rows, dtype=float)


class TestBlinks:
  def test_events_and_sides(self):
    cs = car([[0, 5, 0, 0], [1, 5, 1, 0], [3, 5, 0, 0], [5, 5, 0, 1], [6, 5, 0, 0]])
    assert nav.blinks(cs) == [(1.0, 3.0, -1), (5.0, 6.0, 1)]

  def test_short_gap_is_one_blinker(self):
    cs = car([[0, 5, 1, 0], [2, 5, 0, 0], [2.5, 5, 1, 0], [4, 5, 0, 0], [9, 5, 1, 0], [10, 5, 0, 0]])
    assert nav.blinks(cs) == [(0.0, 4.0, -1), (9.0, 10.0, -1)]

  def test_on_at_both_ends(self):
    cs = car([[0, 5, 0, 1], [1, 5, 0, 1], [2, 5, 0, 1]])
    assert nav.blinks(cs) == [(0.0, 2.0, 1)]


class TestTurns:
  def drive(self, rate_deg, seconds, speed=5.0):
    t = np.arange(0, 20, 0.05)
    yaw = np.where((t >= 5) & (t < 5 + seconds), np.radians(rate_deg), 0.0)
    return np.column_stack([t, yaw]), car([[0, speed, 0, 0], [20, speed, 0, 0]])

  def test_right_turn(self):
    turns = nav.find_turns(*self.drive(30, 3))
    assert len(turns) == 1
    turn_in, end, degrees, speed = turns[0]
    assert 5.0 <= turn_in <= 5.6 and 7.8 <= end <= 8.6 and 80 < degrees < 95 and speed == 5.0

  def test_left_turn_is_negative(self):
    assert nav.find_turns(*self.drive(-30, 3))[0][2] < -80

  def test_a_bend_and_a_fast_curve_are_not_turns(self):
    assert nav.find_turns(*self.drive(10, 3)) == []  # 30 deg
    assert nav.find_turns(*self.drive(30, 3, speed=20.0)) == []  # 45 mph


class TestPick:
  def test_empty_and_first(self):
    from openpilot.sunnypilot.fm_nav.roads import Turn
    assert not nav._pick([]).valid
    p = nav._pick([Turn(30.0, 90.0, "Oak Lane", "residential", False), Turn(55.0, 88.0, "Elm Road", "residential", False)])
    assert (p.valid, p.name, p.count, p.nextGap) == (True, "Oak Lane", 2, 25.0)
