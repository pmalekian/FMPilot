from openpilot.selfdrive.ui.sunnypilot.onroad.fm_torque_readout import make_readout


def test_not_shown_without_a_limit():
  assert make_readout(100, 0, 0, True, True) is None


def test_off_is_grey_dashes():
  r = make_readout(0, 481, 530, False, False)
  assert (r.sent, r.cap, r.max_torque, r.accepted, r.state, r.direction) == ("--", "481", "530", None, 'off', 0)


def test_direction_left_positive():
  assert make_readout(120, 481, 530, True, True).direction == -1
  r = make_readout(-457, 481, 530, True, True)
  assert (r.direction, r.sent) == (1, "457")
  assert make_readout(0, 481, 530, True, True).direction == 0


def test_raw_value_not_capped():
  assert make_readout(577.6, 530, 530, True, True).sent == "578"


def test_states():
  assert make_readout(300, 481, 530, True, True).state == 'normal'
  assert make_readout(457, 481, 530, True, True).state == 'at_cap'
  r = make_readout(457, 481, 530, False, True)
  assert (r.state, r.accepted) == ('refused', False)
