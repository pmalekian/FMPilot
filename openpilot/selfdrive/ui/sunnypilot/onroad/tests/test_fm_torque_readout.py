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


class TestCpuReadout:
  USAGE = [40, 70, 55, 80, 99, 60, 74, 30]  # core 4 at 99

  def test_average_of_the_other_cores(self):
    from openpilot.selfdrive.ui.sunnypilot.onroad.fm_torque_readout import READOUT_CPU_AVG, make_cpu_readout
    text, busy = make_cpu_readout(READOUT_CPU_AVG, self.USAGE, [61.2, 67.6, 0.0], 50.0)
    assert text == "4: 99% / A: 58% / 68°C" and busy  # (40+70+55+80+60+74+30)/7 = 58.4

  def test_busiest_other_core(self):
    from openpilot.selfdrive.ui.sunnypilot.onroad.fm_torque_readout import READOUT_CPU_BUSIEST, make_cpu_readout
    text, busy = make_cpu_readout(READOUT_CPU_BUSIEST, [10, 20, 30, 40, 50, 60, 70, 80], [55.0], 0.0)
    assert text == "4: 50% / B: 80% / 55°C" and not busy

  def test_no_cpu_temps_uses_max_temp_and_short_lists(self):
    from openpilot.selfdrive.ui.sunnypilot.onroad.fm_torque_readout import READOUT_CPU_AVG, make_cpu_readout
    assert make_cpu_readout(READOUT_CPU_AVG, self.USAGE, [], 71.4)[0].endswith("/ 71°C")
    assert make_cpu_readout(READOUT_CPU_AVG, self.USAGE, [], 0.0)[0].endswith("/ --°C")
    assert make_cpu_readout(READOUT_CPU_AVG, [1, 2, 3], [50.0], 0.0) is None  # no core 4 reported

  def test_mode_parsing(self):
    from openpilot.selfdrive.ui.sunnypilot.onroad.fm_torque_readout import parse_readout_mode
    assert [parse_readout_mode(v) for v in (0, 1, 2, 3, "2", None, 7, "x")] == [0, 1, 2, 3, 2, 1, 1, 1]
