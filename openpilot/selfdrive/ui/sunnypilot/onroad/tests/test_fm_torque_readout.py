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


class TestLeadReadout:
  NONE, RADAR, CAMERA = 0, 1, 2

  def frame(self, st, now, prob, source, low_speed=False, ghost=False, drops=0, early=0):
    return st.update(now, prob, source, low_speed, ghost, drops, early)

  def test_lead_mode_parses_and_torque_stays_default(self):
    from openpilot.selfdrive.ui.sunnypilot.onroad.fm_torque_readout import READOUT_LEAD, READOUT_TORQUE, parse_readout_mode
    assert READOUT_LEAD == 4 and parse_readout_mode(4) == READOUT_LEAD and parse_readout_mode("4") == READOUT_LEAD
    assert parse_readout_mode(None) == READOUT_TORQUE and parse_readout_mode(5) == READOUT_TORQUE

  def test_note_examples(self):
    from openpilot.selfdrive.ui.sunnypilot.onroad.fm_torque_readout import LeadReadoutState
    cases = [  # (prob, source, early, drops) -> text, number color (the build note's table)
      ((0.18, self.NONE, 0, 0), "18% – E0 G0", 'gray'),
      ((0.38, self.CAMERA, 0, 0), "38% C E0 G0", 'cyan'),
      ((0.40, self.RADAR, 1, 0), "40% R E1 G0", 'purple'),
      ((0.62, self.CAMERA, 1, 0), "62% C E1 G0", 'white'),
      ((0.87, self.RADAR, 1, 0), "87% R E1 G0", 'white'),
    ]
    for (prob, source, early, drops), text, color in cases:
      r = self.frame(LeadReadoutState(), 10.0, prob, source, early=early, drops=drops)
      assert (r.text, r.prob_color) == (text, color)

  def test_colors(self):
    from openpilot.selfdrive.ui.sunnypilot.onroad.fm_torque_readout import lead_prob_color
    assert lead_prob_color(0.499, self.RADAR, False) == 'purple'
    assert lead_prob_color(0.499, self.CAMERA, False) == 'cyan'
    assert lead_prob_color(0.5, self.RADAR, False) == 'white' and lead_prob_color(0.5, self.CAMERA, False) == 'white'
    assert lead_prob_color(0.9, self.NONE, False) == 'gray'  # no lead counted
    assert lead_prob_color(0.1, self.RADAR, True) == 'gray'  # low-speed radar path: R with a gray number
    assert lead_prob_color(0.9, self.RADAR, True) == 'gray'

  def test_low_speed_radar_path_shows_r_gray(self):
    from openpilot.selfdrive.ui.sunnypilot.onroad.fm_torque_readout import LeadReadoutState
    r = self.frame(LeadReadoutState(), 1.0, 0.12, self.RADAR, low_speed=True)
    assert r.text == "12% R E0 G0" and r.prob_color == 'gray' and r.letter_color == 'white'

  def test_ghost_flashes_x_for_two_seconds(self):
    from openpilot.selfdrive.ui.sunnypilot.onroad.fm_torque_readout import LeadReadoutState
    st = LeadReadoutState()
    r = self.frame(st, 5.0, 0.74, self.CAMERA, ghost=True, drops=1, early=1)
    assert r.text == "74% X E1 G1" and r.letter_color == 'red' and r.prob_color == 'white'
    assert self.frame(st, 5.3, 0.74, self.CAMERA, drops=1, early=1).letter_color == 'red_dim'  # blinking
    assert self.frame(st, 5.6, 0.74, self.CAMERA, drops=1, early=1).letter_color == 'red'
    assert self.frame(st, 6.9, 0.74, self.CAMERA, drops=1, early=1).letter == 'X'
    r = self.frame(st, 7.01, 0.74, self.CAMERA, drops=1, early=1)
    assert r.letter == 'C' and r.letter_color == 'white'

  def test_ghost_held_while_dropping_and_missed_frames_still_flash(self):
    from openpilot.selfdrive.ui.sunnypilot.onroad.fm_torque_readout import LeadReadoutState
    st = LeadReadoutState()
    self.frame(st, 0.0, 0.7, self.RADAR, drops=0)
    for t in (1.0, 2.0, 3.0):  # the guard keeps dropping the point
      self.frame(st, t, 0.7, self.CAMERA, ghost=True, drops=1)
    assert self.frame(st, 4.9, 0.7, self.CAMERA, drops=1).letter == 'X'
    assert self.frame(st, 5.1, 0.7, self.CAMERA, drops=1).letter == 'C'
    # a drop the UI never saw a frame of: the count went up
    assert self.frame(st, 6.0, 0.7, self.RADAR, drops=2).letter == 'X'

  def test_new_drive_count_reset_does_not_flash(self):
    from openpilot.selfdrive.ui.sunnypilot.onroad.fm_torque_readout import LeadReadoutState
    st = LeadReadoutState()
    self.frame(st, 0.0, 0.7, self.RADAR, drops=3)
    r = self.frame(st, 100.0, 0.2, self.NONE, drops=0)
    assert r.letter == '–' and r.counts == "E0 G0"

  def test_prob_text_edges(self):
    from openpilot.selfdrive.ui.sunnypilot.onroad.fm_torque_readout import lead_prob_text
    assert [lead_prob_text(p) for p in (0.0, 0.004, 0.29, 0.4999, 0.5, 1.0, 1.2, -0.1, float('nan'))] == \
           ["0%", "0%", "29%", "49%", "50%", "100%", "100%", "0%", "--%"]
