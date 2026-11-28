"""Actual planner selection/publication with a deterministic MPC output boundary.

FM Pilot: the boost step is the Accel Boost picker now (fm_accel_boost_mode.py): "Acceleration Control on" is picker mode
4 (AP Accel Control), "off" mode 0 (Off). The base's AccelBoostSP / AccelerationControl selection tests became picker
tests (one boost at a time; the toggles are not read), and every mode's published accelBoost is checked.

The generated MPC optimizer and unrelated SP controllers are seams. The production
planner update, cruise limiter, plan interpolation, the Accel Boost picker, AccelerationControl,
min-selection, clipping and both publishers execute unchanged. No road/replay claim.
"""
from types import SimpleNamespace

import numpy as np
import pytest

from opendbc.car import structs
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.cereal import log, custom
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner, T_IDXS_MPC
from openpilot.sunnypilot.selfdrive.controls.lib.accel_boost import ACCEL_BOOST_MAX as SUNNY_MAX
from openpilot.sunnypilot.selfdrive.controls.lib.acceleration_control import ACCELERATION_CONTROL_BOOST
from openpilot.sunnypilot.selfdrive.controls.lib.fm_accel_boost_mode import (
  MODES, MODE_AP_ACCEL_CONTROL, MODE_OFF, MODE_OP_PERSONALITY, MODE_SUNNY, PARAM_MODE, READ_FRAMES, FmAccelBoostMode,
)
from openpilot.sunnypilot.selfdrive.controls.lib.fm_op_personality_boost import ACCEL_BOOST as OP_MAX
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerSP
from openpilot.sunnypilot.selfdrive.controls.lib.tests.test_acceleration_control import _sm

Source = log.LongitudinalPlan.LongitudinalPlanSource
SP_UPDATE = LongitudinalPlannerSP.update  # _planner stubs it; the boost selection tests need the real one


class DeterministicMPC:
  """Only the solver result boundary is controlled; interpolation is production code."""
  crash_cnt = 0
  solve_time = 0.001
  source = Source.lead0

  def __init__(self, accel):
    self.v_solution = 25 + accel * T_IDXS_MPC
    self.a_solution = np.full(len(T_IDXS_MPC), accel)
    self.j_solution = np.zeros(len(T_IDXS_MPC) - 1)

  def set_weights(self, prev_accel_constraint, personality):
    pass

  def set_cur_state(self, v, a):
    pass

  def update(self, radar, personality):
    pass


class Messages(dict):
  logMonoTime = {'modelV2': 0}

  def all_checks(self, service_list=None):
    return True


class Publisher:
  def __init__(self):
    self.messages = {}

  def send(self, service, message):
    # Real binary serialization at the parent -> SP publisher boundary.
    with log.Event.from_bytes(message.to_bytes()) as decoded:
      self.messages[service] = decoded.to_dict()[service]


class ModeParams:
  def __init__(self, mode):
    self.mode = mode

  def get(self, key, return_default=False):
    assert key == PARAM_MODE
    return self.mode


def _planner(monkeypatch, raw, cruise, lead, enabled=True, mode=None):
  # Isolate unrelated controllers without replacing update's selection algorithms.
  monkeypatch.setattr(LongitudinalPlannerSP, 'update', lambda self, sm: None)
  monkeypatch.setattr(LongitudinalPlannerSP, 'update_targets',
                      lambda self, sm, v_ego, a_ego, v_cruise: (v_cruise, a_ego))
  planner = object.__new__(LongitudinalPlanner)
  planner.CP = structs.CarParams(openpilotLongitudinalControl=True, longitudinalActuatorDelay=0.35)
  planner.mpc = DeterministicMPC(lead)
  mode = (MODE_AP_ACCEL_CONTROL if enabled else MODE_OFF) if mode is None else mode
  planner.accel_boost = FmAccelBoostMode(planner.CP, ModeParams(mode))
  # curve control's lateral throttle interlock, inactive
  planner.scc = SimpleNamespace(governor=SimpleNamespace(throttle_scale=lambda: 1.0))
  planner.dt = DT_MDL
  planner.v_desired_filter = FirstOrderFilter(25, 2, DT_MDL)
  planner.output_a_target = 0
  planner.a_cruise = cruise
  sm = Messages(_sm(v_ego=25, desired_accel=raw))
  sm['carState'].vCruise = (25 + cruise) * 3.6
  sm['controlsState'] = log.ControlsState.new_message(longControlState='pid')
  sm['radarState'] = log.RadarState.new_message(leadOne={'present': True})
  return planner, sm


def _publication_fields(planner):
  # Explicit neutral SP state; publish_longitudinal_plan_sp runs its real body.
  default = custom.LongitudinalPlanSP.new_message()
  planner.source = custom.LongitudinalPlanSP.LongitudinalPlanSource.cruise
  planner.output_v_target = 30
  planner.events_sp = SimpleNamespace(to_msg=list)
  vision = SimpleNamespace(state=default.smartCruiseControl.vision.state, output_v_target=30, output_a_target=0,
                           current_lat_acc=0, max_pred_lat_acc=0, is_enabled=False, is_active=False)
  map_source = SimpleNamespace(state=default.smartCruiseControl.map.state, output_v_target=30, output_a_target=0,
                               is_enabled=False, is_active=False)
  curve = SimpleNamespace(state=default.smartCruiseControl.curve.state, output_v_target=30, output_a_target=0,
                          lat_accel_target=0, is_enabled=False, is_active=False)
  planner.scc = SimpleNamespace(vision=vision, map=map_source, curve=curve,
                                governor=SimpleNamespace(load=0, ceiling=1, is_active=False,
                                                         throttle_scale=lambda: 1.0))
  planner.resolver = SimpleNamespace(speed_limit=0, speed_limit_last=0, speed_limit_final=0, speed_limit_final_last=0,
                                     speed_limit_valid=False, speed_limit_last_valid=False, speed_limit_offset=0,
                                     distance=0, source=default.speedLimit.resolver.source)
  planner.sla = SimpleNamespace(state=default.speedLimit.assist.state, is_enabled=False, is_active=False,
                                output_v_target=30, output_a_target=0)
  planner.e2e_alerts_helper = SimpleNamespace(green_light_alert=False, lead_depart_alert=False)


@pytest.mark.parametrize('raw,cruise,lead,enabled,expected,source', [
  (0.1, 1, 1, True, 0.3, Source.e2e),
  (0.1, 0.1, 1, True, 0.1, Source.cruise),
  (0.1, 1, 0.05, True, 0.05, Source.lead0),
  # Production tie order: MPC then cruise then model.
  (0, 0, 0, True, 0, Source.lead0),
  # 94.5 kph and 26.25 m/s are exactly representable, so this is a real tie.
  (1.25, 1.25, 2, False, 1.25, Source.cruise),
  (0.5, 1, 0.5, False, 0.5, Source.lead0),
  (4, 4, 4, True, ACCEL_MAX, Source.cruise),
  (-5, 1, 1, True, ACCEL_MIN, Source.e2e),
])
def test_actual_planner_winners_ties_and_limits(monkeypatch, raw, cruise, lead, enabled, expected, source):
  planner, sm = _planner(monkeypatch, raw, cruise, lead, enabled)
  planner.update(sm)
  assert planner.output_a_target == pytest.approx(expected, abs=1e-6)
  assert planner.mpc.source == source


@pytest.mark.parametrize('cruise,lead', [(0.1, 1), (1, 0.05)])
def test_parent_to_sp_requested_telemetry_and_disable(monkeypatch, cruise, lead):
  planner, sm = _planner(monkeypatch, 0.1, cruise, lead)
  planner.update(sm)
  # Real capnp vCruise is float32; its kph -> m/s conversion has sub-micro precision.
  assert planner.output_a_target <= 0.1 + 1e-6
  _publication_fields(planner)
  pm = Publisher()
  planner.publish(sm, pm)
  shared = pm.messages['longitudinalPlan']
  fork = pm.messages['longitudinalPlanSP']
  assert 'accelerationControlBoost' not in shared
  assert fork['accelerationControlBoostRequested'] == pytest.approx(0.2)
  assert fork['accelerationControlBoostRequested'] > shared['aTarget']
  # Same object, positive preceding request: disabled update must publish zero.
  sm['selfdriveState'].enabled = False
  planner.update(sm)
  planner.publish(sm, pm)
  assert pm.messages['longitudinalPlanSP']['accelerationControlBoostRequested'] == 0


def _planner_with_mode(monkeypatch, mode, personality=log.LongitudinalPersonality.standard):
  """A whole planner frame with the real SP update and a model that wants every boost (no reason to slow, 25 m/s)."""
  planner, sm = _planner(monkeypatch, 0.1, 1, 1, mode=mode)
  monkeypatch.setattr(LongitudinalPlannerSP, 'update', SP_UPDATE)
  planner.events_sp = SimpleNamespace(clear=lambda: None)
  planner.e2e_alerts_helper = SimpleNamespace(update=lambda sm, events: None)
  probs = [0.0, 0.05, 0.0, 0.0, 0.0, 0.0]
  sm['modelV2'] = log.ModelDataV2.new_message(action={'desiredAcceleration': 0.1},
                                              meta={'disengagePredictions': {'brakePressProbs': probs, 'gasPressProbs': probs}})
  sm['selfdriveState'].personality = personality
  return planner, sm


def _publish(planner, sm):
  _publication_fields(planner)
  pm = Publisher()
  planner.publish(sm, pm)
  return pm.messages


def test_ap_accel_control_alone_in_the_whole_frame(monkeypatch):
  planner, sm = _planner_with_mode(monkeypatch, MODE_AP_ACCEL_CONTROL)
  for _ in range(60):
    planner.update(sm)
    # the model's 0.1 plus Acceleration Control's 0.2 for Standard, and nothing from any other boost
    assert planner.output_a_target <= 0.3 + 1e-6
  assert planner.output_a_target == pytest.approx(0.3, abs=1e-6)
  msgs = _publish(planner, sm)
  assert msgs['longitudinalPlan']['accelBoost'] == pytest.approx(0.2, abs=1e-6)  # the bar's field: what it added
  assert msgs['longitudinalPlanSP']['accelerationControlBoostRequested'] == pytest.approx(0.2)
  assert msgs['fmPlan']['accelBoostMode'] == 'apAccelControl'


def test_sunny_boost_alone_in_the_whole_frame(monkeypatch):
  planner, sm = _planner_with_mode(monkeypatch, MODE_SUNNY)
  for _ in range(60):
    planner.update(sm)
  # Standard's 0.25 m/s^2 on the model's 0.1, reached at 0.2 m/s^3 well inside 3 s
  assert planner.output_a_target == pytest.approx(0.35, abs=1e-6)
  msgs = _publish(planner, sm)
  assert msgs['longitudinalPlan']['accelBoost'] == pytest.approx(SUNNY_MAX[1], abs=1e-6)
  assert msgs['longitudinalPlanSP']['accelerationControlBoostRequested'] == 0
  assert msgs['fmPlan']['accelBoostMode'] == 'sunnyBoost'


def test_switching_from_ap_to_the_removed_gas_mode_mid_drive_drops_the_lift(monkeypatch):
  planner, sm = _planner_with_mode(monkeypatch, MODE_AP_ACCEL_CONTROL)
  for _ in range(5):
    planner.update(sm)
  planner.accel_boost.params = ModeParams(1)  # 1 was Gas (Gas Override Boost), removed: reads as Off
  for _ in range(READ_FRAMES + 1):
    planner.update(sm)
  assert planner.accel_boost.mode == MODE_OFF
  assert planner.output_a_target == pytest.approx(0.1, abs=1e-6)


@pytest.mark.parametrize('mode', MODES)
def test_boost_bar_field_for_every_mode(monkeypatch, mode):
  """longitudinalPlan.accelBoost is what the picked mode added to the model's target this frame, in every mode."""
  planner, sm = _planner_with_mode(monkeypatch, mode, personality=log.LongitudinalPersonality.aggressive)
  sm['carState'].vCruise = 40 * 3.6  # far above: the cruise ramp leaves room for EVP Set Speed
  planner.a_cruise = 2.0
  for frame in range(200):
    sm['carState'].gasPressed = 20 <= frame < 100  # no mode needs it now (Gas Override Boost is gone); kept as a disturbance
    planner.update(sm)
  msgs = _publish(planner, sm)
  boost = msgs['longitudinalPlan']['accelBoost']
  assert boost == pytest.approx(planner.accel_boost.applied)
  expected = {MODE_OFF: 0.0, MODE_SUNNY: SUNNY_MAX[0], MODE_AP_ACCEL_CONTROL: ACCELERATION_CONTROL_BOOST[0], MODE_OP_PERSONALITY: OP_MAX[0]}
  if mode in expected:
    assert boost == pytest.approx(expected[mode], abs=1e-6)
  else:  # EVP Set Speed: toward its 0.7 cap
    assert boost > 0.0
  names = {v: k for k, v in custom.FmPlan.AccelBoostMode.schema.enumerants.items()}
  assert msgs['fmPlan']['accelBoostMode'] == names[mode]
