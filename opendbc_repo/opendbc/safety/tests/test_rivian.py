#!/usr/bin/env python3
import unittest
from types import SimpleNamespace

import numpy as np

from opendbc.car import structs
from opendbc.car.rivian.carcontroller import CarController
from opendbc.car.structs import CarParams
from opendbc.safety.tests.common import CANPackerSafety
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.sunnypilot.car.rivian.values import RivianFlagsSP, RivianSafetyFlagsSP, max_torque_level
import opendbc.safety.tests.common as common
from opendbc.car.rivian.values import CarControllerParams, RivianSafetyFlags

class TestRivianSafetyBase(common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest, common.SteerRequestCutSafetyTest,
                           common.LongitudinalAccelSafetyTest, common.VehicleSpeedSafetyTest):

  DBC = "rivian_primary_actuator"
  SAFETY_MODEL = CarParams.SafetyModel.rivian

  TX_MSGS = [[0x120, 0], [0x321, 2], [0x162, 2]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x120,), 2: (0x321, 0x162)}
  FWD_BLACKLISTED_ADDRS = {0: [0x321, 0x162], 2: [0x120]}

  MAX_TORQUE_LOOKUP = [9, 25, 27], [385, 295, 275]
  DYNAMIC_MAX_TORQUE = True
  MAX_RATE_UP = 3
  MAX_RATE_DOWN = 5

  MAX_RT_DELTA = 125

  DRIVER_TORQUE_ALLOWANCE = 100
  DRIVER_TORQUE_FACTOR = 2

  MIN_VALID_STEERING_FRAMES = 89
  MAX_INVALID_STEERING_FRAMES = 2

  cnt_speed = 0
  cnt_speed_2 = 0
  cnt_torque = 0
  cnt_brake = 0
  cnt_pcm = 0

  RX_QUALITY_FLAGS = {
    "ESP_Status": {"ESP_Vehicle_Speed_Q": (1,)},
    "VDM_PropStatus": {"VDM_VehicleSpeedQ": (1,)},
    "iBESP2": {"iBESP2_BrakePedalApplied_Q": (1,)},
  }

  def _torque_driver_msg(self, torque, counter=None):
    values = {"EPAS_TorsionBarTorque": torque / 100.0, "EPAS_SystemStatus_Counter": self.cnt_torque % 15 if counter is None else counter}
    self.__class__.cnt_torque += 1
    return self.packer.make_can_msg_safety("EPAS_SystemStatus", 0, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"ACM_lkaStrToqReq": torque, "ACM_lkaActToi": steer_req}
    return self.packer.make_can_msg_safety("ACM_lkaHbaCmd", 0, values)

  def _speed_msg(self, speed):
    values = {"ESP_Vehicle_Speed": speed * 3.6, "ESP_Status_Counter": self.cnt_speed % 15, "ESP_Vehicle_Speed_Q": 1}
    self.__class__.cnt_speed += 1
    return self.packer.make_can_msg_safety("ESP_Status", 0, values)

  def _speed_msg_2(self, speed):
    # Rivian has a dynamic max torque limit based on speed, so it checks two sources
    return self._user_gas_msg(0, speed)

  def _user_brake_msg(self, brake, quality_flag=True, counter=None):
    values = {"iBESP2_BrakePedalApplied": brake, "iBESP2_BrakePedalApplied_Q": 1 if quality_flag else 2,
              "iBESP2_AliveCounter": self.cnt_brake % 15 if counter is None else counter}
    self.__class__.cnt_brake += 1
    return self.packer.make_can_msg_safety("iBESP2", 0, values)

  def _user_gas_msg(self, gas, speed=0):
    values = {"VDM_AcceleratorPedalPosition": gas, "VDM_VehicleSpeed": speed * 3.6,
              "VDM_PropStatus_Counter": self.cnt_speed_2 % 15, "VDM_VehicleSpeedQ": 1}
    self.__class__.cnt_speed_2 += 1
    return self.packer.make_can_msg_safety("VDM_PropStatus", 0, values)

  def _pcm_status_msg(self, enable, counter=None):
    values = {"ACM_FeatureStatus": enable, "ACM_Unkown1": 1, "ACM_Status_Counter": self.cnt_pcm % 15 if counter is None else counter}
    self.__class__.cnt_pcm += 1
    return self.packer.make_can_msg_safety("ACM_Status", 2, values)

  def _accel_msg(self, accel: float):
    values = {"ACM_AccelerationRequest": accel}
    return self.packer.make_can_msg_safety("ACM_longitudinalRequest", 0, values)

  def _torque_loop_setup(self, angle=150.0, speed=11.4, timer_offset_frames=0):
    """the real CarController driving the panda model frame by frame (10 ms) at a high wheel angle, where it
    blips the TOI request about every 0.9 s"""
    CP_SP = structs.CarParamsSP()
    if max_torque_level(self.MAX_TORQUE_SETTING) >= 1:  # FM Pilot "Max Torque": the car side of the same setting
      CP_SP.flags |= RivianFlagsSP.HIGH_TORQUE_PANDA.value
      CP_SP.safetyParam = self._max_torque_param_sp()
    self.controller = CarController({"pt": "rivian_primary_actuator"}, structs.CarParams(), CP_SP)
    out = structs.CarState()
    out.vEgoRaw = speed
    out.steeringAngleDeg = angle
    self.cs = SimpleNamespace(out=out, acm_lka_hba_cmd={"ACM_hbaSysState": 0, "ACM_hbaLamp": 0, "ACM_hbaOnOffState": 0,
                                                        "ACM_slifOnOffState": 0},
                              sccm_wheel_touch={"SCCM_WheelTouch_Counter": 0, "SCCM_WheelTouch_HandsOn": 0,
                                                "SCCM_WheelTouch_CapacitiveValue": 0, "SCCM_WheelTouch_Calibration": 100,
                                                "SCCM_WheelTouch_ResistiveValue": 0},
                              vdm_adas_status=[])
    self.safety.init_tests()
    self._reset_safety_hooks()
    self.safety.set_controls_allowed(True)
    # the panda's 250 ms real-time interval starts at its first torque message. Send one now and start the
    # controller timer_offset_frames later so the interval timer is out of step with the controller's blip cycle
    self.safety.set_timer(int(1e6))
    self.assertTrue(self._tx(self._torque_cmd_msg(0, steer_req=1)))
    self.frame = max(1, timer_offset_frames)
    self.rx_speed = speed
    for _ in range(10):
      self._torque_loop_rx()

  def _torque_loop_rx(self):
    self._rx(self._speed_msg(self.rx_speed))
    self._rx(self._speed_msg_2(self.rx_speed))
    self._rx(self._torque_driver_msg(0))

  def _torque_loop_frame(self, demand):
    """one 10 ms controller frame sent through the panda; returns whether the panda let the torque message through"""
    self.safety.set_timer(int(1e6) + self.frame * 10000)
    self.frame += 1
    cc = structs.CarControl()
    cc.latActive = True
    cc.actuators.torque = demand
    actuators, can_sends = self.controller.update(cc.as_reader(), structs.CarControlSP(), self.cs, 0)
    self.last_sent_torque = actuators.torqueOutputCan
    ok = True
    for addr, dat, bus in can_sends:
      if addr == 0x120:
        ok = self._tx(libsafety_py.make_CANPacket(addr, bus, dat))
    self._torque_loop_rx()
    return ok

  def test_toi_fault_release_is_never_refused(self):
    """Releasing the TOI request to clear a latched EPAS ToiFlt (torque 0, request low) must pass the panda wherever it
    lands: mid-ramp, next to a TOI blip, at any phase of the panda's real-time interval. So must the resume, which
    starts from the frozen torque (route 2bba20cd6136cc27/0000007a--9d80d483b8: the latch lasted 59 s and 75 s)."""
    blocked = []
    no_release = []
    for sign in (1.0, -1.0):
      for offset in range(0, 26, 5):  # panda interval timer against the blip cycle
        for start in range(0, 100, 7):  # where in the ramp / blip cycle the latch lands
          self._torque_loop_setup(timer_offset_frames=offset)
          released_at = None
          for i in range(200):
            # like the EPAS: latched from start until a couple of frames after the first release
            self.cs.toi_fault = start <= i and (released_at is None or i < released_at + 2)
            if not self._torque_loop_frame(sign):
              blocked.append((sign, offset, start, i))
              break
            if released_at is None and self.controller.toi_clear_cooldown > 0:
              released_at = i
          if released_at is None:
            no_release.append((sign, offset, start))
    self.assertEqual(blocked, [], f"panda refused around a ToiFlt release at (sign, timer offset, latch start, frame): {blocked[:6]}")
    self.assertEqual(no_release, [], f"latch was never released: {no_release[:6]}")

  def test_toi_dropout_release_is_never_refused(self):
    """The same release, sent because the EPAS stopped taking torque with no ToiFlt (route
    2bba20cd6136cc27/000000ea--8e6d69adcc: ignored for 231 s), must pass the panda wherever it lands, and so must the resume."""
    blocked = []
    no_release = []
    for sign in (1.0, -1.0):
      for offset in range(0, 26, 5):  # panda interval timer against the blip cycle
        for start in range(0, 100, 7):  # where in the ramp / blip cycle the dropout lands
          self._torque_loop_setup(timer_offset_frames=offset)
          released_at = None
          for i in range(250):
            # like the EPAS: stops taking torque at start and only takes it again a couple of frames after a release
            self.cs.toi_active = i < start or (released_at is not None and i >= released_at + 2)
            if not self._torque_loop_frame(sign):
              blocked.append((sign, offset, start, i))
              break
            if released_at is None and i >= start and self.controller.toi_clear_cooldown > 0:
              released_at = i
          if released_at is None:
            no_release.append((sign, offset, start))
    self.assertEqual(blocked, [], f"panda refused around a dropout release at (sign, timer offset, dropout start, frame): {blocked[:6]}")
    self.assertEqual(no_release, [], f"dropout was never released: {no_release[:6]}")

  def test_torque_ramp_through_blip_is_never_blocked(self):
    """A full-rate torque ramp that straddles the TOI blip must never be refused. The panda's real-time check
    only refreshes its reference every 250 ms and a blip frame restarts that timer without refreshing the
    reference, so the reference can be about 0.5 s old. A ramp at the maximum rate (3 counts per frame) then
    climbs more than the allowed 125 counts from it, the panda refuses the frame and zeroes its torque memory,
    and every following frame is refused too (route 4440a486580ed7c6/00000112 seg 15, 5 s lockout)."""
    blocked = []
    for sign in (1.0, -1.0):
      for offset in range(0, 26, 2 if sign > 0 else 5):  # sweep the panda interval timer against the blip cycle ...
        for start in range(0, 100, 4 if sign > 0 else 8):  # ... and the start of the ramp across the cycle
          self._torque_loop_setup(timer_offset_frames=offset)
          for _ in range(start):
            self._torque_loop_frame(0.0)
          for i in range(200):
            if not self._torque_loop_frame(sign):
              blocked.append((sign, offset, start, i))
              break
    self.assertEqual(blocked, [], f"panda refused the controller's torque at (sign, timer offset, demand start, frame): {blocked[:6]}")

  def _fm_lookup(self):
    """the most the car code may send at each speed: AP's curve, or with Max Torque above 385 the larger of it and the
    high curve (CarControllerParams.steer_max_lookup_high)"""
    base = CarControllerParams.STEER_MAX_LOOKUP
    if max_torque_level(self.MAX_TORQUE_SETTING) < 1:
      return lambda v: round(float(np.interp(v, *base)))
    high = CarControllerParams.steer_max_lookup_high(self.MAX_TORQUE_LOOKUP[1][0])
    return lambda v: max(round(float(np.interp(v, *base))), round(float(np.interp(v, *high))))

  def test_fm_demand_past_cap_is_never_refused(self):
    """FM Pilot: openpilot may ask past 1.0 (Headroom). Whatever it asks, the controller must stay inside the panda's
    limit at every speed and reach the cap: AP's curve at 385, the higher curve above it."""
    cap_at = self._fm_lookup()
    for speed in self.FM_SPEEDS:
      for sign in (1.0, -1.0):
        self._torque_loop_setup(angle=30.0, speed=speed)
        peak = 0
        for i in range(400):
          self.assertTrue(self._torque_loop_frame(sign * 1.6), f"refused at {speed} m/s, sign {sign}, frame {i}")
          peak = max(peak, abs(self.last_sent_torque))
        self.assertLessEqual(peak, cap_at(speed), f"{speed} m/s")
        self.assertGreaterEqual(peak, cap_at(speed) - 3, f"{speed} m/s: never reached the cap")

  def test_fm_unit_demand_unchanged(self):
    """FM Pilot: a request of 1.0 sends the same torque whatever Max Torque is (AP's curve)."""
    for speed in (4.0, 9.0, 17.0, 30.0):
      self._torque_loop_setup(angle=30.0, speed=speed)
      for _ in range(200):
        self._torque_loop_frame(1.0)
      self.assertEqual(abs(self.last_sent_torque), round(float(np.interp(speed, *CarControllerParams.STEER_MAX_LOOKUP))))

  def test_fm_high_angle_cap(self):
    """past 90 degrees: AP's 95% cap at 385; the full cap with Max Torque above 385 (Full Torque Past 90°, folded)"""
    cap = self._fm_lookup()(4.0)
    expected = cap if max_torque_level(self.MAX_TORQUE_SETTING) >= 1 else round(cap * 0.95)
    self._torque_loop_setup(angle=150.0, speed=4.0)
    peak = 0
    for i in range(400):
      self.assertTrue(self._torque_loop_frame(1.6), f"refused, frame {i}")
      peak = max(peak, abs(self.last_sent_torque))
    self.assertEqual(peak, expected)

  def test_fm_car_code_ramp(self):
    """5 per frame below 25 mph with Max Torque above 385 (Faster Torque Ramp, folded), else AP's 3; 3 above 25 mph"""
    def frames_to(speed, target):
      self._torque_loop_setup(angle=30.0, speed=speed)
      for i in range(400):
        self._torque_loop_frame(1.6)
        if abs(self.last_sent_torque) >= target:
          return i
      return 400
    low = frames_to(6.0, 300)
    if max_torque_level(self.MAX_TORQUE_SETTING) >= 1:
      self.assertLess(low, 300 / 3 - 15)  # clearly faster than 3 per frame
      self.assertGreaterEqual(low, 300 / 5 - 1)  # never faster than 5 per frame
    else:
      self.assertGreaterEqual(low, 300 / 3 - 1)
    self.assertGreaterEqual(frames_to(12.0, 250), 250 / 3 - 1)  # 27 mph: the normal ramp

  def test_torque_recovers_after_panda_refusal(self):
    """If the panda ever does refuse a frame it zeroes its torque memory. The controller, told the frame was
    refused, must restart from zero instead of repeating a request the panda will keep refusing."""
    self._torque_loop_setup()
    for _ in range(80):
      self.assertTrue(self._torque_loop_frame(0.8))
    # force a refusal: an out-of-range request makes the panda drop its torque memory
    self.safety.set_timer(int(1e6) + self.frame * 10000)
    self.assertFalse(self._tx(self._torque_cmd_msg(self.MAX_TORQUE + 50, steer_req=1)))
    self.cs.torque_tx_refused = True
    results = [self._torque_loop_frame(0.8)]
    self.cs.torque_tx_refused = False
    results += [self._torque_loop_frame(0.8) for _ in range(30)]
    self.assertTrue(all(results), f"controller kept sending requests the panda refuses: {results}")

  def test_wheel_touch(self):
    # For hiding hold wheel alert on engage
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)
      values = {
        "SCCM_WheelTouch_HandsOn": 1 if controls_allowed else 0,
        "SCCM_WheelTouch_CapacitiveValue": 100 if controls_allowed else 0,
        "SCCM_WheelTouch_Calibration": 100,
      }
      self.assertTrue(self._tx(self.packer.make_can_msg_safety("SCCM_WheelTouch", 2, values)))

  cnt_stalk = 0

  def _stalk_msg(self, req, bus=0, counter=None):
    values = {"VDM_UserAdasRequest": req, "VDM_AdasStatus_Counter": self.cnt_stalk % 15 if counter is None else counter}
    self.__class__.cnt_stalk += 1
    return self.packer.make_can_msg_safety("VDM_AdasSts", bus, values)

  # "Use enhanced Rivian MADS". Off, the panda has no MADS button, as in stock sunnypilot.
  ENHANCED_MADS = False
  # FM Pilot "Max Torque" (385 = stock AP: nothing sent)
  MAX_TORQUE_SETTING = 385
  # speeds (m/s) the real controller is run against the panda at; around every breakpoint of both curves
  FM_SPEEDS = (0.5, 4.0, 8.0, 9.0, 12.0, 13.0, 17.0, 21.0, 25.0, 27.0, 33.0)

  def _max_torque_param_sp(self):
    level = max_torque_level(self.MAX_TORQUE_SETTING)
    return (RivianSafetyFlagsSP.HIGH_TORQUE | (level << RivianSafetyFlagsSP.TORQUE_LEVEL_SHIFT)) if level >= 1 else 0

  def _param_sp(self):
    return (RivianSafetyFlagsSP.ENHANCED_MADS if self.ENHANCED_MADS else 0) | self._max_torque_param_sp()

  @property
  def _safety_param_sp(self):
    return self._param_sp()

  def test_pcm_rising_edge_resets_main_heartbeat(self):
    self.assertTrue(self._rx(self._pcm_status_msg(False)))
    for _ in range(2):
      self.safety.set_heartbeat_engaged_mismatches(2)
      self.assertTrue(self._rx(self._pcm_status_msg(True)))
      self.assertTrue(self.safety.get_controls_allowed())
      self.assertEqual(self.safety.get_heartbeat_engaged_mismatches(), 0)
      self.safety.set_heartbeat_engaged_mismatches(2)
      self.assertTrue(self._rx(self._pcm_status_msg(True)))
      self.assertEqual(self.safety.get_heartbeat_engaged_mismatches(), 2)
      self.assertTrue(self._rx(self._pcm_status_msg(False)))
      self.assertFalse(self.safety.get_controls_allowed())

  def test_mads_parameter_reinitialization_and_bus(self):
    for reinitialize in (False, True):
      if reinitialize:
        self.safety.init_tests()
      self._reset_safety_hooks()
      expected_param = self._param_sp()
      self.assertEqual(self.safety.get_current_safety_param_sp(), expected_param)
      for cruise in (False, True):
        for bus in (2, 0):
          self._reset_safety_hooks()
          self.assertTrue(self._rx(self._pcm_status_msg(cruise)))
          self.safety.set_mads_button_press(-1)
          # Generate fresh frames with valid DBC checksum/counter on BOTH buses.
          # The bus-zero positive control proves rejection is not malformed data.
          for _ in range(10):
            msg = self._stalk_msg(1, bus)
            self.assertTrue(self._rx(msg))
          expected = (0 if cruise else 1) if self.ENHANCED_MADS and bus == 0 else -1
          self.assertEqual(self.safety.get_mads_button_press(), expected, (reinitialize, cruise, bus))

  def test_mads_button_gated_on_cruise(self):
    """UP_1 counts as the MADS button only while stock ACC is NOT engaged: with ACC active
    python treats UP_1 as cancel-only, and counting it in the panda desyncs the two MADS
    state machines (root cause of the EPAS AngleControlCntr fault, route c17ea97d.../6 seg 3).
    With enhanced MADS off the stalk is never the MADS button."""
    self.safety.set_mads_button_press(-1)
    for cruise in (False, True):
      self._rx(self._pcm_status_msg(1 if cruise else 0))
      self._rx(self._stalk_msg(1))
      if self.ENHANCED_MADS:
        expected = 0 if cruise else 1  # MADS_BUTTON_NOT_PRESSED / MADS_BUTTON_PRESSED
      else:
        expected = -1  # MADS_BUTTON_UNAVAILABLE
      self.assertEqual(self.safety.get_mads_button_press(), expected, f"cruise={cruise}")
      self._rx(self._stalk_msg(0))

  def test_remaining_rx_checks(self):
    # The DBC defines counters for the remaining monitored RX messages too.
    for make_msg in (self._torque_driver_msg, self._user_brake_msg, self._pcm_status_msg, self._stalk_msg):
      self._reset_safety_hooks()
      self.safety.set_controls_allowed(True)
      for _ in range(10):
        self.assertTrue(self._rx(make_msg(0) if make_msg == self._torque_driver_msg else make_msg(False)))

      msg = make_msg(0) if make_msg == self._torque_driver_msg else make_msg(False)
      msg[0].data[0] ^= 0xff
      self.assertFalse(self._rx(msg))
      self.assertFalse(self.safety.get_controls_allowed())

      if make_msg == self._user_brake_msg:
        self._reset_safety_hooks()
        self.safety.set_controls_allowed(True)
        self.assertFalse(self._rx(self._user_brake_msg(False, quality_flag=False)))
        self.assertFalse(self.safety.get_controls_allowed())

      # Pack fixed counters BEFORE CRC calculation. Mutating payload bytes after
      # packing would test checksum rejection instead of the counter threshold.
      self._reset_safety_hooks()
      value = True if make_msg == self._pcm_status_msg else 0
      for counter in (1, 2):
        self.assertTrue(self._rx(make_msg(value, counter=counter)))
      self.safety.set_controls_allowed(True)
      for _ in range(common.MAX_WRONG_COUNTERS - 1):
        self.assertTrue(self._rx(make_msg(value, counter=2)))
        self.assertTrue(self.safety.get_controls_allowed())
      self.assertFalse(self._rx(make_msg(value, counter=2)))
      self.assertFalse(self.safety.get_controls_allowed())


class TestRivianStockSafety(TestRivianSafetyBase):

  LONGITUDINAL = False

  def test_adas_status(self):
    # For canceling stock ACC
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)
      for interface_status in range(4):
        values = {"VDM_AdasInterfaceStatus": interface_status}
        self.assertTrue(self._tx(self.packer.make_can_msg_safety("VDM_AdasSts", 2, values)))

  def test_fm_every_max_torque_level_is_never_refused(self):
    """FM Pilot: the real controller against the panda at every Max Torque step above 385: never refused, reaches the
    cap (run once, here; the subclasses check their own level)"""
    if type(self) is not TestRivianStockSafety:
      self.skipTest("run once")
    try:
      for value in range(400, 581, 15):
        self.MAX_TORQUE_SETTING = value
        self.MAX_TORQUE_LOOKUP = [9, 25, 27], [value, 295, 275]
        with self.subTest(value=value):
          self.test_fm_demand_past_cap_is_never_refused()
    finally:
      del self.MAX_TORQUE_SETTING, self.MAX_TORQUE_LOOKUP


class TestRivianLongitudinalSafety(TestRivianSafetyBase):

  SAFETY_PARAM = RivianSafetyFlags.LONG_CONTROL

  TX_MSGS = [[0x120, 0], [0x321, 2], [0x160, 0], [0x162, 2]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x120, 0x160), 2: (0x321, 0x162)}
  FWD_BLACKLISTED_ADDRS = {0: [0x321, 0x162], 2: [0x120, 0x160]}

  def test_adas_status(self):
    # VDM_AdasSts is forwarded to the ACM in long mode so openpilot can hide ACC engage requests it would refuse
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)
      for user_request in range(5):
        values = {"VDM_UserAdasRequest": user_request}
        self.assertTrue(self._tx(self.packer.make_can_msg_safety("VDM_AdasSts", 2, values)))


class TestRivianStockSafetyEnhancedMads(TestRivianStockSafety):
  ENHANCED_MADS = True


class TestRivianLongitudinalSafetyEnhancedMads(TestRivianLongitudinalSafety):
  ENHANCED_MADS = True


# FM Pilot "Max Torque" above 385: every inherited steering test runs against the higher envelope. Those tests run at
# standstill, where the panda allows the faster ramp (5 per frame).
class TestRivianStockSafetyMaxTorque400(TestRivianStockSafety):
  MAX_TORQUE_SETTING = 400
  MAX_TORQUE_LOOKUP = [9, 25, 27], [400, 295, 275]
  MAX_RATE_UP = 5


class TestRivianStockSafetyMaxTorque475(TestRivianStockSafety):
  MAX_TORQUE_SETTING = 475
  MAX_TORQUE_LOOKUP = [9, 25, 27], [475, 295, 275]
  MAX_RATE_UP = 5


class TestRivianStockSafetyMaxTorque580(TestRivianStockSafety):
  MAX_TORQUE_SETTING = 580
  MAX_TORQUE_LOOKUP = [9, 25, 27], [580, 295, 275]
  MAX_RATE_UP = 5
  FM_SPEEDS = tuple(sorted({float(x) for x in range(35)} | {0.5, 8.5, 9.5, 12.5, 16.5, 17.5, 24.5, 25.5}))  # 0-34 m/s at his setting

  def _rate_up_allowed(self, speed, step):
    for _ in range(6):
      self.assertTrue(self._rx(self._speed_msg(speed)))
      self.assertTrue(self._rx(self._speed_msg_2(speed)))
    self.safety.set_controls_allowed(True)
    self.safety.set_desired_torque_last(100)
    self.safety.set_rt_torque_last(100)
    self.safety.set_torque_meas(0, 0)
    return self._tx(self._torque_cmd_msg(100 + step))

  def test_fast_ramp_only_at_low_speed(self):
    self.assertTrue(self._rate_up_allowed(5.0, 5))
    self.assertFalse(self._rate_up_allowed(5.0, 6))
    self.assertTrue(self._rate_up_allowed(12.0, 5))
    # at and above 12.5 m/s the ramp is back to 3
    self.assertTrue(self._rate_up_allowed(13.0, 3))
    self.assertFalse(self._rate_up_allowed(13.0, 4))
    self.assertFalse(self._rate_up_allowed(20.0, 4))

  def test_fast_ramp_boundary(self):
    self.assertTrue(self._rate_up_allowed(12.45, 5))
    self.assertFalse(self._rate_up_allowed(12.5, 4))
    self.assertTrue(self._rate_up_allowed(12.5, 3))

  def test_fast_ramp_waits_for_the_highest_recent_speed(self):
    """braking from above the cutoff, the panda keeps the normal ramp until every recent speed sample is below it
    (vehicle_speed.max), so a fast-ramp frame is never allowed early"""
    self._rate_up_allowed(14.0, 3)  # fill the samples at 14 m/s
    self.assertTrue(self._rx(self._speed_msg(11.0)))
    self.assertTrue(self._rx(self._speed_msg_2(11.0)))
    self.safety.set_desired_torque_last(100)
    self.safety.set_rt_torque_last(100)
    self.assertFalse(self._tx(self._torque_cmd_msg(105)))  # still 14 m/s among the recent samples
    self.assertTrue(self._rate_up_allowed(11.0, 5))  # once they are all 11 m/s

  def test_fast_ramp_car_code_is_never_refused(self):
    """the real controller at 5 per frame from zero to the cap, through blips, stays inside the panda's checks"""
    for speed in (2.0, 6.0, 11.0):
      self._torque_loop_setup(angle=150.0, speed=speed)
      for i in range(300):
        self.assertTrue(self._torque_loop_frame(1.6), f"refused at {speed} m/s, frame {i}")


class TestRivianLongitudinalSafetyMaxTorque580(TestRivianLongitudinalSafety):
  MAX_TORQUE_SETTING = 580
  MAX_TORQUE_LOOKUP = [9, 25, 27], [580, 295, 275]
  MAX_RATE_UP = 5


class TestRivianStockSafetyEnhancedMadsMaxTorque580(TestRivianStockSafety):
  ENHANCED_MADS = True
  MAX_TORQUE_SETTING = 580
  MAX_TORQUE_LOOKUP = [9, 25, 27], [580, 295, 275]
  MAX_RATE_UP = 5


class TestRivianMaxTorqueGate(unittest.TestCase):
  """FM Pilot: the panda's ceiling at standstill for each safety param. 385 unless the flag and a level 1-13 come
  together; each level is its own ceiling; the ramp stays AP's 3 at 385."""

  def _safety(self, param_sp):
    packer = CANPackerSafety("rivian_primary_actuator")
    safety = libsafety_py.libsafety
    safety.set_current_safety_param_sp(param_sp)
    safety.set_safety_hooks(CarParams.SafetyModel.rivian, 0)
    safety.init_tests()
    safety.set_controls_allowed(True)
    return packer, safety

  def _allowed(self, safety, packer, torque, last=None):
    # previous torque just below the value, so only the absolute limit is tested (standstill: low-speed envelope)
    last = torque - 1 if last is None else last
    safety.set_desired_torque_last(last)
    safety.set_rt_torque_last(last)
    safety.set_torque_meas(0, 0)
    return safety.safety_tx_hook(packer.make_can_msg_safety("ACM_lkaHbaCmd", 0, {"ACM_lkaStrToqReq": torque, "ACM_lkaActToi": 1}))

  def _param(self, level, flag=True):
    return (RivianSafetyFlagsSP.HIGH_TORQUE if flag else 0) | (level << RivianSafetyFlagsSP.TORQUE_LEVEL_SHIFT)

  def test_stock_keeps_385(self):
    for param_sp in (0, self._param(0), self._param(5, flag=False), self._param(14), self._param(15),
                     RivianSafetyFlagsSP.ENHANCED_MADS):
      packer, safety = self._safety(param_sp)
      self.assertTrue(self._allowed(safety, packer, 385), param_sp)
      self.assertFalse(self._allowed(safety, packer, 386), param_sp)
      self.assertFalse(self._allowed(safety, packer, -386), param_sp)
      self.assertTrue(self._allowed(safety, packer, 103, last=100), param_sp)
      self.assertFalse(self._allowed(safety, packer, 104, last=100), param_sp)  # AP's ramp

  def test_each_level_is_its_own_ceiling(self):
    for value in range(400, 581, 15):
      level = max_torque_level(value)
      packer, safety = self._safety(self._param(level))
      self.assertTrue(self._allowed(safety, packer, value), value)
      self.assertTrue(self._allowed(safety, packer, -value), value)
      self.assertFalse(self._allowed(safety, packer, value + 1), value)
      self.assertFalse(self._allowed(safety, packer, -value - 1), value)
      self.assertTrue(self._allowed(safety, packer, 105, last=100), value)  # the faster ramp
      self.assertFalse(self._allowed(safety, packer, 106, last=100), value)

  def test_levels_cover_the_slider(self):
    self.assertEqual([385 + 15 * max_torque_level(v) for v in range(385, 581, 15)], list(range(385, 581, 15)))


class TestRivianIgnition(common.CanIgnitionSafetyTest):
  DBC = "rivian_primary_actuator"
  IGNITION_VALUES = range(4)
  IGNITION_ON_VALUES = {1}  # VDM_EpasPowerMode_Drive_On
  IGNITION_COUNTER_MOD = 15

  def _ignition_msg(self, value, counter=None, bus=0):
    return self.packer.make_can_msg_safety("VDM_OutputSignals", bus,
                                           {"VDM_OutputSigs_Counter": counter, "VDM_EpasPowerMode": value})


if __name__ == "__main__":
  unittest.main()
