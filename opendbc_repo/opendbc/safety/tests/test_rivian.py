#!/usr/bin/env python3
import unittest

import numpy as np
from types import SimpleNamespace

from opendbc.car import structs
from opendbc.car.rivian.carcontroller import CarController
from opendbc.car.structs import CarParams
from opendbc.sunnypilot.car.rivian.values import RivianFlagsSP, RivianSafetyFlagsSP, max_torque_level
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety
from opendbc.car.rivian.values import CarControllerParams, RivianSafetyFlags
from opendbc.car.rivian.riviancan import checksum as _checksum


def checksum(msg):
  addr, dat, bus = msg
  ret = bytearray(dat)

  # ESP_Status
  if addr == 0x208:
    ret[0] = _checksum(ret[1:], 0x1D, 0xB1)
  elif addr == 0x150:
    ret[0] = _checksum(ret[1:], 0x1D, 0x9A)
  elif addr == 0x162:
    ret[0] = _checksum(ret[1:], 0x1D, 0xD1)

  return addr, ret, bus


class TestRivianSafetyBase(common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest, common.SteerRequestCutSafetyTest,
                           common.LongitudinalAccelSafetyTest, common.VehicleSpeedSafetyTest):

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

  def _torque_driver_msg(self, torque):
    values = {"EPAS_TorsionBarTorque": torque / 100.0}
    return self.packer.make_can_msg_safety("EPAS_SystemStatus", 0, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"ACM_lkaStrToqReq": torque, "ACM_lkaActToi": steer_req}
    return self.packer.make_can_msg_safety("ACM_lkaHbaCmd", 0, values)

  def _speed_msg(self, speed, quality_flag=True):
    values = {"ESP_Vehicle_Speed": speed * 3.6, "ESP_Status_Counter": self.cnt_speed % 15,
              "ESP_Vehicle_Speed_Q": 1 if quality_flag else 0}
    self.__class__.cnt_speed += 1
    return self.packer.make_can_msg_safety("ESP_Status", 0, values, fix_checksum=checksum)

  def _speed_msg_2(self, speed, quality_flag=True):
    # Rivian has a dynamic max torque limit based on speed, so it checks two sources
    return self._user_gas_msg(0, speed, quality_flag)

  def _user_brake_msg(self, brake):
    values = {"iBESP2_BrakePedalApplied": brake}
    return self.packer.make_can_msg_safety("iBESP2", 0, values)

  def _user_gas_msg(self, gas, speed=0, quality_flag=True):
    values = {"VDM_AcceleratorPedalPosition": gas, "VDM_VehicleSpeed": speed * 3.6,
              "VDM_PropStatus_Counter": self.cnt_speed_2 % 15, "VDM_VehicleSpeedQ": 1 if quality_flag else 0}
    self.__class__.cnt_speed_2 += 1
    return self.packer.make_can_msg_safety("VDM_PropStatus", 0, values, fix_checksum=checksum)

  def _pcm_status_msg(self, enable):
    values = {"ACM_FeatureStatus": enable, "ACM_Unkown1": 1}
    return self.packer.make_can_msg_safety("ACM_Status", 2, values)

  def _accel_msg(self, accel: float):
    values = {"ACM_AccelerationRequest": accel}
    return self.packer.make_can_msg_safety("ACM_longitudinalRequest", 0, values)

  def _torque_loop_setup(self, angle=150.0, speed=11.4, timer_offset_frames=0):
    """the real CarController driving the panda model frame by frame (10 ms) at a high wheel angle, where it
    blips the TOI request about every 0.9 s"""
    CP_SP = structs.CarParamsSP()
    if self.HIGH_TORQUE:
      CP_SP.flags |= RivianFlagsSP.HIGH_TORQUE_PANDA.value
      CP_SP.safetyParam = self._high_torque_param_sp()
      if self.FAST_RAMP:
        CP_SP.flags |= RivianFlagsSP.FAST_RAMP.value
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

  def test_fm_demand_past_cap_is_never_refused(self):
    """FM Pilot: openpilot may ask past 1.0 (Higher Torque). Whatever it asks, the controller must stay inside the
    panda's limit at every speed: with the panda flag off it holds AP's curve, with it on FM Pilot's higher curve."""
    lookup = CarControllerParams.STEER_MAX_LOOKUP
    if self.HIGH_TORQUE:
      lookup = CarControllerParams.steer_max_lookup_high(self.MAX_TORQUE_LOOKUP[1][0])
    for speed in (0.5, 4.0, 8.0, 9.0, 12.0, 17.0, 21.0, 25.0, 27.0, 33.0):
      for sign in (1.0, -1.0):
        self._torque_loop_setup(angle=30.0, speed=speed)
        peak = 0
        for i in range(300):
          self.assertTrue(self._torque_loop_frame(sign * 1.6), f"refused at {speed} m/s, sign {sign}, frame {i}")
          peak = max(peak, abs(self.last_sent_torque))
        cap = round(float(np.interp(speed, *lookup)))
        if not self.HIGH_TORQUE:
          cap = round(float(np.interp(speed, *CarControllerParams.STEER_MAX_LOOKUP)))
        self.assertLessEqual(peak, cap, f"{speed} m/s")
        self.assertGreaterEqual(peak, cap - 3, f"{speed} m/s: never reached the cap")

  def test_fm_unit_demand_unchanged(self):
    """FM Pilot: a request of 1.0 or less sends the same torque with or without the panda flag (Off == AP)."""
    for speed in (4.0, 9.0, 17.0, 30.0):
      self._torque_loop_setup(angle=30.0, speed=speed)
      for _ in range(200):
        self._torque_loop_frame(1.0)
      self.assertEqual(abs(self.last_sent_torque), round(float(np.interp(speed, *CarControllerParams.STEER_MAX_LOOKUP))))

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

  def test_rx_hook(self):
    # checksum, counter, and quality flag checks
    for quality_flag in (True, False):
      for msg_type in ("speed", "speed_2"):
        self.safety.set_controls_allowed(True)
        # send multiple times to verify counter checks
        for _ in range(10):
          if msg_type == "speed":
            msg = self._speed_msg(0, quality_flag=quality_flag)
          elif msg_type == "speed_2":
            msg = self._speed_msg_2(0, quality_flag=quality_flag)

          self.assertEqual(quality_flag, self._rx(msg))
          self.assertEqual(quality_flag, self.safety.get_controls_allowed())

        # Mess with checksum to make it fail
        msg[0].data[0] = 0xff
        self.assertFalse(self._rx(msg))
        self.assertFalse(self.safety.get_controls_allowed())

  cnt_stalk = 0

  def _stalk_msg(self, req):
    values = {"VDM_UserAdasRequest": req, "VDM_AdasSts_Counter": self.cnt_stalk % 15}
    self.__class__.cnt_stalk += 1
    return self.packer.make_can_msg_safety("VDM_AdasSts", 0, values, fix_checksum=checksum)

  # "Use enhanced Rivian MADS". Off, the panda has no MADS button, as in stock sunnypilot.
  ENHANCED_MADS = False

  # FM Pilot "Panda Allows Higher Torque". Off, AP's limits.
  HIGH_TORQUE = False
  # FM Pilot "Max Torque" level sent with it (None: no level, the earlier fixed 481)
  MAX_TORQUE_SETTING = None
  # FM Pilot "Faster Torque Ramp" (only counts with HIGH_TORQUE)
  FAST_RAMP = False

  def _high_torque_param_sp(self):
    param_sp = RivianSafetyFlagsSP.HIGH_TORQUE
    if self.MAX_TORQUE_SETTING is not None:
      param_sp |= max_torque_level(self.MAX_TORQUE_SETTING) << RivianSafetyFlagsSP.TORQUE_LEVEL_SHIFT
    if self.FAST_RAMP:
      param_sp |= RivianSafetyFlagsSP.FAST_RAMP
    return param_sp

  def _set_safety_hooks(self, param):
    param_sp = RivianSafetyFlagsSP.ENHANCED_MADS if self.ENHANCED_MADS else 0
    if self.HIGH_TORQUE:
      param_sp |= self._high_torque_param_sp()
    self.safety.set_current_safety_param_sp(param_sp)
    self.safety.set_safety_hooks(CarParams.SafetyModel.rivian, param)

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


class TestRivianStockSafety(TestRivianSafetyBase):

  LONGITUDINAL = False

  def setUp(self):
    self.packer = CANPackerSafety("rivian_primary_actuator")
    self.safety = libsafety_py.libsafety
    self._set_safety_hooks(0)
    self.safety.init_tests()

  def test_adas_status(self):
    # For canceling stock ACC
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)
      for interface_status in range(4):
        values = {"VDM_AdasInterfaceStatus": interface_status}
        self.assertTrue(self._tx(self.packer.make_can_msg_safety("VDM_AdasSts", 2, values)))


class TestRivianLongitudinalSafety(TestRivianSafetyBase):

  TX_MSGS = [[0x120, 0], [0x321, 2], [0x160, 0], [0x162, 2]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x120, 0x160), 2: (0x321, 0x162)}
  FWD_BLACKLISTED_ADDRS = {0: [0x321, 0x162], 2: [0x120, 0x160]}

  def setUp(self):
    self.packer = CANPackerSafety("rivian_primary_actuator")
    self.safety = libsafety_py.libsafety
    self._set_safety_hooks(RivianSafetyFlags.LONG_CONTROL)
    self.safety.init_tests()

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


# FM Pilot: the higher low-speed envelope (AP 5270cbba, Uncle-Tony). Every inherited steering test runs against it.
class TestRivianStockSafetyHighTorque(TestRivianStockSafety):
  HIGH_TORQUE = True
  MAX_TORQUE_LOOKUP = [9, 25, 27], [481, 295, 275]


class TestRivianLongitudinalSafetyHighTorque(TestRivianLongitudinalSafety):
  HIGH_TORQUE = True
  MAX_TORQUE_LOOKUP = [9, 25, 27], [481, 295, 275]


class TestRivianStockSafetyEnhancedMadsHighTorque(TestRivianStockSafety):
  ENHANCED_MADS = True
  HIGH_TORQUE = True
  MAX_TORQUE_LOOKUP = [9, 25, 27], [481, 295, 275]


# FM Pilot "Max Torque": the lowest and highest slider levels, and the default
class TestRivianStockSafetyMaxTorque405(TestRivianStockSafety):
  HIGH_TORQUE = True
  MAX_TORQUE_SETTING = 405
  MAX_TORQUE_LOOKUP = [9, 25, 27], [405, 295, 275]


class TestRivianStockSafetyMaxTorque480(TestRivianStockSafety):
  HIGH_TORQUE = True
  MAX_TORQUE_SETTING = 480
  MAX_TORQUE_LOOKUP = [9, 25, 27], [480, 295, 275]


class TestRivianStockSafetyMaxTorque580(TestRivianStockSafety):
  HIGH_TORQUE = True
  MAX_TORQUE_SETTING = 580
  MAX_TORQUE_LOOKUP = [9, 25, 27], [580, 295, 275]


# FM Pilot "Faster Torque Ramp": the inherited tests run at standstill, where the panda allows 5 per frame
class TestRivianStockSafetyFastRamp(TestRivianStockSafety):
  HIGH_TORQUE = True
  MAX_TORQUE_SETTING = 480
  FAST_RAMP = True
  MAX_TORQUE_LOOKUP = [9, 25, 27], [480, 295, 275]
  MAX_RATE_UP = 5

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

  def test_fast_ramp_car_code_is_never_refused(self):
    """the real controller at 5 per frame from zero to the cap, through blips, stays inside the panda's checks"""
    for speed in (2.0, 6.0, 11.0):
      self._torque_loop_setup(angle=150.0, speed=speed)
      for i in range(300):
        self.assertTrue(self._torque_loop_frame(1.6), f"refused at {speed} m/s, frame {i}")

  def test_car_code_ramps_faster_below_25_mph(self):
    def frames_to(speed, target):
      self._torque_loop_setup(angle=30.0, speed=speed)
      for i in range(400):
        self._torque_loop_frame(1.6)
        if abs(self.last_sent_torque) >= target:
          return i
      return 400
    fast = frames_to(6.0, 300)
    self.assertLess(fast, 300 / 3 - 15)  # clearly faster than 3 per frame
    self.assertGreaterEqual(fast, 300 / 5 - 1)  # never faster than 5 per frame
    self.assertGreaterEqual(frames_to(12.0, 250), 250 / 3 - 1)  # 27 mph: the normal ramp


class TestRivianFastRampNeedsHighTorque(TestRivianStockSafety):
  """the fast ramp bit alone (higher limit off) changes nothing: AP's limits and ramp"""
  FAST_RAMP = True

  def _set_safety_hooks(self, param):
    self.safety.set_current_safety_param_sp(RivianSafetyFlagsSP.FAST_RAMP)
    self.safety.set_safety_hooks(CarParams.SafetyModel.rivian, param)


class TestRivianHighTorqueGate(unittest.TestCase):
  """Without the flag the panda must still refuse anything above AP's 385 at low speed; with it, 481 is the ceiling."""

  def _safety(self, high, level=0):
    packer = CANPackerSafety("rivian_primary_actuator")
    safety = libsafety_py.libsafety
    param_sp = level << RivianSafetyFlagsSP.TORQUE_LEVEL_SHIFT
    if high:
      param_sp |= RivianSafetyFlagsSP.HIGH_TORQUE
    safety.set_current_safety_param_sp(param_sp)
    safety.set_safety_hooks(CarParams.SafetyModel.rivian, 0)
    safety.init_tests()
    safety.set_controls_allowed(True)
    return packer, safety

  def _allowed(self, safety, packer, torque):
    # previous torque set just below the value so only the absolute limit is tested (standstill: low-speed envelope)
    safety.set_desired_torque_last(torque - 1)
    safety.set_rt_torque_last(torque - 1)
    return safety.safety_tx_hook(packer.make_can_msg_safety("ACM_lkaHbaCmd", 0, {"ACM_lkaStrToqReq": torque, "ACM_lkaActToi": 1}))

  def test_flag_off_keeps_385(self):
    packer, safety = self._safety(False)
    self.assertTrue(self._allowed(safety, packer, 385))
    self.assertFalse(self._allowed(safety, packer, 387))
    self.assertFalse(self._allowed(safety, packer, 481))

  def test_flag_on_allows_481_not_more(self):
    packer, safety = self._safety(True)
    self.assertTrue(self._allowed(safety, packer, 481))
    self.assertTrue(self._allowed(safety, packer, -481))
    self.assertFalse(self._allowed(safety, packer, 483))

  def test_each_max_torque_level_is_its_own_ceiling(self):
    for value in range(405, 581, 25):
      packer, safety = self._safety(True, max_torque_level(value))
      self.assertTrue(self._allowed(safety, packer, value), value)
      self.assertTrue(self._allowed(safety, packer, -value), value)
      self.assertFalse(self._allowed(safety, packer, value + 2), value)

  def test_level_without_the_flag_keeps_385(self):
    packer, safety = self._safety(False, max_torque_level(580))
    self.assertTrue(self._allowed(safety, packer, 385))
    self.assertFalse(self._allowed(safety, packer, 387))

  def test_unknown_level_falls_back_to_481(self):
    for level in (9, 15):
      packer, safety = self._safety(True, level)
      self.assertTrue(self._allowed(safety, packer, 481))
      self.assertFalse(self._allowed(safety, packer, 483))


class TestRivianIgnition(unittest.TestCase):
  TX_MSGS: list = []

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.init_tests()
    self.packer = CANPackerSafety("rivian_primary_actuator")

  def _msg(self, counter, mode):
    return self.packer.make_can_msg_safety("VDM_OutputSignals", 0,
                                           {"VDM_OutputSigs_Counter": counter,
                                            "VDM_EpasPowerMode": mode})

  # VDM_EpasPowerMode_Drive_On=1
  def test_ignition_on(self):
    for i in range(15):
      self.safety.init_tests()
      self.safety.ignition_can_hook(self._msg(i, 1))
      self.assertFalse(self.safety.get_ignition_can())
      self.safety.ignition_can_hook(self._msg((i + 1) % 15, 1))
      self.assertTrue(self.safety.get_ignition_can())

  def test_ignition_off(self):
    self.safety.ignition_can_hook(self._msg(0, 1))
    self.safety.ignition_can_hook(self._msg(1, 1))
    self.assertTrue(self.safety.get_ignition_can())
    self.safety.ignition_can_hook(self._msg(2, 0))
    self.safety.ignition_can_hook(self._msg(3, 0))
    self.assertFalse(self.safety.get_ignition_can())


if __name__ == "__main__":
  unittest.main()
