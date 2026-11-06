"""FM Pilot: "Panda Allows Higher Torque" sets the car flag and the panda safety bit together, once per drive."""
from opendbc.car import structs
from opendbc.sunnypilot.car.interfaces import _initialize_rivian_high_torque
from opendbc.sunnypilot.car.rivian.values import (MAX_TORQUE_DEFAULT, RivianFlagsSP, RivianSafetyFlagsSP, high_torque_cap,
                                                 max_torque_level)


def _init(brand, value, max_torque=None):
  CP = structs.CarParams(brand=brand)
  CP_SP = structs.CarParamsSP()
  params = {} if value is None else {"FM_PandaHighTorque": value}
  if max_torque is not None:
    params["FM_MaxTorque"] = max_torque
  _initialize_rivian_high_torque(CP, CP_SP, params)
  return CP_SP


def test_on_sets_flag_and_safety_bit():
  for value in (True, "1"):
    CP_SP = _init('rivian', value)
    assert CP_SP.flags & RivianFlagsSP.HIGH_TORQUE_PANDA
    assert CP_SP.safetyParam & RivianSafetyFlagsSP.HIGH_TORQUE


def test_off_or_unset_leaves_both_clear():
  for value in (False, "0", None):
    CP_SP = _init('rivian', value)
    assert CP_SP.flags == 0
    assert CP_SP.safetyParam == 0


def test_other_brands_ignored():
  CP_SP = _init('tesla', True)
  assert CP_SP.flags == 0
  assert CP_SP.safetyParam == 0


def test_bits_do_not_collide_with_enhanced_mads():
  assert not RivianSafetyFlagsSP.HIGH_TORQUE & RivianSafetyFlagsSP.ENHANCED_MADS
  assert not RivianFlagsSP.HIGH_TORQUE_PANDA & (RivianFlagsSP.ENHANCED_MADS | RivianFlagsSP.LONGITUDINAL_HARNESS_UPGRADE)


def test_max_torque_level_goes_to_the_panda():
  for value in range(405, 581, 25):
    CP_SP = _init('rivian', True, value)
    assert high_torque_cap(CP_SP.safetyParam) == value
    assert CP_SP.safetyParam & RivianSafetyFlagsSP.ENHANCED_MADS == 0


def test_max_torque_default_rounding_and_clamping():
  assert high_torque_cap(_init('rivian', True).safetyParam) == MAX_TORQUE_DEFAULT
  assert high_torque_cap(_init("rivian", True, "520").safetyParam) == 530
  assert high_torque_cap(_init('rivian', True, 900).safetyParam) == 580
  assert high_torque_cap(_init('rivian', True, 100).safetyParam) == 405
  assert high_torque_cap(_init('rivian', True, "junk").safetyParam) == MAX_TORQUE_DEFAULT
  assert high_torque_cap(_init('rivian', False, 580).safetyParam) == 385


def test_levels_fit_their_bits():
  for value in range(405, 581, 25):
    assert 1 <= max_torque_level(value) <= 8
    assert (max_torque_level(value) << RivianSafetyFlagsSP.TORQUE_LEVEL_SHIFT) & (RivianSafetyFlagsSP.ENHANCED_MADS |
                                                                                 RivianSafetyFlagsSP.HIGH_TORQUE) == 0


def test_no_level_is_the_earlier_481():
  assert high_torque_cap(RivianSafetyFlagsSP.HIGH_TORQUE) == 481
