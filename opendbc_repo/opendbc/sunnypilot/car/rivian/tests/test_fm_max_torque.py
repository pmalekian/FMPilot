"""FM Pilot "Max Torque" (FM_CustomMaxTorque): one setting sets the car flag and the panda's safety bits together, once
per drive. 385 is stock AP: nothing is set."""
from opendbc.car import structs
from opendbc.sunnypilot.car.interfaces import _initialize_rivian_max_torque
from opendbc.sunnypilot.car.rivian.values import (MAX_TORQUE_LEVELS, RivianFlagsSP, RivianSafetyFlagsSP, high_torque_cap,
                                                 max_torque_grid, max_torque_level)


def _init(brand, value):
  CP = structs.CarParams(brand=brand)
  CP_SP = structs.CarParamsSP()
  _initialize_rivian_max_torque(CP, CP_SP, {} if value is None else {"FM_CustomMaxTorque": value})
  return CP_SP


def test_385_or_unset_sets_nothing():
  for value in (385, "385", None, 300, "junk", float("nan")):
    CP_SP = _init('rivian', value)
    assert CP_SP.flags == 0 and CP_SP.safetyParam == 0, value


def test_each_step_goes_to_the_panda():
  for value in range(400, 581, 15):
    CP_SP = _init('rivian', value)
    assert CP_SP.flags & RivianFlagsSP.HIGH_TORQUE_PANDA
    assert CP_SP.safetyParam & RivianSafetyFlagsSP.HIGH_TORQUE
    assert high_torque_cap(CP_SP.safetyParam) == value
    assert not CP_SP.safetyParam & RivianSafetyFlagsSP.ENHANCED_MADS


def test_rounding_and_clamping():
  assert high_torque_cap(_init('rivian', "520").safetyParam) == 520
  assert high_torque_cap(_init('rivian', 522).safetyParam) == 520
  assert high_torque_cap(_init('rivian', 523).safetyParam) == 520  # 523 is 138/15 = 9.2 steps over 385
  assert high_torque_cap(_init('rivian', 900).safetyParam) == 580
  assert high_torque_cap(_init('rivian', 392).safetyParam) == 385  # rounds down to stock
  assert high_torque_cap(_init('rivian', 393).safetyParam) == 400


def test_other_brands_ignored():
  for value in (400, 580):
    CP_SP = _init('tesla', value)
    assert CP_SP.flags == 0 and CP_SP.safetyParam == 0


def test_bits_do_not_collide():
  level_bits = RivianSafetyFlagsSP.TORQUE_LEVEL_MASK << RivianSafetyFlagsSP.TORQUE_LEVEL_SHIFT
  assert not level_bits & (RivianSafetyFlagsSP.ENHANCED_MADS | RivianSafetyFlagsSP.HIGH_TORQUE)
  assert not RivianSafetyFlagsSP.HIGH_TORQUE & RivianSafetyFlagsSP.ENHANCED_MADS
  assert not RivianFlagsSP.HIGH_TORQUE_PANDA & (RivianFlagsSP.ENHANCED_MADS | RivianFlagsSP.LONGITUDINAL_HARNESS_UPGRADE)
  assert MAX_TORQUE_LEVELS <= RivianSafetyFlagsSP.TORQUE_LEVEL_MASK


def test_grid_and_levels():
  assert [max_torque_grid(v) for v in range(385, 581, 15)] == list(range(385, 581, 15))
  assert [max_torque_level(v) for v in range(385, 581, 15)] == list(range(14))


def test_cap_decoding_matches_the_panda_rules():
  # flag + level 1-13 only; anything else is stock 385 (the panda reads the same, safety/modes/rivian.h)
  shift = RivianSafetyFlagsSP.TORQUE_LEVEL_SHIFT
  assert high_torque_cap(0) == 385
  assert high_torque_cap(RivianSafetyFlagsSP.HIGH_TORQUE) == 385
  assert high_torque_cap(5 << shift) == 385
  assert high_torque_cap(RivianSafetyFlagsSP.HIGH_TORQUE | (14 << shift)) == 385
  assert high_torque_cap(RivianSafetyFlagsSP.HIGH_TORQUE | (13 << shift)) == 580
