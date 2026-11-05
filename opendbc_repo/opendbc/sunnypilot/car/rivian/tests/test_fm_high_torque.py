"""FM Pilot: "Panda Allows Higher Torque" sets the car flag and the panda safety bit together, once per drive."""
from opendbc.car import structs
from opendbc.sunnypilot.car.interfaces import _initialize_rivian_high_torque
from opendbc.sunnypilot.car.rivian.values import RivianFlagsSP, RivianSafetyFlagsSP


def _init(brand, value):
  CP = structs.CarParams(brand=brand)
  CP_SP = structs.CarParamsSP()
  _initialize_rivian_high_torque(CP, CP_SP, {} if value is None else {"FM_PandaHighTorque": value})
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
