from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from opendbc.car import structs
from openpilot.cereal import custom
from openpilot.selfdrive.car.card import Car


@pytest.mark.parametrize("brand, passive, can_valid, caps, epas", [
  ("rivian", False, True, (366, 385), True),
  ("rivian", False, False, (366, 385), True),
  ("rivian", True, True, (0, 0), True),
  ("mock", False, True, (0, 0), False),
])
def test_fm_car_state_publishes_with_car_output(brand, passive, can_valid, caps, epas):
  card = Car.__new__(Car)
  card.CP = structs.CarParams(brand=brand, passive=passive)
  card.CI = SimpleNamespace(
    CS=SimpleNamespace(toi_active=True),
    CC=None if passive else SimpleNamespace(fm_steer_cap=366, fm_max_torque=385),
  ) if brand == "rivian" else SimpleNamespace()
  card.last_actuators_output = structs.CarControl.Actuators(torqueOutputCan=250)
  card.can_rcv_cum_timeout_counter = 0
  card.rk = SimpleNamespace(remaining=0.)
  card.sm = SimpleNamespace(frame=1, all_checks=Mock(return_value=True))
  card.pm = Mock()

  card.state_publish(structs.CarState(canValid=can_valid), custom.CarStateSP.new_message(), None)

  events = {call.args[0]: call.args[1] for call in card.pm.send.call_args_list}
  fm = events["fmCarState"]
  assert fm.valid is can_valid
  assert fm.fmCarState.epasToiActive is epas
  assert (fm.fmCarState.steerCap, fm.fmCarState.maxTorque) == caps
  assert events["carOutput"].carOutput.actuatorsOutput.torqueOutputCan == 250
