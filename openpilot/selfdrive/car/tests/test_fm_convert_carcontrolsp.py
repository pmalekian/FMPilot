import random

from openpilot.cereal import custom, messaging
from opendbc.car import structs
from openpilot.selfdrive.car.helpers import _FIELDS, convert_carControlSP


def test_log_only_fields_are_dropped():
  # FM Pilot: releaseGuard is in carControlSP for logging only; opendbc's CarControlSP has no such field
  msg = custom.CarControlSP.new_message()
  msg.releaseGuard.active = True
  msg.releaseGuard.amount = 0.5
  msg.mads.enabled = True
  msg.laneCentering.active = True
  cc = convert_carControlSP(msg.as_reader())
  assert cc.mads.enabled
  assert cc.laneCentering.active
  assert not hasattr(cc, "releaseGuard")


def test_turn_assist_and_low_speed_logs_are_dropped():
  msg = custom.CarControlSP.new_message()
  msg.turnAssist.sign = -1
  msg.turnAssist.holdFloor = 0.02
  msg.fmLowSpeed.feedforward = 0.3
  msg.mads.enabled = True
  cc = convert_carControlSP(msg.as_reader())
  assert cc.mads.enabled
  assert not hasattr(cc, "turnAssist") and not hasattr(cc, "fmLowSpeed")


def _whole_message_convert(struct) -> structs.CarControlSP:
  """The conversion as it was before it stopped turning the whole message into a dict: the reference."""
  def remove_deprecated(s: dict) -> dict:
    return {k: v for k, v in s.items() if not k.endswith('DEPRECATED')}

  struct_dict = struct.to_dict()
  known = getattr(structs.CarControlSP, _FIELDS)
  struct_dataclass = structs.CarControlSP(**remove_deprecated({k: v for k, v in struct_dict.items()
                                                               if k in known and not isinstance(k, dict)}))
  struct_dataclass.mads = structs.ModularAssistiveDrivingSystem(**remove_deprecated(struct_dict.get('mads', {})))
  struct_dataclass.leadOne = structs.LeadData(**remove_deprecated(struct_dict.get('leadOne', {})))
  struct_dataclass.leadTwo = structs.LeadData(**remove_deprecated(struct_dict.get('leadTwo', {})))
  struct_dataclass.intelligentCruiseButtonManagement = structs.IntelligentCruiseButtonManagement(
    **remove_deprecated(struct_dict.get('intelligentCruiseButtonManagement', {}))
  )
  struct_dataclass.laneCentering = structs.CarControlSP.LaneCentering(**remove_deprecated(struct_dict.get('laneCentering', {})))
  return struct_dataclass


def _random_message(rng: random.Random):
  msg = messaging.new_message('carControlSP')
  cc = msg.carControlSP
  mads_states = list(custom.ModularAssistiveDrivingSystem.ModularAssistiveDrivingSystemState.schema.enumerants)
  cc.mads.state = rng.choice(mads_states)
  cc.mads.enabled, cc.mads.active, cc.mads.available = (rng.random() < 0.5 for _ in range(3))
  for lead in (cc.leadOne, cc.leadTwo):
    lead.dRel, lead.yRel, lead.vRel, lead.aRel, lead.vLead = (rng.uniform(-50, 150) for _ in range(5))
    lead.dPath, lead.vLat, lead.vLeadK, lead.aLeadK, lead.aLeadTau = (rng.uniform(-5, 5) for _ in range(5))
    lead.fcw, lead.status, lead.radar = (rng.random() < 0.5 for _ in range(3))
    lead.modelProb = rng.random()
    lead.radarTrackId = rng.randrange(-1, 500)
  icbm = cc.intelligentCruiseButtonManagement
  icbm_fields = custom.CarControlSP.schema.fields['intelligentCruiseButtonManagement'].schema.fields
  icbm.state = rng.choice(list(icbm_fields['state'].schema.enumerants))
  icbm.sendButton = rng.choice(list(icbm_fields['sendButton'].schema.enumerants))
  icbm.vTarget = rng.uniform(0, 40)
  lc = cc.laneCentering
  lc.active, lc.holding, lc.clipped, lc.singleLine = (rng.random() < 0.5 for _ in range(4))
  lc.correction, lc.offset = rng.uniform(-0.01, 0.01), rng.uniform(-1, 1)
  lc.lineSide = rng.choice([-1, 0, 1])
  if rng.random() < 0.5:
    params = cc.init('params', rng.randrange(1, 4))
    for i, param in enumerate(params):
      param.key, param.value, param.type = f"Key{i}", bytes([rng.randrange(256) for _ in range(3)]), rng.choice(['string', 'bool', 'int'])
  # the log-only fields the car side never sees
  ta = cc.turnAssist
  ta.sign, ta.source, ta.amount = rng.choice([-1, 0, 1]), rng.choice(['none', 'desire', 'carry']), rng.random()
  ta.requestIn, ta.requestOut, ta.lead, ta.holdFloor, ta.holdPeak = (rng.uniform(-0.2, 0.2) for _ in range(5))
  ta.holdActive, ta.releaseReason = rng.random() < 0.5, rng.choice(['none', 'driverAgainst', 'maxTurned', 'maxTime', 'modelExit'])
  cc.releaseGuard.active, cc.releaseGuard.cut, cc.releaseGuard.limit = rng.random() < 0.5, rng.uniform(-3, 3), rng.uniform(0, 3)
  cc.fmLowSpeed.feedforward, cc.fmLowSpeed.gainScale = rng.uniform(-1, 1), rng.uniform(0.4, 1.0)
  return msg


def test_same_result_as_converting_the_whole_message():
  """card calls this 100 times a second. It now converts only the fields the car side has; what the car side gets must
  be exactly what the whole-message conversion gave, for a reader (as card passes) and a builder alike."""
  rng = random.Random(0)
  for _ in range(300):
    msg = _random_message(rng)
    reader = messaging.log_from_bytes(msg.to_bytes()).carControlSP
    want = _whole_message_convert(reader)
    assert convert_carControlSP(reader) == want
    assert convert_carControlSP(msg.carControlSP) == want
    assert convert_carControlSP(reader).params == want.params
    for name in ("mads", "leadOne", "leadTwo", "intelligentCruiseButtonManagement", "laneCentering"):
      assert type(getattr(convert_carControlSP(reader), name)) is type(getattr(want, name)), name


def test_empty_and_partly_filled_messages():
  # a struct that was never set stays at the car-side dataclass's own defaults, as before
  reader = messaging.log_from_bytes(messaging.new_message('carControlSP').to_bytes()).carControlSP
  assert convert_carControlSP(reader) == _whole_message_convert(reader) == structs.CarControlSP()
  msg = messaging.new_message('carControlSP')
  msg.carControlSP.leadOne.dRel = 12.0
  msg.carControlSP.turnAssist.sign = 1
  reader = messaging.log_from_bytes(msg.to_bytes()).carControlSP
  assert convert_carControlSP(reader) == _whole_message_convert(reader)
  assert convert_carControlSP(reader).leadOne.dRel == 12.0


def test_every_car_side_field_is_converted():
  # a struct field added to opendbc's CarControlSP without its dataclass listed here would arrive as a plain dict
  from openpilot.selfdrive.car.helpers import _CC_SP_PLAIN, _CC_SP_STRUCTS
  known = set(getattr(structs.CarControlSP, _FIELDS))
  assert set(_CC_SP_STRUCTS) <= known
  assert known - set(_CC_SP_STRUCTS) == {"params"}
  # every car-side struct is read field by field today, and each of its fields that capnp has is read
  for name, (cls, plain) in _CC_SP_PLAIN.items():
    schema = custom.CarControlSP.schema.fields[name].schema
    want = [f for f in getattr(cls, _FIELDS) if not f.endswith("DEPRECATED") and f in schema.fields]
    assert [f for f, _ in plain] == want and plain, name
    enums = {f for f, is_enum in plain if is_enum}
    assert enums == {f for f in want if schema.fields[f].proto.slot.type.which() == 'enum'}, name
  assert {f for f, e in _CC_SP_PLAIN['mads'][1] if e} == {'state'}
  assert {f for f, e in _CC_SP_PLAIN['intelligentCruiseButtonManagement'][1] if e} == {'state', 'sendButton'}


def test_enums_arrive_as_their_names():
  msg = messaging.new_message('carControlSP')
  msg.carControlSP.mads.state = 'enabled'
  msg.carControlSP.intelligentCruiseButtonManagement.sendButton = 'increase'
  reader = messaging.log_from_bytes(msg.to_bytes()).carControlSP
  cc, want = convert_carControlSP(reader), _whole_message_convert(reader)
  assert cc.mads.state == want.mads.state == 'enabled'
  assert cc.intelligentCruiseButtonManagement.sendButton == want.intelligentCruiseButtonManagement.sendButton == 'increase'
  assert type(cc.mads.state) is type(want.mads.state)
  assert type(cc.intelligentCruiseButtonManagement.sendButton) is type(want.intelligentCruiseButtonManagement.sendButton)
