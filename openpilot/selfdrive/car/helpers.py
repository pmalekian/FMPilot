import capnp
from typing import Any

from openpilot.cereal import custom
from opendbc.car import structs

_FIELDS = '__dataclass_fields__'  # copy of dataclasses._FIELDS


def is_dataclass(obj):
  """Similar to dataclasses.is_dataclass without instance type check checking"""
  return hasattr(obj, _FIELDS)


def _asdictref_inner(obj) -> dict[str, Any] | Any:
  if is_dataclass(obj):
    ret = {}
    for field in getattr(obj, _FIELDS):  # similar to dataclasses.fields()
      ret[field] = _asdictref_inner(getattr(obj, field))
    return ret
  elif isinstance(obj, (tuple, list)):
    return type(obj)(_asdictref_inner(v) for v in obj)
  else:
    return obj


def asdictref(obj) -> dict[str, Any]:
  """
  Similar to dataclasses.asdict without recursive type checking and copy.deepcopy
  Note that the resulting dict will contain references to the original struct as a result
  """
  if not is_dataclass(obj):
    raise TypeError("asdictref() should be called on dataclass instances")

  return _asdictref_inner(obj)


def convert_to_capnp(struct: structs.CarParamsSP | structs.CarStateSP) -> capnp.lib.capnp._DynamicStructBuilder:
  struct_dict = asdictref(struct)

  if isinstance(struct, structs.CarParamsSP):
    struct_capnp = custom.CarParamsSP.new_message(**struct_dict)
  elif isinstance(struct, structs.CarStateSP):
    struct_capnp = custom.CarStateSP.new_message(**struct_dict)
  else:
    raise ValueError(f"Unsupported struct type: {type(struct)}")

  return struct_capnp


# The car-side dataclass for each struct field of carControlSP that opendbc defines one for
_CC_SP_STRUCTS = {
  'mads': structs.ModularAssistiveDrivingSystem,
  'leadOne': structs.LeadData,
  'leadTwo': structs.LeadData,
  'intelligentCruiseButtonManagement': structs.IntelligentCruiseButtonManagement,
  'laneCentering': structs.CarControlSP.LaneCentering,
}


def _capnp_to_plain(value):
  """A capnp struct as a dict, a capnp list as a list of those, an enum as its name: what to_dict() gives for one field."""
  if hasattr(value, 'to_dict'):
    return value.to_dict()
  if hasattr(value, '_as_str'):
    return value._as_str()
  if isinstance(value, (str, bytes, int, float, bool)) or value is None:
    return value
  return [_capnp_to_plain(v) for v in value]


def _plain_fields(cls, schema) -> list[tuple[str, bool]]:
  """(name, is an enum) for the fields of a car-side dataclass that its capnp struct has, when they all hold a plain
  value there (number, bool, enum, text), so they can be read one by one instead of through to_dict()."""
  plain = []
  for name in getattr(cls, _FIELDS):
    field = schema.fields.get(name)
    if name.endswith('DEPRECATED') or field is None:
      continue
    kind = field.proto.slot.type.which() if field.proto.which() == 'slot' else 'group'
    if kind in ('group', 'struct', 'list', 'interface', 'anyPointer'):
      return []  # a nested struct or list: leave this dataclass to to_dict()
    plain.append((name, kind == 'enum'))
  return plain


# For each struct field of carControlSP that opendbc has a dataclass for: (dataclass, its plain fields)
_CC_SP_PLAIN = {name: (cls, _plain_fields(cls, custom.CarControlSP.schema.fields[name].schema)) for name, cls in _CC_SP_STRUCTS.items()}


def convert_carControlSP(struct: capnp.lib.capnp._DynamicStructReader) -> structs.CarControlSP:
  # TODO: recursively handle any car struct as needed
  # FM Pilot: only the fields the car-side dataclasses know are read. carControlSP also carries log-only fields
  # (releaseGuard, turnAssist, fmLowSpeed) that opendbc doesn't define; passing them crashed card with a TypeError and
  # blocked engagement. This runs in card at 100 Hz. It used to turn the whole message into a dict (struct.to_dict())
  # and throw most of it away; now each car-side field is read directly (enums as their names, as to_dict() gives them).
  fields = {}
  for name in getattr(structs.CarControlSP, _FIELDS):
    # a struct or list that was never set is left to the dataclass's own default, as to_dict() leaves it out
    if name.endswith('DEPRECATED') or not struct._has(name):
      continue
    value = getattr(struct, name)
    cls, plain = _CC_SP_PLAIN.get(name, (None, None))
    if cls is None:
      fields[name] = _capnp_to_plain(value)
    elif plain:
      fields[name] = cls(**{f: (getattr(value, f)._as_str() if is_enum else getattr(value, f)) for f, is_enum in plain})
    else:
      fields[name] = cls(**{k: v for k, v in value.to_dict().items() if not k.endswith('DEPRECATED')})
  # struct_dataclass.params = [structs.CarControlSP.Param(**remove_deprecated(p)) for p in struct_dict.get('params', [])]
  return structs.CarControlSP(**fields)
