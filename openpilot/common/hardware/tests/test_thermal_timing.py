from openpilot.common.hardware.base import ThermalConfig, ThermalZone


class FakeZone(ThermalZone):
  def __init__(self, name, value):
    super().__init__(name)
    self.value = value

  def read(self) -> float:
    return self.value


def _config():
  return ThermalConfig(cpu=[FakeZone("cpu0", 40.), FakeZone("cpu1", 41.)], gnss=FakeZone("gnss", 30.))


def test_get_msg_unchanged_without_callback():
  assert _config().get_msg() == {"cpuTempC": [40., 41.], "gnssTempC": 30.}


def test_get_msg_times_every_zone():
  seen = []
  msg = _config().get_msg(lambda name, ms: seen.append((name, ms)))
  assert msg == {"cpuTempC": [40., 41.], "gnssTempC": 30.}
  assert [n for n, _ in seen] == ["cpu0", "cpu1", "gnss"]
  assert all(ms >= 0. for _, ms in seen)


def test_get_msg_times_a_failing_read():
  class Broken(ThermalZone):
    def read(self) -> float:
      raise OSError("EIO")

  seen = []
  try:
    ThermalConfig(gnss=Broken("gnss")).get_msg(lambda name, ms: seen.append(name))
  except OSError:
    pass
  assert seen == ["gnss"]
