"""
FM Pilot: Turn Pulse on Pull-Away.

modeld hands the driving model a turn desire as a one-frame pulse when the desire starts (it only passes the rising
edge), and the model keeps that pulse in its 5 s input history. A turn held through a long wait at a corner has dropped
out of that history by the time the truck pulls away, so the model starts the turn late: in 0000005e--e26703ace2--2,
after a 12 s stop with the turn held, it asked for about half the curve the truck drove (the driver steered the rest).

With the setting on, once the truck has stopped during a held turn, the model's desire input drops to none for one
model frame as the truck starts moving again (0.3 m/s, where openpilot starts steering again), so the next frame is a
fresh pulse. Only the model's input changes: the turn itself (laneTurnDirection, the "Turning" alert, Turn Assist,
Blendful's no-desire check) carries on as before. Lane changes are not touched.
"""
from openpilot.cereal import log

PARAM = "FM_TurnPulseOnPullAway"
STOP_SPEED = 0.1  # m/s: counts as stopped
PULL_AWAY_SPEED = 0.3  # m/s: controlsd's standstill line, where openpilot steers again
PARAMS_PERIOD_FRAMES = 50
TURN_DESIRES = (log.Desire.turnLeft, log.Desire.turnRight)


class FmTurnPulse:
  def __init__(self, params=None):
    if params is None:
      from openpilot.common.params import Params
      params = Params()
    self.params = params
    self.enabled = False
    self.frame = 0
    self.stopped = False  # the truck has stopped during the current turn
    self.read_params()

  def read_params(self) -> None:
    self.enabled = self.params.get_bool(PARAM)

  def update(self, desire, v_ego: float):
    """The desire to feed the model this frame."""
    self.frame += 1
    if self.frame % PARAMS_PERIOD_FRAMES == 0:
      self.read_params()
    if not self.enabled or desire not in TURN_DESIRES:
      self.stopped = False
      return desire
    if v_ego < STOP_SPEED:
      self.stopped = True
    elif self.stopped and v_ego >= PULL_AWAY_SPEED:
      self.stopped = False
      return log.Desire.none  # one frame off, so the next frame reaches the model as a fresh pulse
    return desire
