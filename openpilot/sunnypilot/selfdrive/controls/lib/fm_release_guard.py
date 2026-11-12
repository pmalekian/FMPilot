"""
FM Pilot: Release Guard (Rivian, off by default).

In a slow turn, when you hold the wheel tighter than openpilot's target, the torque controller's proportional term
pushes back against your hands. At turn speeds that term is huge (KP_INTERP is about 30 at 7 mph against 1.0 on the
highway), so even a small gap saturates it, and the sent torque against you climbs as far as the car's hand-torque
rule allows. When you let go, that torque is still there: the wheel unwinds, crosses center with it still pushing, and
it only decays at 5 counts per message.

Nov 11 logs (open-loop replay through the car's hand rule and ramp, which reproduces the logged sent torque):
  - 00000067--4e0a74a75c--0 +32.8 to +34.9 s (Rebel Legion, left turn, 8-11 mph, ~2 Nm into the turn, the truck 30%
    tighter than asked): up to 485 sent against you, 400 still pushing at center, past center for 0.80 s; the wheel went
    to -39 deg. With at most 150 against you: 159 max, 90 at center, 0.18 s.
  - 00000068--29f57c5eb1--1 +0 to +1.7 s (TSFDOM, right turn, 12-15 mph, Turn Hold + Hold Adds Your Steering on but
    already released): up to 410 against you, 290 at center, 0.60 s past center. With at most 150: 110, 29, 0.06 s.

This limits only that proportional push against you to Most Torque Against You (150 or 75 counts, converted to the
controller's lateral acceleration units the same way the Low-Speed Steering Feedforward converts its counts, so Higher
Torque's Full mode doesn't change it). It acts only:
  - while you press the wheel into the curve the truck is already turning (hand torque toward the turn and the truck
    turning that way by more than 0.003 1/m), so the wheel's own reaction, which reads against openpilot, can't trigger
    it, and catching an overshoot (hand torque the same way as openpilot's push) never does;
  - under 15 mph, fading out by 20 mph.
When you let go it fades out over 0.5 s, and ends at once when the proportional term flips (the wheel has passed
openpilot's target: from then on that term is the pull back and is never touched) or when your hand torque reverses.
Feedforward (including the Low-Speed Steering Feedforward), friction, the integrator (frozen while you press anyway) and
every torque limit after the controller are unchanged: it only ever removes torque that pushes against you.
Turning the setting on or off fades over 0.5 s. Not with Lateral Jerk Torque Controller or Neural Network Lateral
Control, which replace the controller's output (it stands down, as the feedforward does).
"""
import math

import numpy as np

from openpilot.common.constants import CV
from openpilot.common.realtime import DT_CTRL

PARAM_ENABLED = "FM_ReleaseGuard"
PARAM_MAX = "FM_ReleaseGuardMax"
MAX_CHOICES = (75, 150)  # counts against you, at most
MAX_DEFAULT = 150
REPLACES_OUTPUT = ("LateralJerkTorqueController", "NeuralNetworkLateralControl")  # sunnypilot settings
SPEEDS = (15 * CV.MPH_TO_MS, 20 * CV.MPH_TO_MS)  # full below, gone above
MIN_CURVATURE = 0.003  # 1/m: the truck must already be turning that way (as Hold Adds Your Steering)
RELEASE_S = 0.5  # s: fade out after you let go
BLEND_S = 0.5  # s: setting on/off
BASE_COUNTS = 385  # counts per 1.0 of torque request below 9 m/s (CarControllerParams.STEER_MAX_LOOKUP)
PARAMS_PERIOD = 1.0  # s


class FmReleaseGuard:
  def __init__(self, CP, params=None):
    self.available = getattr(CP, "brand", None) == "rivian"
    self.params = params
    self.target = 0.0
    self.enabled = 0.0  # blends to target
    self.max_counts = float(MAX_DEFAULT)
    self.frame = -1
    self._reset_state()
    if self.available:
      if self.params is None:
        from openpilot.common.params import Params
        self.params = Params()
      self.available = not any(self.params.get_bool(k) for k in REPLACES_OUTPUT)
    if self.available:
      self.read_params()
      self.enabled = self.target  # the drive starts at the setting, no blend

  def _reset_state(self) -> None:
    self.amount = 0.0     # 1 = full guard (pressing into the turn below 15 mph), fades after release
    self.against = 0.0    # sign (right-positive) of the proportional push against you, from the last press
    self.active = False   # cut something this frame
    self.cut = 0.0        # lateral accel removed from P this frame, right-positive
    self.limit = 0.0      # lateral accel P may push against you this frame

  def read_params(self) -> None:
    try:
      enabled = self.params.get_bool(PARAM_ENABLED)
      value = self.params.get(PARAM_MAX, return_default=True)
      counts = float(MAX_DEFAULT if value is None else value)
    except (TypeError, ValueError):
      enabled, counts = False, float(MAX_DEFAULT)
    if not math.isfinite(counts) or int(counts) not in MAX_CHOICES:
      counts = float(MAX_DEFAULT)
    self.target = 1.0 if enabled else 0.0
    self.max_counts = counts

  def idle(self) -> None:
    """Lateral is off: nothing is cut and the guard starts fresh next time."""
    self.active, self.cut, self.amount, self.against = False, 0.0, 0.0, 0.0

  def update(self, pid, active: bool, v_ego: float, steering_pressed: bool, steering_torque: float,
             measured_curvature: float, lat_accel_factor: float, fm_scale: float = 1.0) -> float:
    """Limit pid.p's push against the driver in place and return the PID's new (clipped) output, right-positive.
    steering_torque is the column's hand torque (left-positive); measured_curvature is right-positive."""
    self.active, self.cut = False, 0.0
    if not self.available:
      return float(pid.control)
    self.frame += 1
    if self.frame % int(PARAMS_PERIOD / DT_CTRL) == 0:
      self.read_params()
    step = DT_CTRL / BLEND_S
    self.enabled = float(np.clip(self.target, self.enabled - step, self.enabled + step))

    if not active or self.enabled <= 0.0:
      self.amount, self.against = 0.0, 0.0
      return float(pid.control)

    hand = -steering_torque  # right-positive, like the controller
    into_turn = steering_pressed and hand * measured_curvature > 0.0 and abs(measured_curvature) > MIN_CURVATURE
    if into_turn:
      self.amount = float(np.interp(v_ego, SPEEDS, [1.0, 0.0]))
      self.against = -math.copysign(1.0, hand)
    elif steering_pressed:
      self.amount = 0.0  # pressing, but not into the turn: steering against it, or the truck isn't turning yet
    else:
      self.amount = max(self.amount - DT_CTRL / RELEASE_S, 0.0)
      if self.amount > 0.0 and pid.p * self.against <= 0.0:
        self.amount = 0.0  # P flipped: the wheel passed openpilot's target, so P is now the pull back

    self.limit = self.max_counts / BASE_COUNTS * lat_accel_factor / max(fm_scale, 1e-3)
    amount = self.amount * self.enabled
    if amount <= 0.0 or pid.p * self.against <= self.limit:
      return float(pid.control)

    excess = abs(pid.p) - self.limit
    self.cut = math.copysign(excess * amount, pid.p)
    self.active = True
    pid.p -= self.cut
    pid.control = float(np.clip(pid.p + pid.i + pid.d + pid.f, pid.neg_limit, pid.pos_limit))
    return float(pid.control)
