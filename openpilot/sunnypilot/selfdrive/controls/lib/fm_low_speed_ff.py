"""
FM Pilot: Low-Speed Steering Feedforward (Rivian, off by default).

openpilot's torque controllers build their feedforward from lateral acceleration (curvature x speed^2). That covers
highway steering, but at crawl speed lateral acceleration is tiny even in a tight turn, while holding the wheels at an
angle still takes real torque (tire scrub, self-centering, rack friction). So below about 10 mph the feedforward asks
for almost nothing and the proportional correction supplies all of it, only once a gap has opened: late, then overshoot.

Measured on Frankie's Nov 11 drives (00000061, 00000068, 00000069, 0000006a; 3300 frames where openpilot steered, hands
off, and the sent torque followed the request): below 10 mph the existing feedforward gave about 0 counts (-4 to +5)
while 50-70 counts were sent at 15-45 degrees of wheel, all of it from the correction, and the truck drove 15-19% wider
than asked. At 10-20 mph the existing feedforward already gave 75-85% of the torque and the truck was within 2-8%.

This adds torque straight from the steering-wheel angle the asked curvature needs (vehicle model, no road roll), from
a table in CAN counts fitted so that at Full it never adds more than the gap measured in any bin (gap = sent torque
minus the existing feedforward, with 00000061's Higher Torque Full mode (x1.5) taken out): 20-45 deg under 10 mph 65
added vs 66 measured, 90-220 deg at 10-16 mph 55 vs 54, 8-20 deg under 10 mph 32 vs 52. Tight turns under 10 mph had no
clean frames (always ramp-limited or hands on), so the table above 45 degrees rests on the 10-16 mph bins and is flat
from 120 degrees. Under 20 degrees the logs split by side (left about 70-80 counts, right about 0: road crown helping
right turns), and 20-45 degrees under 10 mph had right turns only, so the symmetric table leans low for left turns and a
little high for gentle right curves at crawl (up to about 8 counts at Gentle). Full below 10 mph (4.5 m/s), fading out by about 16 mph (7 m/s), where the
existing feedforward covers most of it. Strength in four steps: Gentle 40 % (default), Medium 60 %, Firm 80 %, Full 100 %
of the table. Changes blend over a second. Same in both directions (the logs couldn't separate a left/right difference
from road crown). The counts are what is sent, also in Higher Torque's Full mode. It only adds to the request: every
torque limit, the ramp and the panda's checks still apply.
"""
import math

import numpy as np

from openpilot.common.realtime import DT_CTRL

PARAM_ENABLED = "FM_LowSpeedFFEnabled"
PARAM_STEP = "FM_LowSpeedFFStep"
STEPS = {1: 0.4, 2: 0.6, 3: 0.8, 4: 1.0}  # Gentle, Medium, Firm, Full
STEP_DEFAULT = 1
# |steering wheel angle| (deg) -> counts at Full: never more than the measured gap in any bin of the Nov 11 logs
TABLE_DEG = [0.0, 10.0, 30.0, 65.0, 120.0, 180.0]
TABLE_COUNTS = [0.0, 20.0, 70.0, 95.0, 105.0, 110.0]
SPEEDS = (4.5, 7.0)  # m/s: full below, gone above (about 10 and 16 mph)
BASE_COUNTS = 385  # counts per 1.0 of torque request below 9 m/s (CarControllerParams.STEER_MAX_LOOKUP)
BLEND_S = 1.0
PARAMS_PERIOD = 1.0  # s


class FmLowSpeedFF:
  def __init__(self, CP, params=None):
    self.available = getattr(CP, "brand", None) == "rivian"
    self.params = params
    self.target = 0.0
    self.strength = 0.0
    self.frame = -1
    if self.available:
      if self.params is None:
        from openpilot.common.params import Params
        self.params = Params()
      self.read_params()
      self.strength = self.target  # the drive starts at the setting, no blend

  def read_params(self) -> None:
    enabled = self.params.get_bool(PARAM_ENABLED)
    try:
      step = int(self.params.get(PARAM_STEP, return_default=True) or STEP_DEFAULT)
    except (TypeError, ValueError):
      step = STEP_DEFAULT
    self.target = STEPS.get(step, STEPS[STEP_DEFAULT]) if enabled else 0.0

  @staticmethod
  def counts(wheel_deg: float, v_ego: float) -> float:
    """Feedforward at Full, in counts (>= 0), for a steering-wheel angle and speed."""
    return float(np.interp(abs(wheel_deg), TABLE_DEG, TABLE_COUNTS)) * float(np.interp(v_ego, SPEEDS, [1.0, 0.0]))

  def update(self, desired_curvature: float, v_ego: float, VM, lat_accel_factor: float, fm_scale: float = 1.0) -> float:
    """Extra feedforward in the controller's lateral acceleration units (positive to the right, like
    desired_curvature), or 0."""
    if not self.available:
      return 0.0
    self.frame += 1
    if self.frame % int(PARAMS_PERIOD / DT_CTRL) == 0:
      self.read_params()
    step = DT_CTRL / BLEND_S
    self.strength = float(np.clip(self.target, self.strength - step, self.strength + step))
    if self.strength <= 0.0 or v_ego >= SPEEDS[1] or desired_curvature == 0.0:
      return 0.0
    # controlsd's curvature is right-positive and -VM.calc_curvature(left-positive angle), so the VM curvature is -k
    wheel_deg = math.degrees(VM.get_steer_from_curvature(-desired_curvature, max(v_ego, 0.0), 0.0))
    torque = self.counts(wheel_deg, v_ego) * self.strength / BASE_COUNTS / max(fm_scale, 1e-3)
    return math.copysign(torque * lat_accel_factor, desired_curvature)
