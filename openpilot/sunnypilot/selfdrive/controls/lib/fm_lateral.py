"""
FM Pilot: lateral additions applied to the requested curvature in controlsd, after AP's lane centering and before
clip_curvature, so they work the same with torque and angle steering.

  - Lane Guard (from gerrylum's rx-wb, with its Rivian tune): a small correction only when the planned path brings the
    body too close to a lane line or road edge. Exclusive with AP's lane centering: it never runs while lane centering
    is on.
  - Rear-axle off-tracking compensation (from rx-wb): widens turns by gain x the rear axle's cut-in. Rivian only.
"""
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.lane_guard import STOCK, TUNES, LaneGuard
from openpilot.sunnypilot.selfdrive.controls.lib.rivian_offtrack import offtrack_curvature

PARAM_LANE_GUARD = "LaneGuardEnabled"
PARAM_OFFTRACK = "RivianOfftrackCompensationEnabled"
PARAM_OFFTRACK_GAIN = "RivianOfftrackCompensationGain"
PARAMS_PERIOD = 1.0  # s


class FmLateral:
  def __init__(self, CP, params):
    self.CP = CP
    self.params = params
    self.lane_guard = LaneGuard(TUNES.get(CP.brand, STOCK))
    self.lane_guard_enabled = False
    self.offtrack_gain = 0.0
    self.left_hand_traffic = False
    self.frame = 0
    self.read_params()

  def read_params(self) -> None:
    self.lane_guard_enabled = self.params.get_bool(PARAM_LANE_GUARD)
    self.left_hand_traffic = self.params.get_bool("IsRhdDetected")
    gain = 0.0
    if self.CP.brand == 'rivian' and self.params.get_bool(PARAM_OFFTRACK):
      gain = float(self.params.get(PARAM_OFFTRACK_GAIN, return_default=True) or 0.0)
    self.offtrack_gain = gain

  def update(self, desired_curvature: float, model_v2, CS, lat_active: bool, lane_centering_enabled: bool) -> float:
    self.frame += 1
    if self.frame % int(PARAMS_PERIOD / DT_CTRL) == 0:
      self.read_params()

    # Lane Guard and lane centering are exclusive: lane centering wins if both are on
    guard_active = self.lane_guard_enabled and not lane_centering_enabled and lat_active and not CS.steeringPressed
    guard = self.lane_guard.update(CS.vEgo, model_v2, guard_active, bool(CS.leftBlinker or CS.rightBlinker), self.left_hand_traffic)
    if not lat_active:
      return desired_curvature

    desired_curvature += guard
    return offtrack_curvature(desired_curvature, self.CP.wheelbase, self.offtrack_gain)
