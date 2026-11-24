"""FM Pilot same-car handoffs, using the asymmetric handoff design in the Build B note."""
import math

from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.selfdrive.controls.lib.fm_radar import ADMIT_HOLD, BLEND, HOLD, FmLeadTrace

RELEASE_MARGIN = 0.05
HOLD_FRAMES = round(0.5 / DT_MDL)
BLEND_FRAMES = round(0.75 / DT_MDL)
SAME_CAR_LATERAL = 1.5  # m
SAME_CAR_SPEED = 4.0   # m/s
HOLD_PATH_MARGIN = 2.0  # m from the model path where the camera sees the object


def same_car_distance_margin(distance):
  """4-10 m: 10% of the gap."""
  return max(4.0, min(10.0, 0.1 * distance))


def same_car(predicted, measured):
  if predicted is None or measured is None:
    return False
  return (abs(measured['dRel'] - predicted['dRel']) < same_car_distance_margin(predicted['dRel']) and
          abs(measured['yRel'] - predicted['yRel']) < SAME_CAR_LATERAL and
          abs(measured['vRel'] - predicted['vRel']) < SAME_CAR_SPEED)


def camera_on_path(lead_msg, model_path):
  if model_path is None:
    return True
  x, y = model_path.x, model_path.y
  distance = lead_msg.x[0]
  for i in range(1, min(len(x), len(y))):
    if x[i - 1] <= distance <= x[i] and x[i] > x[i - 1]:
      fraction = (distance - x[i - 1]) / (x[i] - x[i - 1])
      lateral = y[i - 1] + fraction * (y[i] - y[i - 1])
      return abs(lead_msg.y[0] - lateral) < HOLD_PATH_MARGIN
  return False


class FmLeadHandoff:
  """Track the last measurement for association and the last published estimate for prediction."""

  def __init__(self):
    self.reset()

  def reset(self):
    self.measurement = None
    self.estimate = None
    self.ego_speed = 0.0
    self.lateral_speed = 0.0
    self.held_frames = 0
    self.blend_frames = 0

  def predict(self, lead, ego_speed):
    if lead is None:
      return None
    speed_change = lead['aLeadK'] * DT_MDL
    next_speed = max(0.0, lead['vLead'] + speed_change)
    relative_speed = lead['vRel'] + next_speed - lead['vLead'] - (ego_speed - self.ego_speed)
    return dict(lead, dRel=max(0.0, lead['dRel'] + 0.5 * DT_MDL * (lead['vRel'] + relative_speed)),
                vRel=relative_speed, vLead=next_speed, vLeadK=max(0.0, lead['vLeadK'] + speed_change),
                yRel=lead['yRel'] + self.lateral_speed * DT_MDL)

  def update(self, lead, camera, camera_supports_hold, ego_speed, probability, trace: FmLeadTrace):
    prediction = self.predict(self.estimate, ego_speed)
    reference = self.predict(self.measurement, ego_speed)
    if trace.low_speed or trace.ghost is not None:
      self.reset()
      return lead

    if not lead['present']:
      if (self.measurement is None or not self.measurement['radar'] or not camera_supports_hold or
          self.held_frames >= HOLD_FRAMES or not same_car(reference, camera)):
        self.reset()
        return lead

      self.held_frames += 1
      trace.phase = HOLD
      trace.admission = ADMIT_HOLD
      self.measurement = reference
      self.estimate = dict(prediction, modelProb=probability)
      self.ego_speed = ego_speed
      return self.estimate

    if not all(math.isfinite(lead[key]) for key in ('dRel', 'yRel', 'vRel', 'vLead', 'vLeadK', 'aLeadK', 'aLeadTau')):
      self.reset()
      return lead

    if same_car(reference, lead):
      source_changed = self.measurement['radar'] != lead['radar'] or self.measurement['radarTrackId'] != lead['radarTrackId']
      if source_changed or self.held_frames:
        self.lateral_speed = 0.0
        if not self.blend_frames or self.held_frames:
          self.blend_frames = BLEND_FRAMES
      else:
        lateral_speed = (lead['yRel'] - self.measurement['yRel']) / DT_MDL
        self.lateral_speed = 0.75 * self.lateral_speed + 0.25 * max(-2.5, min(2.5, lateral_speed))
    else:
      self.reset()

    out = lead
    if self.blend_frames:
      out = dict(lead)
      # Compare against the kinematic prediction, so an expected change in gap isn't mistaken for good or bad news.
      for field in ('dRel', 'vRel', 'vLead', 'vLeadK', 'aLeadK'):
        if lead[field] > prediction[field]:
          out[field] = prediction[field] + (lead[field] - prediction[field]) / self.blend_frames
      trace.phase = BLEND
      self.blend_frames -= 1

    self.held_frames = 0
    self.measurement = lead
    self.estimate = out
    self.ego_speed = ego_speed
    return out
