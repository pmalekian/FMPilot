#!/usr/bin/env python3
"""Report Build B shadow logs, or replay lead admission and the real longitudinal planner on an rlog."""
import argparse
import csv
from collections import Counter, defaultdict
from contextlib import nullcontext
from dataclasses import dataclass
from pathlib import Path

import fmlog

SLOTS = ('leadOne', 'leadTwo')
LEAD_METRICS = ('source', 'confidence', 'radarTrackId', 'dRel', 'vRel', 'phase', 'admission',
                'ghostDropped', 'ghostTrackId', 'lowSpeedLead', 'hysteresis')
PLAN_SOURCES = {v: k for k, v in fmlog.log.LongitudinalPlan.LongitudinalPlanSource.schema.enumerants.items()}


@dataclass(frozen=True)
class LeadFrame:
  time: float
  variant: str
  decisions: tuple


@dataclass(frozen=True)
class PlanFrame:
  time: float
  variant: str
  source: str
  accel: float
  recorded_source: str
  recorded_accel: float
  long_active: bool
  lead_time: float | None


def name(radar, camera, smooth):
  return f'{radar:.0%}/{camera:.0%} {"on" if smooth else "off"}'


def display(decision):
  enum_fields = ('source', 'phase', 'admission')
  return {key: str(getattr(decision, key)) if key in enum_fields else getattr(decision, key) for key in LEAD_METRICS}


def shadow_report(events, start):
  results, plans = [], []
  radar_time = None
  long_active = False
  for event in events:
    t = (event.logMonoTime - start) / 1e9
    kind = event.which()
    if kind == 'carControl':
      long_active = event.carControl.longActive
    elif kind == 'radarState':
      radar_time = t
    elif kind == 'fmRadarLead' and event.valid:
      msg = event.fmRadarLead
      if len(msg.decisions) != 2:
        continue
      t = radar_time if radar_time is not None and 0 <= t - radar_time < 0.05 else t
      active = name(msg.radarThreshold, msg.cameraThreshold, msg.smoothHandoffs)
      results.append(LeadFrame(t, f'active {active}', tuple(map(display, msg.decisions))))
      for shadow in msg.shadow:
        variant = name(shadow.radarThreshold, shadow.cameraThreshold, shadow.smoothHandoffs)
        results.append(LeadFrame(t, variant, tuple(map(display, shadow.decisions))))
    elif kind == 'longitudinalPlan' and event.valid:
      plan = event.longitudinalPlan
      source = str(plan.longitudinalPlanSource)
      plans.append(PlanFrame(t, 'recorded', source, plan.aTarget, source, plan.aTarget, long_active, radar_time))
  return results, plans


def replay(events, start, guard=None):
  from openpilot.cereal import custom, messaging
  from opendbc.car.structs import car
  from openpilot.common.gps import get_gps_location_service
  from openpilot.common.params import Params
  from openpilot.common.prefix import OpenpilotPrefix
  from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
  from openpilot.selfdrive.controls.radard import RadarD
  from openpilot.sunnypilot.selfdrive.controls.lib.fm_radar import FmLeadConfig, PARAM_GUARD

  logged_params = fmlog.raw_params(events)
  cp = next((e.carParams for e in events if e.which() == 'carParams'), None)
  cp_sp = next((e.carParamsSP for e in events if e.which() == 'carParamsSP'), None)
  if cp is None and 'CarParams' in logged_params:
    cp = messaging.log_from_bytes(logged_params['CarParams'], car.CarParams)
  if cp_sp is None and 'CarParamsSP' in logged_params:
    cp_sp = messaging.log_from_bytes(logged_params['CarParamsSP'], custom.CarParamsSP)
  if cp is None or cp_sp is None:
    raise ValueError('replay requires recorded CarParams and CarParamsSP')
  if guard is None:
    guard = logged_params.get(PARAM_GUARD) == b'1'

  configs = [FmLeadConfig(radar, camera, smooth)
             for radar, camera in ((.50, .50), (.50, .40), (.30, .50), (.30, .40)) for smooth in (False, True)]
  lead_frames, plan_frames = [], []
  with OpenpilotPrefix():
    params = Params()
    for key in (k.decode() for k in params.all_keys()):
      if (raw := logged_params.get(key)) and (value := params.cpp2python(key, raw)) is not None:
        params.put(key, value, block=True)
    gps = get_gps_location_service(params)
    services = ('modelV2', 'radarTracks', 'radarState', 'carState', 'carControl', 'controlsState', 'vehicleParameters',
                'selfdriveState', 'liveMapDataSP', 'carStateSP', 'selfdriveStateSP', gps, 'fmNavTurn', 'modelDataV2SP')
    sm = messaging.SubMaster(services)
    radars = [RadarD(cp, cp_sp, cp.radarDelay) for _ in configs]
    planners = [LongitudinalPlanner(cp, cp_sp) for _ in configs]
    for radar, config in zip(radars, configs, strict=True):
      radar.fm_radar.config, radar.fm_radar.guard = config, guard

    models = {e.logMonoTime: e.modelV2 for e in events if e.which() == 'modelV2'}
    radar_time = None
    for event in events:
      kind = event.which()
      t = (event.logMonoTime - start) / 1e9
      if kind in services:
        sm.data[kind] = getattr(event, kind)
        sm.logMonoTime[kind] = event.logMonoTime
        sm.recv_frame[kind] += 1
        sm.seen[kind] = True

      # RadarD and plannerd both poll modelV2. Their published messages tell us which model frame each saw.
      if kind == 'radarState':
        model_time = event.radarState.mdMonoTime
        if model_time not in models or not sm.seen['radarTracks'] or not sm.seen['carState']:
          continue
        sm.data['modelV2'], sm.logMonoTime['modelV2'] = models[model_time], model_time
        sm.frame += 1
        for radar, config in zip(radars, configs, strict=True):
          radar.update(sm, sm['radarTracks'])
          if radar.fm_lead_msg is not None:
            decisions = tuple(map(display, radar.fm_lead_msg.decisions))
            lead_frames.append(LeadFrame(t, name(config.radar, config.camera, config.smooth), decisions))
        radar_time = t

      elif kind == 'longitudinalPlan':
        recorded = event.longitudinalPlan
        if recorded.modelMonoTime not in models or radar_time is None:
          continue
        sm.data['modelV2'], sm.logMonoTime['modelV2'] = models[recorded.modelMonoTime], recorded.modelMonoTime
        for radar, planner, config in zip(radars, planners, configs, strict=True):
          sm.data['radarState'] = radar.radar_state
          planner.sla.update_buttons(sm['selfdriveStateSP'].buttonsReleaseToggle)
          planner.update(sm)
          plan_frames.append(PlanFrame(t, name(config.radar, config.camera, config.smooth), PLAN_SOURCES[int(planner.mpc.source)],
                                       float(planner.output_a_target), str(recorded.longitudinalPlanSource),
                                       recorded.aTarget, sm['carControl'].longActive, radar_time))
  return lead_frames, plan_frames, guard


def summarize(leads, plans, baseline, braking_window=None, braking_decel=-0.5):
  by_variant = defaultdict(list)
  for frame in leads:
    by_variant[frame.variant].append(frame)
  baseline_by_time = {frame.time: frame.decisions for frame in leads
                      if frame.variant == baseline or (baseline == 'active' and frame.variant.startswith('active '))}
  plan_by_time = {frame.time: frame for frame in plans if frame.variant == baseline}
  observed_by_lead = {frame.lead_time: frame for frame in plans if frame.variant == 'recorded'}
  observed = [p for p in plans if p.variant == 'recorded']
  if observed:
    print(f'  recorded planner sources={dict(Counter(p.source for p in observed))}'
          f' minimum aTarget={min(p.accel for p in observed):.3f} m/s²'
          f' long active={sum(p.long_active for p in observed)}')
  for variant, frames in by_variant.items():
    for i, slot in enumerate(SLOTS):
      counts = Counter()
      observed_sources = Counter()
      jumps = {'dRel': 0.0, 'vRel': 0.0}
      previous = None
      for frame in frames:
        decision = frame.decisions[i]
        present = decision['source'] != 'none'
        counts[decision['source']] += 1
        counts['guard rejects'] += decision['ghostDropped']
        counts['holds'] += decision['phase'] == 'hold'
        counts['blends'] += decision['phase'] == 'blend'
        reference = baseline_by_time.get(frame.time)
        if present and reference is not None and reference[i]['source'] == 'none':
          counts['candidate only'] += 1
          if (observed := observed_by_lead.get(frame.time)) is not None:
            observed_sources[observed.source] += 1
        if previous is not None:
          counts['dropouts'] += previous['source'] != 'none' and not present
          counts['source switches'] += previous['source'] not in ('none', decision['source']) and present
          counts['track swaps'] += (present and previous['source'] == decision['source'] == 'radar' and
                                    previous['radarTrackId'] != decision['radarTrackId'])
          if present and previous['source'] != 'none':
            for metric in jumps:
              jumps[metric] = max(jumps[metric], abs(decision[metric] - previous[metric]))
        previous = decision

      print(f'  {variant:20s} {slot}: radar={counts["radar"]} camera={counts["camera"]} candidate only={counts["candidate only"]}'
            f' loss={counts["dropouts"]} switches={counts["source switches"]} swaps={counts["track swaps"]}'
            f' guard={counts["guard rejects"]} hold={counts["holds"]} blend={counts["blends"]}'
            f' max jump={jumps["dRel"]:.2f} m/{jumps["vRel"]:.2f} m/s')
      if observed_sources:
        print(f'      observed planner on candidate-only frames: {dict(observed_sources)}')

    variant_plans = [p for p in plans if p.variant == variant]
    if variant_plans:
      sources = Counter(p.source for p in variant_plans)
      biggest_jump = max((abs(b.accel - a.accel) for a, b in zip(variant_plans, variant_plans[1:], strict=False)), default=0.0)
      deltas = [abs(p.accel - plan_by_time[p.time].accel) for p in variant_plans if p.time in plan_by_time]
      print(f'      planner frames={len(variant_plans)} sources={dict(sources)} max accel jump={biggest_jump:.3f} m/s²'
            f' max delta vs {baseline}={max(deltas, default=0):.3f} m/s²'
            f' long active={sum(p.long_active for p in variant_plans)}')
      if braking_window is not None:
        onset = next((p.time for p in variant_plans if braking_window[0] <= p.time <= braking_window[1] and
                      p.source.startswith('lead') and p.accel <= braking_decel), None)
        print(f'      first lead-limited accel <= {braking_decel:g} m/s²: {onset:.3f} s' if onset is not None else
              '      braking window: no lead-limited target crossed the requested threshold')


def write_csv(writer, segment, leads, plans):
  for frame in leads:
    for slot, decision in zip(SLOTS, frame.decisions, strict=True):
      writer.writerow(dict(segment=segment, time=frame.time, variant=frame.variant, slot=slot, **decision))
  for frame in plans:
    writer.writerow({
      'segment': segment, 'time': frame.time, 'variant': frame.variant, 'source': frame.source, 'accel': frame.accel,
      'recorded_source': frame.recorded_source, 'recorded_accel': frame.recorded_accel,
      'long_active': frame.long_active, 'lead_time': frame.lead_time,
    })


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('logs', nargs='+')
  parser.add_argument('--replay', action='store_true', help='rerun radard and plannerd, requires a native openpilot build')
  parser.add_argument('--guard', action=argparse.BooleanOptionalAction, default=None, help='override the recorded guard setting')
  parser.add_argument('--csv', type=Path, help='save the full timeline for labeling real/false leads and braking events')
  parser.add_argument('--braking-window', nargs=2, type=float, metavar=('START', 'END'), help='segment-local seconds')
  parser.add_argument('--braking-decel', type=float, default=-0.5, help='deceleration threshold in m/s²')
  args = parser.parse_args()
  with args.csv.open('w', newline='') if args.csv else nullcontext() as output:
    if output is not None:
      writer = csv.DictWriter(output, fieldnames=('segment', 'time', 'variant', 'slot', *LEAD_METRICS, 'accel',
                                                  'recorded_source', 'recorded_accel', 'long_active', 'lead_time'))
      writer.writeheader()
    for path in fmlog.expand(args.logs):
      keep = None if args.replay else {'fmRadarLead', 'radarState', 'longitudinalPlan', 'carState', 'carControl'}
      events = fmlog.read(path, keep)
      start = next((e.logMonoTime for e in events if e.which() == 'carState'), events[0].logMonoTime if events else 0)
      if args.replay:
        leads, plans, guard = replay(events, start, args.guard)
        baseline = '50%/50% off'
        print(f'{path}: open-loop replay, guard={"on" if guard else "off"}')
      else:
        leads, plans = shadow_report(events, start)
        baseline = 'active'
        print(f'{path}: recorded shadow decisions and observed planner')
      if not leads:
        parser.exit(1, 'No usable lead frames. ' +
                    ('Replay requires modelV2, radarTracks and radarState.\n' if args.replay else
                     'Shadow reports require Build B fmRadarLead messages.\n'))
      summarize(leads, plans, baseline, args.braking_window, args.braking_decel)
      if output is not None:
        segment = fmlog.route_segment(path)
        write_csv(writer, str(path) if segment[0] is None else f'{segment[0]}--{segment[1]}', leads, plans)


if __name__ == '__main__':
  main()
