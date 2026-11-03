#!/usr/bin/env python3
"""Steering timeline of one segment: what the model asked, what was finally requested, what the truck did, torque
requested and sent, lane lines, the driver's hands, blinker and turn desire, in steps of --step seconds.

usage: timeline.py rlog.zst [rlog.zst ...] [--step 2] [--start 0] [--end 60] [--each]
  Segments of one drive are joined; --start/--end are seconds from the start of the drive, and each row says which
  segment it is in ("--3 +12.4s").

Sign conventions (this fork, Rivian): curvature and lateral accel are positive to the RIGHT (controlsd's
-VM.calc_curvature); steering angle and hand torque are positive to the LEFT; actuators/sent torque are positive to the
LEFT. Lane line y is positive to the right. See README.md.
"""
import numpy as np

from fmlog import MPH, load_drives, log_args, model_name, version


def main(drive, a):
  evs, T = drive.evs, drive.t
  cs, ctl, cc, co, mv, sp = [], [], [], [], [], []
  for e in evs:
    w = e.which()
    if w == "carState":
      c = e.carState
      cs.append((T(e), c.vEgo * MPH, c.steeringAngleDeg, c.steeringTorque, float(c.steeringPressed), float(c.leftBlinker),
                 float(c.rightBlinker), float(c.steerFaultTemporary), float(c.accFaulted)))
    elif w == "controlsState":
      ctl.append((T(e), e.controlsState.desiredCurvature, e.controlsState.curvature))
    elif w == "carControl":
      cc.append((T(e), float(e.carControl.latActive), e.carControl.actuators.torque))
    elif w == "carOutput":
      co.append((T(e), e.carOutput.actuatorsOutput.torque))
    elif w == "modelV2":
      m = e.modelV2
      L = m.laneLines
      mv.append((T(e), m.action.desiredCurvature, L[1].y[0], L[2].y[0], m.laneLineProbs[1], m.laneLineProbs[2], m.modelExecutionTime))
    elif w == "modelDataV2SP":
      sp.append((T(e), float(e.modelDataV2SP.laneTurnDirection.raw)))
  if not cs or not ctl or not cc or not mv:
    print(f"{drive.label()}: missing carState / controlsState / carControl / modelV2")
    return
  cs, ctl, cc, mv = (np.array(x, float) for x in (cs, ctl, cc, mv))
  co = np.array(co, float) if co else np.full((1, 2), -1e9)
  sp = np.array(sp, float) if sp else np.zeros((1, 2))
  print(f"{drive.label()}: version {version(evs)}, model {model_name(evs)}, model run {np.median(mv[:, 6]) * 1000:.0f} ms median")
  print("time        speed  wheel(min..max)  lat | model / final / truck curvature e-3 | torque req (sent) | R line m (prob) "
        "L line | hands Nm (pressed) | blink turn fault")
  skipped = 0.0
  for s in np.arange(max(a.start, 0), min(a.end, cs[-1, 0]), a.step):
    m = (cs[:, 0] >= s) & (cs[:, 0] < s + a.step)
    if not m.any():
      skipped += a.step
      continue
    if skipped:
      print(f"  ... no log for {skipped:.0f} s (missing segment?)")
      skipped = 0.0
    w = lambda arr: (arr[:, 0] >= s) & (arr[:, 0] < s + a.step)  # noqa: E731
    k, c, o, v, p = w(ctl), w(cc), w(co), w(mv), w(sp)
    turn = {0: "-", 1: "L", 2: "R"}.get(int(sp[p, 1].max()) if p.any() else 0, "?")
    blink = ("L" if cs[m, 5].max() else "") + ("R" if cs[m, 6].max() else "") or "-"
    fault = ("steer " if cs[m, 7].max() else "") + ("acc" if cs[m, 8].max() else "") or "-"
    sent = f"{co[o, 1].mean():+.2f}" if o.any() else "  n/a"
    print(f"{drive.where(max(s, cs[m, 0][0])):11s} {cs[m, 1].mean():5.1f}  {cs[m, 2].min():6.0f}..{cs[m, 2].max():6.0f}  {cc[c, 1].mean():.1f} | "
          f"{mv[v, 1].mean() * 1e3:+6.2f} / {ctl[k, 1].mean() * 1e3:+6.2f} / {ctl[k, 2].mean() * 1e3:+6.2f} | "
          f"{cc[c, 2].mean():+.2f} ({sent}) | {mv[v, 3].mean():+.2f} ({mv[v, 5].mean():.2f}) {mv[v, 2].mean():+.2f} ({mv[v, 4].mean():.2f}) | "
          f"{np.abs(cs[m, 3]).max():4.1f} ({cs[m, 4].mean():.1f}) | {blink:5s} {turn:4s} {fault}")


if __name__ == "__main__":
  args = log_args(__doc__, **{"--step": {"type": float, "default": 2.0}, "--start": {"type": float, "default": 0.0},
                              "--end": {"type": float, "default": 1e9}}).parse_args()
  for d in load_drives(args.rlog, args.each):
    main(d, args)
