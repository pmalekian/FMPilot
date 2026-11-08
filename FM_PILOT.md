# FM Pilot

Frankie Malekian's personal fork of [AdventurePilot](https://github.com/AdventurePilotDev/AdventurePilot) (`dev`), for a
2022 Rivian R1S (Gen 1) with a comma 4. Branch: `fm-dev-max`. Everything AdventurePilot does is kept; FM Pilot adds the
features below. Every FM Pilot commit starts with "FM Pilot".

## Features

| Feature | Settings (Sunnylink page) | Code | Default |
|---|---|---|---|
| Accel Boost, one of three (exclusive, never stacked): **Off**; **Gas** (Gas Override Boost: after the driver presses the gas while the model holds the car below what the lead and set speed allow, adds up to +0.2 m/s² to the model's acceleration for that engagement); **Predictive** (sunnypilot's predictive boost: adds acceleration whenever the model predicts no reason to slow, up to +0.4 / 0.25 / 0.1 m/s² for aggressive / standard / relaxed, fading out in tight curves and when the model wants to stop) | `FM_AccelBoostMode` (Cruise > Driving Personality Settings); what it added is logged as `longitudinalPlanSP.fmAccelBoost` | `selfdrive/controls/lib/fm_boost_mode.py`, `accel_boost.py`, `sunnypilot/selfdrive/controls/lib/accel_boost.py`, `longitudinal_planner.py` | Gas |
| Lane keeping mode, one of: **Off**, **Lane Centering** (AP's), **Lane Guard**, **Blendful**. Replaces the separate Lane Centering and Lane Guard toggles; seeded once at boot from them (lane centering wins if both were on) | `FM_LaneKeepMode` (Steering > Lane Centering) | `sunnypilot/selfdrive/controls/lib/fm_lane_keep_mode.py`, `sunnypilot/system/params_migration.py` | from old toggles |
| Lane Guard: small correction only when the planned path brings the body too close to a line or edge | `FM_LaneKeepMode` = Lane Guard | `selfdrive/controls/lib/lane_guard.py`, `sunnypilot/selfdrive/controls/lib/fm_lateral.py` | off |
| Blendful (alpha): trust-weighted nudge toward the lane center added to the model's curvature in modeld; weights each line by the model's confidence, yields when the model goes around something, off below 5 m/s and during any desire or lane change. Logged in `modelDataV2SP` (`blendful*`); blue lane center line on the comma 4 | `FM_LaneKeepMode` = Blendful | `sunnypilot/modeld_v2/blendful.py`, `modeld.py`, `selfdrive/ui/sunnypilot/onroad/blendful.py` | off |
| Rear-axle off-tracking compensation: widens tight turns by the rear axle's cut-in | `RivianOfftrackCompensationEnabled`, `…Gain` (Steering > Lane Centering) | `sunnypilot/selfdrive/controls/lib/rivian_offtrack.py`, `fm_lateral.py` | off |
| Ignore Stationary Radar Ghosts: a standing-still radar point (under 1 m/s) is never paired with a camera lead that is clearly moving (over 5 m/s, 11 mph) and not braking hard (gentler than -1.5 m/s²); that car is then followed from the camera. Stops guardrail/barrier points on ramps and curves from braking the truck. The low-speed radar path (under 4 m/s) is untouched | `FM_RadarStationaryGuard` (Cruise > Smart Cruise Control) | `selfdrive/controls/radard.py`, `sunnypilot/selfdrive/controls/lib/fm_radar.py` | off |
| Lead Detection Sensitivity: how sure the camera has to be (25–50%) that a car is ahead before it counts as a lead and is paired with the radar. Lower picks leads up sooner and farther away, so the lead-following planner gets radar distance and speed earlier; pair with Ignore Stationary Radar Ghosts | `FM_LeadDetectionThreshold` (Cruise > Smart Cruise Control) | `selfdrive/controls/radard.py`, `sunnypilot/selfdrive/controls/lib/fm_radar.py` | 50 (openpilot's value) |
| Quiet road-edge block: small gray message, no sound (always on; `FM_QuietRoadEdgeBlock` is no longer read) | n/a | `selfdrived.py`, `sunnypilot/selfdrive/selfdrived/events.py`, `cereal/custom.capnp` | always |
| Road Edge Turn Desire: signaling toward a road edge (20–45 mph) plans a turn; every turn starts on the blinker alone and holds until the blinker is cancelled | `FM_RoadEdgeTurnDesire`, `FM_RoadEdgeTurnMaxSpeed` (Models > Model Behavior) | `sunnypilot/selfdrive/controls/lib/fm_turn_desire.py`, `desire_helper.py` | off |
| Turn Boost: tighter requests during low-speed blinker turns, per side | `FM_TurnBoostEnabled`, `FM_TurnBoostLeft`, `FM_TurnBoostRight` (Steering > FM Pilot: Turn Tweaks) | `sunnypilot/selfdrive/controls/lib/fm_turn_assist.py`, `fm_lateral.py` | off (1.10 / 1.20) |
| Turn Hold: keeps a model from letting go of a turn halfway (time stopped doesn't count toward its 8 s). Option Hold Path Source: when starting a turn from a stop or a crawl (under about 2 mph), also takes the turn from the planned path, never tighter than Turn Path Tightest Radius. Option Hold Adds Your Steering: steering into the turn never releases the hold and sets the curve to hold, up to full lock (not limited by the radius setting); steering against it still releases | `FM_TurnHoldEnabled`, `FM_TurnHoldPathSource`, `FM_TurnHoldDriverAdds` (Steering > FM Pilot: Turn Tweaks) | `fm_turn_assist.py` | off |
| Turn Lead: starts blinker turns on time when the model turns in late (from Turn Lead Min Speed, 2–7 mph, to about 16 mph), from the planned path's curve 1.3 s ahead, handing back as the truck catches up; never tighter than Turn Path Tightest Radius (shared with Hold Path Source). Can run with Turn Hold; one at a time recommended | `FM_TurnLeadEnabled`, `FM_TurnLeadMinSpeed`, `FM_TurnPathMinRadius` (Steering > FM Pilot: Turn Tweaks) | `fm_turn_assist.py` | off (7 mph; 12 m, 8–20) |
| Higher Torque (testing): up to **Max Torque** (405–580 in steps of 25, default 480) under about 20 mph (AP: 385), back to AP's curve by 38 mph, highway unchanged. **Panda Allows Higher Torque** and **Max Torque** (off-road only, apply next drive) set the panda's limit: the chosen level goes to the panda in the safety param (bits 2–5) and the panda enforces exactly it; **Higher Torque** picks Off / Headroom (more only where it used to max out) / Full (every low-speed request scaled by Max Torque / 385), live, blending over 1 s. With the panda setting off, every mode stays at 385. The slider is for finding the right number; afterwards it goes back to one fixed high value | `FM_PandaHighTorque`, `FM_MaxTorque`, `FM_TorqueMode` (Steering, Torque Control) | `opendbc/safety/modes/rivian.h`, `opendbc/car/rivian/carcontroller.py`, `values.py`, `sunnypilot/selfdrive/controls/lib/fm_torque_mode.py`, `latcontrol_torque.py` and `latcontrol_torque_v0.py` (both torque controllers sunnypilot can pick) | off |
| Faster Torque Ramp (testing): with Panda Allows Higher Torque on, steering torque may rise 5 per frame instead of 3 under 25 mph (full torque in about 1 s instead of 1.6 s). The panda allows it below 12.5 m/s (28 mph) only when the drive started with it on; the 250 ms real-time limit (125) and everything above those speeds are unchanged. Off-road only, applies next drive | `FM_TorqueFastRamp` (Steering, Torque Control) | `opendbc/safety/modes/rivian.h`, `opendbc/car/rivian/carcontroller.py`, `values.py`, `opendbc/sunnypilot/car/interfaces.py` | off |
| Full Torque Past 90° (testing): with Panda Allows Higher Torque on, skips AP's 95% torque cap past 90° of steering wheel (`HIGH_ANGLE_CAP_FRAC`), so tight turns can use the full Max Torque. Car code only; the panda's limit is the same. Off-road only, applies next drive | `FM_TorqueFullCapHighAngle` (Steering, Torque Control) | `opendbc/car/rivian/carcontroller.py`, `opendbc/sunnypilot/car/interfaces.py` | off |
| Accel Boost bar: green bar in the comma 4 side panel (in place of the confidence ball) while Gas or Predictive boost is on; fills to that mode's maximum only while the boosted model target sets the speed | `FM_AccelBoostIndicator` (Cruise > Driving Personality Settings) | `selfdrive/ui/sunnypilot/mici/onroad/accel_boost_indicator.py`, `fm_boost_mode.py` | off |
| Torque readout: one line at the top center of the comma 4 road view, `sent / limit now / Max Torque ✓`. Sent is the raw `torqueOutputCan` (arrow = direction); limit now is the speed curve incl. the 95% past-90° cap; Max Torque is the panda limit this drive (385 without Panda Allows Higher Torque); ✓/✗ is the EPAS `ToiActive` bit (taking the overlay, not how much it applied). Amber at 95% of the limit, red ✗ when refused, grey while steering is off. Logged in `carStateSP` (`fmEpasToiActive`, `fmSteerCap`, `fmMaxTorque`) | always on (Rivian) | `selfdrive/ui/sunnypilot/onroad/fm_torque_readout.py`, `selfdrive/ui/mici/onroad/hud_renderer.py`, `opendbc/car/rivian/carcontroller.py`, `carstate.py`, `cereal/custom.capnp` | on |

Removed: dual model mode (two small models do not fit the comma 4's 50 ms model step); the custom (splice) model, its
build script, the on-device compile fix (`FM_CONTIGUOUS_OUTPUTS`) and the model check tools (Nov 8, 2026, before the move
to sunnypilot's v25 model list; in git history before that date).

## Credits

- **Lane Guard** and its Rivian tune: gerrylum's rx-wb fork, commit `17ea338` (MIT).
- **Rear-axle off-tracking compensation**: gerrylum's rx-wb fork, commit `ce7e5c8` (MIT). AdventurePilot `dev` has since
  added its own version from the same work (`0d85a260b`); the two will be merged at the next sync.
- **Gas Override Boost**: openpilot #39015 (Harald Schäfer), via sunnypilot master (MIT). Upstream AdventurePilot now
  carries the same file; ours gives way to it at the next sync.
- **Blendful**: sunnypilot #2073, commit `97135965e` (Oct 29, 2026; Blendful math by James Vecellio-Grant /
  Discountchubbs, with royjr, rav4kumar, nayan8teen and Jason Wen; MIT), `blendful.py` and its tests unchanged. FM Pilot
  changes: switched by `FM_LaneKeepMode` instead of its own toggle, and only the blue center line is drawn on screen.
- **Accel Boost bar**: sunnypilot #2090 (rav4kumar and Jason Wen, Nov 4, 2026; MIT; open there). FM Pilot changes: Gas and
  Predictive, reads the logged boost, own setting.
- **Predictive Boost**: sunnypilot `AccelBoostSP`, branch `booooost` (James Vecellio-Grant / Discountchubbs, Nov 3, 2026;
  MIT), logic unchanged. When sunnypilot ships it and AP takes it in, ours gives way to theirs and the mode selector stays.
- **Turn Lead**, **Hold Path Source** and **Hold Adds Your Steering**: StarPilot's Model Turn Assist "turn lead" and
  curvature hold (plan source, driver-confirmed wind), `openpilot/starpilot/lateral/model_turn_assist.py`, Dom branch,
  Nov 2026, firestar5683 and StarPilot contributors (MIT). FM Pilot changes: smooth speed fade for Turn Lead, its low end
  as a setting, and the radius cap as a setting.
- **Higher Torque**: the 481 low-speed limit and its panda envelope shape come from AdventurePilot `5270cbba` (Uncle-Tony,
  May 16, 2026, MIT), backed off upstream in `5c5a1c98`. FM Pilot keeps only the low-speed raise and adds the
  Headroom mode and the settings.
- **Lead Detection Sensitivity**: FrogPilot's "Lead Detection Sensitivity" (`LeadDetectionThreshold`), FrogAi, FrogPilot
  branch (Oct 31, 2026), `frogpilot/common/frogpilot_variables.py` and `selfdrive/controls/radard.py` (MIT). FM Pilot
  keeps 50% as the default (FrogPilot: 35%) and adds it to sunnypilot's radard alongside the ghost guard.
- Everything else here is FM Pilot's own, built on AdventurePilot, sunnypilot and openpilot (MIT).

## Upstreaming to AdventurePilot

FM Pilot's history is personal: one early commit (`9a3d302db`) adds four features at once, and dual model mode was added
and later removed. To offer a feature upstream, start a fresh branch from AP's `dev` and carry over just that feature's
files and settings (listed in the table), with its tests, as one commit. Good candidates: Road Edge Turn Desire,
Turn Boost / Turn Hold, and the Quiet road-edge block.
