# FM Pilot

Frankie Malekian's personal fork of [AdventurePilot](https://github.com/AdventurePilotDev/AdventurePilot) (`dev`), for a
2022 Rivian R1S (Gen 1) with a comma 4. Branch: `fm-dev-max`. Everything AdventurePilot does is kept; FM Pilot adds the
features below. Every FM Pilot commit starts with "FM Pilot".

## Features

| Feature | Settings (Sunnylink page) | Code | Default |
|---|---|---|---|
| Accel Boost, one of three (exclusive, never stacked): **Off**; **Gas** (Gas Override Boost: after the driver presses the gas while the model holds the car below what the lead and set speed allow, adds up to +0.2 m/s² to the model's acceleration for that engagement); **Predictive** (sunnypilot's predictive boost: adds acceleration whenever the model predicts no reason to slow, up to +0.4 / 0.25 / 0.1 m/s² for aggressive / standard / relaxed, fading out in tight curves and when the model wants to stop) | `FM_AccelBoostMode` (Cruise) | `selfdrive/controls/lib/fm_boost_mode.py`, `accel_boost.py`, `sunnypilot/selfdrive/controls/lib/accel_boost.py`, `longitudinal_planner.py` | Gas |
| Lane Guard: small correction only when the planned path brings the body too close to a line or edge; exclusive with lane centering | `LaneGuardEnabled` (Steering) | `selfdrive/controls/lib/lane_guard.py`, `sunnypilot/selfdrive/controls/lib/fm_lateral.py` | off |
| Rear-axle off-tracking compensation: widens tight turns by the rear axle's cut-in | `RivianOfftrackCompensationEnabled`, `…Gain` (Steering) | `sunnypilot/selfdrive/controls/lib/rivian_offtrack.py`, `fm_lateral.py` | off |
| Ignore Stationary Radar Ghosts: a standing-still radar point (under 1 m/s) is never paired with a camera lead that is clearly moving (over 5 m/s, 11 mph) and not braking hard (gentler than -1.5 m/s²); that car is then followed from the camera. Stops guardrail/barrier points on ramps and curves from braking the truck. The low-speed radar path (under 4 m/s) is untouched | `FM_RadarStationaryGuard` (Cruise) | `selfdrive/controls/radard.py`, `sunnypilot/selfdrive/controls/lib/fm_radar.py` | off |
| Lead Detection Sensitivity: how sure the camera has to be (25–50%) that a car is ahead before it counts as a lead and is paired with the radar. Lower picks leads up sooner and farther away, so the lead-following planner gets radar distance and speed earlier; pair with Ignore Stationary Radar Ghosts | `FM_LeadDetectionThreshold` (Cruise) | `selfdrive/controls/radard.py`, `sunnypilot/selfdrive/controls/lib/fm_radar.py` | 50 (openpilot's value) |
| Quiet road-edge block: small gray message, no sound | `FM_QuietRoadEdgeBlock` (Steering) | `selfdrived.py`, `sunnypilot/selfdrive/selfdrived/events.py`, `cereal/custom.capnp` | off |
| Road Edge Turn Desire: signaling toward a road edge (20–45 mph) plans a turn; every turn starts on the blinker alone and holds until the blinker is cancelled | `FM_RoadEdgeTurnDesire`, `FM_RoadEdgeTurnMaxSpeed` (Steering) | `sunnypilot/selfdrive/controls/lib/fm_turn_desire.py`, `desire_helper.py` | off |
| Turn Boost: tighter requests during low-speed blinker turns, per side | `FM_TurnBoostEnabled`, `FM_TurnBoostLeft`, `FM_TurnBoostRight` (Steering) | `sunnypilot/selfdrive/controls/lib/fm_turn_assist.py`, `fm_lateral.py` | off (1.10 / 1.20) |
| Turn Hold: keeps a model from letting go of a turn halfway (time stopped doesn't count toward its 8 s). Option Hold Path Source: when starting a turn from a stop or a crawl (under about 2 mph), also takes the turn from the planned path, never tighter than Turn Path Tightest Radius. Option Hold Adds Your Steering: steering into the turn never releases the hold and sets the curve to hold, up to full lock (not limited by the radius setting); steering against it still releases | `FM_TurnHoldEnabled`, `FM_TurnHoldPathSource`, `FM_TurnHoldDriverAdds` (Steering) | `fm_turn_assist.py` | off |
| Turn Lead: starts blinker turns on time when the model turns in late (from Turn Lead Min Speed, 2–7 mph, to about 16 mph), from the planned path's curve 1.3 s ahead, handing back as the truck catches up; never tighter than Turn Path Tightest Radius (shared with Hold Path Source). Can run with Turn Hold; one at a time recommended | `FM_TurnLeadEnabled`, `FM_TurnLeadMinSpeed`, `FM_TurnPathMinRadius` (Steering) | `fm_turn_assist.py` | off (7 mph; 12 m, 8–20) |
| Higher Torque (branch `fm-dev-torque`, testing): up to 481 steering torque under about 20 mph (AP: 385), back to AP's curve by 38 mph, highway unchanged. **Panda Allows Higher Torque** (off-road only, applies next drive) raises the panda's limit; **Higher Torque** picks Off / Headroom (more only where it used to max out) / Full (every low-speed request about 25% stronger), live, blending over 1 s. With the panda setting off, every mode stays at 385 | `FM_PandaHighTorque`, `FM_TorqueMode` (Steering, Torque Control) | `opendbc/safety/modes/rivian.h`, `opendbc/car/rivian/carcontroller.py`, `values.py`, `sunnypilot/selfdrive/controls/lib/fm_torque_mode.py`, `latcontrol_torque.py` and `latcontrol_torque_v0.py` (both torque controllers sunnypilot can pick) | off |
| Custom (spliced) model: load a model compiled on the device in place of its base model; on-screen "Splice Model / Custom Model \| ms" label | `FM_CustomModelEnabled` (Models); `FM_CustomModelPath` / `FM_CustomModelBaseRef` set by the build script | `sunnypilot/modeld_v2/modeld.py`, `fm_build_custom_model.sh`, `fm_custom_label.py`, HUD renderers | on once built |
| On-device compile fix for the comma GPU (QCOM, IMAGE=1) miscompile of the point-policy residual add (NaN lane lines, meta and pose) | `FM_CONTIGUOUS_OUTPUTS` (build script sets `linear_2`) | `sunnypilot/modeld_v2/fm_compile_fixes.py`, `compile_modeld.py` | used by the build script |
| Model check tools | n/a | `fm_check_model.py` (NaN check, `--compare`), `fm_debug_nan.py`, `fm_debug_variants.py` | n/a |

Removed: dual model mode (two small models do not fit the comma 4's 50 ms model step).

## Credits

- **Lane Guard** and its Rivian tune: gerrylum's rx-wb fork, commit `17ea338` (MIT).
- **Rear-axle off-tracking compensation**: gerrylum's rx-wb fork, commit `ce7e5c8` (MIT). AdventurePilot `dev` has since
  added its own version from the same work (`0d85a260b`); the two will be merged at the next sync.
- **Gas Override Boost**: openpilot #39015 (Harald Schäfer), via sunnypilot master (MIT). Upstream AdventurePilot now
  carries the same file; ours gives way to it at the next sync.
- **Predictive Boost**: sunnypilot `AccelBoostSP`, branch `booooost` (James Vecellio-Grant / Discountchubbs, Nov 3, 2026;
  MIT), logic unchanged. When sunnypilot ships it and AP takes it in, ours gives way to theirs and the mode selector stays.
- **Turn Lead**, **Hold Path Source** and **Hold Adds Your Steering**: StarPilot's Model Turn Assist "turn lead" and
  curvature hold (plan source, driver-confirmed wind), `openpilot/starpilot/lateral/model_turn_assist.py`, Dom branch,
  Nov 2026, firestar5683 and StarPilot contributors (MIT). FM Pilot changes: smooth speed fade for Turn Lead, its low end
  as a setting, and the radius cap as a setting.
- **Higher Torque**: the 481 low-speed limit and its panda envelope come from AdventurePilot `5270cbba` (Uncle-Tony,
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
Turn Boost / Turn Hold, Quiet road-edge block, and the compile fix.
