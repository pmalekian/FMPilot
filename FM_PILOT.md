# FM Pilot

Frankie Malekian's personal fork of [AdventurePilot](https://github.com/AdventurePilotDev/AdventurePilot) for a 2022
Rivian R1S Launch Edition (Gen 1) with a comma 4 on the torque harness. Every FM Pilot commit starts with "FM Pilot".

## Branch `fm-dev-sync`

- **Base:** AdventurePilot `dev-next-src` `cf3ad0ec` (the torque line: `steerControlType = torque`; the `-a` branches
  are the angle line) merged with sunnypilot master `c410198c` (Nov 24, 2026). opendbc stays copied in the repo
  (`opendbc_repo/`) as AdventurePilot does: re-synced to sunnypilot's opendbc pin `3db0b06` with AP's Rivian changes
  kept. Everything AdventurePilot and sunnypilot do is kept; FM Pilot adds the features below.
- **Install:** `pmalekian/fm-dev-sync` (comma's installer takes GitHub owner/branch). Switch branches from the device's
  Software settings; uninstalling wipes settings. The older branches (`fm-dev-nav-settle`, `fm-dev-max`, ...) stay as
  fallbacks.
- **How it was built:** one GitHub PR per feature into `fm-dev-sync`, in the order of
  `claude/FM_DEV_SYNC_BUILD_PLAN.md` (project doc). Code is ported from `fm-dev-nav-settle` `ca08aac9`.
- **Defaults:** every FM setting starts at stock/off on a fresh install. On the first boot of `fm-dev-sync` a one-time
  migration copies the values of the old FM settings into the new or reshaped ones (see Settings migration). Behavior
  that used to be a toggle and is now fixed is listed in `FM_FOLDED_DEFAULTS.md`.
- **Dropped from earlier FM branches:** all map features (Nav Assist `fm_navassistd`, Download Nav Map, Map Turn Desire,
  Nav Turn Speed, `sunnypilot/fm_nav/`, `fmNavTurn`, `tools/fm/nav.py`); Turn Boost; Road Edge Turn Desire; Lead Shadow
  Log; Radar Badge. FM's own copies of the v25 model list ports, Blendful's base file, Predictive Boost and gerrylum's
  off-tracking are replaced by the upstream ones already in the base.

## Features

Added PR by PR; each row names its settings (Sunnylink page), code and default.

| Feature | Settings | Code | Default |
|---|---|---|---|
| Max Torque (Rivian): the most steering torque openpilot may use under about 20 mph, 385-580 in steps of 15, back to AP's curve by about 38 mph (envelope [9,25,27] m/s -> [Max,295,275]; car code the larger of AP's curve and [9,17,25,27] -> [Max,332,295,275]), highway unchanged. The panda enforces exactly this level (safety param bit 1 + level 1-13 in bits 2-5). Above 385 also, folded: torque builds 5 per frame instead of 3 below 25 mph (panda below 12.5 m/s), no 95% cap past 90° of wheel, and both torque controllers (v0 and v1) may request up to the higher curve (Headroom: requests unscaled, so only where they would have saturated; blends in over 1 s; with Lateral Jerk or NNLC the PID's torque-unit limits follow it). Overriding takes a firmer hand as the limit rises, because the panda's and car code's driver-torque rule (unchanged) is built on the limit: below about 10 m/s openpilot's torque is fully cancelled at about 2.9 Nm of your steering at 385, 3.4 Nm at 475, 3.9 Nm at 580 (the same as on `fm-dev-nav-settle`). 385 = stock AP: nothing changes. Off-road only, applies next drive | `FM_CustomMaxTorque` (Steering > FM Custom Torque Settings) | `opendbc/sunnypilot/car/rivian/values.py`, `opendbc/sunnypilot/car/interfaces.py`, `opendbc/car/rivian/carcontroller.py`, `values.py`, `opendbc/safety/modes/rivian.h`, `sunnypilot/selfdrive/controls/lib/fm_torque_mode.py`, `latcontrol_torque.py`, `latcontrol_torque_v0.py` | 385 |
| Release Guard (Rivian): while you steer into a slow turn tighter than openpilot wants, limits the torque controller's P push against your hands to 75 or 150 counts (converted with latAccelFactor), so letting go doesn't swing the wheel past center. Only while you press into a curve the truck is already turning (> 0.003 1/m), under 15 mph, gone by 20; fades over 0.5 s after you let go, ends at once when P flips or your hand reverses. Feedforward, friction, integrator and all limits untouched; off with Lateral Jerk / NNLC. v0 and v1. Logged in `fmControls.releaseGuard` (`turns.py` release line) | `FM_ReleaseGuardLimit` drop-down Stock / 75 / 150 (Steering > FM Custom Torque Settings) | `sunnypilot/selfdrive/controls/lib/fm_release_guard.py`, `latcontrol_torque.py`, `latcontrol_torque_v0.py`, `controlsd_ext.py` | Stock |
| Low-Speed Steering Feedforward (Rivian): adds torque for the wheel angle the asked curve needs (vehicle model; table [0,10,30,65,120,180] deg -> [0,20,70,95,105,110] counts at Full, fitted on the Nov 11 drives, never above the measured gap), full under 10 mph (4.5 m/s), gone by 16 mph (7 m/s); per side, blends over 1 s. Off both sides = off (master toggle folded). Stands down with Lateral Jerk / NNLC. v0 and v1; logged in `fmControls.lowSpeed.feedforward` | `FM_LowSpeedFFLeft`, `FM_LowSpeedFFRight`: Off / Gentle 40 / Medium 60 / Firm 80 / Full 100 (Steering > Feedforward Settings) | `sunnypilot/selfdrive/controls/lib/fm_low_speed_ff.py`, `latcontrol_torque.py`, `latcontrol_torque_v0.py` | Off |
| Low-Speed Correction Strength (Rivian; was Calmer Low-Speed Correction): scales the torque controller's proportional gain (KP_INTERP) below 7 m/s (16 mph), back to normal by 9 m/s (20 mph); highway unchanged; blends over 1 s. The slider alone decides it (toggle folded). v0 and v1, also with Lateral Jerk / NNLC (as before); logged in `fmControls.lowSpeed.gainScale` | `FM_LowSpeedCorrectionStrength` 40-100 % (Steering > FM Smoothing Settings) | `sunnypilot/selfdrive/controls/lib/fm_low_speed_gain.py`, `latcontrol_torque.py`, `latcontrol_torque_v0.py` | 100 % (stock) |
| Lane Keeping selector: one choice of Off, Lane Centering (AP's), Lane Guard or Blendful, so two never run together. The device's Lane Centering toggle still turns Lane Centering on and off | `FM_LaneKeepMode` drop-down (Steering > Lane Centering) | `sunnypilot/selfdrive/controls/lib/fm_lane_keep_mode.py`, `lane_centering.py`, `modeld_v2/modeld.py`, `ui_state.py`, `webrtcd.py` | Off |
| Lane Guard (from gerrylum's rx-wb, Rivian tune): a small correction only when the planned path brings the body too close to a lane line (in curves) or a road edge; fades out while you steer, pauses around lane changes. Added to the request after lane centering, before AP's off-tracking | Lane Keeping = Lane Guard | `selfdrive/controls/lib/lane_guard.py`, `sunnypilot/selfdrive/controls/lib/fm_lateral.py`, `controlsd.py` | Off |
| Blendful (sunnypilot's): works from 0.3 m/s instead of 5 m/s, so it centers in slow traffic; the comma 4 draws only its blue center line (no readout), only while it's selected | Lane Keeping = Blendful | `sunnypilot/modeld_v2/blendful.py`, `selfdrive/ui/sunnypilot/onroad/blendful.py`, `mici/onroad/model_renderer.py` | Off |
| Model Lateral Smoothing: the small-model choice sets the curvature filter and the plan look-ahead offset. The big-model choice sets only the curvature filter, including on Chestnut or Jetlink | `FM_SmallModelSmoothingMode`, `FM_BigModelSmoothingMode`: Model Default / 0.00 s / 0.10 s (Steering > Model Lateral Smoothing) | `modeld_v2/fm_lateral_smoothing.py`, both modelds | Model Default |
| Calmer Crawl-Speed Steering raises AP's action-head curvature speed floor from 1.0 to 2.5 m/s. Calmer Straight Pull-Away uses the higher floor only from a straight stop and blends back over 0.5 s on a turn signal, turn desire, or a sustained steering press | `FM_LowSpeedCurvatureFloor`, `FM_StraightPullAwayFloor` (Steering > FM Smoothing Settings) | `modeld_v2/fm_pull_away.py`, both modelds and the Jetlink action adapter | Off |
| Turn Hold: during a blinker turn under 15 mph (gone by 20), keeps a model from letting go of the turn halfway: holds the request at 90% of the turn's peak, the floor dropping 15% of the peak per second after it; releases at 75° turned, after 8 s of turning, when you steer against the turn, or when the turn ends (a blinker cancel fades it over 0.5 s). Built in (folded, always on with it): Turn Nudge (a 0.8 Nm nudge into the turn for 0.15 s at 2-8 mph adds up to 0.01 1/m for 0.6 s when the path just ahead bends that way), Hold Through Early Correction (straightening at the start of a turn, under 5 mph and 20°, passes through), Ignore Brief Hand Blips (a press against the turn counts after 0.25 s or from 1.5 Nm), Hold Starts at Turn-In (the 8 s count from 5° turned; before 10° a press against the turn passes through unless it lasts 1.5 s), Hold Only Counts While Steering (no learning, clock or degrees while openpilot isn't steering), Hold Ignores Crawl Spikes (below 5 mph the peak only learns a request kept up for 0.3 s). Options: Hold Path Source (from a stop or a crawl under 2 mph, also take the turn from the planned path, never tighter than Turn Path Tightest Radius), Hold Adds Your Steering (steering into the turn keeps your curve, up to full lock), Hold Follows the Model's Exit (past 45°, fades out over 0.5 s once the model asks under half the peak), Stop Before Turning Starts Over (a stop before 10° turned starts the turn over). Only changes the requested curvature, before Lane Guard and AP's off-tracking; every limit after it applies | `FM_TurnHoldEnabled`, `FM_TurnHoldPathSource`, `FM_TurnHoldDriverAdds`, `FM_TurnHoldFollowExit`, `FM_TurnRestartAfterStop` (Steering > Turns > Turn Hold Settings); `FM_TurnPathMinRadius` 8-20 m (on Turn Lead Settings, shared with Turn Lead and Turn Nudge) | `sunnypilot/selfdrive/controls/lib/fm_turn_assist.py`, `fm_lateral.py`, `controlsd.py` | Off (12 m) |
| Turn Lead: starts blinker turns on time when the model turns in late. From Turn Lead Min Speed (2-7 mph; full strength about 2.2 mph above it) to about 16 mph it reads the planned path's curve 1.3 s ahead (4-14 m) and, when that bends harder toward the turn than the model's request, asks for 85% of it, handing back as the truck's own curve catches up (gone once the truck turns as tightly as the path asks); stays out when stopping short of that point, when the model steers away from the turn, in a lane change and while openpilot isn't steering; never tighter than Turn Path Tightest Radius. Runs before Turn Hold, so Turn Hold holds what Turn Lead asked for; one at a time makes a turn easier to judge. Options: **Adaptive Look-Ahead** (reads 0.5 s ahead for a gentle curve, 50 m radius or wider, up to 0.9 s for a tight one, 12.5 m or tighter, at least 3 m, judged by the plan's tightest bend in the next 17 m; off is the fixed 1.3 s), **Wait for the Corner** (acts only once the point where the plan has turned 10° toward the turn, 2-3 m into a 12 m corner, is within **Turn Lead Start Distance**, 3-10 m, default 5, fading in over the 2 m before it), **Turn Lead Yields to Your Hands** (a press against the turn while Turn Lead is adding, 0.25 s or 1.5 Nm at once, pauses it before 10° turned, back 0.5 s after you let go, and ends it for the rest of the turn after that; a press before Turn Lead starts changes nothing; `000000af--8` +52.3 s he pushed out at 1.75 Nm and Turn Lead kept adding 0.017-0.040 1/m). Only changes the requested curvature, like Turn Hold | `FM_TurnLeadEnabled` (Steering > Turns); `FM_TurnLeadMinSpeed`, `FM_TurnLeadAdaptive`, `FM_TurnLeadGate`, `FM_TurnLeadGateDist`, `FM_TurnLeadYield`, `FM_TurnPathMinRadius` (Steering > Turns > Turn Lead Settings) | `fm_turn_assist.py` | Off (7 mph, 5 m, 12 m); options off |
| Finish Turn After Signal Cancels: when the turn desire ends (the blinker cancels or self-cancels) while the truck is still turning that way (0.01 1/m, a 100 m radius, or tighter), Turn Lead and Turn Hold keep the turn going until the truck straightens out (under 0.008 1/m), for at most 6 s; it ends at once on a press against the turn (0.25 s, or 0.5 Nm), openpilot not steering, a lane change or 20 mph, and an active hold then fades over 0.5 s as for a cancel. Only Turn Assist's own idea of the turn is kept: the model's desire, the Turning alert and lane changes follow the blinker. Built in (folded, always on with it): Carry Only at Turn-In (no carry when the desire ends after 45° turned or once the truck is straightening out, curving under 70% of the turn's tightest; that cancel is the end of the turn). Logged as `fmControls.turnAssist.source` = carry | `FM_TurnCarryThrough` (Steering > Turns, needs Turn Hold or Turn Lead) | `fm_turn_assist.py` | Off |
| Smooth Turn Start: what Turn Lead, Turn Hold and its nudge add on top of the model's request rises through a 0.12 s low-pass (about 90% in 0.3 s) instead of in one step, and still drops at once; the model's own request is never filtered | `FM_TurnPhaseIn` (Steering > FM Smoothing Settings, first; needs Turn Hold or Turn Lead) | `fm_turn_assist.py` | Off |
| Turn Assist log: every control step `fmControls.turnAssist` records the turn being assisted and where it came from (`source`: the blinker's desire, or `carry` for Finish Turn After Signal Cancels), the request in and out, what Turn Lead, Hold Adds Your Steering, Hold Path Source, Turn Hold's floor, Turn Nudge and a fading hold each added, what Smooth Turn Start held back, Turn Hold's peak, floor, degrees and seconds counted, why it let go, the degrees turned, Turn Lead Yields to Your Hands (`leadYield` 1 paused, 2 off for the turn) and restarts (`turns.py` assist line). Turn Boost's field is gone. Logging only | none | `fm_turn_assist.py`, `controlsd_ext.py` | always |
| Rivian steerLimitTimer 0.8 s (AP 0.4): the steering-required alert and anything reading steerSaturated wait 0.8 s of saturation | none | `opendbc/car/rivian/interface.py` | always |
| Off-tracking near-zero curvature crash fix: AP's rear off-tracking is worked in curvature, so a curvature below ~1e-154 can no longer raise OverflowError in controlsd; same result everywhere else | none | `sunnypilot/selfdrive/controls/lib/rivian_offtrack.py` | always |
| Quiet road-edge block: the road-edge lane change block is a small gray message with no sound instead of the orange alert and beep. Folded (was `FM_QuietRoadEdgeBlock`) | none (always on) | `selfdrived.py`, `sunnypilot/selfdrive/selfdrived/events.py` (`fmLaneChangeRoadEdgeQuiet`) | always |
| FM log tools: fmlog, timeline, events, turns (with the path-vs-request turn-in timing from `fm-dev-max` `d189ccc`), lead, settle, sway, dm, jetlink, maclog, and their tests. They read `fm-dev-sync` drives with the repo's schema and drives from `fm-dev-nav-settle` and earlier with a copy of that branch's schema, picked per file (`FM_SCHEMA=old|new` forces one) | none | `tools/fm/` (`fmlog.py`, `schema/fm-dev-nav-settle/`) | n/a |

## Log messages

FM's log data is in FM's own messages (`cereal/custom.capnp`, the free `CustomReserved` slots taken from the top, each
keeping its struct ID), not in fields added to sunnypilot's structs. Filled in by the feature PRs that publish them.

| Message (Event field) | From | What |
|---|---|---|
| `fmControls` @145 | controlsd, 100 Hz | Turn Assist, Release Guard, Low-Speed Feedforward / Correction Strength |
| `fmModel` @144 | modeld / modeld_v2, 20 Hz | Queue Guard, Settle After Turn |
| `fmPlan` @143 | plannerd, 20 Hz | Accel Boost mode (0 Off, 1 Gas, 2 Sunny Boost, 3 EVP Set Speed, 4 AP Accel Control, 5 OP Personality); the amount is `longitudinalPlan.accelBoost` |
| `fmCarState` @142 | card, 100 Hz | Rivian torque readout: EPAS taking torque, cap now, Max Torque in force |
| `fmRadarLead` @141 | radard, 20 Hz | lead source, confidence, cutoffs, ghost drops, decisions for both lead slots |
| `fmJetlinkStatus` @140 | fm_jetlinkstatusd, 1 Hz | which Jetlink model drives, link state |

FM's one event, `fmLaneChangeRoadEdgeQuiet`, is `OnroadEventSP` @38, after AP's Jetlink events (@35-37). Slot 10
(@136) is sunnypilot's `gpsLocationSP`. `tools/fm` reads both this layout and the older one (see its README).

## Settings migration

On the first boot of `fm-dev-sync` (manager start, before unset params get their defaults),
`sunnypilot/system/fm_migration.py` copies the values of the old FM settings into the new or reshaped ones, so the
truck drives as before. The old keys stay declared and are never written, so switching back to `fm-dev-nav-settle`
restores the old setup (switch from the device's Software settings; uninstalling wipes settings). A migration only
writes a new key that is still unset, runs once (`FM_SyncMigration` lists the ones that ran), and is retried on the next
boot if it fails. A fresh install has none of the old keys and starts at stock/off. Settings whose meaning didn't
change keep their keys.

| Setting | From the old settings |
|---|---|
| Lane Keeping (`FM_LaneKeepMode`) | kept if set on `fm-dev-nav-settle`; else `LaneCenteringEnabled` -> Lane Centering, then `LaneGuardEnabled` -> Lane Guard, then sunnypilot's `Blendful` -> Blendful; none -> nothing written (Off) |
| Low-Speed Feedforward (`FM_LowSpeedFFLeft`, `FM_LowSpeedFFRight`) | `FM_LowSpeedFFEnabled` off -> both Off; on -> `FM_LowSpeedFFStepLeft` / `FM_LowSpeedFFStepRight` (0-4; unset -> Gentle) |
| Low-Speed Correction Strength (`FM_LowSpeedCorrectionStrength`) | `FM_LowSpeedGainEnabled` off -> 100; on -> `FM_LowSpeedGainPercent` (40-100; unset -> 70) |
| Release Guard (`FM_ReleaseGuardLimit`) | `FM_ReleaseGuard` off -> Stock (0); on -> `FM_ReleaseGuardMax` (75 / 150; unset -> 150) |
| Model Lateral Smoothing (`FM_SmallModelSmoothingMode`, `FM_BigModelSmoothingMode`) | Small: `FM_LateralSmoothingEnabled` off -> Model Default, on -> `FM_LateralSmoothingSeconds` 0.00 -> 0.00 s, 0.10 or 0.20 (or unset) -> 0.10 s. Big: `FM_BigModelSmoothing` off -> Model Default, on -> 0.10 s |
| Calmer Crawl-Speed Steering and Calmer Straight Pull-Away | Keep `FM_LowSpeedCurvatureFloor` and `FM_StraightPullAwayFloor`; existing values carry forward under the same keys |
| Turn Hold Settings, Smooth Turn Start, Turn Path Tightest Radius (`FM_TurnHoldEnabled`, `FM_TurnHoldPathSource`, `FM_TurnHoldDriverAdds`, `FM_TurnHoldFollowExit`, `FM_TurnRestartAfterStop`, `FM_TurnPhaseIn`, `FM_TurnPathMinRadius`) | None needed: same keys and meaning, existing values carry forward (on `fm-dev-nav-settle` Follows the Model's Exit and Smooth Turn Start defaulted on, and that stored value stays). Turn Nudge, Hold Through Early Correction, Ignore Brief Hand Blips, Hold Starts at Turn-In, Hold Only Counts While Steering and Hold Ignores Crawl Spikes are folded, nothing copied |
| Turn Lead Settings (`FM_TurnLeadEnabled`, `FM_TurnLeadMinSpeed`, `FM_TurnLeadAdaptive`, `FM_TurnLeadGate`, `FM_TurnLeadGateDist`, `FM_TurnLeadYield`) | None needed: same keys and meaning, existing values carry forward. On `fm-dev-nav-settle` Adaptive Look-Ahead and Wait for the Corner defaulted on; fresh installs start with them off here, a stored value stays |
| Finish Turn After Signal Cancels (`FM_TurnCarryThrough`) | None needed: same key and meaning, an existing value carries forward. On `fm-dev-nav-settle` it defaulted on; fresh installs start off here. Carry Only at Turn-In (`FM_TurnCarryTurnInOnly`) is folded (always on with it), nothing copied; a stored off no longer turns it off |
| Max Torque (`FM_CustomMaxTorque`) | `FM_PandaHighTorque` off -> 385; on -> `FM_MaxTorque` rounded to the 15 grid (580 -> 580; unset -> the old default 480 -> 475). Faster ramp, full cap past 90° and the Higher Torque mode are folded, nothing copied |

## Settings pages

Sunnylink Steering page: sunnypilot's own sections stay where they are; FM's follow in this order: FM Custom Torque
Settings, Feedforward Settings, FM Smoothing Settings, Model Lateral Smoothing, Turns. Lane Keeping is in Steering >
Lane Centering. sunnypilot's Blendful toggle is hidden from the Models page (Sunnylink and the device): Blendful is one
choice of the Lane Keeping selector. FM Smoothing Settings, in order: Smooth Turn Start, Low-Speed Correction Strength,
Calmer Crawl-Speed Steering, Calmer Straight Pull-Away. Turns: the Turn Hold, Turn Lead and Finish Turn After Signal
Cancels toggles (the last enabled by Turn Hold or Turn Lead). Turn Hold Settings
(shown when Turn Hold is on) holds Hold Path Source, Hold Adds Your Steering, Hold Follows the Model's Exit and Stop
Before Turning Starts Over. Turn Lead Settings (shown when Turn Lead is on) holds Turn Lead Min Speed, Adaptive
Look-Ahead, Wait for the Corner, Turn Lead Start Distance, Turn Lead Yields to Your Hands and Turn Path Tightest Radius
(shared with Hold Path Source, so with Turn Lead off it keeps its value but isn't shown).

## Credits

- **openpilot** (comma.ai), **sunnypilot** (sunnypilot contributors) and **AdventurePilot** (AdventurePilotDev,
  Uncle-Tony and contributors), all MIT: the base of everything here.
- **Max Torque**: the higher low-speed limit's panda envelope shape and the high curve's shape come from AdventurePilot
  `5270cbba` (Uncle-Tony, May 16, 2026, MIT; 481 there, backed off upstream in `5c5a1c98`), the fixed 481 replaced by the
  setting. Headroom, the slider, the faster ramp and full cap past 90° are FM Pilot's own.
- **Near-zero curvature crash fix** and **Rivian steerLimitTimer 0.8**: EVPilot `a0500916` (Nov 2, 2026) and the
  `steerLimitTimer` line of `9214103e` (Oct 28, 2026), Gerry / gerrylum (MIT); the fix applied to AP's off-tracking
  here, the tests are FM Pilot's.
- **Lane Guard**: gerrylum's rx-wb fork, commit `17ea338` (Gerry / gerrylum, MIT), with its Rivian tune, used as is
  (`selfdrive/controls/lib/lane_guard.py` and its tests). The selector and its wiring are FM Pilot's.
- **Blendful**: sunnypilot #2073 (James Vecellio-Grant / Discountchubbs, MIT); FM changes only its minimum speed
  (0.3 m/s, the value its author runs) and what the comma 4 draws.
- **Big-model lateral smoothing**: zoompilot's BigModelSmoothing, Zeph Leggett, `zoompilot/zoompilot` `6cb59d3` (MIT).
- **Crawl-speed curvature floor**: sunnypilot #2091, James Vecellio-Grant (MIT). Straight pull-away comes from `fm-dev-nav-settle` `ca08aac9`.
- **Hold Path Source** and **Hold Adds Your Steering** (Turn Hold): StarPilot's Model Turn Assist curvature hold (plan
  source, driver-confirmed wind), `openpilot/starpilot/lateral/model_turn_assist.py`, Dom branch, Nov 2026, firestar5683
  and StarPilot contributors (MIT); same thresholds, ported from `fm-dev-nav-settle` `ca08aac9`. The rest of Turn Hold
  and Smooth Turn Start are FM Pilot's own.
- **Turn Lead**: StarPilot's Model Turn Assist "turn lead" (`TURN_LEAD_*`, `_plan_circle_curvature`, the dual probe),
  same file, branch and authors (MIT), ported from `fm-dev-nav-settle` `ca08aac9`. FM Pilot changes: a smooth speed fade
  in place of StarPilot's hard cutoff at 7 m/s, the low end as a setting (StarPilot: fixed 3 m/s), and the radius cap as
  a setting (StarPilot: fixed 0.12 1/m) defaulting gentler, to 12 m. Adaptive Look-Ahead, Wait for the Corner and Turn
  Lead Yields to Your Hands are FM Pilot's own.
- **Finish Turn After Signal Cancels** (with Carry Only at Turn-In): FM Pilot's own, ported from `fm-dev-nav-settle`
  `ca08aac9` (`5a5ab9e`, `812b394` there).
- Credits for each borrowed feature are added with the PR that ports it.
- Everything else here is FM Pilot's own, built on AdventurePilot, sunnypilot and openpilot (MIT).
