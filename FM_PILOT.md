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
| Low-Speed Feedforward (`FM_LowSpeedFFLeft`, `FM_LowSpeedFFRight`) | `FM_LowSpeedFFEnabled` off -> both Off; on -> `FM_LowSpeedFFStepLeft` / `FM_LowSpeedFFStepRight` (0-4; unset -> Gentle) |
| Low-Speed Correction Strength (`FM_LowSpeedCorrectionStrength`) | `FM_LowSpeedGainEnabled` off -> 100; on -> `FM_LowSpeedGainPercent` (40-100; unset -> 70) |
| Release Guard (`FM_ReleaseGuardLimit`) | `FM_ReleaseGuard` off -> Stock (0); on -> `FM_ReleaseGuardMax` (75 / 150; unset -> 150) |
| Max Torque (`FM_CustomMaxTorque`) | `FM_PandaHighTorque` off -> 385; on -> `FM_MaxTorque` rounded to the 15 grid (580 -> 580; unset -> the old default 480 -> 475). Faster ramp, full cap past 90° and the Higher Torque mode are folded, nothing copied |

## Settings pages

Sunnylink Steering page: sunnypilot's own sections stay where they are; FM's follow in this order: FM Custom Torque
Settings, Feedforward Settings, FM Smoothing Settings, Model Lateral Smoothing, Turns. Lane Keeping is in Steering >
Lane Centering. sunnypilot's Blendful toggle is hidden from the Models page (Sunnylink and the device): Blendful is one
choice of the Lane Keeping selector.

## Credits

- **openpilot** (comma.ai), **sunnypilot** (sunnypilot contributors) and **AdventurePilot** (AdventurePilotDev,
  Uncle-Tony and contributors), all MIT: the base of everything here.
- **Max Torque**: the higher low-speed limit's panda envelope shape and the high curve's shape come from AdventurePilot
  `5270cbba` (Uncle-Tony, May 16, 2026, MIT; 481 there, backed off upstream in `5c5a1c98`), the fixed 481 replaced by the
  setting. Headroom, the slider, the faster ramp and full cap past 90° are FM Pilot's own.
- **Near-zero curvature crash fix** and **Rivian steerLimitTimer 0.8**: EVPilot `a0500916` (Nov 2, 2026) and the
  `steerLimitTimer` line of `9214103e` (Oct 28, 2026), Gerry / gerrylum (MIT); the fix applied to AP's off-tracking
  here, the tests are FM Pilot's.
- Credits for each borrowed feature are added with the PR that ports it.
- Everything else here is FM Pilot's own, built on AdventurePilot, sunnypilot and openpilot (MIT).
