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
| (each mapping lands with its feature's PR) | |

## Settings pages

Sunnylink Steering page: sunnypilot's own sections stay where they are; FM's follow in this order: FM Custom Torque
Settings, Feedforward Settings, FM Smoothing Settings, Model Lateral Smoothing, Turns. Lane Keeping is in Steering >
Lane Centering. sunnypilot's Blendful toggle is hidden from the Models page (Sunnylink and the device): Blendful is one
choice of the Lane Keeping selector.

## Credits

- **openpilot** (comma.ai), **sunnypilot** (sunnypilot contributors) and **AdventurePilot** (AdventurePilotDev,
  Uncle-Tony and contributors), all MIT: the base of everything here.
- Credits for each borrowed feature are added with the PR that ports it.
- Everything else here is FM Pilot's own, built on AdventurePilot, sunnypilot and openpilot (MIT).
