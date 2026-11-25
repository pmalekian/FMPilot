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
| FM log tools: fmlog, timeline, events, turns (with the path-vs-request turn-in timing from `fm-dev-max` `d189ccc`), lead, settle, sway, dm, jetlink, maclog, and their tests. They read drives from `fm-dev-nav-settle` and earlier with a copy of that branch's schema | none | `tools/fm/` (`schema/fm-dev-nav-settle/`) | n/a |

## Settings migration

Lands with the settings framework (PR 3); filled in there.

## Credits

- **openpilot** (comma.ai), **sunnypilot** (sunnypilot contributors) and **AdventurePilot** (AdventurePilotDev,
  Uncle-Tony and contributors), all MIT: the base of everything here.
- Credits for each borrowed feature are added with the PR that ports it.
- Everything else here is FM Pilot's own, built on AdventurePilot, sunnypilot and openpilot (MIT).
