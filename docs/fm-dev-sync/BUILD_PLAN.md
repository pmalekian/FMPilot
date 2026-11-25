# fm-dev-sync: PR order and build plan (final, Nov 24, 2026; nothing built yet)

Goes with `claude/FM_DEV_SYNC_INVENTORY.md` (what is kept, dropped, reshaped) and `claude/FM_FOLDED_DEFAULTS.md` (behavior
folded into defaults). Code reference for every port: `fm-dev-nav-settle` `ca08aac9`.

## Ground rules (from Frankie)
- **No drive-log replays.** Frankie does his own drives and testing. The build session makes sure the code is right:
  build, unit tests, lint, Sunnylink compile, panda safety tests, and synthetic same-input checks of ported modules
  against their `fm-dev-nav-settle` versions (random/constructed inputs only, no rlogs).
- **Merge everything without waiting for drives.** Milestones M1 / M2 / M3 are merge points only.
- **Torque PR:** behavior is unchanged from how he uses it today (panda limit follows Max Torque; 385 = stock), so it
  doesn't need his review. It does get an independent code review in a **fresh Claude Opus session** (prompt in the
  project chat of Nov 24).
- New/reshaped settings default to stock/off; a one-time migration copies his current values.
- Commits: one logical change each, message starts `FM Pilot`, author `Frankie Malekian
  <107580415+pmalekian@users.noreply.github.com>`, borrowed code credited in the message, the source file and `FM_PILOT.md`.

## Verified facts
- `dev-next-src` is the **torque** line: `opendbc/car/rivian/interface.py` sets `steerControlType = torque`.
  `dev-a-next-src` is the angle line (needs the xnor extreme angle harness, sets `ANGLE_HARNESS`).
- **opendbc:** AP keeps opendbc copied inside the repo (`opendbc_repo/`, a normal folder, ~543 files incl. AP's Rivian
  work); sunnypilot uses a submodule pointing at `sunnypilot/opendbc`. Decision: **follow AP** (copied in-repo). On each
  sunnypilot sync, re-sync the copy to sunnypilot's new opendbc pin while keeping AP's and FM's Rivian changes.
- Free log slots in the base: `CustomReserved10`–`19` (`log.capnp` `customReserved10 @136` … `customReserved19 @145`).

## Step 0: base branch (no FM features)
1. Cut `fm-dev-sync` from AP `dev-next-src` `cf3ad0ec`.
2. Merge sunnypilot master (`c410198c` as of Nov 24; check for newer). Expected conflicts: opendbc copy (re-sync to
   sunnypilot's pin), `custom.capnp` / `log.capnp` (GPS fuser #2112), `params_keys.h`, settings JSON (recompile).
3. Checks: build; selfdrive / modeld / controls / Sunnylink suites (known pre-existing failures listed, not FM's); Rivian
   panda safety tests; Sunnylink compile + validator; ruff.

## Step 1: groundwork
- **PR 1 Docs and tools:** fresh `FM_PILOT.md`, `FM_PILOT_HANDOFF.md`, `FM_FOLDED_DEFAULTS.md` (from the project copy);
  `tools/fm` minus `nav.py` and its test, incl. `turns.py` timing tweak from `fm-dev-max` `d189ccc`; earlier branch history
  moved into `FM_PILOT_FINDINGS.md`.
- **PR 2 Log schema, fresh numbering (no placeholders):**
  - FM's own messages and per-frame FM data go into **FM-owned structs in the free `CustomReserved` slots** (e.g. one
    FM controls/turn-assist message, `fmRadarLead`, `fmJetlinkStatus`, `fmLowSpeed`), instead of extra fields appended to
    sunnypilot's structs (`carControlSP`, `carStateSP`, `modelDataV2SP`, `longitudinalPlanSP`). Then sunnypilot or AP
    adding fields to their structs later never collides with FM.
  - FM events (quiet road-edge block, Jetlink alerts if not already AP's, turn/settle alerts) are appended after AP's
    last event number (AP uses `@27–34` for angle-lane events, `@35–37` for Jetlink).
  - Dropped features' fields (Nav Turn, Turn Boost, Lead Shadow, Radar Badge) are simply not carried.
  - `tools/fm` keeps a copy of the `fm-dev-nav-settle` schema and picks old or new per route, so older drives still read.
  - Accel Boost mode values: 0 Off, 1 Gas, 2 Sunny Boost, 3 EVP Set Speed, 4 AP Accel Control, 5 OP Personality.
- **PR 3 Settings framework:** FM sub-pages in Sunnylink YAML and the on-device UI, the one-time migration runner, the
  quiet road-edge block (folded, always on), Blendful hidden from the Models page.

## Step 2: feature PRs (each its own PR into `fm-dev-sync`, merged in order)
**M1**
4. FM Custom Torque Settings: Max Torque 385–580 in steps of 15 (description "385 = stock AP"; panda limit follows it;
   off-road only, next drive), Headroom, faster ramp, full cap past 90°, steerLimitTimer 0.8 + near-zero curvature fix
   (EVPilot). Rivian panda safety tests. **Fresh-Opus code review before merge.**
5. Release Guard drop-down (Stock / 75 / 150), v0 and v1 controllers.
6. Feedforward Settings (Left / Right drop-downs) + Low-Speed Correction Strength (40–100 %, 100 = stock); v0 and v1;
   stand down with Lateral Jerk / NNLC.
7. Lane Keeping selector (Off / Lane Centering / Lane Guard / Blendful), Lane Guard, Blendful 0.3 m/s + blue line only.
8. Model Lateral Smoothing (Small / Big drop-downs: Model Default / 0.00 / 0.10, notes on the difference), both modelds.
9. Calmer Crawl-Speed Steering + Calmer Straight Pull-Away (on AP's 1.0 floor).

**M2**
10. Turn Hold (folded behaviors; toggles Path Source, Adds Your Steering, Follows the Model's Exit, Stop Before Turning
    Starts Over), Smooth Turn Start, Turn Assist logging.
11. Turn Lead (Min Speed, Adaptive Look-Ahead, Wait for the Corner + distance, Yields to Your Hands, Tightest Radius).
12. Finish Turn After Signal Cancels (Carry Only at Turn-In folded).
13. Pre/Post Turn Behavior: Queue Guard, Turn Pulse on Pull-Away, Settle After Turn + Early Right Push.

**M3**
14. Accel Boost picker (Gas / Sunny Boost / EVP Set Speed / AP Accel Control / OP Personality from openpilot
    #39430 + #39440) + Boost bar working for all five.
15. Lead: Radar-matched / Camera-only confidence, Ignore Stationary Radar Ghosts, Smooth Lead Handoffs, `fmRadarLead`.
16. Top Readout (Torque / CPU / Lead) + torque log fields.
17. Jetlink extras: zoompilot delta (API 4, v0.9.0 fixes), Jetlink Status log, icon states.
18. hardwared stall diagnostics.

## Checks for every PR
Unit tests ported plus new ones for reshaped settings and migration; synthetic same-input equivalence vs the
`fm-dev-nav-settle` module wherever FM logic is unchanged (differences explained); ruff; Sunnylink recompile + validator;
panda tests where touched. Each PR description lists what couldn't be checked (truck response, on-device CPU, UI look).

## Settings migration (once, on first boot; a flag param marks it done)
Old keys stay declared and untouched, so switching back to `fm-dev-nav-settle` restores his setup. Switch branches from
the device's Software settings (uninstalling wipes settings). Mappings: Max Torque (panda off -> 385, else rounded to the
15 grid; 580 -> 580); Release Guard (off -> Stock, else 75/150); Feedforward (master off -> both Off, else per side);
Low-Speed Correction Strength (off -> 100, else percent); Small Model Smoothing (off -> Model Default, 0 -> 0.00,
0.1 -> 0.10, 0.2 -> 0.10); Big Model Smoothing (off -> Model Default, on -> 0.10); Accel Boost mode 0–3 unchanged;
unchanged-meaning settings keep their keys.

## Install
`pmalekian/fm-dev-sync` (comma's installer takes GitHub owner/branch). Old branches stay as fallbacks.
