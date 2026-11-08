# FM Pilot: handoff for new chats (short)

State as of Nov 8, 2026. This is the current state only. Background findings, past drive analyses and
the ideas backlog are in `claude/FM_PILOT_FINDINGS.md`: read it only when a task needs that history. Features, settings
and credits are in `FM_PILOT.md`; log tools in `tools/fm/README.md`.

## Setup

- **Truck:** 2022 Rivian R1S Launch Edition (Gen 1), comma 4, torque harness.
- **Repo:** `pmalekian/fmpilot`. Active branch **`fm-dev-set`** (v25 model list and tinygrad 92b742f since Nov 8), cut from `fm-dev-torque` `bece23e`
  (Higher Torque + radar settings) and adding the settings reorganization, Blendful and the boost bar; `fm-dev-torque`
  stays at `bece23e` as the fallback. `fm-dev-max` (`d189ccc`) is the everyday branch, installer `pmalekian/fm-dev-max`. Own fork of AdventurePilot `dev` (last synced `980dd5c`, Oct 31).
  **No more AP syncs**; borrow single pieces only if wanted.
- **Code work:** attach the repo (push access), clone the branch. Commit as
  `Frankie Malekian <107580415+pmalekian@users.noreply.github.com>`, messages start "FM Pilot", one change per commit.
  Push with `--no-verify` (LFS hook 403s). Tests run off-device with a shim stubbing `Params` from `params_keys.h` (and
  `msgq` names, for `cereal.messaging`), with `opendbc_repo` on the path; deps: numpy pytest pycapnp zstandard
  setproctitle psutil pyzmq pycryptodome tqdm, plus `tinygrad` (pip) for modeld_v2 tests and `raylib` + `xvfb-run` for
  UI tests. Known off-device failures: `test_latcontrol_torque_ext.py` (NNLC data submodule not cloned).
  After YAML edits recompile `openpilot/sunnypilot/sunnylink/tools/compile_settings_ui.py` (and run
  `validate_settings_ui.py`).
- **Frankie's side:** Windows PowerShell. Comma at `192.168.1.246`, user `comma`. Always `scp -O`. Say which prompt each
  command runs at (`PS C:\...>` or after `ssh comma@192.168.1.246`). He can also download single rlogs from Comma
  Connect on his phone and attach them (rename each to its segment, e.g. `<route>--<n>--rlog.zst`). The PC can be
  linked to the chat; the bridge can read `C:\Users\frank\fm-logs` (the full Nov 3 drive `0000002d`, 64 segments) and
  `C:\fmlogs` once granted (`C:\fmlogs\2026-11-05` folders are empty), but the PC shell VM doesn't start
  (virtualization off), so the chat can't pull from the comma itself.
- **Logs:** one-minute segments, `<route>--<n>/rlog.zst`. Model name only logged in `--0` (tools infer it otherwise).
  `tools/fm` joins segments; tests `cd tools/fm && python -m pytest tests`. In a cloud chat run from `tools/fm` (its
  `fmlog` import puts the repo's `openpilot` package on the path). rlogs include `radarTracks` (raw radar points), so
  radard can be replayed offline. EPAS signals: decode `EPAS_SystemStatus` (0x380, bus 0) with cantools and the DBC.
- **Signs:** curvature and lateral accel positive **right**; angle, hand torque and torque commands (incl.
  `carOutput.actuatorsOutput.torqueOutputCan`, same sign as `actuators.torque`) positive **left**. Hand torque reads
  *against* openpilot from wheel reaction when loose.
- **Real torque sent:** `carOutput.actuatorsOutput.torqueOutputCan` (CAN units, cap 385 / 481). Learned torque values
  are in `lateralTorqueParameters` (not `liveTorqueParameters`).
- **Torque controller:** with `EnforceTorqueControl` on and `TorqueControlTune` 0 (his settings), or Enforce off,
  sunnypilot runs **v0** (`latcontrol_torque_v0.py`), not v1. `torqueState.version` in the log says which.
  `torqueState.saturated` is always false below 10 m/s.

## What he drives now

- **TSFDOM**, Experimental Mode, lane centering on (One-Line on; on `fm-dev-set` this is `FM_LaneKeepMode` = Lane
  Centering), Lane Guard off, off-tracking off, Camera Offset 0.05 m
  (since Nov 4, not yet analyzed). Aggressive personality, following time offset -20% (Nov 3 backup).
- Road Edge Turn Desire on (max 35 mph in the Nov 5 logs), quiet road-edge block on.
- Accel Boost: **Predictive** (mode 2) on the Nov 5 drives (up to +0.4 m/s² on aggressive).
- Turn Assist on Nov 5: Turn Hold + Hold Path Source + Hold Adds Your Steering on; Lead and Boost off; radius 8 m.
- Nov 5 drives were on `fm-dev-torque` `03b6805` (before the fix), panda setting on: `00000038` Headroom, `00000039`
  (morning commute) Full. **Both actually ran at 385** (see below).
- Custom splice model: **removed** Nov 8 (with the on-device compile fix and model check tools), for the v25 move.

## Active work

### 00. v25 model list + tinygrad 92b742f on `fm-dev-set` (Nov 8): built, not yet on the device

- Why: new sunnypilot models (e.g. **Cheddar**, openpilot PR #39255) are only in sunnypilot's model list v25+, which
  needs selector version 21, whole-file downloads and the plain GPU memory model format on tinygrad 92b742f.
- Ported from sunnypilot master-dev, one commit each: #2050, #2053, #2062, #2051, #2071, #2052, #2072, #2093.
  `selfdrive/modeld` (SConscript, helpers, stock modeld, dmonitoringmodeld loading) moved to sunnypilot's, because the
  new tinygrad dropped `--device-input` and changed Buffer/UOp APIs. **DM model file unchanged** (FM's Aug model; sunnypilot
  has openpilot's newer Sep "Super Leicht", not taken). FM's own: Blendful hook kept in modeld_v2; load_oob rejoins
  chunked files (FM prebuilds still chunk >95 MB). Not taken: #2092 (CI), .github changes.
- Also new from upstream: modeld_v2 and stock modeld seed the previous curvature from `controlsState` while lateral is
  off (openpilot #39091), chestnut retry; `modeld_warmup` runs offroad. Because of #39091, `tools/fm` now reads the raw
  `modelDataV2SP.modelCurvature` as "model asked" while lateral is off (`fmlog.model_requests`).
- **Calmer Crawl-Speed Steering** (`FM_LowSpeedCurvatureFloor`, Steering > FM Pilot: Turn Tweaks, **off**): sunnypilot
  #2091 as a setting. Speed floor 2.5 m/s instead of 1.0 when turning the action into curvature: below 5.6 mph the
  request drops (64% at 4.5 mph, 16% at 2.2 mph). Calms crawl twitch; may soften turns from a stop. Try as its own drive
  on the same corners, with and without.
- Off-device tests: modeld_v2 121 pass, models 28 pass (onnx hash test fails only because LFS isn't pulled); no new
  failures vs `60ddd2a` across sunnypilot/modeld/ui/controls tests.
- **First boot:** build takes longer (DM and stock model recompile). The saved v23 selection (selector 20) no longer
  parses, so modeld most likely runs the default model until TSFDOM (same ref) is selected again and re-downloads
  (from the code, not yet seen on the device); leave it offroad for the warmup. **Check before driving:** DM works (face seen, no DM faults), model label/picker
  shows the selected model, no modeld errors. Then try Cheddar as its own drive.

### 0b. Jetlink on `fm-dev-jet` (Nov 8): built, off, not yet on the device

- Branch `fm-dev-jet` = `fm-dev-set` + Jetlink (zoompilot/jetlink v0.9.0, API 4). Sources: AP PR #12 (params, events,
  selfdrived, hardwared, process, UI) and zoompilot develop (adapter, modeld hooks for the v25 modeld_v2). Setting
  Models > Jetlink (Off / USB / iOS, offroad), `JetlinkLink` 0 by default. Events `jetlinkReady/LinkLost/Chime` @28-30.
- Big model = comma's large models from Jetlink's index, picked in the Jetlink app (FM doesn't use the model-list hooks).
  It runs stock modeld's action path + Blendful (`fm_jetlink_action.py`); Calmer Crawl-Speed Steering doesn't apply to it.
- Frankie's host: MacBook M1 Pro (tested platform; ~20 s prepare, up to 10 s load). Needs a USB 3 data cable, the comma
  powered from the harness (Jetlink cuts USB-C draw at boot when on), the Mac on its own power and awake. S25 Ultra
  Android app is experimental.
- Off-device: adapter conformance, both modeld seams, joining-model contract, fallback: pass. Not run on a device.
- Order: drive v25 on `fm-dev-set` first; then `fm-dev-jet` with Jetlink off (same as fm-dev-set); then on, parked first.

### 0a. ChatGPT commits Nov 6 (not yet driven or analyzed)

- `b925e3a` Turn Hold fades over 0.5 s when the blinker is cancelled mid-hold; `398cd6b` **Turn Nudge**
  (`FM_TurnNudgeEnabled`, off, needs Turn Hold): 2–8 mph, 0.8 Nm into the turn for 0.15 s adds up to 0.01 1/m for 0.6 s
  if the path bends that way; `dd313ff` torque readout stays visible (moved down) during the Turning Left/Right notice.
  Reviewed Nov 8: tests pass (78), Sunnylink JSON matches its YAML; docs added to FM_PILOT.md; garbled docstring fixed.

### 0. `fm-dev-set`: built, not yet driven (Nov 5 evening)

- Frankie plans to drive it with Lane Centering selected, testing Higher Torque at the same time. If anything feels
  off, he goes back to `fm-dev-torque` `bece23e` alone.
- **Lane keeping mode** (`FM_LaneKeepMode`: 0 Off / 1 Lane Centering / 2 Lane Guard / 3 Blendful) replaces the
  `LaneCenteringEnabled` / `LaneGuardEnabled` toggles as the switch. No default: `params_migration` seeds it once at boot
  from the old toggles (centering wins). Everything reads it through `fm_lane_keep_mode.get_lane_keep_mode`.
- **Blendful** (sunnypilot #2073, unchanged math) in modeld_v2, picked by the mode; returns exactly 0.0 when not
  selected. Logged in `modelDataV2SP` (`blendfulCurvature`, `blendfulTrust`, `modelCurvature`, `blendfulGoingAround`,
  `blendfulLeftProb/RightProb`, `blendfulLaneWidth`). Plan before he drives it: replay on Nov 3 pull-right and curvy
  segments and show the nudge it would add.
- **Boost:** `longitudinalPlanSP.fmAccelBoost` (what the boost added) and `fmAccelBoostMode` are now logged. Side-panel
  bar from sunnypilot #2090, `FM_AccelBoostIndicator`, off. Quiet road-edge block is always on.
- **Sunnylink:** Steering > Lane Centering dropdown (+ off-tracking at the bottom); Lane Change holds the road-edge
  block; "FM Pilot: Turn Tweaks" with Turn Boost / Hold / Lead settings pages; Road Edge Turn Desire moved to Models >
  Model Behavior (that page is `remote_configurable: false`; check he can change it from Sunnylink); Cruise > Driving
  Personality Settings and Smart Cruise > Curve Speed Settings pages. Sub-page links may render after a section's items.
- **First log check:** Sunnylink shows Lane Centering before driving; `modelDataV2SP.blendfulCurvature` 0 all drive;
  lane centering active as before; plus the Higher Torque check below.
- Also discussed, parked: sunnypilot #2083 (Set Speed Assist, a candidate 4th Accel Boost mode), #2034 (smoother final stop), openpilot #39059 (torqued long-accel filter), #39225 (e2e
  curve speed). Predictive Boost math in sunnypilot master (#2089) matches ours.

### 1. Higher Torque: working; Max Torque slider, faster ramp and full cap past 90° built, not yet driven

- **Nov 5 evening drive `0000003a` (`0941590e`, Headroom, panda on) confirms it works:** `torqueOutputCan` reached 457
  (`--81`–`--83`), no faults. Only short stretches above 385 (max 1.8 s, `--82` +16 s right turn from a stop).
- **What still limits slow turns:** (1) the ramp: 3 per frame, ~1.5 s to full, while the request sat at 1.25;
  (2) AP's 95% cap past 90° (`HIGH_ANGLE_CAP_FRAC`), hence 457 not 481. The 90° TOI blip workaround is not the cause
  (torque came straight back after the one blip seen; wheel stalled ~160–196° while 340–457 was being sent).
- **Built Nov 5 night (all off by default, off-road only, next drive, all need Panda Allows Higher Torque):**
  - `c514b3e` **Max Torque** (`FM_MaxTorque`, 405–580 step 25, default 480): level sent to the panda in safety param
    bits 2–5 (n → 380 + 25 n); the panda enforces exactly it. No level = 481. Temporary: once a number is settled,
    back to fixed standard / high modes.
  - `143ece8` **Faster Torque Ramp** (`FM_TorqueFastRamp`): car code 5/frame below 25 mph; panda allows 5 below 12.5 m/s
    when safety param bit 6 is set. 250 ms limit (125) unchanged.
  - `3d7f292` **Full Torque Past 90°** (`FM_TorqueFullCapHighAngle`): car code only.
- **Torque readout** (`b514a01`, always on): top-center `sent / limit now / Max Torque ✓` on the comma 4. In logs:
  `carStateSP.fmEpasToiActive`, `fmSteerCap`, `fmMaxTorque` next to `torqueOutputCan`. Not yet seen on the device.
- **Test plan:** 480 + ramp + full cap, Turn Lead off, normal Loop A turns up to 25 mph. If the wheel still stalls at
  the cap, go off-road, step Max Torque 505 → 530 → 555 → 580, one at a time. Before driving each new step: parked
  check, wheel toward lock, lateral on. Send rlogs per step.
- **Turn Lead** cut the curb on `0000003a--83` +4 s and +22 s rights: it asked ~12 m about 1–2 s before the model got
  there (look-ahead 1.3 s plus slow ramp). Keep it off; possible fix: shorter look-ahead (~0.7–0.8 s).
- **Turn Hold** likely released in `--82` +18 s on +1.5 Nm wheel reaction (guess; hold state isn't logged). Blinker
  self-cancel mid-turn (`--83` +46 s) ended the turn desire and openpilot pushed out of the turn for ~3 s.
- Don't reset the torque learner unless he settles on Full.

### 2. Radar lead settings: built, both off, not yet driven (`50d1997`, `613c760`)

- **Ignore Stationary Radar Ghosts** (`FM_RadarStationaryGuard`, Cruise, off): radard won't pair a radar point under
  1 m/s with a camera lead over 5 m/s (11 mph) unless the camera sees it braking harder than -1.5 m/s²; that car is
  followed from the camera instead. Fixes guardrail/barrier phantom brakes on ramps and curves (`00000038--19` on-ramp
  +16.8 s: -1.8 to -2.5 m/s², 19 → 13 mph; +32.6 s; `--13`; `00000039--47` +56.6 s, where his gas press was against a
  -3.5 phantom). Low-speed radar path (under 4 m/s) untouched.
- **Lead Detection Sensitivity** (`FM_LeadDetectionThreshold`, Cruise, 25–50%, default 50 = openpilot): camera
  confidence needed to count a lead and pair it with radar. From FrogPilot (MIT). Use with the guard.
- **Replay** (real radard on the 9 Nov 5 segments; stock replay matches logged leads 96–100%): phantom frames 148 → 5
  with the guard (the 5 are camera leads at 9–12 mph, by design; one is the `--48` red-light car at 107 m, correctly
  kept); nothing else changed. 40%/35%: leads 0.2–0.9 s earlier, 1–2 short extra camera leads in `--1`; at 35% without
  the guard phantoms rose to 163. No fast approach to stopped traffic in these logs.
- **Test plan:** after the Higher Torque drive. Guard on, same on-ramp. Then threshold 40%, then 35%, in normal driving
  with slowdowns for traffic at 35–45 mph; watch for braking with no car ahead. Untested: a real hard stop behind a car.
- Checked and not taken: FrogPilot's radar-only "far lead" (moving targets only, no help for stopped traffic) and
  "Human-Like Following" (brakes later and closer; model-gated). StarPilot Dom has none of these for non-Honda cars.

### 3. Garage spiral (`00000039--61` to `--63`): mostly explained

- TSFDOM asked nearly straight at lock (median R 77–121 m vs 5.1 m driven), so openpilot sent max torque **left, out of
  the turn**, against his right steering. Blinker off in `--63`; turn desire only in `--62` 0–6.4 s.
- Lockouts are FM/AP's own **TOI-dropout watchdog**: past ~450° the EPAS mostly stopped taking torque (`ToiActive` 9–33%,
  no ToiFlt/derate/hands-override); after 1.05 s the watchdog set `steerTempUnavailableSilent` and lateral was off
  exactly 3.0 s (7 of 8; the first was him overpowering at -6 Nm).
- Turn Hold released for good at `--61` +57.5 s (pressed torque +1–1.8 Nm against the right turn, wheel moved 12° left),
  so Hold Adds Your Steering never acted. The "7–14 m" was `turns.py`'s brief tightest-asked peak, not a hold.
- **Still open:** what held the wheel at lock with `ToiActive` off and hands light (he says it doesn't stay there by
  itself). Unknown whether the EPAS refuses all overlay near the end stop or only away-from-lock (every near-lock request
  was out of lock).
- Options, not built: don't count `ToiActive` off near lock (> ~480°) toward the watchdog (discuss first); require
  against-turn torque to last ~0.3 s before releasing Turn Hold.

### 4. Wide right curve (`00000039--47` +45 s to `--48` +7 s)

- He wants more steering, **not less speed** (curve speed control rejected).
- 12 mph: hands off, not at cap, drove 90% of the request; v0's integrator held a stale -0.03 to -0.13 m/s² against the
  turn (frozen below 5 m/s). 20–21 mph: ~97%. "Model let go" at `--48` +7–8 s (0.023 → 0.014 1/m) coincided with his gas
  press and right steering.
- Options, not built: let v0's integrator decay at low speed instead of freezing; a curve hold without a blinker;
  Predictive Boost curve fade lower (now 1.5 → 2.5 m/s²; this curve was ~0.8 at 12 mph, full boost).

### 5. Smoother stops / regen: analyzed, nothing built (details in findings)

- openpilot sends **one** accel number (0x160); the VDM alone picks regen vs friction. No CAN control of the split, and
  no regen-level signal. Motor torque and brake pressure aren't on the comma's buses; friction is `ESP_BrkTrq` (0x208).
- Nov 3 drive: openpilot braking is all regen to about -2 m/s²; friction comes in only past that, and in the last
  ~0.5 s of a stop (as `stopping` starts), then holds ~-700 Nm.
- openpilot stops end harder than his one-pedal stops: -0.44..-0.55 vs -0.16..-0.40 m/s² 0.3 s before standstill,
  release jerk 4.4–5.5 vs 2.0–3.7 m/s³. Cause: the plan still asks -0.3..-0.45 in the last second, and below ~3 mph the
  truck over-brakes the plan by ~0.1 (the drag FF is ramped out there).
- Waiting on Frankie: does the end of openpilot stops feel abrupt to him; his regen setting (Standard/Low); 3–5 stops
  from 30–45 mph on current build. Candidate (discuss first): Rivian final-approach taper below ~3 mph, plus carrying
  part of the drag FF down to standstill.

## Next steps

0. Update the comma to `fm-dev-set` (v25), do the first-boot checks in item 00, re-select TSFDOM, then try Cheddar.
1. Torque test day on `fm-dev-set` (item 1 test plan). Then decide the Max Torque number and drop the slider.
2. Radar settings test (above).
3. Smoother stops: his answers to item 5, then decide whether to build the taper (off by default).
4. Older: analyze Nov 4 routes `0000002e`, `0000002f`, `00000030`, `00000033`, `00000034`, `00000035` (Turn Assist with
   fixes) and the Camera Offset pull test (`C:\fmlogs\2026-11-04`). Turn Boost / Hold speed limit to 20 → 25 mph.
5. Later: nav port (`claude/FM_PILOT_NAV_PLAN.md`).

## Key facts in one line each (details in claude/FM_PILOT_FINDINGS.md)

- At Loop A speeds the torque cap, not the model's request, is what fails.
- His settings run sunnypilot's v0 torque controller; any controller hook must go in both v0 and v1.
- Two small models don't fit the comma 4's 50 ms step; dual mode removed.
- Models: sunnypilot list v25 (selector 21), tinygrad 92b742f; selfdrive/modeld follows sunnypilot master-dev.
- The VDM decides regen vs friction; openpilot only sends a target accel (truck delivers ~0.23 m/s² more decel than sent).
- Lane Centering, Lane Guard and Blendful are one setting, `FM_LaneKeepMode`; read it via `get_lane_keep_mode`.
- Never loosen driver monitoring.
- For other assistants (ChatGPT, Gemini), paste `claude/FM_PILOT_BRIEF_FOR_OTHER_AIS.md`. Check their claims before use.
