# FM Pilot: handoff for new chats (short)

State as of Nov 5, 2026 (late evening). This is the current state only. Background findings, past drive analyses and
the ideas backlog are in `claude/FM_PILOT_FINDINGS.md`: read it only when a task needs that history. Features, settings
and credits are in `FM_PILOT.md`; log tools in `tools/fm/README.md`.

## Setup

- **Truck:** 2022 Rivian R1S Launch Edition (Gen 1), comma 4, torque harness.
- **Repo:** `pmalekian/fmpilot`. Active branch **`fm-dev-set`** (head of the branch; code as of `5ead1c3`), cut from `fm-dev-torque` `bece23e`
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
- Custom splice model (OPM16D + Rebel Legion): built, switched off.

## Active work

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
- Also discussed, parked: sunnypilot #2083 (Set Speed Assist, a candidate 4th Accel Boost mode), #2091 (low-speed
  action curvature floor), #2034 (smoother final stop), openpilot #39059 (torqued long-accel filter), #39225 (e2e
  curve speed). Predictive Boost math in sunnypilot master (#2089) matches ours.

### 1. Higher Torque: fixed, not yet driven working (`14cac9b`)

- Cause: the hook was only in sunnypilot's v1 torque controller; his settings run **v0**, so `actuators.torque`
  stopped at 1.00 and `torqueOutputCan` at 385. v0 now has the same hook; controller tests cover v0 and v1 and every
  controller `initialize_lateral_control` can pick. All Nov 5 "Headroom/Full" drives = Off for torque.
- Settings: **Panda Allows Higher Torque** (`FM_PandaHighTorque`, off-road only) and **Higher Torque**
  (`FM_TorqueMode`: 0 Off / 1 Headroom / 2 Full). Panda envelope `[9,25,27] -> [481,295,275]`; car cap
  `[9,17,25,27] -> [481,332,295,275]`.
- **First log check:** `actuators.torque` passes 1.00 and `torqueOutputCan` passes 385 in a tight turn.
- **Evidence it will help (Nov 5, at 385):** in normal turns the EPAS took 385 97–100% of the time, no faults. Cleanest:
  `00000038--19` +16.8–18.9 s, 17 mph right, 2.1 s at cap with ~0.5 Nm hands, wheel creeping ~16°/s, 88% of the asked
  lat accel. `--1` +11–12 s, 12 mph left: at the cap the wheel unwound 244° → 185° with hands 0.36 Nm.
- **Next:** a Headroom drive with slow no-blinker curves (Loop A, `--19` ramp); then decide Headroom vs Full. Don't
  reset the torque learner unless he settles on Full.

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

1. Update the comma to `fm-dev-set` between tests (Lane Centering selected). Drive Higher Torque (Headroom), check the
   first log (item 0 and item 1 checks).
2. Radar settings test (above).
3. Smoother stops: his answers to item 5, then decide whether to build the taper (off by default).
4. Older: analyze Nov 4 routes `0000002e`, `0000002f`, `00000030`, `00000033`, `00000034`, `00000035` (Turn Assist with
   fixes) and the Camera Offset pull test (`C:\fmlogs\2026-11-04`). Turn Boost / Hold speed limit to 20 → 25 mph.
5. Later: nav port (`claude/FM_PILOT_NAV_PLAN.md`).

## Key facts in one line each (details in claude/FM_PILOT_FINDINGS.md)

- At Loop A speeds the torque cap, not the model's request, is what fails.
- His settings run sunnypilot's v0 torque controller; any controller hook must go in both v0 and v1.
- Two small models don't fit the comma 4's 50 ms step; dual mode removed.
- On-device compile (QCOM, IMAGE=1) needs `FM_CONTIGUOUS_OUTPUTS=linear_2` or outputs go NaN.
- The VDM decides regen vs friction; openpilot only sends a target accel (truck delivers ~0.23 m/s² more decel than sent).
- Lane Centering, Lane Guard and Blendful are one setting, `FM_LaneKeepMode`; read it via `get_lane_keep_mode`.
- Never loosen driver monitoring.
- For other assistants (ChatGPT, Gemini), paste `claude/FM_PILOT_BRIEF_FOR_OTHER_AIS.md`. Check their claims before use.
