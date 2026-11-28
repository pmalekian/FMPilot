# FM Pilot: handoff for new chats (short)

State as of Nov 25, 2026: **`fm-dev-sync` plan PRs 1-18 are all merged and the M3 full suites passed** (head `023ff40f`; only the known base and sandbox-only failures; PR 10-13 equivalence scripts show only the documented PR 13 differences). Built PR by PR from the agreed plan (project docs
`claude/FM_DEV_SYNC_BUILD_PLAN.md`, `claude/FM_DEV_SYNC_INVENTORY.md`, `claude/FM_FOLDED_DEFAULTS.md`; repo copy
`FM_FOLDED_DEFAULTS.md`). Planning and building are finished: don't re-plan. **Next: Frankie drives `fm-dev-sync` (now with the Nov 27 upstream PRs #29-#33) and shares rlogs.** History before `fm-dev-sync` (every earlier branch, build
and drive analysis, the old handoff word for word) is in `FM_PILOT_FINDINGS.md`: read it only when a task needs it.
Features, settings and credits: `FM_PILOT.md`. Log tools: `tools/fm/README.md`.

**Nov 27, 2026 upstream check** (openpilot master `05c5c54f` Nov 24 -> `e7d349f8`; nightly = v0.11.3 `0ee1c770`, master as
of Nov 26 `c65119ce`; sunnypilot master only bumped opendbc since `c410198c`). Taken as PRs #29-#32 (below), merged Nov 27
with Frankie's explicit go-ahead. Frankie chose (Nov 27) to match openpilot's personality retune fully, incl. Relaxed 1.45 s, with the curve gate as a setting that defaults on. Not taken: tinygrad bump #39484 (new modeld/tinygrad API and
re-exported models: wait for sunnypilot), device-permission removal #39485 and the rest of #39481 (need AGNOS newer
than 19.9), opendbc #4102 heartbeat reset on every engagement (panda safety: Frankie's call), opendbc #4045 `ToiFlt`
(FM has its own debounced version), comma four UI and test changes. Crumbly Cheddar became openpilot's default model
(#39451). Next check starts from openpilot `e7d349f8` and commaai/opendbc `66b40028`.

## Setup

- **Truck:** 2022 Rivian R1S Launch Edition (Gen 1), comma 4, torque harness.
- **Repo:** `pmalekian/FMPilot`. **Active branch `fm-dev-sync`**, install string `pmalekian/fm-dev-sync`. Base: AP
  `dev-next-src` `cf3ad0ec` + sunnypilot master `c410198c` (merge commit `4efb05a`). Code is ported from
  `fm-dev-nav-settle` `ca08aac9` (his last everyday build line); that branch and the older ones stay as fallbacks.
  Switch from the device's Software settings (uninstalling wipes settings).
- **How PRs land:** each PR is its own branch `fm-sync/prNN-<name>` and GitHub PR into `fm-dev-sync`, merged once its
  checks pass, in the plan's order (Step 0, PRs 1-3 groundwork, M1 = 4-9, M2 = 10-13, M3 = 14-18). No drive-log replays:
  builds, unit tests, ruff, Sunnylink compile + validator, Rivian panda safety tests where touched, and synthetic
  same-input checks against the `fm-dev-nav-settle` modules. Each PR description lists what couldn't be checked.
  **PR 4 (torque) gets an independent fresh-Opus code review before merge.**
- **Opening and merging PRs:** the GitHub connector's `create_pull_request` returns 403 for this repo; use the shell:
  `gh api repos/pmalekian/FMPilot/pulls -f title=... -f head=<branch> -f base=fm-dev-sync -F body=@<file>`, merge with
  `gh api -X PUT repos/pmalekian/FMPilot/pulls/<n>/merge -f merge_method=merge -f sha=<head sha>`. The session's permission
  check may refuse a merge without Frankie's explicit permission in chat; ask him (Nov 27 he gave it for #29-#33).
- **Commits:** author `Frankie Malekian <107580415+pmalekian@users.noreply.github.com>`, message starts "FM Pilot", one
  logical change each, borrowed code credited in the message, the source file and `FM_PILOT.md`. Push with
  `--no-verify` and `-c push.negotiate=true`.
- **Migration:** a feature PR that reshapes a setting declares the old keys it reads (never written) and its new
  keys (stock/off defaults) in `params_keys.h`, adds an `@migration("name")` function to `fm_migration.py` using
  `put_if_unset`, its mapping row in `FM_PILOT.md`, and tests on real `Params` (old values -> new; fresh install ->
  nothing written). Unchanged-meaning settings keep their keys and get stock/off defaults (his stored values stay:
  manager only writes defaults for unset keys).
- **opendbc:** copied in-repo as AP does (`opendbc_repo/`, a normal folder, no submodule). On a sunnypilot sync, re-sync
  the copy to sunnypilot's new opendbc pin and re-apply AP's and FM's changes: diff AP's copy against the pin it was
  vendored from, cherry-pick that onto the new pin (done for Step 0: `48611ae9` -> `3db0b06`).
- **Keeping this file current:** whenever this handoff changes, also write it into the Project as
  `FM_PILOT_HANDOFF (3).md` (replacing the old one) in the same turn. Keep `FM_FOLDED_DEFAULTS.md` (repo) and
  `claude/FM_FOLDED_DEFAULTS.md` (project) identical.

## Checks off-device (cloud chat, Nov 25 on the base)

```
git submodule update --init --depth 1   # then fetch each submodule's pinned commit if it checked out the default branch
uv sync --frozen --all-extras && source .venv/bin/activate && uv pip install pytest pytest-timeout
scons -j2 -k        # ~5 min on 2 cores; only the two LFS model compiles fail (onnx files are LFS pointers)
export PYTHONPATH=$PWD:$PWD/opendbc_repo:$PWD/msgq_repo:$PWD/rednose_repo:$PWD/tinygrad_repo:$PWD/teleoprtc_repo:$PWD/panda
python -m pytest -q --timeout=120 --continue-on-collection-errors openpilot/sunnypilot openpilot/selfdrive/controls \
  openpilot/selfdrive/ui/sunnypilot openpilot/system/hardware/tests openpilot/system/manager \
  opendbc_repo/opendbc/car/rivian opendbc_repo/opendbc/sunnypilot/car/rivian opendbc_repo/opendbc/safety/tests/test_rivian.py
(cd tools/fm && python -m pytest tests)
python openpilot/sunnypilot/sunnylink/tools/compile_settings_ui.py && python openpilot/sunnypilot/sunnylink/tools/validate_settings_ui.py
ruff check .
```

- **Base results (Step 0, `4efb05a`):** 1162 passed, 167 skipped, 2 failed, both pre-existing and not FM's:
  `test_default_model::test_compare_onnx_hashes` (LFS not pulled) and `test_manager::test_set_params_with_default_value`
  (AP's `ScreenSaverCustomText`). sunnypilot's new GPS fuser tests pass. Sunnylink compile unchanged, validator 10/10.
  ruff: 10 findings, all in AP files unchanged by the merge (Rivian `carcontroller.py`, `test_route_replay.py`,
  `layouts/settings/cruise.py`, `jetlink_adapter`, `system/manager/process.py`). Running every opendbc safety test
  (all cars) takes well over an hour on 2 cores; `test_rivian.py` covers the Rivian and MADS changes.
- **Not checkable here:** truck response, on-device CPU, UI look, the model compiles (LFS).

## Progress on `fm-dev-sync`

| Step | State |
|---|---|
| Step 0 base | done, `4efb05a`, pushed |
| PR 1 docs and tools | merged, PR #5 (`fba53fc`) |
| PR 2 log schema | merged, PR #6 (`f887f49`): FM's own messages `fmControls` @145, `fmModel` @144, `fmPlan` @143, `fmCarState` @142, `fmRadarLead` @141, `fmJetlinkStatus` @140 (free slots from the top: sunnypilot master took slot 10 for `gpsLocationSP`), event `fmLaneChangeRoadEdgeQuiet` @38; `tools/fm` picks the old or new schema per file |
| PR 3 settings framework | merged, PR #7 (`aa95387`): quiet road-edge block (folded), Blendful hidden from the Models page, FM sections on the Sunnylink Steering page, one-time migration runner (`sunnypilot/system/fm_migration.py`, flag `FM_SyncMigration`) |
| PR 4 torque | merged, PR #8 (`44d8a07`) after the fresh-Opus review (findings and fixes in the PR's comment: Headroom kept Lateral Jerk / NNLC's PID limits, reads vEgoRaw, tighter safety tests, override force documented): Max Torque `FM_CustomMaxTorque` 385-580 step 15 (panda follows it; above 385 faster ramp, full cap past 90°, Headroom), steerLimitTimer 0.8, off-tracking near-zero fix; migration `max_torque` |
| PR 5 Release Guard | merged, PR #9 (`1653723`): `FM_ReleaseGuardLimit` Stock / 75 / 150, v0 + v1, migration `release_guard`; controlsd publishes `fmControls` |
| PR 6 feedforward + correction strength | merged, PR #10 (`60c057b`): `FM_LowSpeedFFLeft/Right` (Off default), `FM_LowSpeedCorrectionStrength` (100 default), v0 + v1, migrations `low_speed_feedforward`, `low_speed_correction` |
| PR 7 Lane Keeping | merged, PR #11 (`99a2b44`): `FM_LaneKeepMode` Off / Lane Centering / Lane Guard / Blendful (Off default), Lane Guard (gerrylum rx-wb `17ea338`) via `fm_lateral.py`, Blendful at 0.3 m/s with the blue line only; migration `lane_keep_mode` |
| PR 8 Model Lateral Smoothing | merged, PR #12 (`31be347ac`): Small and Big Model Default / 0.00 s / 0.10 s choices, both modelds; migration `model_lateral_smoothing` |
| PR 9 Calmer Crawl-Speed Steering and Calmer Straight Pull-Away | merged, PR #13 (`ce45f40de`): two Off-by-default toggles for the action-head curvature floor in both modelds and Jetlink. Existing settings retain their keys. |
| PR 10 Turn Hold, Smooth Turn Start, Turn Assist logging | merged, PR #16 (`19574d7`): `fm_turn_assist.py` via `fm_lateral.py` (controlsd passes its SubMaster with `modelDataV2SP` and its curvature), logged in `fmControls.turnAssist`; six Turn Hold options folded, old keys declared, not read; keys unchanged, no migration. Turn Path Tightest Radius (`FM_TurnPathMinRadius`) is read but has no Sunnylink entry until PR 11's Turn Lead Settings. Turn Lead and the carry plug into `update()` where its comments say |
| PR 11 Turn Lead | merged, PR #17 (`b1b1800`): Turn Lead in `fm_turn_assist.py` before Turn Hold, Turn Lead Settings sub-page (Min Speed, Adaptive Look-Ahead, Wait for the Corner + Start Distance, Yields to Your Hands, Turn Path Tightest Radius); keys unchanged, no migration; Adaptive Look-Ahead and Wait for the Corner now default off (on in `ca08aac9`), stored values carry; `lead`/`leadYield` logged; Smooth Turn Start works with Turn Lead alone |
| PR 12 Finish Turn After Signal Cancels | merged, PR #18 (`48c343f`): the carry in `fm_turn_assist.py` supplies the turn's sign once the desire ends (`source` = carry), Turn Lead and Turn Hold follow it; toggle on Steering > Turns, enabled by Turn Hold or Turn Lead; Carry Only at Turn-In folded (`FM_TurnCarryTurnInOnly` declared, not read); key unchanged, no migration; now defaults off (on in `ca08aac9`), a stored value carries |
| PR 13 Pre/Post Turn Behavior | merged, PR #19 (`d3406ff`): Queue Guard (`fm_queue_guard.py` in `desire_helper.py`, both modelds, `fmModel.queueGuard`; Turn Assist reads the hold so it never starts a carry), Turn Pulse on Pull-Away (`fm_turn_pulse.py`, the model's desire `DH.model_desire` in both modelds), Settle After Turn + Early Right Push (`modeld_v2/fm_turn_settle.py` in the Blendful hook, `fmModel.turnSettle*`; a Finish Turn carry, read from `fmControls`, counts as the turn still on, so the window opens when the carry ends). New sub-page Steering > Turns > Pre/Post Turn Behavior; keys unchanged, all off (as on `ca08aac9`), no migration. PR 17 restores the Blendful hook for the Jetlink large model |
| PR 14 Accel Boost picker + Boost bar | merged, PR #20 (`e9ba455`): `FM_AccelBoostMode` 0 Off / 1 Gas / 2 Sunny Boost / 3 EVP Set Speed / 4 AP Accel Control / 5 OP Personality (Off default; same key, 0-3 carry) in `fm_accel_boost_mode.py`, one boost at a time; `AccelBoostSP` and `AccelerationControl` hidden and not read (LongitudinalPlannerSP no longer swaps boosts); AP Accel Control runs without Gas Override Boost on top (the base stacked them); `longitudinalPlan.accelBoost` = what the mode added (Gas: scaled, was upstream's unscaled total), `fmPlan.accelBoostMode` published; Boost bar `FM_AccelBoostIndicator` for all five; migration `accel_boost_mode` (stored mode kept; unset + `AccelBoostSP` on alone -> 2) |
| PR 15 Lead | merged, PR #21 (`998d062`): Ignore Stationary Radar Ghosts, Radar-matched / Camera-only Lead Confidence (25-50 %, 50 = stock), Smooth Lead Handoffs in radard (`fm_radar.py`, `fm_lead_handoff.py`), Cruise > Smart Cruise Control (Sunnylink); keys unchanged, defaults off / 50 / 50 / off (as on `ca08aac9`), no migration; old `FM_LeadDetectionThreshold` not read. radard publishes `fmRadarLead` with each radarState (for PR 16's Lead readout and `tools/fm/lead.py`, which reads both layouts and replays on either). Lead Shadow Log and Radar Badge dropped. All defaults: radarState byte-identical to the base on constructed drives |
| PR 16 Top Readout | merged, PR #24 (`97121bc`): Sunnylink `FM_TopReadout` selects Off, Torque, CPU (Avg), CPU (Busiest) or Lead on comma 4. Fresh installs default to Off and stored settings carry forward. Card publishes `fmCarState` and the Lead mode reads `fmRadarLead` |
| PR 17 Jetlink extras | merged, PR #22 (`8e737bb`): zoompilot API 4 / v0.9.0 adapter delta on AP JetPilot, Blendful and Settle After Turn restored on the Jetlink big model, `fmJetlinkStatus` published onroad with Jetlink on, comma 4 model icon states. Existing settings carry forward |
| PR 18 hardwared stall diagnostics | merged, PR #23 (`3674674`): `hardwared_slow_loop` reports untimed work, thermal reads and watchdog data. Keeps sunnypilot #2109's params-lock fix |
| Audit fixes | merged, PR #25 (`4324841`): H-1 (`UIStateSP` Accel Boost defaults), M-1 (Headroom torque-unit PID limits with Lateral Jerk / NNLC), L-1 (`max_torque_grid` non-finite fallback to 385), L-2 (`FmTorqueMode.update` returns 1.0 on non-finite `v_ego`), L-3 (grey out `FM_ReleaseGuardLimit` with Lateral Jerk / NNLC), L-4 (`fmRadarLead`/`fmJetlinkStatus` in `fmlog.NEW_FM_MSGS`), L-5 (`FM_LaneKeepMode` titled "Lane Keeping", `BUILD_PLAN.md` feedforward note), L-6 (`OpenpilotPrefix` isolation in controller tests). No change to driving behavior |
| Turn exit fixes | merged, PR #26 (`2aa8757`): Hold Adds Your Steering stops latching new driver input at 45° turned (pre-45° additions stay frozen in peak until normal exit) and driver pressure into the turn no longer blocks model-exit fade or release past 45°; Turn Hold ends its floor with a 0.5 s fade (and does not restart for the turn) once `fmModel` shows Settle After Turn / Early Right Push active for the current turn (audit M-3) |
| Lane keeping waits for the turn carry | merged, PR #27 (`8eb0613`): while Finish Turn After Signal Cancels carries a turn (`fmControls.turnAssist.source == carry`), Blendful (small and Jetlink big models, `modeld_v2/modeld.py`), AP Lane Centering (`controlsd.py`) and Lane Guard (`fm_lateral.py`) add no steering and resume normally as after a blinker turn once the carry ends; Settle After Turn carry handling unchanged (audit M-2) |
| Turn exit and carry follow-ups | merged, PR #28 (`a438c61`): 45° crossing stays remembered for the rest of a turn (`fm_turn_assist.py`, M-4); leftover Blendful / AP Lane Centering / Lane Guard correction fades to zero over ~0.2 s on carry start and stays zero for the rest of the carry while keeping Lane Guard's 1 s cooldown armed (M-2); Blendful waits for a fresh post-clear `fmControls` status and resumes on the first fresh not-carrying status (`BlendfulCarryGate`, M-3); disabling the last Turn Assist feature clears published carry/turn state (M-3); `FmLateral` shares one settings snapshot between carry prediction and Turn Assist (L-1); Sunnylink 45° Turn Hold descriptions updated + recompiled (L-2); Headroom override limit test polls after headroom settles (L-3). NNLC M-1 skipped by choice |
| Remove Gas Accel Boost | merged, PR #29 (`fa404c9`): mode 1 and `selfdrive/controls/lib/accel_boost.py` (Gas Override Boost) removed; numbers kept (0, 2-5), a stored 1 reads and shows as Off; comma 4 button `FmAccelBoostPicker` stores mode numbers |
| Driving Personality Retune from openpilot | merged, PR #30 (`d04d4b1`): mode 5 runs openpilot's `AccelBoost` (#39248) unchanged at `selfdrive/controls/lib/accel_boost.py` (+0.3 / 0.15 / 0 bias plus 20 / 10 / 0 % gain, only while the plan runs ahead), reset with the planner as openpilot; Relaxed follows at 1.45 s like Standard (#39440); Hold Boost in Curves `FM_OpBoostCurveHold` (on by default) gates it with openpilot's CurveSpeed (`curve_speed.py`, read only, never added to the target; plannerd reads `vehicleParameters`, ignored for validity); `test_fm_personality_settings.py` covers follow time, Following Time Offset, stop distance + personality link and following distance for every mode |
| Rivian counters 0-14 | merged, PR #31 (`c0e665b`): commaai/opendbc #4077 in `opendbc_repo/` (packer and parser; openpilot now checks Rivian counters like the panda) |
| hardwared NetworkMetered on change | merged, PR #32 (`fd96b7b`): openpilot #39481's hardwared lines |

## Reference (unchanged from before)

- **Frankie's side:** Windows PowerShell. Comma at `192.168.1.246`, user `comma`. Always `scp -O`. Say which prompt each
  command runs at (`PS C:\...>` or after `ssh comma@192.168.1.246`). He mostly talks from the Claude app on his phone.
- **His settings and usage:** never inferred from this file; he has driven all FM features. The migration copies what
  is set on the device.
- **Logs:** one-minute segments, `<route>--<n>/rlog.zst`. Model name only logged in `--0`. `tools/fm` joins segments.
- **Signs:** curvature and lateral accel positive **right**; angle, hand torque and torque commands (incl.
  `carOutput.actuatorsOutput.torqueOutputCan`, same sign as `actuators.torque`) positive **left**. Hand torque reads
  *against* openpilot from wheel reaction when loose.
- **Torque controller:** with `EnforceTorqueControl` on and `TorqueControlTune` 0, or Enforce off, sunnypilot runs
  **v0** (`latcontrol_torque_v0.py`), not v1. `torqueState.version` says which.
- **Safety:** never weaken driver monitoring; discuss panda safety limit or steering torque cap changes with him first
  (the Max Torque slider following the panda limit is agreed, 385 = stock).
