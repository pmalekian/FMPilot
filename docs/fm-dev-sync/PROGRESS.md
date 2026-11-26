# fm-dev-sync progress

Plan PR numbers (BUILD_PLAN.md) differ from GitHub PR numbers. Status as of Nov 25, 2026.

## Merged
| Plan PR | What | GitHub | Merge commit |
|---|---|---|---|
| 1 | Docs and tools | #5 | `fba53fc` |
| 2 | Log schema | #6 | `f887f49` |
| 3 | Settings framework | #7 | `aa95387` |
| 4 | FM Custom Torque Settings | #8 | `44d8a07` |
| 5 | Release Guard | #9 | `1653723` |
| 6 | Feedforward + Low-Speed Correction Strength | #10 | `60c057b` |
| 7 | Lane Keeping, Lane Guard, Blendful | #11 | `99a2b44` |
| 8 | Model Lateral Smoothing | #12 | `31be347ac` |
| 9 | Calmer Crawl-Speed Steering and Calmer Straight Pull-Away | #13 | `ce45f40de` |
| 10 | Turn Hold, Smooth Turn Start, Turn Assist logging | #16 | `19574d7` |
| 11 | Turn Lead | #17 | `b1b1800` |
| 12 | Finish Turn After Signal Cancels | #18 | `48c343f` |
| 13 | Pre/Post Turn Behavior (Queue Guard, Turn Pulse, Settle After Turn + Early Right Push) | #19 | `d3406ff` |
| 14 | Accel Boost picker + Boost bar | #20 | `e9ba455` |
| 15 | Lead (confidence, stationary ghosts, Smooth Lead Handoffs, `fmRadarLead`) | #21 | `998d062` |
| 16 | Top Readout | #24 | `97121bc` |
| 17 | Jetlink extras | #22 | `8e737bb` |
| 18 | hardwared stall diagnostics | #23 | `3674674` |
| — | Audit fixes (H-1, M-1, L-1..L-6; PR title rule) | #25 | `4324841` |
| — | Turn exit fixes (Hold Adds Your Steering 45° cutoff + M-3 Settle coordination) | #26 | `2aa8757` |

## Owners and next PR
- **Antigravity (post-audit fixes):** Audit fixes merged (#25, `4324841`). Turn exit fixes merged (#26, `2aa8757`): Hold Adds Your Steering stops latching and no longer blocks exit/release past 45° turned (pre-45° additions stay frozen until normal exit); Turn Hold ends its floor with a 0.5 s fade and does not restart once `fmModel` shows Settle After Turn / Early Right Push active for the current turn (M-3). `FM_FOLDED_DEFAULTS.md` (both copies) and `FM_PILOT.md` updated. Next: Lane keeping waits for the turn carry (`fm-sync/lane-keep-carry`, M-2).
- **ChatGPT:** plan PRs 8, 9, 16, 17, 18. PRs 16, 17 and 18 are merged. PR 16 Top Readout is merged (#24, checks run by Claude): card publishes
  `fmCarState` and the HUD uses it for Torque and `fmRadarLead` for Lead. `FM_TopReadout` preserves stored choices and
  defaults to Off on fresh installs. Retain the Top Readout status row when updating the pending PR 18 branch.
  **PR 17 Jetlink extras** is merged (#22, `8e737bb`, checks run by Claude): the Jetlink big model runs the Blendful and
  Settle After Turn callback again (`modeld_v2/fm_jetlink_action.py`); `fm_jetlinkstatusd` publishes `fmJetlinkStatus`
  while driving with Jetlink on; comma 4 icon states. Jetlink off changes nothing.
  **PR 18 hardwared stall diagnostics** is merged (#23, `3674674`, checks run by Claude): `hardwared_slow_loop` reports
  untimed work, thermal reads and watchdog data (`loop_timer.py`); keeps sunnypilot #2109's params-lock fix. Python-only,
  no native tests; the real stall report needs a drive on the comma 4.
- **Claude:** plan PRs 10-15. Done with 10-15. **End of M3 done** (Nov 25, 2026): full suites passed on `023ff40f` (only the 2 known base failures plus sandbox-only ones; `test_agnos_updater::test_manifest` needs network; PR 10-13 equivalence scripts: only the documented PR 13 Queue Guard carry and Settle-after-carry differences) and the final docs pass is done. Nothing next: Frankie drives and shares rlogs.
  PR 15 for PR 16 (Top Readout): radard publishes `fmRadarLead` in `RadarD.publish` right after radarState whenever
  the model has 2+ leads (fields: source, leadProb, liveLeadProb, earlyLead, lowSpeedLead, earlyRadarMatches,
  ghostDropped/DRel/YRel/VLead/TrackId, ghostDrops, guardEnabled, radarThreshold, cameraThreshold, smoothHandoffs,
  decisions[2]; counters reset when radard starts). `tools/fm/lead.py --replay` was fixed (it was broken on the base).
  ca08aac9's copy of the old shared `FM_LeadDetectionThreshold` into both confidence keys was not ported (only matters
  coming from a branch older than the split).
  PR 14: `fm_accel_boost_mode.py` alone picks the boost; `AccelBoostSP` and `AccelerationControl` are hidden and never
  read. Migration `accel_boost_mode` keeps a stored mode (ca08aac9's 0 was already no boost); only an unset mode with
  `AccelBoostSP` on becomes 2. `longitudinalPlan.accelBoost` = what the mode added. OP Personality verified against
  openpilot `ff300ecf` (#39430) and `6daa874b` (#39440); #39440's Relaxed follow-time change is not ported.
  For PR 15: plannerd now publishes `fmPlan`; planner test harness `test_acceleration_control_planner.py`.
  For PR 16 (Top Readout): `ui_state` and `augmented_road_view` have Boost bar hooks from PR 14.
  M2 full suites passed on `da5b06a` (same known/sandbox-only failures as M1; PR 10-13 equivalence scripts: only the
  documented PR 13 Queue Guard carry and Settle-after-carry differences).
  M1 full suites passed on `d02379c` (only the 2 known failures plus sandbox-only ones: no network/display/GPU; the
  jetlink `test_seam` large-model test fails in the sandbox because tinygrad probes `/sys`, also before PR 9).
  PR 13: Queue Guard holds now count as a wait in Turn Assist (controlsd subscribes to `fmModel`), so a hold can't
  start a carry (ca08aac9 could). Settle After Turn reads the carry from `fmControls` (modeld_v2 subscribes) and opens
  its window when the carry ends. Pre/Post Turn Behavior is a plain sub-page; all four toggles were already off.
  Fresh-install defaults now off where ca08aac9 had them on (stored values carry over): Hold Follows the Model's Exit,
  Smooth Turn Start (PR 10); Adaptive Look-Ahead, Wait for the Corner (PR 11); Finish Turn After Signal Cancels (PR 12).
  Turn Path Tightest Radius is on the Turn Lead page, so it shows only while Turn Lead is on.

## Conventions
- FM log messages use the free slots 14-19, from the top: `fmControls` @145, `fmModel` @144, `fmPlan` @143,
  `fmCarState` @142, `fmRadarLead` @141, `fmJetlinkStatus` @140. sunnypilot master took slot 10 (`gpsLocationSP`).
  Put FM data in these structs, not in sunnypilot's `*SP` structs.
- FM events go after AP's last event (quiet road-edge `fmLaneChangeRoadEdgeQuiet` @38).
- Migration pattern (`openpilot/sunnypilot/system/fm_migration.py`): an `@migration("name")` function using
  `put_if_unset`, a row in FM_PILOT.md's migration table, tests on real `Params` (old values -> new; fresh install ->
  nothing written). Old keys are read only, never written.
- `FM_CustomMaxTorque` is the new Max Torque key; the old `FM_MaxTorque` is left untouched.
- The full suites have 2 known base failures: `test_default_model::test_compare_onnx_hashes` (LFS) and
  `test_manager::test_set_params_with_default_value` (AP's `ScreenSaverCustomText`). Anything else is yours.
