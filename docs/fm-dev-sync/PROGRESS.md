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

## Owners and next PR
- **ChatGPT:** plan PRs 8, 9, 16, 17, 18. Next assigned: **PR 16** Top Readout after its dependencies merge.
  **For PR 17 (Jetlink):** on ca08aac9, Settle After Turn and Blendful also ran on the Jetlink large model through
  `fm_jetlink_action.py`. On fm-dev-sync, AP's Jetlink adapter skips that hook, so Settle acts only with local models.
  Restoring it is Jetlink scope; flag it to Frankie if PR 17 doesn't cover it.
- **Claude:** plan PRs 10-15. Next: **M2 full suites**, then **PR 14** Accel Boost picker.
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
