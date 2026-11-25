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

## Owners and next PR
- **ChatGPT:** plan PRs 8, 9, 16, 17, 18. Next assigned: **PR 16** Top Readout after its dependencies merge.
- **Claude:** plan PRs 10-15. Next: **PR 11** Turn Lead.
  M1 full suites passed on `d02379c` (only the 2 known failures plus sandbox-only ones: no network/display/GPU; the
  jetlink `test_seam` large-model test fails in the sandbox because tinygrad probes `/sys`, also before PR 9).
  PR 10 notes for PR 11: `FM_TurnPathMinRadius` is declared and read (Hold Path Source, Turn Nudge) but has no Sunnylink
  entry yet; add it on the Turn Lead Settings sub-page (trigger `FM_TurnLeadEnabled`). Turn Lead plugs into
  `update()` before Turn Hold (marked); add `lead_off`/`lead_paused` back to the Stop Before Turning Starts Over reset;
  fill the `lead`/`leadYield` log fields; add Turn Lead to Smooth Turn Start's enablement and description.
  `LEAD_DECEL_GATE` / `LEAD_MODEL_OPPOSE` are already in the module. For PR 12: use `SOURCE_CARRY`; restore
  `HOLD_AGAINST_TORQUE` (0.5 Nm), dropped in PR 10. Fresh-install defaults for Hold Follows the Model's Exit and Smooth
  Turn Start are now off (were on in ca08aac9); stored values carry over.
  PR 9 kept the two existing floor keys and AP's 1.0 m/s stock floor. The optional floor applies only to the
  action-head curvature conversion, independently of Turn Hold and Model Lateral Smoothing.

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
