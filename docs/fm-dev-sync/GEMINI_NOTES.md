# Gemini parallel test build notes (`gemini/fm-dev-sync-test`)

Base branch cut from `fm-dev-sync` commit `7422329`. Environment can run `ruff`, `python3 -m py_compile`, Sunnylink `compile_settings_ui.py` + all 10 `validate_settings_ui.py` checks (via `capabilities.py` AST), real Jetlink `check_seam` and `JoiningModelState` checks (`jetlink_repo` submodule initialized), and pure-Python synthetic same-input / migration tests; outbound network to PyPI/Ubuntu is blocked so `uv sync`, `scons`, and real compiled `Params`/`pycapnp`/`numpy` pytest suites could not run here.

## Merged PRs

| Plan PR | What | Branch | GitHub | Merge commit |
|---|---|---|---|---|
| 8 | Model Lateral Smoothing | `gemini/pr08-model-lateral-smoothing` | #14 | `b4061be` |
| 9 | Calmer Crawl-Speed Steering + Calmer Straight Pull-Away | `gemini/pr09-calmer-crawl` | #15 | `12662f1` |

## Plan PR 8: Model Lateral Smoothing (PR #14, merge `b4061be`)
- **Scope:** Small Models (`FM_SmallModelLatSmoothing`) and Big Models (`FM_BigModelLatSmoothing`) drop-downs (`Model Default` / `0.00 s` / `0.10 s`, default `Model Default`), both `modeld_v2` and stock `modeld`, migrations `small_model_lat_smoothing` and `big_model_lat_smoothing`, Sunnylink UI under `fm_model_smoothing`, docs (`FM_PILOT.md`, `FM_PILOT_HANDOFF.md` with plan PR 7 marked merged as `#11` `99a2b44`).
- **Checks run and results:**
  - `ruff check` on all 7 changed/added Python files: **PASS** (0 findings).
  - `python3 -m py_compile` on all 7 changed/added Python files: **PASS**.
  - `params_keys.h` entry/duplicate/format verification: **PASS**.
  - `compile_settings_ui.py` + `--check`: **PASS**.
  - `validate_settings_ui.py` (all 10 checks via `CAPABILITY_FIELDS` AST): **PASS** (10/10).
  - `jetlink.openpilot.testing.check_seam` on `openpilot/selfdrive/modeld/modeld.py` and `openpilot/sunnypilot/modeld_v2/modeld.py`: **PASS** (0 errors).
  - Synthetic same-input equivalence vs `ca08aac9` `fm_big_model_smoothing.py` (200-frame 1 Hz sine wave + all model configs), migration matrix checks, and real `JoiningModelState` tests (with lightweight shims for unavailable `numpy`/`Params`): **PASS**.
- **Not checkable here:** `scons` build, real `Params` / `pycapnp` / `numpy` pytest runs, truck response, on-device CPU, UI look, LFS model compiles.
- **Surprises / gotchas:**
  - `openpilot/sunnypilot/jetlink_adapter/tests/test_seam.py` documents a subtle crash when stock `selfdrive/modeld/modeld.py`'s `get_action_from_model` takes a 6th positional argument (`lat_smooth_seconds`) because `modeld_v2` passes `blendful` as a 6th argument. Making `lat_smooth_seconds` **keyword-only** (`*, lat_smooth_seconds=LAT_SMOOTH_SECONDS`) in stock `modeld.py` lets small-model smoothing work in stock `modeld.py` while statically preventing any 6th positional argument from reaching `smooth_value`.
  - In `ca08aac9`, `fm_apply_model_settings` wrote `FM_LateralSmoothingEnabled` to every `ModelState` in `fm_own_model_states` (including a chestnut `ModelState(chestnut=True)`). With separate Small/Big drop-downs, checking `getattr(m, 'chestnut', False)` ensures the Small Models setting never shifts `lat_delay` on a chestnut big model, while Big Models `0.00 s` can still clear a chestnut bundle override and `0.10 s` stays post-action only.

## Plan PR 9: Calmer Crawl-Speed Steering + Calmer Straight Pull-Away (PR #15, merge `12662f1`)
- **Scope:** `FM_LowSpeedCurvatureFloor` (sunnypilot #2091 2.5 m/s floor as an opt-in setting on AP's 1.0 m/s default) and `FM_StraightPullAwayFloor` (`fm_pull_away.py` `StraightPullAway` + per-frame `fm_set_action_floor`), both off by default with unchanged keys (no migration needed), Jetlink `JoiningModelState` safe, Sunnylink UI toggles under `fm_smoothing` after `FM_LowSpeedCorrectionStrength`, docs (`FM_PILOT.md`, `FM_PILOT_HANDOFF.md`).
- **Checks run and results:**
  - `ruff check` on all 6 changed/added Python files: **PASS** (0 findings).
  - `python3 -m py_compile` on all 6 changed/added Python files: **PASS**.
  - Real `pytest -q openpilot/sunnypilot/modeld_v2/tests/test_fm_pull_away.py openpilot/sunnypilot/modeld_v2/tests/test_action_curvature_floor.py`: **PASS** (19/19 passed).
  - `params_keys.h` entry/duplicate/format verification: **PASS**.
  - `compile_settings_ui.py` + `--check`: **PASS**.
  - `validate_settings_ui.py` (all 10 checks via `CAPABILITY_FIELDS` AST): **PASS** (10/10).
  - `jetlink.openpilot.testing.check_seam` on `openpilot/selfdrive/modeld/modeld.py` and `openpilot/sunnypilot/modeld_v2/modeld.py`: **PASS** (0 errors).
  - Synthetic same-input equivalence of `StraightPullAway` vs `ca08aac9` across 2,000 randomized + edge (`NaN`, standstill, handover) frames, Sunnylink UI item order under `fm_smoothing`, and real `JoiningModelState` check for `fm_set_action_floor`: **PASS** (bit-for-bit identical).
- **Not checkable here:** `scons` build, real `Params` / `pycapnp` / `numpy` pytest runs (`test_fm_low_speed_floor.py`, `test_fm_model_settings.py` end-to-end), truck response, on-device CPU, UI look, LFS model compiles.
- **Surprises / gotchas:**
  - AdventurePilot's base branch includes `openpilot/sunnypilot/modeld_v2/tests/test_action_curvature_floor.py`, which regex-matched literal `max(1.0, v_ego)` in both `selfdrive/modeld/modeld.py` and `sunnypilot/modeld_v2/modeld.py`. Replacing `1.0` with `speed_floor` in `modeld_v2/modeld.py` would have failed that guard test; updated it so stock `modeld.py` is still checked for literal `1.0` and `modeld_v2/modeld.py` is guarded at `ACTION_SPEED_FLOOR = 1.0` (catching any accidental upstream sync back to 2.5 by default).
  - Note: `FM_PILOT_HANDOFF.md` in PR 9 records PR 8 as merged under `#14` (`b4061be`) on `gemini/fm-dev-sync-test` (still referencing the test base PR number rather than `fm-dev-sync`).
