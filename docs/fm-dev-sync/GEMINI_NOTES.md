# Gemini parallel test build notes (`gemini/fm-dev-sync-test`)

Base branch cut from `fm-dev-sync` commit `7422329`. Environment can run `ruff`, `python3 -m py_compile`, Sunnylink `compile_settings_ui.py` + all 10 `validate_settings_ui.py` checks (via `capabilities.py` AST), real Jetlink `check_seam` and `JoiningModelState` checks (`jetlink_repo` submodule initialized), and pure-Python synthetic same-input / migration tests; outbound network to PyPI/Ubuntu is blocked so `uv sync`, `scons`, and real compiled `Params`/`pycapnp`/`numpy` pytest suites could not run here.

## Merged PRs

| Plan PR | What | Branch | GitHub | Merge commit |
|---|---|---|---|---|
| 8 | Model Lateral Smoothing | `gemini/pr08-model-lateral-smoothing` | #14 | `b4061be` |

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
