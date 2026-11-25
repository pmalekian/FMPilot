# FM Pilot: handoff for new chats (short)

State as of Nov 25, 2026: **`fm-dev-sync` is being built PR by PR** from the agreed plan (project docs
`claude/FM_DEV_SYNC_BUILD_PLAN.md`, `claude/FM_DEV_SYNC_INVENTORY.md`, `claude/FM_FOLDED_DEFAULTS.md`; repo copy
`FM_FOLDED_DEFAULTS.md`). Planning is finished: don't re-plan. History before `fm-dev-sync` (every earlier branch, build
and drive analysis, the old handoff word for word) is in `FM_PILOT_FINDINGS.md`: read it only when a task needs it.
Features, settings and credits: `FM_PILOT.md`. Log tools: `tools/fm/README.md`.

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
- **Commits:** author `Frankie Malekian <107580415+pmalekian@users.noreply.github.com>`, message starts "FM Pilot", one
  logical change each, borrowed code credited in the message, the source file and `FM_PILOT.md`. Push with
  `--no-verify` and `-c push.negotiate=true`.
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
| PR 1 docs and tools | this PR: `tools/fm` without nav (reads old drives with a copy of `fm-dev-nav-settle`'s schema), `turns.py` path timing from `fm-dev-max` `d189ccc`, fresh `FM_PILOT.md` / handoff, `FM_FOLDED_DEFAULTS.md`, `FM_PILOT_FINDINGS.md` |
| PRs 2-18 | not started |

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
