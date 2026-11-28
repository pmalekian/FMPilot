# FM Pilot: instructions for Claude Code sessions in this repo

Frankie Malekian's fork of AdventurePilot (sunnypilot / openpilot) for his 2022 Rivian R1S Launch Edition (Gen 1) with a
comma 4 on the torque harness. This file is for local sessions (WSL2 on his Windows PC, often driven from his phone via
Remote Control). It mirrors the rules of the claude.ai "FM Pilot" Project, which local sessions can't see.

## Start here

1. Work on the branch Frankie names; otherwise `git fetch` and use the most recently updated active branch
   (currently `fm-dev-sync`; feature/PR branches are `fm-sync/...`). State branch and commit you start from.
2. Read `FM_PILOT_HANDOFF.md` (current state, findings, next steps) and `FM_PILOT.md` (features, settings, credits).
   GitHub is the source of truth for both. History is in `FM_PILOT_FINDINGS.md`; read it only when needed.
3. Log tools and how to use them: `tools/fm/README.md`.

## Goal and scope

Improve torque-based lateral control while keeping longitudinal smooth in city, suburban and highway driving. Focus on
Experimental Mode with lane centering and acceleration assist. Chill Mode and Dynamic Experimental Mode are out of
scope unless Frankie brings them up. His model choices and settings come from the handoff or from the logs, never
guessed. He favors balanced overall behavior.

## Safety (non-negotiable)

- Never weaken driver monitoring.
- Discuss any change to panda safety limits or the steering torque cap with Frankie before making it
  (the Max Torque slider following the panda limit is agreed; 385 = stock).
- Before suggesting he drive a new model or a change that could materially affect driving, test it offline against the
  base model or last known-good build on the same logs and settings. Name the baseline, report regressions, and say
  what remains unverified.
- New features are off by default unless he says otherwise.

## This machine

- WSL2 Ubuntu 24.04 on Windows 11, RTX 3080 Ti (CUDA via the Windows driver; never install a Linux NVIDIA driver).
  Set up by `tools/fm/wsl_setup.sh` (safe to re-run).
- Repo lives at `~/fmpilot` (Linux filesystem). Don't work under `/mnt/c` (slow).
- Comma: `ssh comma` (alias for `comma@192.168.1.246`). Pull segments into a working folder outside the repo, keeping
  the `<route>--<n>/rlog.zst` layout the tools expect, e.g.
  `for n in 3 4 5; do mkdir -p ~/fm-logs/R--$n; scp -O comma:/data/media/0/realdata/R--$n/rlog.zst ~/fm-logs/R--$n/; done`
  Always `scp -O`. The comma is only reachable when the car is on/awake and on home Wi-Fi.
- Live camera streaming from the comma into WSL is not supported; work from logs.

## Analysis

Frankie tunes by driving, sharing rlogs and describing events. Use `tools/fm` (`turns.py`, `timeline.py`, `events.py`,
`lead.py`, `sway.py`, `dm.py`, `settle.py`, `jetlink.py`). Lead with the plain-language finding, then the timestamps
(`--3 +12.4s` = segment 3, 12.4 s in), signals or code. Separate observations from hypotheses. If his impression
conflicts with your read, re-check the segment and account for model, build, mode or settings differences.
Sign conventions: curvature and lateral accel positive right; angle, hand torque and torque commands positive left.

## Checks before pushing

```bash
source .venv/bin/activate
scons -j$(nproc) -k
export PYTHONPATH=$PWD:$PWD/opendbc_repo:$PWD/msgq_repo:$PWD/rednose_repo:$PWD/tinygrad_repo:$PWD/teleoprtc_repo:$PWD/panda
SCALE=1 xvfb-run -a python -m pytest -q -p no:xdist --timeout=120 --continue-on-collection-errors \
  openpilot/sunnypilot openpilot/selfdrive/controls openpilot/selfdrive/ui/sunnypilot \
  openpilot/system/hardware/tests openpilot/system/manager \
  opendbc_repo/opendbc/car/rivian opendbc_repo/opendbc/sunnypilot/car/rivian opendbc_repo/opendbc/safety/tests/test_rivian.py
(cd tools/fm && python -m pytest tests)
python openpilot/sunnypilot/sunnylink/tools/compile_settings_ui.py && python openpilot/sunnypilot/sunnylink/tools/validate_settings_ui.py
ruff check openpilot opendbc_repo tools
```
Known pre-existing failures and expected results are listed in the handoff's "Checks off-device" section. Recompile
Sunnylink settings after any change to its YAML. Add or update tests for behavior changes. Report checks you couldn't run.

## Commits and pushes

- One logical change per commit. Messages start with `FM Pilot`. Author: `Frankie Malekian
  <107580415+pmalekian@users.noreply.github.com>` (set in this clone's git config).
- Credit borrowed code in the commit message and the source file, and record it in `FM_PILOT.md`.
- Fetch and check for intervening remote changes before pushing. Build, test, commit and push requested work so he can
  update from the comma.
- Update `FM_PILOT_HANDOFF.md` when a build, finding or decision changes what the next session needs; update
  `FM_PILOT.md` when features, settings or credits change.

## Talking to Frankie

He's usually on his phone. Keep replies short and plain. When he must run something, give exact copy-paste commands
and label the shell: Windows PowerShell (`PS C:\...>`), Ubuntu (WSL), or the comma over SSH.
