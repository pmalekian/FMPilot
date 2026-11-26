# Rules for any agent building an fm-dev-sync PR
Read first: PROGRESS.md (incl. Conventions), BUILD_PLAN.md, INVENTORY.md, FM_FOLDED_DEFAULTS.md (this folder), and the status table in FM_PILOT_HANDOFF.md.
- Before starting a PR: pull fm-dev-sync and check in PROGRESS.md that the PR is yours and its dependencies are merged. Only touch your PR's scope; another agent works on the same branch.
- Port the feature from fm-dev-nav-settle (ca08aac9) onto current fm-dev-sync, reshaped as BUILD_PLAN.md and INVENTORY.md describe. New or reshaped settings default to stock/off; their migration copies the old values (pattern in Conventions).
- Branch fm-sync/prNN-short-name. One logical change per commit, messages starting "FM Pilot", author Frankie Malekian <107580415+pmalekian@users.noreply.github.com>. Credit borrowed code in the commit message, the source file and FM_PILOT.md.
- In the PR, update FM_PILOT.md (feature, settings, migration row), FM_PILOT_HANDOFF.md's status table, and docs/fm-dev-sync/FM_FOLDED_DEFAULTS.md if a folded behavior changes.
- Checks: build; unit tests for the files you touch, plus new tests for reshaped settings and the migration; ruff on changed files; Sunnylink settings recompile + validator if YAML changed. Only the 2 known base failures are acceptable. No drive-log replays.
- Never weaken driver monitoring. Don't touch panda safety code or torque limits.
- PR titles are 'FM Pilot: <feature name>' with no plan PR numbers.
- Open the PR into fm-dev-sync with a short description: what it does, the checks run and their results, what couldn't be checked.
- Merge only if you actually ran the checks and they passed. If your environment can't run the build or tests, don't merge: label the PR "needs-checks" and say so in PROGRESS.md; Claude will run the checks and merge it.
- After merging, update PROGRESS.md in a separate commit: merged, its GitHub # and merge commit, anything the next PR needs to know.
- If the plan looks wrong or your files conflict with the other agent's work, stop and tell Frankie instead of working around it.
