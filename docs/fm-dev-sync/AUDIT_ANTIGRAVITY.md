# Independent Audit of `fm-dev-sync` (Antigravity)

- **Audited branch / commit:** `fm-dev-sync` @ `aca552ece780502f2b3d54482829b858580d6ec7`
- **Audit branch:** `audit/antigravity-2026-11-25`
- **Reference commits checked against:**
  - AdventurePilot `dev-next-src`: `cf3ad0ec0ccfbc507dd08e970c6cc2065d01f68c`
  - sunnypilot `master` (merge base in Step 0 `4efb05a`): `c410198c` (fetched tip `6732fcc52a26ad6eda10f436595ca67c892c58cf`)
  - FM Pilot `fm-dev-nav-settle`: `ca08aac90e91236124da2317988535e8a9190758`
- **Environment capability:** Full Linux (Ubuntu 24.04) build + test environment available (`uv`, `scons`, `arm-none-eabi-gcc`, `pytest`, `ruff`, `cppcheck` MISRA, Sunnylink YAML compiler/validator).
  - **Could run:** Full `scons -j2 -k` (C++/Cython/capnp/STM32H7 panda firmware/`libpanda`/`libsafety`), MISRA static analysis (`opendbc/safety/tests/misra/test_misra.sh`), Sunnylink YAML compile & 10-check validator, `ruff`, `tools/fm` pytest, PR 10 equivalence script against `ca08aac9`, and full pytest suites (`sunnypilot`, `controls`, `selfdrived`, `car/tests/test_fm_car_state.py`, `modeld`, `ui/sunnypilot`, `system/hardware/tests`, `common/hardware/tests`, `system/manager`, Rivian car/safety).
  - **Could not run / Not audited dynamically:** On-device comma 4 driving/UI rendering/CPU profiling, GPU/OpenCL/display-dependent tests, network-dependent updater tests, the 2 LFS ONNX model compiles (`dmonitoring_model_tinygrad.pkl`, `driving_tinygrad.pkl` — LFS pointers not pulled), and the full all-cars `selfdrive/car` interface matrix; PR 11–13 multi-million-step Monte Carlo equivalence scripts were cancelled after ~3–5 min on 2 cores to avoid starving the pytest suites (their unit tests and code diffs were all audited directly).

---

## Severity-Ranked Findings

### High

#### [H-1] Cold-Boot / Onroad UI Restart `AttributeError` Crash in `UIStateSP` (PR 14)
- **Severity:** High
- **Status:** Confirmed (empirically reproduced)
- **Location:** `openpilot/selfdrive/ui/sunnypilot/ui_state.py:45-77`, `openpilot/selfdrive/ui/sunnypilot/ui_state.py:191-192`, `openpilot/selfdrive/ui/sunnypilot/mici/onroad/accel_boost_indicator.py:67-69`
- **What's wrong:** `UIStateSP.__init__` initializes `self.blendful_enabled` (line 61) and `self.fm_top_readout` (line 76), but **never initializes `self.fm_accel_boost_indicator` or `self.fm_accel_boost_mode`**. Both attributes are only assigned inside `UIStateSP.update_params()` (lines 191–192), which runs asynchronously on the daemon thread `_params_refresh_worker` spawned on the first `ui_state.update()` call (`openpilot/selfdrive/ui/ui_state.py:153-155`), after ~30 param reads and `jetlink_adapter.status()`.
- **Concrete driving / runtime failure scenario:** If `ui` starts while already onroad (e.g., cold boot with ignition on, or `ui` watchdog restart mid-drive), `AugmentedRoadView._render` (`openpilot/selfdrive/ui/mici/onroad/augmented_road_view.py:282`) invokes `self._accel_boost_indicator.render()` -> `_update_state()` -> `indicator_wanted()` before `_params_refresh_worker` finishes its first pass. Reading `ui_state.fm_accel_boost_indicator` raises `AttributeError: 'UIState' object has no attribute 'fm_accel_boost_indicator'`, crashing `ui` (which can enter a crash loop on slow I/O).
- **Evidence:**
  - Direct instantiation of `UIStateSP`: `hasattr(d, "fm_accel_boost_indicator") == False`, `hasattr(d, "fm_accel_boost_mode") == False` (while `blendful_enabled` and `fm_top_readout` are `True`).
  - `openpilot/selfdrive/ui/sunnypilot/mici/tests/test_accel_boost_indicator.py:34-35` used `monkeypatch.setattr(ui_state, "fm_accel_boost_indicator", ..., raising=False)`, which masked the missing attribute during unit tests.
- **Suggested fix:** Initialize both defaults in `UIStateSP.__init__`:
  ```python
  self.fm_accel_boost_indicator: bool = False
  self.fm_accel_boost_mode: int = MODE_OFF
  ```

---

### Medium

#### [M-1] Headroom + Lateral Jerk / NNLC Leaves `pid` Limits in Lateral-Accel Units (2.8× Too High) After Startup Blend (PR 4)
- **Severity:** Medium
- **Status:** Confirmed (empirically reproduced on both v0 and v1 torque controllers)
- **Location:** `openpilot/selfdrive/controls/lib/latcontrol_torque.py:78-90`, `openpilot/sunnypilot/selfdrive/controls/lib/latcontrol_torque_v0.py:80-92`, `openpilot/sunnypilot/selfdrive/controls/lib/latcontrol_torque_ext_base.py:64`, `openpilot/sunnypilot/selfdrive/controls/lib/latcontrol_torque_ext.py:21`
- **What's wrong:** `LatControlTorqueExtBase.__init__` sets `self._pid = PIDController(KP, KI)` (a dummy PID instance distinct from `lac_torque.pid`). `self._pid` is only rebound to `lac_torque.pid` inside `LatControlTorqueExt.update()` on the first **active** frame (`active=True`). Meanwhile, `FmTorqueMode.update(CS.vEgoRaw)` runs on every frame including `active=False` and ramps `w_headroom` from `0.0 -> 1.0` during the first 1.0 s of onroad startup. Each ramp frame triggers `if fm_max != self.steer_max:` -> `self.update_limits()` (writing lateral-accel limits, e.g. `±4.22`, to the real `self.pid`) followed by `self.extension.update_limits()` (which writes torque-unit limits `±1.51` to the **dummy** `_pid`, leaving `self.pid` at `±4.22`!).
- **Concrete driving / runtime failure scenario:** Driver starts a Rivian with `FM_CustomMaxTorque = 580` and sunnypilot's `LateralJerkTorqueController` (or `NeuralNetworkLateralControl`) enabled. The truck sits disengaged for ≥ 1 s (normal), so `w_headroom` reaches `1.0` and `self.steer_max = 580/385 = 1.5065`. The driver then engages below 9 m/s (20 mph), where `_ratio(v_ego)` is flat at `1.5065`:
  - On frame 0 of engagement (even with `lateralTorqueParameters.useParams == True`), `extension.update()` binds `self._pid = self.pid` and runs torque-space PID with `pos_limit = 4.2182` instead of `1.5065`.
  - If `lateralTorqueParameters.useParams` is `False` (Custom Torque Tuning, Real-Time Tuning, or cold live torque before convergence), `fm_max != self.steer_max` never fires while speed stays ≤ 9 m/s, so `self.pid.pos_limit` stays stuck at **`4.2182`** (2.8× the intended torque limit) indefinitely below 20 mph.
  - Also, `if self.extension.update_override_torque_params(self.torque_params): self.update_limits()` (line 78–79 / 80–81) resets `self.pid` to accel limits without calling `self.extension.update_limits()`.
  - *Why not Critical:* Both `CarController` (`apply_driver_steer_torque_limits` + `rt_limiter`) and the Rivian panda safety code still hard-cap CAN torque at `steer_cap` (≤ 580), so physical torque cannot exceed the configured safety limit. However, PID integrator windup (at 5–9 m/s where integrator is unfrozen) and `CC.actuators.torque` overshoot (triggering false `steer_limited_by_safety` and wrong saturation timing) occur when Lateral Jerk or NNLC is enabled.
- **Evidence:**
  - Live reproduction on both `LatControlTorque` (v1) and `LatControlTorqueV0` (v0) at `FM_CustomMaxTorque=580`, `LateralJerkTorqueController=True`: after 120 disengaged frames at 0 m/s, engaging at 5 m/s yields `steer_max=1.5065, pid.pos_limit=4.2182` on frames 0 through 4+.
  - `test_fm_headroom_limits.py:61-62` skipped `i == 0` with a comment, started immediately with `active=True`, and forgot to set `CS.vEgoRaw`, missing the realistic disengaged-startup scenario.
- **Suggested fix:** Bind `self._pid = lac_torque.pid` in `LatControlTorqueExtBase.__init__` (or have `LatControlTorqueJerkAware.update_limits` / `NeuralNetworkLateralControl.update_limits` set limits on `self.lac_torque.pid`), and call `self.extension.update_limits()` after `update_override_torque_params` as well.

#### [M-2] Cross-Feature Conflict: `Blendful` (and `Lane Centering`) Activate Immediately During a `Finish Turn After Signal Cancels` Carry (PR 7 + PR 12)
- **Severity:** Medium
- **Status:** Confirmed (static trace)
- **Location:** `openpilot/sunnypilot/modeld_v2/modeld.py:555`, `openpilot/selfdrive/controls/controlsd.py:148-153`
- **What's wrong:** PR 13 explicitly taught `TurnSettle` to treat a `Finish Turn After Signal Cancels` carry (`fmControls.turnAssist.source == carry`) as the turn still in progress (`carry_from_sm(sm)`). However, **`Blendful`** (`modeld_v2/modeld.py:555`: `blendful_active = desire == log.Desire.none and DH.lane_change_state == log.LaneChangeState.off`) and **AP `Lane Centering`** (`controlsd.py:148-150`, gated on `bool(CS.leftBlinker or CS.rightBlinker)`) do **not** know about an active Turn Assist carry.
- **Concrete driving / runtime failure scenario:** Driver has `FM_LaneKeepMode = Blendful` (whose `MIN_SPEED` FM lowered from 5.0 m/s to **0.3 m/s** in PR 7) or `Lane Centering`, plus `FM_TurnHoldEnabled` / `FM_TurnLeadEnabled` + `FM_TurnCarryThrough`. Mid-turn (e.g., 30° turned at 10 mph), the blinker self-cancels. Turn Assist enters `carry` mode to finish the turn, but on the very next frame `DH.desire` becomes `none` and `CS.leftBlinker/rightBlinker` become `False`, allowing `Blendful` (down to 0.3 m/s) or `Lane Centering` to immediately inject lane-centering curvature if lane lines are visible while Turn Assist is simultaneously trying to hold/lead the turn.
- **Evidence:** `modeld_v2/modeld.py:555` checks only `desire == log.Desire.none` while line 561 passes `sm` to `settle_from_model` specifically to read `fmControls` carry. `controlsd.py:148-153` runs `self.lane_centering.update(...)` before `self.fm_lateral.update(...)` using only raw blinker booleans.
- **Suggested fix:** Gate `blendful_active` in `modeld_v2/modeld.py` with `and carry_from_sm(sm) == 0`, and pass `blinker or self.fm_lateral.turn_assist.carry_sign != 0` into `self.lane_centering.update(...)` in `controlsd.py`.

#### [M-3] Cross-Feature Conflict: `Early Right Push` (`FM_TurnSettleEarlyPush`) Can Fight `Turn Hold` While Blinker Remains Active After Straightening (PR 10 + PR 13)
- **Severity:** Medium
- **Status:** Confirmed (static trace)
- **Location:** `openpilot/sunnypilot/modeld_v2/fm_turn_settle.py:268-276`, `openpilot/sunnypilot/selfdrive/controls/lib/fm_turn_assist.py:584-641`
- **What's wrong:** `Early Right Push` intentionally starts its settle window *before* the turn desire ends, once `turned >= 45°` and the truck has straightened (`toward < min(0.3 * peak, 0.02)` for 0.3 s). It adds rightward curvature (`+0.004` to `+0.01 1/m`) directly to `modelV2.action.desiredCurvature` in `modeld_v2`. If the blinker is still on (hasn't self-cancelled yet), `controlsd`'s `Turn Hold` is still active on the same turn:
  - After a **left** turn (`sign = -1`) with `FM_TurnHoldFollowExit = False` (now the fresh-install default!): `Turn Hold` sees `toward = -desired_curvature < floor` and **overrides Early Right Push back to the left-turn floor** until 75° / 8 s / blinker cancel! Even with `FM_TurnHoldFollowExit = True`, the 0.5 s `exit_fade` still pulls left against Early Right Push.
  - After a **right** turn (`sign = +1`): Early Right Push adds positive rightward curvature into `desiredCurvature`, which inflates `model_toward` seen by `Turn Hold` and can keep `Turn Hold`'s floor alive or delay `Hold Follows the Model's Exit`.
- **Concrete driving / runtime failure scenario:** Left turn of 55° where the blinker stays on 1–2 s past turn exit with `FM_TurnHoldEnabled = True` (`FM_TurnHoldFollowExit = False`) and `FM_TurnSettle = True`, `FM_TurnSettleEarlyPush = True`: `modeld_v2` adds rightward push while `controlsd` clamps curvature back to the leftward hold floor, cancelling Early Right Push and logging contradictory `fmModel.turnSettle > 0` vs `fmControls.turnAssist.holdFloor > 0`.
- **Evidence:** `fm_turn_settle.py:268-276` starts the window while `desire_sign == self.sign`; `fm_turn_assist.py:584-641` applies Turn Hold downstream in `controlsd` without checking `fmModel.turnSettle`.
- **Suggested fix:** Either suppress `Turn Hold` floor once `fmModel.turnSettlePhase != idle` (or `turnSettle != 0`), or document clearly that `Hold Follows the Model's Exit` should be enabled alongside `Early Right Push` and check `fmModel` in `fm_lateral.py`.

---

### Low

#### [L-1] `max_torque_grid(float('inf'))` Clamps to Maximum Torque (`580`) Instead of Safe Stock Fallback (`385`) (PR 4)
- **Severity:** Low
- **Status:** Confirmed (empirically reproduced)
- **Location:** `opendbc_repo/opendbc/sunnypilot/car/rivian/values.py:36-45`
- **What's wrong:** `max_torque_grid` checks `if v != v:` (which catches `NaN`), but does not check `math.isfinite(v)`. Passing `float('inf')` or `"inf"` falls through to `min(max(inf, 385), 580)` -> **`580` (level 13)** instead of falling back to stock `385`.
- **Concrete driving / runtime failure scenario:** Only reachable if `FM_CustomMaxTorque` param storage is corrupted with `"inf"` (normal UI/Params writes are `INT`-typed). If it occurred, the fallback would pick maximum torque (580) rather than stock (385).
- **Evidence:** `max_torque_grid(float('inf')) == 580`, `max_torque_level(float('inf')) == 13`.
- **Suggested fix:** Replace `if v != v:` with `if not math.isfinite(v):`.

#### [L-2] `FmTorqueMode._ratio` Does Not Guard Non-Finite `v_ego` (PR 4)
- **Severity:** Low
- **Status:** Confirmed (static inspection)
- **Location:** `openpilot/sunnypilot/selfdrive/controls/lib/fm_torque_mode.py:25-28`
- **What's wrong:** `b = round(float(np.interp(v_ego, *base)))` raises `ValueError: cannot convert float NaN to integer` if `CS.vEgoRaw` is ever `NaN`. (`carcontroller.py:153` has the same pre-existing AP pattern.)
- **Concrete driving / runtime failure scenario:** A `NaN` `vEgoRaw` in `carState` would crash `controlsd` inside `FmTorqueMode.update` before `controlsd`'s `math.isfinite` actuator sanitizer runs.
- **Suggested fix:** Guard `if not math.isfinite(v_ego): return 1.0` in `FmTorqueMode.update()`.

#### [L-3] Sunnylink `FM_ReleaseGuardLimit` Not Greyed Out When Lateral Jerk / NNLC Is Enabled (PR 5)
- **Severity:** Low
- **Status:** Confirmed (static inspection)
- **Location:** `openpilot/sunnypilot/sunnylink/settings_ui_src/pages/steering.yaml:476-492`
- **What's wrong:** Unlike `FM_LowSpeedFFLeft` / `FM_LowSpeedFFRight` (lines 515–520, 536–541), which have Sunnylink `enablement` rules greying them out when `LateralJerkTorqueController` or `NeuralNetworkLateralControl` is true, `FM_ReleaseGuardLimit` only checks `brand == rivian` even though `FmReleaseGuard` stands down with both (`fm_release_guard.py:64`) and its description says "off with Lateral Jerk or NNLC".
- **Concrete driving / runtime failure scenario:** Cosmetic/UX: user can toggle Release Guard in Sunnylink while Lateral Jerk/NNLC is on and wonder why it has no effect.
- **Suggested fix:** Add the same two `type: not` enablement conditions for `LateralJerkTorqueController` and `NeuralNetworkLateralControl` to `FM_ReleaseGuardLimit` and recompile `settings_ui.json`.

#### [L-4] `tools/fm/fmlog.py` `NEW_FM_MSGS` Omits `fmRadarLead` and `fmJetlinkStatus` (PR 2)
- **Severity:** Low
- **Status:** Confirmed (static inspection)
- **Location:** `tools/fm/fmlog.py:36`
- **What's wrong:** `NEW_FM_MSGS = frozenset(("fmControls", "fmModel", "fmPlan", "fmCarState"))`. On `fm-dev-sync`, `fmJetlinkStatus` is at `@140` (`CustomReserved14`) and `fmRadarLead` is at `@141` (`CustomReserved15`), slots that were completely unused on `fm-dev-nav-settle` (where they were `@137` and `@138`). Omitting them from `NEW_FM_MSGS` means a synthetic or filtered log from a non-`fm-dev-sync` branch name containing only `fmRadarLead` or `fmJetlinkStatus` would be misidentified as `old` by `pick_schema()`.
- **Concrete driving / runtime failure scenario:** Log-analysis tool edge case only (full rlogs always contain `fmControls`/`fmCarState`).
- **Suggested fix:** Include `"fmRadarLead"` and `"fmJetlinkStatus"` in `NEW_FM_MSGS`.

#### [L-5] Minor Doc / UI Naming Discrepancies
- **Severity:** Low
- **Status:** Confirmed (static inspection)
- **Location:** `docs/fm-dev-sync/BUILD_PLAN.md:56-57` vs `FM_PILOT.md:34` & `fm_low_speed_gain.py:41`; `openpilot/sunnypilot/sunnylink/settings_ui_src/pages/steering.yaml:94` vs `docs/fm-dev-sync/INVENTORY.md:65`
- **What's wrong:**
  1. `BUILD_PLAN.md:57` ends PR 6 with "stand down with Lateral Jerk / NNLC", whereas `INVENTORY.md:34`, `FM_PILOT.md:34`, `ca08aac9`, and `fm_low_speed_gain.py` intentionally only stand down Feedforward (not Low-Speed Correction Strength) with Lateral Jerk / NNLC.
  2. `steering.yaml:94` titles the `FM_LaneKeepMode` dropdown `"Lane Centering"` (same as the section title) instead of `"Lane Keeping"` as described in `INVENTORY.md:65` and `FM_PILOT.md:36`.
- **Suggested fix:** Optionally rename `steering.yaml:94` title to `"Lane Keeping"` (and recompile JSON) if desired.

#### [L-6] Global `Params()` Leak in `test_fm_low_speed_ff.py` and `test_fm_release_guard.py` (PR 5, PR 6)
- **Severity:** Low
- **Status:** Confirmed (empirically observed during pytest run)
- **Location:** `openpilot/sunnypilot/selfdrive/controls/lib/tests/test_fm_low_speed_ff.py:160-191`, `openpilot/sunnypilot/selfdrive/controls/lib/tests/test_fm_release_guard.py:230-266`
- **What's wrong:** Unlike tests subclassing `OpenpilotTestCase` (which isolate `OPENPILOT_PREFIX`), these standalone pytest functions instantiate `LatControlTorque` / `LatControlTorqueV0` against the real global `Params()` directory (`~/.comma/params/d/`). If `LateralJerkTorqueController` or `NeuralNetworkLateralControl` is set to `True` in the ambient `Params()` store, 10 tests fail (`available == False`).
- **Concrete driving / runtime failure scenario:** Test-only flake on developer workstations or CI environments with leftover params; does not affect driving.
- **Evidence:** Running `pytest openpilot/sunnypilot` while ambient `LateralJerkTorqueController=True` failed 10 tests across these two files; clearing the ambient param made all 48 tests pass in 1.35 s.
- **Suggested fix:** Wrap controller creation in `with OpenpilotPrefix():` (or use `OpenpilotTestCase`).

---

## Audit Section Summaries (Sections 1–8)

### 1. Safety
- **Rivian panda safety (`opendbc_repo/opendbc/safety/modes/rivian.h`):**
  - `RIVIAN_STEERING_LIMITS` (stock) is unchanged from AP `dev-next-src` (`cf3ad0ec`): max torque `385`, lookup `[9, 25, 27] -> [385, 295, 275]`, `max_rate_up = 3`, `max_rate_down = 5`, `max_rt_delta = 125`, `driver_torque_multiplier = 2`, `driver_torque_allowance = 100`.
  - `rivian_init` decodes bit 1 (`HIGH_TORQUE = 2`) and bits 2–5 (`level = (current_safety_param_sp >> 2U) & 0xFU`), accepting **only** `1 <= level <= 13`, computing `rivian_high_torque_cap = 385 + 15 * level` (`400..580`). Maximum possible cap is strictly `580`. Any invalid/missing/0/14/15 level or unset `HIGH_TORQUE` bit sets `rivian_high_torque = false` and `rivian_high_torque_cap = 385` (exact stock AP behavior).
  - `RIVIAN_FAST_RAMP_MAX_SPEED = 12.5` m/s uses `vehicle_speed.max / VEHICLE_SPEED_FACTOR` (`1000.0` double, no integer truncation) so rising speed drops back to `max_rate_up = 3` immediately; `max_rt_delta = 125` and driver-torque rules are untouched.
  - **Car vs. Panda envelope verified empirically:** 0 violations across all 13 levels (`400..580` step 15) × 4,001 speeds (`0..40` m/s, step 0.01) against panda's actual `lateral.h` interpolation (`fudged_speed = v - 1.0`, `+1`, clamped to `max_torque`).
  - **MISRA static analysis (`test_misra.sh`):** **PASS** (0 errors/warnings across 268 active checkers).
  - **Panda STM32H7 firmware compilation:** **PASS** (`-Wall -Wextra -Wstrict-prototypes -Werror`).
  - Minor findings: `[M-1]` (PID limits with Lateral Jerk/NNLC — still capped by carcontroller & panda), `[L-1]` (`inf` in `max_torque_grid`), `[L-2]` (`NaN` `v_ego` in `FmTorqueMode`).
- **Driver monitoring:** **100% unchanged** vs AP `dev-next-src` (`git diff ap/dev-next-src..aca552e -- openpilot/selfdrive/monitoring/ openpilot/sunnypilot/selfdrive/monitoring/` is empty; `selfdrived.py` DM escalation & lockout logic untouched).
- **`steerLimitTimer` 0.8 & off-tracking fix:** `opendbc_repo/opendbc/car/rivian/interface.py:26` sets `ret.steerLimitTimer = 0.8`. `openpilot/sunnypilot/selfdrive/controls/lib/rivian_offtrack.py:16-26` computes `(x*x) / (1 + sqrt(1 - x*x))` directly in curvature space, algebraically identical to the radius form while eliminating `OverflowError` for near-zero curvature (`|k| < 1e-154`) and guarding `gain <= 0`, non-finite `curvature`, and `|x| >= 1`.

### 2. Cross-Feature Conflicts, Execution Order & Compatibility
- **Lateral pipeline order verified:**
  1. `modeld_v2`: raw model output -> `StraightPullAway` (`action_speed_floor`: 1.0 or 2.5 m/s) -> `lane_keep` callback (`Blendful` [MIN_SPEED 0.3 m/s] + `TurnSettle` / `Early Right Push`) -> small/big model smoothing (`fm_lateral_smoothing.py`).
  2. `modeld_v2` publishes `fmModel` **before** `modelDataV2SP` so `controlsd` never observes a Queue Guard-hidden turn desire before `fmModel.queueGuard`.
  3. `controlsd`: `lane_centering` (AP, gated by `FM_LaneKeepMode == 1`) -> `fm_lateral.update()` (`FmTurnAssist` [Carry -> Turn Lead -> Turn Hold (Driver Adds -> Hold Path -> Hold Floor) -> Turn Nudge -> Smooth Turn Start], then `LaneGuard` [gated by `FM_LaneKeepMode == 2`]) -> `rivian_offtrack` -> `clip_curvature` -> `LaC.update()` (`FmLowSpeedGain` -> `FmTorqueMode` -> `FmLowSpeedFF` -> PID -> `FmReleaseGuard` -> `LatControlTorqueExt`).
- **Longitudinal pipeline order verified:**
  - `radard` (`FmRadarSettings` -> stationary ghost guard -> dual-threshold radar/camera admission -> `FmLeadHandoff` -> publishes `radarState` + `fmRadarLead`) -> `plannerd` (`LongitudinalPlanner` -> `FmAccelBoostMode` [single active boost among 0–5; `AccelBoostSP` & `AccelerationControl` never read] -> `min(mpc, cruise, e2e)` -> publishes `longitudinalPlan` with `accelBoost = applied` + `fmPlan`).
- **Confirmed bugs/conflicts found:** `[H-1]`, `[M-1]`, `[M-2]`, `[M-3]`, `[L-1]`, `[L-2]`.

### 3. Base Merge & opendbc Re-Sync
- Audited Step 0 merge commit `4efb05a` against both `ap/dev-next-src` (`cf3ad0ec`) and sunnypilot `c410198c`:
  - `opendbc_repo/` diff vs AP touches 19 files (Hyundai updates, `mads.h` upstream #562 alignment keeping AP's reset-on-exit, `mads_common.py`, `libsafety/safety.c`); **0 AP Rivian files lost or altered**.
  - GPS fuser #2112 (`gps_fuser.py`, `gpsLocationSP @136`, `LastGPSPositionFused`, `locationd_llk` removed) cleanly merged with 0 stale `LastGPSPositionLLK` or `locationd_llk` references.
  - `hardwared.py` kept sunnypilot #2109 params-lock fix (tici check hoisted before loop) alongside AP's loop timer and Jetlink alert.
  - AP's `screen_saver.py` preserved over sunnypilot's.

### 5. Settings Migration & Defaults
- `openpilot/sunnypilot/system/fm_migration.py` is called from `params_migration.run_migration()` in `manager.py:53` **before** `manager.py:56-59` populates default values for unset params.
- All 7 `@migration` functions (`lane_keep_mode`, `max_torque`, `release_guard`, `low_speed_feedforward`, `low_speed_correction`, `model_lateral_smoothing`, `accel_boost_mode`):
  - Run once (recorded in `FM_SyncMigration`; exceptions logged and retried next boot).
  - Write only via `put_if_unset` (or `migrate_lane_keep_mode`, which checks `parse_mode(params.get(PARAM_MODE)) is not None`).
  - Never write any old key (all old keys declared in `params_keys.h` with **no default**, preserving rollback to `fm-dev-nav-settle`).
  - Match `FM_PILOT.md`'s migration table row-for-row.
- Every new/reshaped FM key in `params_keys.h` defaults to stock/off on fresh install (`FM_CustomMaxTorque=385`, `FM_LowSpeedCorrectionStrength=100`, `FM_RadarLeadConfidence=50`, `FM_CameraLeadConfidence=50`, `FM_TurnPathMinRadius=12.0`, `FM_TurnLeadMinSpeed=7.0`, `FM_TurnLeadGateDist=5.0`, all toggles/modes `0`).

### 6. Log Schema
- All 6 FM messages use `CustomReserved14`–`19` (`@140`–`@145`) keeping original struct IDs: `fmJetlinkStatus @140`, `fmRadarLead @141`, `fmCarState @142`, `fmPlan @143`, `fmModel @144`, `fmControls @145`. Zero fields added to sunnypilot's `*SP` structs.
- `gpsLocationSP` occupies `@136` (`CustomReserved10`); `@137`–`@139` remain free.
- `OnroadEventSP.fmLaneChangeRoadEdgeQuiet` is at `@38`, cleanly after AP's Jetlink events (`@35`–`@37`).
- `services.py` and generated `services.h` include all 6 FM services.
- `tools/fm` (`87 passed`, `ruff` clean) reads both `fm-dev-nav-settle` and `fm-dev-sync` schemas via `fmlog.py` (minor `[L-4]` note).

### 7. Dropped Features Truly Gone
- Verified via repo-wide grep across `openpilot/`, `opendbc_repo/`, and `tools/fm/` (excluding `tools/fm/schema/fm-dev-nav-settle/` and historical docs): **0 leftover code, settings, UI entries, or dead references** for Nav Assist (`fm_navassistd`, `sunnypilot/fm_nav/`, `fmNavTurn`, `tools/fm/nav.py`), Download Nav Map, Map Turn Desire, Nav Turn Speed, Turn Boost, Road Edge Turn Desire, Lead Shadow Log, and Radar Badge.

### 8. Settings UI
- `compile_settings_ui.py` produces **0 diff** against committed `settings_ui.json`; `validate_settings_ui.py` passes **10/10** checks.
- Sunnylink Steering page layout matches `INVENTORY.md`: Lane Keeping (`FM_LaneKeepMode` under Lane Centering), FM Custom Torque Settings, Feedforward Settings, FM Smoothing Settings (Smooth Turn Start, Low-Speed Correction Strength, Calmer Crawl-Speed Steering, Calmer Straight Pull-Away), Model Lateral Smoothing (Small / Big with delay vs. smoothing-only notes), Turns (Turn Hold Settings, Turn Lead Settings, Pre/Post Turn Behavior, Finish Turn After Signal Cancels).
- Blendful removed from Sunnylink `models.yaml`, on-device `layouts/settings/models.py`, and mici `mici/layouts/models.py`; appears only in `FM_LaneKeepMode`.
- `AccelBoostSP` and `AccelerationControl` hidden from Sunnylink `cruise.yaml`, `layouts/settings/cruise.py`, and replaced by `FM_AccelBoostMode` on comma 4 `mici/layouts/settings/toggles.py`.
- AP's `screen_saver.py` kept over sunnypilot's.
- Minor notes: `[L-3]`, `[L-5]`.

---

## Cross-Feature Interaction Matrix

| Feature Pair | Pipeline Ordering | Risk / Conflict | Notes & Evidence |
|---|---|---|---|
| **Max Torque Headroom** × **Lateral Jerk / NNLC** | `LaC.update`: `FmTorqueMode.update` -> `self.update_limits()` -> `self.extension.update_limits()` | **Conflict (`[M-1]`)** | `LatControlTorqueExtBase.__init__` uses a dummy `_pid` until first active frame; startup blend finishes while `active=False`, leaving real `self.pid` in lateral-accel units (`4.22` vs `1.51`) indefinitely below 20 mph if `useParams` is False. Carcontroller & panda still cap CAN torque. |
| **Max Torque Headroom** × **Release Guard / Low-Speed FF / Low-Speed Correction Strength** | `LaC.update`: `FmLowSpeedGain` -> `FmTorqueMode` -> `FmLowSpeedFF` -> PID -> `FmReleaseGuard` | **Clean** | `FmReleaseGuard` and `FmLowSpeedFF` convert counts using `BASE_COUNTS = 385` (matching `carcontroller.py`'s `steer_max` scaling, since Headroom is unscaled); both stand down with Lateral Jerk/NNLC; `FmLowSpeedGain` scales `KP_INTERP` cleanly in both v0 and v1. |
| **Max Torque** × **AP High-Angle 95% Cap / Faster Ramp / `TorqueRtLimiter`** | `carcontroller.py`: `apply_driver_steer_torque_limits` -> optional 95% cap -> `rt_limiter.limit` | **Clean** | At `385`, exact stock AP (`max_rate_up=3`, 95% cap > 90°, `RIVIAN_STEERING_LIMITS`). Above `385`, car fast ramp (< 25 mph) sits strictly inside panda fast ramp (< 12.5 m/s ≈ 28 mph), and `TorqueRtLimiter` (`120` max delta vs panda `125`) caps 250 ms bursts even at 5 counts/frame (`5 × 25 = 125`). |
| **Turn Lead** × **Turn Hold** × **Smooth Turn Start** | `fm_turn_assist.py`: Carry -> Turn Lead -> Turn Hold (Driver Adds / Hold Path / Floor) -> Turn Nudge -> Smooth Turn Start | **Clean** | Turn Lead runs before Turn Hold so Turn Hold captures Turn Lead's output; Turn Nudge runs after Turn Hold so it never feeds peak/exit fade; Smooth Turn Start low-passes only the added delta (`out - desired_curvature`) on rise, drops immediately on release, and clamps `hold_exit_curvature` to applied value. |
| **Finish Turn After Signal Cancels (`carry`)** × **Blendful / AP Lane Centering** | `modeld_v2` (`Blendful`) -> `controlsd` (`lane_centering` -> `FmTurnAssist` -> `LaneGuard`) | **Conflict (`[M-2]`)** | When blinker cancels mid-turn and Turn Assist enters `carry`, `DH.desire` becomes `none` and blinkers become `False`. `Blendful` (active down to 0.3 m/s) and AP `Lane Centering` do not check `carry`, so they can inject lane-centering curvature while Turn Assist is carrying the turn. `LaneGuard` is less affected due to its 8 m/s minimum speed and 1 s `LANE_CHANGE_COOLDOWN`. |
| **Finish Turn After Signal Cancels (`carry`)** × **Settle After Turn** | `controlsd` (`fmControls.turnAssist.source`) -> `modeld_v2` (`carry_from_sm`, 3-frame `CARRY_GRACE`) | **Clean** | `TurnSettle._turn_sign` treats active carry as turn-in-progress and uses 3-frame grace (`0.15 s`) to absorb IPC latency and undo premature window opening if a carry arrives 1–3 model frames after `DH.desire` clears. |
| **Early Right Push (`FM_TurnSettleEarlyPush`)** × **Turn Hold** | `modeld_v2` (`TurnSettle` adds rightward curvature) -> `controlsd` (`Turn Hold` clamps toward turn) | **Conflict (`[M-3]`)** | Early Right Push starts while blinker/desire is still active once straightened past 45°. On a left turn with `FM_TurnHoldFollowExit` off (fresh default), Turn Hold overrides the rightward push back to the left hold floor; on a right turn, Early Right Push inflates `model_toward` seen by Turn Hold. |
| **Queue Guard** × **Finish Turn After Signal Cancels** | `modeld_v2` publishes `fmModel.queueGuard` **before** `modelDataV2SP` -> `controlsd` passes `queue_hold` to `FmTurnAssist._carry` | **Clean** | Publishing `fmModel` first prevents race; `_carry` treats `queue_hold=True` as a wait (`carry_done=True`, returns 0) so a Queue Guard hold can never start a carry. |
| **Queue Guard** × **Turn Pulse on Pull-Away** | `desire_helper.py`: `LaneTurnController` -> `FmQueueGuard` -> `FmTurnPulse` | **Clean** | While Queue Guard holds, `desire = none` resets `FmTurnPulse.stopped = False`; releasing Queue Guard emits a fresh rising-edge desire naturally; if the truck remains stopped after release, `FmTurnPulse` arms normally for the eventual pull-away. |
| **Calmer Straight Pull-Away** × **Turn Assist / Blinker** | `modeld_v2` (`StraightPullAway`: blinker/desire/0.3 s press blends back to usual floor over 0.5 s) | **Clean** | Any blinker or turn desire keeps/blends to `base_floor` (1.0 m/s unless Calmer Crawl-Speed is on); wheel angle is only checked at standstill (`< 0.1 m/s`) so model torque cannot re-trigger a floor jump while moving. |
| **Lane Keeping Selector (`FM_LaneKeepMode`)** × **AP Lane Centering / Lane Guard / Blendful** | Single enum (`0..3`) read by `lane_centering.py`, `fm_lateral.py`, `modeld_v2/modeld.py`, `ui_state.py`, `webrtcd.py` | **Clean** | Mutually exclusive by construction; `fm_lateral.py` also double-gates `guard_active = ... and not lane_centering_enabled`. |
| **Accel Boost Picker (5 modes)** × **Lead MPC / Cruise / CSC Interlock / MADS** | `longitudinal_planner.py`: single active mode in `FmAccelBoostMode` -> `min(mpc, cruise, e2e)` | **Clean** | `AccelBoostSP` and `AccelerationControl` toggles are never read; mode switch resets all 5 boost instances cleanly; AP Accel Control still receives `self.scc.governor.throttle_scale()` and no longer stacks with Gas Override Boost; Lead MPC & cruise ceiling still win via `min()`. MADS (`longActive` independent of `latActive`) is unaffected. |
| **Accel Boost Indicator (Boost Bar)** × **`UIStateSP` Cold Boot** | `UIStateSP.__init__` / async `_params_refresh_worker` -> `AugmentedRoadView` -> `AccelBoostIndicator` | **Crash (`[H-1]`)** | Missing `fm_accel_boost_indicator` / `fm_accel_boost_mode` initialization in `UIStateSP.__init__` causes `AttributeError` if `AugmentedRoadView` renders before background param thread completes. |
| **Smooth Lead Handoffs** × **Ghost Guard / Dual Confidence / Low-Speed Radar** | `radard.py`: preferred track -> radar match + ghost guard -> camera fallback -> low-speed override -> `FmLeadHandoff.update` | **Clean** | Low-speed override and ghost drops reset handoff state cleanly; any change in Lead settings resets `fm_handoffs`; all-default (`False / 50 / 50 / False`) matches stock openpilot. |
| **Jetlink Big Model** × **Blendful / Settle After Turn / Model Lateral Smoothing / Straight Pull-Away** | `fm_jetlink_action.py` (`with_blendful` + `action_speed_floor`) -> `modeld_v2/modeld.py` (`smooth_big_model_action`) | **Clean** | `with_blendful` forwards `action_speed_floor` to stock `get_action_from_model` (`LAT_SMOOTH_SECONDS = 0.0`), applies `lane_keep` (Blendful + TurnSettle, verified identical `model_output` keys/shapes between stock & `modeld_v2` parsers), then `modeld_v2` applies `big_model_smoothing` after swap. |

---

## Per-PR Verdict Table

| PR # | Feature | Status | Files / Commits Checked | Notes |
|---|---|---|---|---|
| Step 0 | Base merge (AP `cf3ad0ec` + SP `c410198c`, opendbc re-sync) | **Matches** | `4efb05a` (67 files, `opendbc_repo/` 19 files, `mads.h`, `gps_fuser.py`, `hardwared.py`, `screen_saver.py`) | Zero AP Rivian files lost; GPS fuser #2112 clean; AP `screen_saver.py` wins; SP #2109 kept. |
| PR 1 | Docs and `tools/fm` | **Matches** | `fba53fc` (`FM_PILOT.md`, `FM_PILOT_HANDOFF.md`, `FM_FOLDED_DEFAULTS.md`, `tools/fm/` incl. `turns.py` `d189ccc` path timing) | `nav.py` dropped; `FM_FOLDED_DEFAULTS.md` identical at root and `docs/fm-dev-sync/`; `tools/fm` pytest 87/87, ruff clean. |
| PR 2 | Log schema (CustomReserved 14–19, `tools/fm` dual schema) | **Matches** (minor `[L-4]`) | `f887f49` (`custom.capnp`, `log.capnp`, `services.py`, `tools/fm/fmlog.py`, `test_schema.py`) | Fresh FM structs at `@140`–`@145` preserving struct IDs; zero fields on `*SP` structs; `tools/fm` reads both schemas. Minor: `NEW_FM_MSGS` omits `fmRadarLead`/`fmJetlinkStatus`. |
| PR 3 | Settings framework, migration runner, quiet road-edge, Blendful hidden | **Matches** | `aa95387` (`fm_migration.py`, `params_migration.py`, `selfdrived.py`, `events.py`, `models.yaml`, `steering.yaml`, `models.py`) | `fmLaneChangeRoadEdgeQuiet @38` after AP `@35-37`; Blendful removed from Models pages; migration runs before default-param population. |
| PR 4 | FM Custom Torque Settings, steerLimitTimer 0.8, off-tracking fix | **Problems** (`[M-1]`, `[L-1]`, `[L-2]`) | `44d8a07` (`rivian.h`, `carcontroller.py`, `values.py`, `interface.py`, `sunnypilot/car/interfaces.py`, `fm_torque_mode.py`, `latcontrol_torque*.py`, `rivian_offtrack.py`, `test_rivian.py`) | Panda safety, MISRA, envelope math (0/52,013 violations), 385=stock AP, `steerLimitTimer=0.8`, off-tracking fix, and migration are solid. Confirmed `[M-1]` PID limits stay in accel units with Lateral Jerk/NNLC after disengaged startup; minor `[L-1]` (`inf`) and `[L-2]` (`NaN`). |
| PR 5 | Release Guard (v0 + v1) | **Matches** (minor `[L-3]`) | `1653723` (`fm_release_guard.py`, `latcontrol_torque*.py`, `controlsd_ext.py`, `steering.yaml`, `fm_migration.py`) | Exact sign conventions, only reduces opposing P, stands down with Lateral Jerk/NNLC, clean migration. Minor `[L-3]`: Sunnylink lacks enablement grey-out for Lateral Jerk/NNLC. |
| PR 6 | Feedforward + Low-Speed Correction Strength (v0 + v1) | **Matches** (minor `[L-5]`) | `60c057b` (`fm_low_speed_ff.py`, `fm_low_speed_gain.py`, `latcontrol_torque*.py`, `controlsd_ext.py`, `steering.yaml`, `fm_migration.py`) | Matches `INVENTORY.md`, `FM_FOLDED_DEFAULTS.md`, `FM_PILOT.md`, and `ca08aac9`; migrations exact. (`BUILD_PLAN.md:57` loosely said "stand down" for both; `INVENTORY.md` & `ca08aac9` only stand down FF.) |
| PR 7 | Lane Keeping selector, Lane Guard, Blendful 0.3 m/s + blue line | **Deviations** (`[M-2]`, `[L-5]`) | `99a2b44` (`fm_lane_keep_mode.py`, `fm_lateral.py`, `lane_guard.py`, `lane_centering.py`, `blendful.py`, `model_renderer.py`, `ui_state.py`, `webrtcd.py`, `steering.yaml`) | `lane_guard.py` byte-identical to `ca08aac9`; Blendful `MIN_SPEED=0.3`, blue line only, gated on `FM_LaneKeepMode==3`. Cross-feature `[M-2]`: Blendful/Lane Centering don't defer to PR 12's Turn Assist carry. Minor `[L-5]`: YAML widget title says "Lane Centering". |
| PR 8 | Model Lateral Smoothing (Small / Big) | **Matches** | `31be347` (`fm_lateral_smoothing.py`, `selfdrive/modeld/modeld.py`, `modeld_v2/modeld.py`, `steering.yaml`, `fm_migration.py`) | Small affects filter + delay, Big affects filter only (incl. Chestnut & Jetlink); 0.20 s -> 0.10 s migration exact; descriptions clearly explain delay vs smoothing-only. |
| PR 9 | Calmer Crawl-Speed Steering + Calmer Straight Pull-Away | **Matches** | `ce45f40` (`fm_pull_away.py`, both `modeld.py`, `jetlink_adapter`, `steering.yaml`) | Matches `ca08aac9` on AP's 1.0 floor, defaults off, existing keys preserved, clean 0.5 s blend back on turn/press. |
| PR 10 | Turn Hold, Smooth Turn Start, Turn Assist logging | **Matches** | `19574d7` (`fm_turn_assist.py`, `fm_lateral.py`, `controlsd*.py`, `steering.yaml`, `pr10_turn_hold_equivalence.py`) | All 6 folded Turn Hold options always on, old keys unread; PR 10 equivalence script verified identical across 2.56M random + 704 constructed steps. |
| PR 11 | Turn Lead | **Matches** | `b1b1800` (`fm_turn_assist.py`, `fm_lateral.py`, `steering.yaml`, `test_fm_turn_assist.py`) | Turn Lead runs before Turn Hold; all settings & defaults match plan (`Adaptive`, `Wait for the Corner` default off on fresh install, stored values carry); unit tests pass. *(Monte Carlo `pr11_turn_lead_equivalence.py` cancelled to prioritize pytest.)* |
| PR 12 | Finish Turn After Signal Cancels | **Deviations** (`[M-2]`) | `48c343f` (`fm_turn_assist.py`, `steering.yaml`, `test_fm_turn_assist.py`) | Carry Only at Turn-In folded; default off on fresh install; unit tests pass; cross-feature `[M-2]` with Blendful / Lane Centering. *(Monte Carlo `pr12_turn_carry_equivalence.py` cancelled to prioritize pytest.)* |
| PR 13 | Pre/Post Turn Behavior (Queue Guard, Turn Pulse, Settle After Turn + Early Right Push) | **Deviations** (`[M-3]`) | `d3406ff` (`fm_queue_guard.py`, `fm_turn_pulse.py`, `fm_turn_settle.py`, `desire_helper.py`, both `modeld.py`, `fm_lateral.py`, `steering.yaml`, `test_fm_queue_guard.py`, `test_fm_turn_pulse.py`, `test_fm_turn_settle*.py`) | Queue Guard + Turn Pulse logic matches `ca08aac9`; `fmModel` published before `modelDataV2SP`; Settle handles carry with 3-frame grace; all unit tests pass. Cross-feature `[M-3]` between Early Right Push and Turn Hold while blinker stays on. *(Monte Carlo `pr13_pre_post_turn_equivalence.py` cancelled to prioritize pytest.)* |
| PR 14 | Accel Boost picker (5 modes) + Boost bar | **Problems** (`[H-1]`) | `e9ba455` (`fm_accel_boost_mode.py`, `set_speed_assist.py`, `fm_op_personality_boost.py`, `longitudinal_planner.py`, `plannerd.py`, `accel_boost_indicator.py`, `ui_state.py`, `cruise.yaml`, `toggles.py`, `cruise.py`, `fm_migration.py`) | Planner logic, OP Personality (#39430/#39440: 0/0.05/0.15, 5->10 m/s, -1.0->-0.5 m/s²), EVP Set Speed, single-boost isolation, Boost bar scaling, and migration all match spec. Confirmed `[H-1]`: missing `UIStateSP.__init__` attributes crash `ui` on cold-boot/onroad restart. |
| PR 15 | Lead (confidence, stationary ghosts, Smooth Lead Handoffs, `fmRadarLead`) | **Matches** | `998d062` (`radard.py`, `fm_radar.py`, `fm_lead_handoff.py`, `cruise.yaml`, `tools/fm/lead.py`) | Matches `ca08aac9` minus dropped Lead Shadow Log & old shared key; defaults off/50/50/off; `tools/fm/lead.py` reads old & new schemas and replays cleanly. |
| PR 16 | Top Readout (Torque / CPU / Lead) + torque log fields | **Matches** | `97121bc` (`fm_torque_readout.py`, `hud_renderer.py`, `card.py`, `carcontroller.py`, `ui_state.py`, `visuals.yaml`) | Default Off (safe fallback on invalid values); `fmCarState` published alongside `carOutput`; `UIStateSP.__init__` properly initializes `fm_top_readout`. |
| PR 17 | Jetlink extras (API 4 / v0.9.0, `fmJetlinkStatus`, icon states) | **Matches** | `8e737bb` (`jetlink_adapter/__init__.py`, `fm_jetlink_action.py`, `fm_jetlink/`, `fm_jetlink_icon.py`, `ui_state.py`, `hud_renderer.py`, `home.py`, `process_config.py`) | Clean Blendful/Settle restoration on Jetlink big model (`with_blendful`); defensive read-only `fm_jetlinkstatusd` on small cores; icon states match spec. |
| PR 18 | `hardwared` stall diagnostics | **Matches** | `367467c` (`loop_timer.py`, `hardwared.py`, `common/hardware/base.py`, tests) | `loop_timer.py` byte-identical to `ca08aac9`; preserves sunnypilot #2109 params-lock fix; watchdog safely catches all exceptions. |

---

## Section 9: Tests, Linting & Docs Verification

### Build & Static Checks
- **`scons -j2 -k`:** All native C++/Cython/capnp/rednose/panda STM32H7 firmware (`-Wall -Wextra -Wstrict-prototypes -Werror`)/`libpanda`/`libsafety` targets built cleanly; **only** the 2 expected LFS ONNX model compiles failed (`dmonitoring_model_tinygrad.pkl`, `driving_tinygrad.pkl`).
- **MISRA (`opendbc_repo/opendbc/safety/tests/misra/test_misra.sh`):** **PASS** (cppcheck 2.21.0, 268 active checkers, 0 errors/warnings).
- **Sunnylink UI compile + validator:** `compile_settings_ui.py` produced **0 diff** against committed `settings_ui.json`; `validate_settings_ui.py` **10/10 PASS**.
- **`ruff check .`:** **7 findings**, all in pre-existing AP files untouched by FM Pilot (`opendbc/car/rivian/tests/test_route_replay.py`, `ui/sunnypilot/layouts/settings/cruise.py`, `jetlink_adapter/tests/conftest.py`, `system/manager/process.py`). **0 ruff findings in FM Pilot code.**
- **`ruff check --config tools/fm/ruff.toml tools/fm`:** **PASS** (0 findings).

### Test Suites (Verifying the "2 Known Base Failures" Claim)
- **`openpilot/sunnypilot`:** **1,720 passed, 47 skipped, 1 failed** (after clearing ambient `LateralJerkTorqueController=True`, which exposed `[L-6]` in `test_fm_low_speed_ff.py` and `test_fm_release_guard.py`; isolated re-run of those 48 tests passed 48/48 in 1.35 s).
  - The 1 failure is `openpilot/sunnypilot/models/tests/test_default_model.py::TestDefaultModel::test_compare_onnx_hashes` (**confirmed known LFS base failure**).
- **`openpilot/selfdrive/controls`, `selfdrive/car/tests/test_fm_car_state.py`, `selfdrive/modeld`, `selfdrive/ui/sunnypilot`, `system/hardware/tests`, `common/hardware/tests`, `system/manager`, `opendbc/car/rivian`, `opendbc/sunnypilot/car/rivian`, `opendbc/safety/tests/test_rivian.py`:** **932 passed, 174 skipped, 489 subtests passed, 1 failed**.
  - The 1 failure is `openpilot/system/manager/test/test_manager.py::TestManager::test_set_params_with_default_value` (`ScreenSaverCustomText` — **confirmed 2nd known base failure**).
- **`openpilot/selfdrive/selfdrived`:** **12 passed, 0 failed**.
- **`tools/fm/tests`:** **87 passed, 0 failed**.
- **PR 10 equivalence (`pr10_turn_hold_equivalence.py` vs `ca08aac9`):** **PASS** — 3,000 random sequences (2,564,993 steps) + 704 constructed sequences 100% identical; 0 folded keys read.
- **Claim verified:** Across **2,751 passing tests** (+ 500 subtests), **only the 2 documented base failures occurred** (plus ambient-param test-isolation flake `[L-6]`).

### Test Coverage & Credits
- **Coverage:** ~7,484 lines of dedicated FM Pilot tests covering every feature PR (PRs 1–18), Rivian panda safety limits (`test_rivian.py` + `test_fm_max_torque.py`), and all 7 settings migrations on real `Params` (`test_fm_migration.py`). Gaps noted in `[H-1]` (`raising=False` hid missing `UIStateSP` attrs) and `[M-1]` (`test_fm_headroom_limits.py` did not test disengaged startup or `CS.vEgoRaw`).
- **Credits in `FM_PILOT.md` (lines 135–188):** All required upstream credits present and accurate — AdventurePilot (AP), sunnypilot, openpilot, StarPilot, ZoomPilot (`zoompilot`), EVPilot, gerrylum (plus FrogPilot).
- **Doc-to-code accuracy:** High overall; `FM_FOLDED_DEFAULTS.md` at repo root and `docs/fm-dev-sync/FM_FOLDED_DEFAULTS.md` are byte-identical. Minor items in `[L-5]`.

---

## Explicit GO / NO-GO Verdict

- **Driving Safety / Vehicle Control:** **GO** — **0 Critical findings.** Rivian panda safety (`rivian.h`), MISRA compliance, 385–580 step-15 envelope enforcement (0/52,013 car-vs-panda violations), stock-AP equivalence at 385, unchanged driver monitoring, `steerLimitTimer = 0.8`, off-tracking near-zero crash fix, longitudinal single-boost isolation, and rollback-safe settings migration all check out cleanly.
- **Branch Release Verdict:** **NO-GO until `[H-1]` is fixed** (a trivial 2-line fix in `openpilot/selfdrive/ui/sunnypilot/ui_state.py` initializing `self.fm_accel_boost_indicator = False` and `self.fm_accel_boost_mode = MODE_OFF` in `UIStateSP.__init__`). While `[H-1]` cannot cause unsafe actuation, it is a confirmed `AttributeError` crash in the `ui` process if `ui` starts or restarts onroad before the background params thread finishes its first pass.
- **Recommended follow-ups (non-blocking unless you run these feature combinations):**
  1. Fix `[M-1]` (2-line fix binding `LatControlTorqueExtBase._pid = lac_torque.pid`) before driving with **Max Torque > 385 + Lateral Jerk or NNLC**.
  2. Fix `[M-2]` (gate `Blendful` and AP `Lane Centering` on Turn Assist carry) before combining **Blendful (0.3 m/s) + Finish Turn After Signal Cancels**.
  3. Fix `[M-3]` (coordinate `Turn Hold` with `fmModel.turnSettlePhase`) before combining **Turn Hold + Settle After Turn: Early Right Push**.

