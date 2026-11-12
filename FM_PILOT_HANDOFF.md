# FM Pilot: handoff for new chats (short)

State as of Nov 12, 2026 (early; Release Guard added). This is the current state only. Background findings, past drive analyses and
the ideas backlog are in `claude/FM_PILOT_FINDINGS.md`: read it only when a task needs that history. Features, settings
and credits are in `FM_PILOT.md`; log tools in `tools/fm/README.md`.

## Setup

- **Truck:** 2022 Rivian R1S Launch Edition (Gen 1), comma 4, torque harness.
- **Repo:** `pmalekian/FMPilot`. Active branch **`fm-dev-set`** (v25 model list and tinygrad 92b742f since Nov 8), cut from `fm-dev-torque` `bece23e`
  (Higher Torque + radar settings) and adding the settings reorganization, Blendful and the boost bar; `fm-dev-torque`
  stays at `bece23e` as the fallback. `fm-dev-max` (`d189ccc`) is the everyday branch, installer `pmalekian/fm-dev-max`. Own fork of AdventurePilot `dev` (last synced `980dd5c`, Oct 31).
  **No more AP syncs**; borrow single pieces only if wanted.
- **Code work:** attach the repo (push access), clone the branch. Commit as
  `Frankie Malekian <107580415+pmalekian@users.noreply.github.com>`, messages start "FM Pilot", one change per commit.
  Push with `--no-verify` (LFS hook 403s). Tests run off-device with a shim stubbing `Params` from `params_keys.h` (and
  `msgq` names, for `cereal.messaging`), with `opendbc_repo` on the path; deps: numpy pytest pycapnp zstandard
  setproctitle psutil pyzmq pycryptodome tqdm, plus `tinygrad` (pip) for modeld_v2 tests and `raylib` + `xvfb-run` for
  UI tests. Known off-device failures: `test_latcontrol_torque_ext.py` (NNLC data submodule not cloned).
  After YAML edits recompile `openpilot/sunnypilot/sunnylink/tools/compile_settings_ui.py` (and run
  `validate_settings_ui.py`).
- **Frankie's side:** Windows PowerShell. Comma at `192.168.1.246`, user `comma`. Always `scp -O`. Say which prompt each
  command runs at (`PS C:\...>` or after `ssh comma@192.168.1.246`). He can also download single rlogs from Comma
  Connect on his phone and attach them (rename each to its segment, e.g. `<route>--<n>--rlog.zst`). The PC can be
  linked to the chat; the bridge can read `C:\Users\frank\fm-logs` (the full Nov 3 drive `0000002d`, 64 segments) and
  `C:\fmlogs` once granted (`C:\fmlogs\2026-11-05` folders are empty), but the PC shell VM doesn't start
  (virtualization off), so the chat can't pull from the comma itself.
- **Logs:** one-minute segments, `<route>--<n>/rlog.zst`. Model name only logged in `--0` (tools infer it otherwise).
  `tools/fm` joins segments; tests `cd tools/fm && python -m pytest tests`. In a cloud chat run from `tools/fm` (its
  `fmlog` import puts the repo's `openpilot` package on the path). rlogs include `radarTracks` (raw radar points), so
  radard can be replayed offline. EPAS signals: decode `EPAS_SystemStatus` (0x380, bus 0) with cantools and the DBC.
- **Signs:** curvature and lateral accel positive **right**; angle, hand torque and torque commands (incl.
  `carOutput.actuatorsOutput.torqueOutputCan`, same sign as `actuators.torque`) positive **left**. Hand torque reads
  *against* openpilot from wheel reaction when loose.
- **Real torque sent:** `carOutput.actuatorsOutput.torqueOutputCan` (CAN units, cap 385 / 481). Learned torque values
  are in `lateralTorqueParameters` (not `liveTorqueParameters`).
- **Torque controller:** with `EnforceTorqueControl` on and `TorqueControlTune` 0 (his settings), or Enforce off,
  sunnypilot runs **v0** (`latcontrol_torque_v0.py`), not v1. `torqueState.version` in the log says which.
  `torqueState.saturated` is always false below 10 m/s.

## What he drives now

- **TSFDOM**, Experimental Mode, **Blendful** as his daily lane keeping (`FM_LaneKeepMode` 3, in the Nov 11 logs; said
  Nov 11), Lane Guard off, off-tracking off, Camera Offset 0.05 m
  (since Nov 4, not yet analyzed). Aggressive personality, following time offset -20% (Nov 3 backup).
- Road Edge Turn Desire on (max 35 mph in the Nov 5 logs), quiet road-edge block on.
- Accel Boost: **Predictive** (mode 2) on the Nov 5 drives (up to +0.4 m/s² on aggressive).
- Turn Assist on Nov 5: Turn Hold + Hold Path Source + Hold Adds Your Steering on; Lead and Boost off; radius 8 m.
- Nov 5 drives were on `fm-dev-torque` `03b6805` (before the fix), panda setting on: `00000038` Headroom, `00000039`
  (morning commute) Full. **Both actually ran at 385** (see below).
- Custom splice model: **removed** Nov 8 (with the on-device compile fix and model check tools), for the v25 move.

## Active work

### 00. v25 model list + tinygrad 92b742f on `fm-dev-set` (Nov 8): built, not yet on the device

- Why: new sunnypilot models (e.g. **Cheddar**, openpilot PR #39255) are only in sunnypilot's model list v25+, which
  needs selector version 21, whole-file downloads and the plain GPU memory model format on tinygrad 92b742f.
- Ported from sunnypilot master-dev, one commit each: #2050, #2053, #2062, #2051, #2071, #2052, #2072, #2093.
  `selfdrive/modeld` (SConscript, helpers, stock modeld, dmonitoringmodeld loading) moved to sunnypilot's, because the
  new tinygrad dropped `--device-input` and changed Buffer/UOp APIs. **DM model file unchanged** (FM's Aug model; sunnypilot
  has openpilot's newer Sep "Super Leicht", not taken). FM's own: Blendful hook kept in modeld_v2; load_oob rejoins
  chunked files (FM prebuilds still chunk >95 MB). Not taken: #2092 (CI), .github changes.
- Also new from upstream: modeld_v2 and stock modeld seed the previous curvature from `controlsState` while lateral is
  off (openpilot #39091), chestnut retry; `modeld_warmup` runs offroad. Because of #39091, `tools/fm` now reads the raw
  `modelDataV2SP.modelCurvature` as "model asked" while lateral is off (`fmlog.model_requests`).
- **Calmer Crawl-Speed Steering** (`FM_LowSpeedCurvatureFloor`, Steering > FM Pilot: Turn Tweaks, **off**): sunnypilot
  #2091 as a setting. Speed floor 2.5 m/s instead of 1.0 when turning the action into curvature: below 5.6 mph the
  request drops (64% at 4.5 mph, 16% at 2.2 mph). Calms crawl twitch; may soften turns from a stop. Try as its own drive
  on the same corners, with and without.
- Off-device tests: modeld_v2 121 pass, models 28 pass (onnx hash test fails only because LFS isn't pulled); no new
  failures vs `60ddd2a` across sunnypilot/modeld/ui/controls tests.
- **First boot:** build takes longer (DM and stock model recompile). The saved v23 selection (selector 20) no longer
  parses, so modeld most likely runs the default model until TSFDOM (same ref) is selected again and re-downloads
  (from the code, not yet seen on the device); leave it offroad for the warmup. **Check before driving:** DM works (face seen, no DM faults), model label/picker
  shows the selected model, no modeld errors. Then try Cheddar as its own drive.

### 0i. Following feels harsh in traffic (Nov 12 logs `00000070--11/12/17`); Set Speed grace fade built, not driven

- **What he felt:** slowing for traffic is harsher than before, "like lifting off too fast with regen", with Predictive and
  Set Speed both. Morning was colder.
- **Findings:** the boosts don't drive these slowdowns (the lead MPC is the planner's min; --17's swings happened in Gas
  mode with boost 0). openpilot's requests are smooth (fast wiggle 0.01–0.02 m/s²). The 55–59 mph swings in --17 track a
  lead wandering 52–59 mph (~1.5 s lag, corr 0.73) at a 1.35 s gap (Aggressive, -20%).
- **Truck over-braking vs the sent request** (incl. drag FF), same request levels: Nov 3 6 pm (`0000002d`, 42 min)
  -0.26..-0.27 when braking, -0.19..-0.20 cruising; Nov 12 8:22 am -0.34..-0.39 braking (one stop, ~8 s, thin),
  -0.21..-0.23 cruising. So ~0.1 more when braking on the cold morning. Regen setting: Standard on every drive (he said).
  The long PID (ki 0.2, no kp) closes a steady gap in ~5–10 s and resets each engage: too slow for transitions.
- **Built (`fm-dev-jet`):** Set Speed grace fade. On --11 an engage at 35 mph with the model at -0.5..-0.75 held the
  target ~0 for 3 s, then stepped -0.03 → -0.76 in one frame. Now when the window runs out with the model still braking,
  the boost is trimmed to the part above min(lead MPC, cruise) and fades at RAMP_DOWN 1.5 m/s³ (~0.6 s here). A stop,
  braking below the floor, low speed, a turn or a disengage still hand back at once. Default in Set Speed mode.
- **Candidate (discuss first):** a braking-only drag correction, ~0.1–0.15 m/s², off by default, if more cold mornings
  show the same. Changes the request, not panda limits.
- **Test:** Set Speed, engage while slowing behind a lead; `fmAccelBoost` should fall over ~0.5 s, not in one frame.

### 0h. Release Guard on `fm-dev-jet` (Nov 12): built, OFF by default, not driven

- **Problem (his):** in a slow turn, holding the wheel tighter than openpilot wants makes the torque controller push back;
  letting go, that torque is still being sent and swings the wheel past center. It is the **P term**: v0's integrator is
  frozen while pressing or under 5 m/s, and KP is ~30 at 7 mph, so a 0.01 1/m gap already saturates it. The car's hand rule
  (`apply_driver_steer_torque_limits`: -200 counts per Nm above 1 Nm) only trims what is sent, never the request.
- **Logs:** `00000067--4e0a74a75c--0` +32.8-34.9 s (Rebel Legion, left, 8-11 mph, ~2 Nm into it, 30% tighter than
  asked, Turn Hold off): up to 485 against him, let go at +34.9 s with 433 out of the turn, 396 at center, wheel to -39 deg
  (about 75 deg past the model's target), push past center 0.78-0.80 s, he caught it at +35.1 s. `00000068--29f57c5eb1--1`
  +0-1.7 s (TSFDOM, right, 12-15 mph): same, 306 at center, 0.64 s, wheel +12 deg; **Turn Hold + Hold Adds Your Steering
  were on** and turn desire active until +1.05 s, but the request stayed below his curve: the hold had most likely
  released at its 75 deg limit (guess; hold state isn't logged). So Hold Adds doesn't cover turn exits.
  `00000061--10` (Cheddar, hands off, -580 -> +580 flip) is a different problem: no hands, the guard never acts; only
  Calmer Correction or a faster ramp-down (panda, deferred) touch it.
- **Built (`e2aa8b3`):** `fm_release_guard.py`, hooked after `pid.update` in v0 and v1. While he presses into the curve
  the truck is already turning (> 0.003 1/m, hand torque toward it), under 15 mph and gone by 20, `pid.p`'s push against
  him is limited to **Most Torque Against You** (`FM_ReleaseGuardMax` 150 default or 75 counts, converted like the
  feedforward via latAccelFactor / fm_scale) and the clipped output recomputed. After release it fades over 0.5 s and ends
  at once when P flips (wheel past openpilot's target) or his hand reverses. Feedforward, friction, integrator and all
  limits after the controller unchanged. Off by default (`FM_ReleaseGuard`), Steering > Torque Control, live (read every
  1 s, on/off blends 0.5 s), stands down with Lateral Jerk / NNLC. A percentage of P was rejected: P runs 10+ here, so
  even 25% left ~300 counts.
- **Logging (`fd3f1f0`):** `carControlSP.releaseGuard` (active, amount, cut, limit). **`turns.py` (`4497333`):** release
  line per turn with a press into it of 0.3 s or more (press time, against-you max/avg, at release, at center, deg past
  center, push past center, guard share/cut).
- **Replay** (real module on logged P / I / FF, then the car's hand rule and ramp, seeded from the log; matches logged sent
  within median 5-14 counts):

  | | Rebel Legion: held avg / at release / at center / past center | TSFDOM 68--1: same |
  |---|---|---|
  | as driven | 197 / 445 / 400 / 0.80 s | 380 / 391 / 306 / 0.64 s |
  | guard 150 | 58 / 125 / 80 / 0.16 s | 172 / 114 / 29 / 0.06 s |
  | guard 75 | 14 / 55 / 10 / 0.02 s | 153 / 39 / -39 / 0 |
  | Calmer 70% alone | 201 / 398 / 353 / 0.71 s | 373 / 328 / 243 / 0.50 s |
  | Calmer 70% + guard 150 | 59 / 125 / 80 / 0.16 s | 172 / 113 / 28 / 0.06 s |

  68--1's press began in `--0` with ~365 already built, so the guard could only bring it down at the ramp (held avg falls
  less). Open loop: shows torque, not where the wheel would have stopped. With the guard on, Calmer changes nothing here
  (the limit is absolute).
- Tests: `test_fm_release_guard.py` 20 pass (incl. both controllers, every selectable controller has the hook, logging);
  tools 44 pass; Sunnylink validator 10/10. Off-device suite: no new failures vs `06dbd26` (same 190 shim failures).
- **Test:** after 0g has been driven, its own drive on the same corners: on at 150, hold the wheel into a few slow turns
  and let go as usual. `turns.py` release line vs `00000067--0`. Try 75 if it still swings past center; if turn-in feels
  like openpilot stopped helping while you steer, that's expected (it only limits push *against* you).

### 0g. Request-side turn fixes on `fm-dev-jet` (Nov 12): built, ON by default, not driven

Frankie's choice after the faster-ramp discussion (below): fix the request first, no safety-limit changes. All five
built, all on by default (his call), to be tested together. One commit each:
- `b7b2d77` **Smooth Turn Start** (`FM_TurnPhaseIn`): Turn Assist's extra request (Boost / Lead / Hold / Nudge) rises
  through a 0.12 s low-pass (~90% in 0.3 s), drops at once; model request never filtered.
- `6bbdcf5` **Turn Lead Adaptive Look-Ahead** (`FM_TurnLeadAdaptive`): 0.5 s (gentle) to 0.9 s (tight) instead of 1.3 s,
  min 3 m; tightness from `plan_max_curvature` (tightest bend toward the turn in the next 17 m).
- `f61de04` **Turn Lead Start Distance gate** (`FM_TurnLeadGate`, `FM_TurnLeadGateDist` 3–10 m, default 5): no lead until
  the plan's 10°-toward-the-turn point is within that distance (fade over 2 m).
- `7573f96` **Calmer Low-Speed Correction** (`FM_LowSpeedGainEnabled`, `FM_LowSpeedGainPercent` 40–100, default 70):
  v0 and v1 `KP_INTERP` scaled below 7 m/s, normal by 9 m/s (`fm_low_speed_gain.py`), Rivian only.
- `0059e09` **`turns.py` timing line**: model / openpilot / truck onsets at 30% of the turn's peak curve, verdict model
  late (< 0.3 s) / wheel late (> 0.8 s) / on time. Nov 11: `00000068--1` +24.7 s L model +1.1 s, openpilot -0.5 s
  (model late, Turn Lead did the work); `00000061--10` +12.2 s R model -0.4 s (on time).
- **Replay** of `00000068--1` through Turn Assist (old settings reproduce the logged request, median diff 0): with the new
  settings Turn Lead's early 0.006–0.021 at +23.9–24.2 s drops to 0.001–0.005 (corner 5–7 m out) and the +24.4 s step
  (0.052 at once) arrives over ~0.2 s; the peak is unchanged. Calmer Correction can't be replayed (closed loop); open
  loop it scales P by 0.7 (the +25.4 s reversal would be ~30% smaller, a guess).
- Tests: 564 pass off-device (was 516), no new failures; Sunnylink compiled and validated.
- **Low-Speed Steering Feedforward** (Nov 12, `FM_LowSpeedFFEnabled` **off** by default, `FM_LowSpeedFFStepLeft` and
  `...StepRight`, each Off / Gentle 40 (default) / Medium 60 / Firm 80 / Full 100): torque for the asked wheel angle (vehicle model), full below 4.5 m/s, gone by 7 m/s,
  both v0 and v1 (`fm_low_speed_ff.py`). Fitted on 18 Nov 11 segments (`00000061` --10/14/15, `00000068` --1/2/5,
  `00000069` --3/4/7/9/10/11/12, `0000006a` --10/11/12/14/15; 3300 openpilot-steered hands-off frames following the
  request): below 10 mph existing feedforward ~0 (-4..+5 counts), correction gave 50-70 at 15-45 deg, truck 15-19% wide;
  10-20 mph existing feedforward 75-85% of torque, truck 2-8% wide. Table counts at Full [0,10,30,65,120,180 deg] ->
  [0,20,70,95,105,110] (`e783540`, refit in review: 00000061's Full-mode x1.5 had inflated the tight-turn gap to 79,
  really 54): Full never exceeds the measured gap in any bin. Tight crawl turns had no clean frames (always
  ramp-limited or hands on): estimated from 10-16 mph. Left/right checked by speed: no reliable side difference (small-angle split
  reverses with speed; tight turns 5 left vs 8 right, within the spread); table symmetric, per-side strength added as
  a tuning knob only (both Gentle). Stands down with Lateral Jerk or NNLC (`7a35e09`). Over 266 s of
  openpilot steering under 16 mph, Gentle adds median 18 / max 44 counts and changes <= 2 counts per step (99th pct)
  vs the P term's 140, so it adds no twitch. Test: turn it on at Gentle with 0g, step up one level per drive if turns still run wide.
- **Watch for:** turn-in feeling softer or later (Calmer Correction 70% builds less torque early; try 80–90% if so);
  Turn Lead starting too late at a tight corner (raise Start Distance to 6–7 m, or Adaptive off); wheel still grabbing
  then swinging (lower Strength to 60%). Note his drives have Turn Lead **and** Turn Hold on.
- Test: Turn Lead on (min 2 mph), known corners incl. the `0000003a--83` curb ones; run `turns.py` (timing and torque
  lines) and compare with `00000068--1`. Then a drive with Turn Lead off to sort model-late vs wheel-late corners.

### 0f. Torque: the ramp, not the cap, holds slow turns back (Nov 11 logs); `turns.py` torque line (`7b61af6`)

- **Logs:** `00000061--142cb659d9--10/14/15` (Cheddar, build `2eaf5852`, Max Torque 580, Faster Ramp on, Full Torque Past
  90° on, Turn Lead/Hold/Boost off) and `00000067--4e0a74a75c--0` (Rebel Legion, `72bb922`, Max Torque 580, Faster Ramp
  on, Full Torque Past 90° **off**, Turn Pulse on). Both had Lane Turn Desire on.
- **With 580 the cap is rarely the limit.** `00000061`: torque sat at 580 only in the `--10` +12.9 s right turn (1.3 s, 14
  mph, hands off) and about 0.5 s in `--15` +45-46.5 s. `00000067`: never; peak 511, limit 551 past 90° (95% of 580).
  Torque climbs at 5 per message (about 490 per second, ~1.2 s to 580) and the request often eased off first.
  Unwinding is slow the same way: when the request flips, sent torque decays at 5 per message (`STEER_DELTA_DOWN`)
  (`--10` +14.2 to 15.3 s: asked +580 while sent was still -450 to -35). The Nov 4 "at the cap" finding was at 385.
- **`turns.py` fix:** the old "torque at limit" column was the controller request at 95% of the base 385 curve, which
  read 61-83% of slow turns. Each turn now has a second line from the logged limit (`carStateSP.fmSteerCap`) and sent
  torque (`torqueOutputCan`): peak sent vs limit, and shares at limit / climbing / held / unwinding. Older logs fall back
  to the speed curve and say so. Details in `tools/fm/README.md`.
- **Software delay:** `LagdToggle` off, `LagdToggleDelay` 0.10 + Rivian `steerActuatorDelay` 0.15 = 0.25 s; the learned
  delay in these logs is 0.242 s (calibrated, std at most 0.006). Not a cause of late or wide turns.
- **Cheddar** under-turns and goes wide, in his drives. Both Cheddar and TSFDOM are generation 12, 20 Hz, small models in
  the sunnypilot list; the only listed difference is lat smoothing (.0 vs .1), which can't make a model ask for less.
  Treated as a model issue. Its comma PR (#39255) is still open as a development model.
- **Lat/long smoothing (explained in chat, nothing built):** sunnypilot's list sets them per model (OPM16D .1/.3, Rebel
  Legion .1/.3, Rebellious Hope .1/.3, TSFDOM .1/.3, Cheddar .0/.3; chestnut v28 is all .0); comma's own `modeld.py` had
  0.0/0.3 at OPM16D and TSFDOM, 0.1/0.1 at Rebel Legion. The value filters the model's request and is also added to the
  delay the model plans for (reads further ahead). Long only affects speed, never steering.
- **Faster ramp discussed Nov 11, deferred (no safety changes):** the car code's `TorqueRtLimiter` (120 over the lowest of
  the last 30 messages) already holds "5 per message" to a 24-up / 6-flat staircase, avg 4.1 (`00000068--1` +24.0 s),
  0 to 580 in ~1.4 s. The panda's 250 ms check (125, reference every 26 messages at 100 Hz) means a faster climb needs
  `max_rate_up` + `max_rt_delta` + car `TORQUE_RT_MAX_DELTA` raised together (e.g. 6/190/180 → 0→580 0.96 s; 8/250/240 →
  0.72 s, zero refusals in jitter sims); unwinding only needs `max_rate_down` + `STEER_DELTA_DOWN` (−450→0: 0.89 s at 5,
  0.56 s at 8). At 5 per message both logs had 0 ToiFlt / reduced / fault bits, ToiActive 100%, 0 panda refusals. Held
  off because the request swings full-scale mid-turn (v0 P term at low speed + Turn Lead / Hold steps) and the slow ramp
  is what smooths it; also `00000061--10` +11.4–11.8 s sat at ~-390 vs -580 asked from the hands-on rule (1.8–2 Nm
  against: him or wheel reaction). Revisit after 0g is driven.
- **Proposed, not built:** drive TSFDOM with Full Torque Past 90° on and Turn Lead on (min 2 mph); Turn Hold + Hold Adds
  Your Steering as its own drive. Release guard: built, item 0h.
  Also lat smoothing 0.0 only during turn desires (0.1 otherwise, TSFDOM). A faster ramp up or down is a panda safety
  limit: **discuss with him first, no code until he agrees.**

### 0e. Turn desire fixes on `fm-dev-jet` (Nov 11): built, not driven

From `0000005e--e26703ace2--2` (Nov 9 build, Cheddar, Road Edge Turn Desire on): a left turn after a 12 s stop.
- **The turn desire did not drop.** turnLeft was held from the previous segment through the stop until 20 s. What felt
  like letting go: (1) the model's desire pulse had aged out of its 5 s history (`desireState` read "none" while
  waiting), so pulling away it asked ~305 m then ~47 m radius while the truck drove ~69 m then ~24 m (he steered);
  (2) **openpilot never steers at a standstill** (`controlsd`: `latActive` needs `vEgo > 0.3` unless
  `steerAtStandstill`); MADS stays enabled, `latActive` came back at 12.89 s; (3) an `e2eChime` alert covered the
  "Turning Left" banner 11.2 to 14.2 s.
- **Rivian blind spot is reported** with the long harness upgrade (`carstate_ext.py`, `enableBsm`), and AP's Lane Turn
  Desire drops the turn while it's lit. Not the cause in this log (no BSM). `tools/fm` doesn't read BSM yet.
- He keeps Road Edge Turn Desire **off** now (false edges, especially on lefts).
- `4ef32d0` Road Edge Turn Desire: turns under 20 mph start without openpilot steering, like AP's, so they start (and
  restart on a blinker cycle) while stopped. Road-edge turns from 20 mph still need it.
- `792475a` **Turn Pulse on Pull-Away** (`FM_TurnPulseOnPullAway`, Models > Model Behavior, off): once stopped during a
  held turn, `DesireHelper.model_desire` drops to none for one model frame at 0.3 m/s, so the model gets a fresh pulse.
  Both modeld's feed `DH.model_desire`; `DH.desire`, `laneTurnDirection`, alert, Turn Assist, Blendful unchanged.
  Replayed on the log above: fires once, at 12.89 s. Works with AP's Lane Turn Desire (Road Edge off).
- `ced5574` Turn Hold option **Hold Through Early Correction** (`FM_TurnHoldEarlyCorrection`, off): under 5 mph and
  under 20 degrees turned, hand torque against the turn doesn't release Turn Hold (request passes through, fade paused,
  same peak after). Later/faster still releases. Only matters with Turn Hold on (it was off in this log).
- Test: Turn Pulse on, Road Edge off, corners with a long wait; compare turn-in with earlier logs (`turns.py`).

### 0d. Lateral Smoothing setting on `fm-dev-jet` (Nov 11): built, off, not driven

- Why: Cheddar's lateral felt bad. Cheddar's list entry has `lat` .0 (unsmoothed), TSFDOM's has .1 (not re-verified in the
  list this session). The tinygrad gap (5d186bd vs 92b742f) is probably not the cause: compiled model files are baked.
- The `lat` value does two things in `modeld_v2/modeld.py`: it low-passes the model curvature (post-FOF models only,
  `smooth_value`) and it is added to the lateral delay given to the model (`lat_action_t`, `lateral_control_params`).
  Higher = calmer but later, and also reads the plan further ahead (earlier). So .2 is not simply "turns sooner".
  controlsd adds the constant 0.0 from `selfdrive/modeld`, not the bundle's value (upstream behavior, left alone).
- Setting: `FM_LateralSmoothingEnabled` (off) + `FM_LateralSmoothingSeconds` slider 0 / 0.1 / 0.2 (default 0.1),
  Steering > FM Pilot: Turn Tweaks > Lateral Smoothing Settings. Replaces the bundle's value while on; read every ~3 s.
- Test: Cheddar, same road, off vs 0.1 vs 0.2. In the rlog compare actual vs desired curvature lag and steer jitter
  (`tools/fm`). Not applied to a Jetlink big model's own action path (stock modeld, unsmoothed).

### 0c. Set Speed Assist on `fm-dev-jet` (Nov 11): built, not driven

- From EVPilot (gerrylum's fork, `evpilotgo/openpilot` `dev`, through `c594381` Nov 10: the boost now shrinks in
  proportion to a model slow-down or a turn instead of cutting off, which fixed its every-4-s pulsing). Ported as
  **Accel Boost mode 3, "Set Speed"** (`FM_AccelBoostMode`, Sunnylink Cruise > Driving Personality Settings); exclusive
  with Gas / Predictive, default still Gas. Logic and numbers unchanged except the end of the grace window (item 0i);
  see FM_PILOT.md for the behavior and credits.
- Only on `fm-dev-jet` (`2002100`), not `fm-dev-set`. Cherry-pick if wanted.
- Test: its own drive in Experimental Mode on roads where the model creeps up to the set speed (20+ mph), boost bar on.
  In logs, `longitudinalPlanSP.fmAccelBoost` (what it added) with `fmAccelBoostMode` 3. Watch for: surging toward the
  set speed into curves, and the 3 s grace after a lead leaves.

- Also on `fm-dev-jet` from EVPilot (Nov 11): the off-tracking / Lane Guard near-zero-curvature crash fix (`868d1b7`) and
  Rivian `steerLimitTimer` 0.8 (`17d4a0d`; only counts above 22 mph, so it doesn't change slow turns). Not taken, decided
  Nov 11: EVPilot's MAP_NORESERVE model load (`6a1ff0b9`; only for ~2 GB models loaded on the comma, which Jetlink avoids),
  its generic resend of panda-blocked frames, and its navigation (openpilot's nav, Mapbox; compare with the StarPilot nav plan first).

### 0b. Jetlink on `fm-dev-jet` (Nov 8): built, off, not yet on the device

- Branch `fm-dev-jet` = `fm-dev-set` + Jetlink (zoompilot/jetlink v0.9.0, API 4). Sources: AP PR #12 (params, events,
  selfdrived, hardwared, process, UI) and zoompilot develop (adapter, modeld hooks for the v25 modeld_v2). Setting
  Models > Jetlink (Off / USB / iOS, offroad), `JetlinkLink` 0 by default. Events `jetlinkReady/LinkLost/Chime` @28-30.
- Big model = comma's large models from Jetlink's index, picked in the Jetlink app (FM doesn't use the model-list hooks).
  It runs stock modeld's action path + Blendful (`fm_jetlink_action.py`); Calmer Crawl-Speed Steering doesn't apply to it.
- Frankie's host: MacBook M1 Pro (tested platform; ~20 s prepare, up to 10 s load). Needs a USB 3 data cable, the comma
  powered from the harness (Jetlink cuts USB-C draw at boot when on), the Mac on its own power and awake. S25 Ultra
  Android app is experimental.
- Model icon (Nov 9), home screen and road view, from zoompilot: Jetlink sets `ui_state.chestnut_state` through jetlink's
  `Status.icon()` (onroad state from its runtime record, `Status.running`): pulsing (download/prepare/join), green
  (ready parked / driving), dimmed green (`WAITING`: disengage fully to switch), orange (problem). Road view green and
  dimmed green stay on (Frankie's choice; the chestnut's fade). A Jetlink host no longer counts as an unknown USB device.
  Alerts kept as AP's wording (Frankie: don't match zoompilot's wording or its louder "lost").
- Off-device: adapter conformance, both modeld seams, joining-model contract, fallback: pass. Not run on a device.
- Order: drive v25 on `fm-dev-set` first; then `fm-dev-jet` with Jetlink off (same as fm-dev-set); then on, parked first.

### 0a. ChatGPT commits Nov 6 (not yet driven or analyzed)

- `b925e3a` Turn Hold fades over 0.5 s when the blinker is cancelled mid-hold; `398cd6b` **Turn Nudge**
  (`FM_TurnNudgeEnabled`, off, needs Turn Hold): 2–8 mph, 0.8 Nm into the turn for 0.15 s adds up to 0.01 1/m for 0.6 s
  if the path bends that way; `dd313ff` torque readout stays visible (moved down) during the Turning Left/Right notice.
  Reviewed Nov 8: tests pass (78), Sunnylink JSON matches its YAML; docs added to FM_PILOT.md; garbled docstring fixed.

### 0. `fm-dev-set`: built, not yet driven (Nov 5 evening)

- Frankie plans to drive it with Lane Centering selected, testing Higher Torque at the same time. If anything feels
  off, he goes back to `fm-dev-torque` `bece23e` alone.
- **Lane keeping mode** (`FM_LaneKeepMode`: 0 Off / 1 Lane Centering / 2 Lane Guard / 3 Blendful) replaces the
  `LaneCenteringEnabled` / `LaneGuardEnabled` toggles as the switch. No default: `params_migration` seeds it once at boot
  from the old toggles (centering wins). Everything reads it through `fm_lane_keep_mode.get_lane_keep_mode`.
- **Blendful** (sunnypilot #2073, unchanged math) in modeld_v2, picked by the mode; returns exactly 0.0 when not
  selected. Logged in `modelDataV2SP` (`blendfulCurvature`, `blendfulTrust`, `modelCurvature`, `blendfulGoingAround`,
  `blendfulLeftProb/RightProb`, `blendfulLaneWidth`). Plan before he drives it: replay on Nov 3 pull-right and curvy
  segments and show the nudge it would add.
- **Boost:** `longitudinalPlanSP.fmAccelBoost` (what the boost added) and `fmAccelBoostMode` are now logged. Side-panel
  bar from sunnypilot #2090, `FM_AccelBoostIndicator`, off. Quiet road-edge block is always on.
- **Sunnylink:** Steering > Lane Centering dropdown (+ off-tracking at the bottom); Lane Change holds the road-edge
  block; "FM Pilot: Turn Tweaks" with Turn Boost / Hold / Lead settings pages; Road Edge Turn Desire moved to Models >
  Model Behavior (that page is `remote_configurable: false`; check he can change it from Sunnylink); Cruise > Driving
  Personality Settings and Smart Cruise > Curve Speed Settings pages. Sub-page links may render after a section's items.
- **First log check:** Sunnylink shows Lane Centering before driving; `modelDataV2SP.blendfulCurvature` 0 all drive;
  lane centering active as before; plus the Higher Torque check below.
- Also discussed, parked: #2034 (smoother final stop), openpilot #39059 (torqued long-accel filter), #39225 (e2e
  curve speed). Predictive Boost math in sunnypilot master (#2089) matches ours.

### 1. Higher Torque: working; Max Torque slider, faster ramp and full cap past 90° built, not yet driven

- **Nov 5 evening drive `0000003a` (`0941590e`, Headroom, panda on) confirms it works:** `torqueOutputCan` reached 457
  (`--81`–`--83`), no faults. Only short stretches above 385 (max 1.8 s, `--82` +16 s right turn from a stop).
- **What still limits slow turns:** (1) the ramp: 3 per frame, ~1.5 s to full, while the request sat at 1.25;
  (2) AP's 95% cap past 90° (`HIGH_ANGLE_CAP_FRAC`), hence 457 not 481. The 90° TOI blip workaround is not the cause
  (torque came straight back after the one blip seen; wheel stalled ~160–196° while 340–457 was being sent).
- **Built Nov 5 night (all off by default, off-road only, next drive, all need Panda Allows Higher Torque):**
  - `c514b3e` **Max Torque** (`FM_MaxTorque`, 405–580 step 25, default 480): level sent to the panda in safety param
    bits 2–5 (n → 380 + 25 n); the panda enforces exactly it. No level = 481. Temporary: once a number is settled,
    back to fixed standard / high modes.
  - `143ece8` **Faster Torque Ramp** (`FM_TorqueFastRamp`): car code 5/frame below 25 mph; panda allows 5 below 12.5 m/s
    when safety param bit 6 is set. 250 ms limit (125) unchanged.
  - `3d7f292` **Full Torque Past 90°** (`FM_TorqueFullCapHighAngle`): car code only.
- **Torque readout** (`b514a01`, always on): top-center `sent / limit now / Max Torque ✓` on the comma 4. In logs:
  `carStateSP.fmEpasToiActive`, `fmSteerCap`, `fmMaxTorque` next to `torqueOutputCan`. Not yet seen on the device.
- **Test plan:** 480 + ramp + full cap, Turn Lead off, normal Loop A turns up to 25 mph. If the wheel still stalls at
  the cap, go off-road, step Max Torque 505 → 530 → 555 → 580, one at a time. Before driving each new step: parked
  check, wheel toward lock, lateral on. Send rlogs per step.
- **Turn Lead** cut the curb on `0000003a--83` +4 s and +22 s rights: it asked ~12 m about 1–2 s before the model got
  there (look-ahead 1.3 s plus slow ramp). Keep it off; possible fix: shorter look-ahead (~0.7–0.8 s).
- **Turn Hold** likely released in `--82` +18 s on +1.5 Nm wheel reaction (guess; hold state isn't logged). Blinker
  self-cancel mid-turn (`--83` +46 s) ended the turn desire and openpilot pushed out of the turn for ~3 s.
- Don't reset the torque learner unless he settles on Full.

### 2. Radar lead settings: built, both off, not yet driven (`50d1997`, `613c760`)

- **Ignore Stationary Radar Ghosts** (`FM_RadarStationaryGuard`, Cruise, off): radard won't pair a radar point under
  1 m/s with a camera lead over 5 m/s (11 mph) unless the camera sees it braking harder than -1.5 m/s²; that car is
  followed from the camera instead. Fixes guardrail/barrier phantom brakes on ramps and curves (`00000038--19` on-ramp
  +16.8 s: -1.8 to -2.5 m/s², 19 → 13 mph; +32.6 s; `--13`; `00000039--47` +56.6 s, where his gas press was against a
  -3.5 phantom). Low-speed radar path (under 4 m/s) untouched.
- **Lead Detection Sensitivity** (`FM_LeadDetectionThreshold`, Cruise, 25–50%, default 50 = openpilot): camera
  confidence needed to count a lead and pair it with radar. From FrogPilot (MIT). Use with the guard.
- **Replay** (real radard on the 9 Nov 5 segments; stock replay matches logged leads 96–100%): phantom frames 148 → 5
  with the guard (the 5 are camera leads at 9–12 mph, by design; one is the `--48` red-light car at 107 m, correctly
  kept); nothing else changed. 40%/35%: leads 0.2–0.9 s earlier, 1–2 short extra camera leads in `--1`; at 35% without
  the guard phantoms rose to 163. No fast approach to stopped traffic in these logs.
- **Test plan:** after the Higher Torque drive. Guard on, same on-ramp. Then threshold 40%, then 35%, in normal driving
  with slowdowns for traffic at 35–45 mph; watch for braking with no car ahead. Untested: a real hard stop behind a car.
- Checked and not taken: FrogPilot's radar-only "far lead" (moving targets only, no help for stopped traffic) and
  "Human-Like Following" (brakes later and closer; model-gated). StarPilot Dom has none of these for non-Honda cars.

### 3. Garage spiral (`00000039--61` to `--63`): mostly explained

- TSFDOM asked nearly straight at lock (median R 77–121 m vs 5.1 m driven), so openpilot sent max torque **left, out of
  the turn**, against his right steering. Blinker off in `--63`; turn desire only in `--62` 0–6.4 s.
- Lockouts are FM/AP's own **TOI-dropout watchdog**: past ~450° the EPAS mostly stopped taking torque (`ToiActive` 9–33%,
  no ToiFlt/derate/hands-override); after 1.05 s the watchdog set `steerTempUnavailableSilent` and lateral was off
  exactly 3.0 s (7 of 8; the first was him overpowering at -6 Nm).
- Turn Hold released for good at `--61` +57.5 s (pressed torque +1–1.8 Nm against the right turn, wheel moved 12° left),
  so Hold Adds Your Steering never acted. The "7–14 m" was `turns.py`'s brief tightest-asked peak, not a hold.
- **Still open:** what held the wheel at lock with `ToiActive` off and hands light (he says it doesn't stay there by
  itself). Unknown whether the EPAS refuses all overlay near the end stop or only away-from-lock (every near-lock request
  was out of lock).
- Options, not built: don't count `ToiActive` off near lock (> ~480°) toward the watchdog (discuss first); require
  against-turn torque to last ~0.3 s before releasing Turn Hold.

### 4. Wide right curve (`00000039--47` +45 s to `--48` +7 s)

- He wants more steering, **not less speed** (curve speed control rejected).
- 12 mph: hands off, not at cap, drove 90% of the request; v0's integrator held a stale -0.03 to -0.13 m/s² against the
  turn (frozen below 5 m/s). 20–21 mph: ~97%. "Model let go" at `--48` +7–8 s (0.023 → 0.014 1/m) coincided with his gas
  press and right steering.
- Options, not built: let v0's integrator decay at low speed instead of freezing; a curve hold without a blinker;
  Predictive Boost curve fade lower (now 1.5 → 2.5 m/s²; this curve was ~0.8 at 12 mph, full boost).

### 5. Smoother stops / regen: analyzed, nothing built (details in findings)

- openpilot sends **one** accel number (0x160); the VDM alone picks regen vs friction. No CAN control of the split, and
  no regen-level signal. Motor torque and brake pressure aren't on the comma's buses; friction is `ESP_BrkTrq` (0x208).
- Nov 3 drive: openpilot braking is all regen to about -2 m/s²; friction comes in only past that, and in the last
  ~0.5 s of a stop (as `stopping` starts), then holds ~-700 Nm.
- openpilot stops end harder than his one-pedal stops: -0.44..-0.55 vs -0.16..-0.40 m/s² 0.3 s before standstill,
  release jerk 4.4–5.5 vs 2.0–3.7 m/s³. Cause: the plan still asks -0.3..-0.45 in the last second, and below ~3 mph the
  truck over-brakes the plan by ~0.1 (the drag FF is ramped out there).
- Waiting on Frankie: does the end of openpilot stops feel abrupt to him; his regen setting (Standard/Low); 3–5 stops
  from 30–45 mph on current build. Candidate (discuss first): Rivian final-approach taper below ~3 mph, plus carrying
  part of the drag FF down to standstill.

## Next steps

00. Drive the request-side turn fixes on `fm-dev-jet` (item 0g), all on, together; send rlogs; `turns.py` timing and torque lines.
00a0. Set Speed grace fade (item 0i): one drive in Set Speed mode with traffic; plus more cold-morning segments with
   braking behind a lead to confirm the extra over-braking.
00a. Then Release Guard (item 0h) on at 150 as its own drive on the same corners; `turns.py` release line.
0. Update the comma to `fm-dev-set` (v25), do the first-boot checks in item 00, re-select TSFDOM, then try Cheddar.
0a. Drive TSFDOM with Full Torque Past 90° on and Turn Lead on (item 0f), then Turn Hold + Hold Adds Your Steering, each as its own drive.
1. Torque test day on `fm-dev-set` (item 1 test plan). Then decide the Max Torque number and drop the slider.
2. Radar settings test (above).
3. Smoother stops: his answers to item 5, then decide whether to build the taper (off by default).
4. Older: analyze Nov 4 routes `0000002e`, `0000002f`, `00000030`, `00000033`, `00000034`, `00000035` (Turn Assist with
   fixes) and the Camera Offset pull test (`C:\fmlogs\2026-11-04`). Turn Boost / Hold speed limit to 20 → 25 mph.
5. Later: nav port (`claude/FM_PILOT_NAV_PLAN.md`).

## Key facts in one line each (details in claude/FM_PILOT_FINDINGS.md)

- At Loop A speeds the torque cap, not the model's request, is what fails.
- His settings run sunnypilot's v0 torque controller; any controller hook must go in both v0 and v1.
- Two small models don't fit the comma 4's 50 ms step; dual mode removed.
- Models: sunnypilot list v25 (selector 21), tinygrad 92b742f; selfdrive/modeld follows sunnypilot master-dev.
- The VDM decides regen vs friction; openpilot only sends a target accel (truck delivers ~0.23 m/s² more decel than sent).
- Lane Centering, Lane Guard and Blendful are one setting, `FM_LaneKeepMode`; read it via `get_lane_keep_mode`.
- Never loosen driver monitoring.
- For other assistants (ChatGPT, Gemini), paste `claude/FM_PILOT_BRIEF_FOR_OTHER_AIS.md`. Check their claims before use.
