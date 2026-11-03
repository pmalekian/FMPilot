# FM Pilot: handoff for new chats

State as of Nov 3, 2026 (evening). Read this with `FM_PILOT.md` (features, settings, credits) and `tools/fm/README.md` (log tools).

## Setup

- **Truck and device:** Frankie's 2022 Rivian R1S Launch Edition (Gen 1), comma 4, torque harness.
- **Fork:** GitHub `pmalekian/fmpilot`, branch **`fm-dev-max`**, latest feature commit at the time of writing `ba7b308`. Installer:
  `pmalekian/fm-dev-max`. Based on AdventurePilot `dev` (`github.com/AdventurePilotDev/AdventurePilot`), synced up to
  `980dd5c` (Oct 31).
- **Working on the code in a new chat:**
  - Attach the repo (`pmalekian/fmpilot`, push access) and clone `fm-dev-max`. Add `upstream` as AdventurePilot.
  - Commit as `Frankie Malekian <107580415+pmalekian@users.noreply.github.com>`, messages starting "FM Pilot", one
    change per commit.
  - Push with `--no-verify`: the LFS pre-push hook tries to upload AP's LFS objects and gets a 403.
  - Tests run off-device with a shim that stubs `Params` from `params_keys.h` (no compiled params library or msgq here).
  - Recompile Sunnylink settings with `openpilot/sunnypilot/sunnylink/tools/compile_settings_ui.py` after YAML edits.
- **Frankie's side:** a Windows PC (PowerShell). The comma is usually at `192.168.1.246`, SSH user `comma`, with the
  key in his GitHub account.
  - Always `scp -O` (plain scp hangs).
  - `scp` runs at the `PS C:\...>` prompt; device commands run after `ssh comma@192.168.1.246`.
  - Give exact, copy-pasteable commands.
- **Logs:** each segment is one minute; send `<route>--<n>/rlog.zst`. The loaded model is only logged in `--0`; the
  tools work it out from initData for other segments and mark it "(from settings)". Comma Connect's download sometimes
  hands out `--0` when later segments haven't uploaded yet.
- **tools/fm** takes several segments, folders or wildcards and joins one drive's segments (`--3 +12.4s` times).
  Tests: `cd tools/fm && python -m pytest tests`. Sign conventions are verified in its README: **curvature and lateral
  accel are positive to the right; angle, hand torque and all torque commands positive to the left.**
- **Hand torque** is the column torsion bar: while openpilot steers, a loose wheel's reaction reads *against*
  openpilot's torque. `turns.py` reports "with openpilot" (certainly the driver) and "against it*" (driver or reaction).

## What Frankie drives now

- **Model:** TSFDOM. He prefers its longitudinal by far; its lateral is "good enough" with his help.
- **Steering:** lane centering on (One-Line Mode on), Lane Guard off, off-tracking off.
- **Turns and road edge:** Road Edge Turn Desire on (max 40 mph), quiet road-edge block on.
- **Longitudinal:** Experimental Mode. **Accel Boost** (`FM_AccelBoostMode`, Cruise page): Off / Gas / Predictive, default
  Gas. Predictive (sunnypilot's model-predicted boost, `c5f251a`) is built but **not yet driven**; on aggressive it adds
  up to +0.4 m/s². It needs the model's gas/brake-press predictions; check `modelV2.meta.disengagePredictions` in a
  TSFDOM rlog has `gasPressProbs`/`brakePressProbs` filled (if empty, Predictive does nothing).
- **Custom model:** the OPM16D + Rebel Legion splice is built on the device but switched off. He gave it up for TSFDOM.
- **Turn Assist** (all off by default, **none driven yet**): Turn Boost, Turn Hold (`1616b20b`), and from StarPilot's
  Model Turn Assist (Nov 3): **Turn Lead** (`111c97a`), Turn Hold options **Hold Path Source** (`2dbc4f8`) and **Hold
  Adds Your Steering** (`ba7b308`), and the shared **Turn Path Tightest Radius** slider (8–20 m, default 12 m; 8 m is
  StarPilot's fixed cap).
  - Frankie's slow turns **start late**, so Turn Lead is first: Turn Lead alone, hands hovering, send turn segments.
    Then Turn Hold (optionally with its two options), then Boost (defaults left 1.10, right 1.20) only if turns still
    go wide. Lead and Hold can run together, but the settings recommend one at a time.
  - Turn Lead works from **Turn Lead Min Speed** (`FM_TurnLeadMinSpeed`, 2–7 mph, default 7 ≈ StarPilot's 3 m/s; full
    strength about 2.2 mph above it) to about 16 mph. Frankie's turns often start late **pulling away from a stop**, so
    lowering it is expected; at 2 mph it meets Hold Path Source. Next step: add path-curve-onset vs request-onset timing
    to `tools/fm/turns.py`, check how steady the path is at low speed, and tune the min speed and radius.
  - Hold Path Source only works under about 2 mph and when the corner is right there (path turned 10° within 1.5 m for
    full weight, none past 5 m). Time stopped no longer counts toward Turn Hold's 8 s.
  - **Builds before `b9c5a55` (Nov 3) had left and right swapped** (curvature is right-positive; Turn Assist assumed
    left-positive), so Boost scaled away-from-the-turn requests and Hold could hold the wrong way. Don't drive either
    feature on an older build.

## Next update to the branch (agreed, not yet done)

1. Turn Boost / Turn Hold speed limit: full strength to 20 mph, fading out by **25 mph** (now 15 → 20).
2. **Sync AP `dev`.** New since our sync: R1T wheelbase from the VIN, AP's own off-tracking compensation, and its device
   menu.
3. **Use AP's off-tracking and remove ours.** It's the same rx-wb code with the same setting names, so keeping both
   would apply it twice. Make sure Turn Boost / Hold run **before** AP's off-tracking line in controlsd, so off-tracking
   sees the boosted request.
4. Re-run the tests. If tinygrad or `compile_modeld.py` changed, rebuild a custom model and check it with
   `fm_check_model.py --compare` before driving it.

When AP takes in the big sunnypilot sync (branch `fix/pr10-personality`, not in dev yet):

- **Gas Override Boost** arrives upstream as the same `accel_boost.py`; drop ours to avoid boosting twice.
- **Predictive Boost** (sunnypilot `AccelBoostSP`, branch `booooost`, likely on by default with its own `AccelBoostSP`
  param): keep FM Pilot's selector as the only switch, point Predictive at their file, and make sure their param can't
  turn on a second boost.
- **Acceleration Control** (a fixed lift by personality) is a new optional setting worth trying.
- **Road-edge lane change** changes (RELC decoupled from blind-spot blocking, nudge override): re-check Road Edge Turn
  Desire and the quiet block.
- **tinygrad bump and new compile paths:** re-verify the IMAGE=1 workaround.

## Findings to remember

- **Two small models don't fit.** Each takes about 37 ms of the comma 4's 50 ms model step, so dual mode is impossible.
- **Model families:** OPM16D and Rebel Legion share identical vision and plan networks, so splicing them is free.
  RHM/RDF and TSFDOM/RDFv2 are separate families. gwmv9 and DTRv6 (2025) ship as two files and steer from the plan,
  with no action head; they can only lend behavior as training teachers.
- **Compiling on the comma (QCOM, IMAGE=1)** miscompiles the point-policy residual add (`add_12 = linear_2 + linear`):
  lane lines, meta and pose come out NaN and locationd blocks engaging.
  - The fix is a contiguous `linear_2`, set via `FM_CONTIGUOUS_OUTPUTS=linear_2` in the build script.
  - IMAGE=0 also works but takes about 245 ms; rewriting the add doesn't help.
  - Official model downloads are fine.
- **Steering torque:** AP's Rivian low-speed cap is 385 (stock 250), and the panda enforces the same cap. Every model
  hits it in tight turns. Older models (gwmv9, DTRv6) ask for tighter radii; TSFDOM eases from about 11 m to 30 m
  mid-turn. Raising the cap was discussed and rejected.
- **The ramp curve (Nov 3):** TSFDOM cut a right curve over the edge line. The controller tracked the request fine.
  Lines and road edge were too uncertain for lane centering or Lane Guard (Lane Guard trusts a line at 70%+ confidence
  and an edge at under 1 m std). Decided: leave the trust limits alone.
- **The garage (Nov 3):**
  - At 2–5 mph TSFDOM asked to go straight even with the turn desire active.
  - openpilot's very stiff low-speed correction (gain 65 at 4–5 mph) pushed left, and the wheel spun right to full lock.
    Likely a fight between openpilot and the power steering amplifying the column twist.
  - The result was a rack refusal and a one-frame `accFaulted` ("Cruise Fault: Restart the Car"). AP logged a
    `rivian_acc_fault_snapshot`.
  - Suggested: raise AP's minimum MADS engage speed to 8–10 mph, or pause steering in garages. Worth reporting to AP.
- **Driver monitoring:** distraction mostly reads as "phone" from fidgeting with a phone (PopSocket grip). Thresholds must
  not be loosened: comma bans forks that weaken driver monitoring.
- **Straight-road sway:** about 0.7° of wheel with a period near 5 s, measured on the OPM16D + Rebel Legion splice.
  Frankie doesn't feel it with TSFDOM. One-Line Mode added about 25% where a line was faint.
- **Blendful** (sunnypilot lane-centering blend, alpha): looked at Nov 3, left alone for now. Centers off one line at half
  strength on straights, ignores road edges.

## Ideas backlog

- **Controller tuning** of the Rivian torque parameters (lat accel factor 2.8, max 2.5, friction 0.07) and the turn-in
  ramp, from Frankie's logs.
- **Retrain TSFDOM's steering head** with OPM16D (or gwmv9) as the teacher, with TSFDOM's acceleration pinned.
  Alternatively, OPM16D with TSFDOM-taught speed.
- **Distillation** via openpilot.distill: chestnut's 1B model is the teacher; RunPod RTX PRO 6000 (Blackwell, 96 GB);
  needs an ONNX export plus a speed test on the device. Frankie saves turn-heavy drives for it.
- **A road-edge guard for curves,** only if more examples show confident edges being crossed.
- **Upstream PR candidates for AP**, each as a clean branch off AP dev: Road Edge Turn Desire, Turn Boost / Hold, the
  quiet road-edge block, and the compile fix.
- **Later:** port StarPilot's navigation, or comma's nav alpha (openpilot #39066, simpler and closer to our code).
- **StarPilot items looked at Nov 3 and set aside:** Force Stop (holds at most 2 s at a light, doesn't fix early
  take-offs; a simple FM "hold until RES/gas" would, if wanted later), Turn Assist's twitch guard, Blendful.
