# FM Pilot: handoff for new chats

State as of Nov 3, 2026 (evening). Read this with `FM_PILOT.md` (features, settings, credits) and `tools/fm/README.md` (log tools).

## Setup

- **Truck and device:** Frankie's 2022 Rivian R1S Launch Edition (Gen 1), comma 4, torque harness.
- **Fork:** GitHub `pmalekian/fmpilot`, branch **`fm-dev-max`**, latest commit at the time of writing `01ca584`. Installer:
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
  - **First drive with Turn Assist: route `0000002d--2f1a05a219` (Nov 3, 63 min, all 64 rlogs).** Lead + Hold + both
    options on, min speed 2 mph, radius 8 m; Boost turned on from Sunnylink at 1:25 (off 53:27, on 61:26), off-tracking
    0.25. Turn Lead turned the truck in when TSFDOM asked for nearly straight (60:50: model -8e-3, request -114e-3).
    Two problems, both fixed: steering into the turn before the truck turned released Turn Hold (`6124165`), and Hold
    Adds Your Steering capped your curve at the slider, so openpilot pushed out of 5–6.6 m loops (`ed7bd8b`: now up
    to full lock). Not yet driven with the fixes.
  - **Builds before `b9c5a55` (Nov 3) had left and right swapped** (curvature is right-positive; Turn Assist assumed
    left-positive), so Boost scaled away-from-the-turn requests and Hold could hold the wrong way. Don't drive either
    feature on an older build.

## Higher Torque: branch `fm-dev-torque` (Nov 5, built, not yet driven)

Goal agreed Nov 4–5: raise the low-speed steering torque cap. FM Pilot is its own fork now: no AP syncs planned; other
forks are only a source of ideas.

- **Branch:** `fm-dev-torque`, cut from `fm-dev-max` `947b16c`. Two commits: `f21de38` (panda) and `b351c7e` (setting).
  Frankie switches the comma to it in the branch selector and back to `fm-dev-max` the same way; the panda is reflashed
  on boot each time, so its limit always matches the branch.
- **Panda Allows Higher Torque** (`FM_PandaHighTorque`, off-road only, off): read at onroad start (card restarts each
  onroad; pandad sends `safetyParamSP` bit 2). Panda envelope `[9,25,27] -> [481,295,275]` (AP's is 385). Ramp rates,
  real-time limit and driver-torque rules unchanged.
- **Higher Torque** (`FM_TorqueMode`, live, Off): car cap `[9,17,25,27] -> [481,332,295,275]`. Headroom raises only the
  controller's max (1.25 under 20 mph); Full scales every request (AP's 481 test was this and felt jerky at 20–29 mph,
  oversteered at 38–50 mph). Blends over 1 s. Panda setting off: every mode stays at 385.
- **Why Headroom:** openpilot's torque is a fraction of the cap, so raising the cap alone (Full) makes every low-speed
  correction 25% stronger. Headroom changes nothing until the controller would have maxed out.
- **Evidence for headroom (Nov 4 logs):** in Loop A `00000032` (`--9`, `--12`, `--15`, `--17`), at the cap the EPAS kept
  `ToiActive` on (96–100%), never set `ToiFlt`, `SteeringReduced` or `SteeringFault`, and the wheel crept (3–60 deg/s)
  until Frankie added 1–2.5 Nm, then moved at 150–420 deg/s. Garage segment (`0000002c--28`): the wheel was moved by
  hand; the command was ~0.5 of the cap until near full lock (-456°), where `ToiActive` dropped as it hit 0.95. Not
  informative about the ceiling. The DBC has no "torque applied" signal; headroom can only be judged from the response.
- **Test plan (one build for all of it):** turn the panda setting on in off-road mode, then Higher Torque Headroom.
  Settings otherwise as Loop A on Nov 4 (TSFDOM, lane centering, Turn Assist and off-tracking off). 1) Empty lot, slow
  turns. 2) Loop A both ways, compare per corner with `00000032` (time at cap, radius vs request, hand help). 3) Normal
  driving 20–45 mph. 4) Full on the same corners, for comparison. 5) Turn Assist back on. Stop on any `ToiFlt` or
  steering fault, unexpected jerk, or a wheel that's hard to override.
- **Check in the first log:** pandad's "setting safety model ... param_sp" line shows bit 2 when the panda setting is on.

## Next update to the branch (agreed, not yet done)

1. Turn Boost / Turn Hold speed limit: full strength to 20 mph, fading out by **25 mph** (now 15 → 20).
2. ~~Sync AP `dev`~~ and ~~use AP's off-tracking~~: dropped Nov 5. FM Pilot no longer syncs AP; borrow individual
   pieces (e.g. AP's R1T wheelbase from the VIN) only if wanted.

Pieces in AP's pending sunnypilot sync (branch `fix/pr10-personality`) worth borrowing on their own, if wanted. No sync planned:

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
  mid-turn. Raising the cap is now the goal (Nov 5): see Higher Torque above.
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
- **Pull to the right, route `0000002d` (Nov 3):** from about minute 30–37 (highway) to the end, the truck sat 5–10 cm
  right of lane centre (earlier 3–7 cm left), and Frankie pushed left over 0.5 Nm for 5–10 s of each minute on
  straights, against a slightly right request (model +0.1 to +0.9e-3 1/m while he pushed). Not Turn Assist (blinker
  off), not the Sunnylink changes after 50 min (it started before), not lane centering (it added a little left). Roads
  were crowned (roll +2 to +2.8°) and the wheel sat about 1° left on straights. Likely TSFDOM's lane position plus
  crown; unproven. Learned torque params: latAccelOffset about -0.35 m/s² all drive (large; worth a look), factor 2.59
  → 2.73. Suggested test: Camera Offset +0.05 m (shifts the model's centre left).
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
