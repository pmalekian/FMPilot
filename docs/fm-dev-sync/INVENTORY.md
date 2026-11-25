# fm-dev-sync: feature inventory (plan complete, nothing built)

Nov 24, 2026. Checked against live refs: AP `dev-next-src` `cf3ad0ec` (Nov 20; sunnypilot master `3d8f211d`, Nov 6),
sunnypilot master `c410198c` (Nov 24), openpilot master `05c5c54f` (Nov 24), FM `fm-dev-nav-settle` `ca08aac9`.

**Agreed:** base = AP `dev-next-src` (torque line, verified `steerControlType = torque`; not the `-a` angle branches) +
merge sunnypilot master. All FM map features dropped (new map feature later). Each FM feature lands as its own PR.
Frankie has driven all FM features; usage is never inferred from the handoff. Behavior folded into defaults is listed in
`claude/FM_FOLDED_DEFAULTS.md` (repo copy `FM_FOLDED_DEFAULTS.md` on `fm-dev-sync`). On first boot of `fm-dev-sync` a
one-time migration copies his current values into new or reshaped settings; fresh installs start at stock/off.
Install string: `pmalekian/fm-dev-sync` (comma's installer takes GitHub owner/branch and clones `<owner>/openpilot`,
which redirects to `pmalekian/FMPilot`). PR order and checks: `claude/FM_DEV_SYNC_BUILD_PLAN.md`.

Marks: **Base** = already in the base. **Port** = carry over as an FM PR. **Drop** = leave behind.

---

## A. sunnypilot foundation (in the base, always kept)

Model runner and picker (`modeld_v2`, v25 list, TSFDOM), tinygrad `92b742f`, Sunnylink, MADS, torque controllers (v0),
custom tuning, Live Learning Steer Delay, Camera Offset, sunnypilot log messages, settings UI, AGNOS 19.9.

**Redundant in FM (drop FM's copy, use upstream's):** v25 model list ports (sunnypilot #2050–#2093), Blendful's base file,
Predictive Boost (sunnypilot #2089, `AccelBoostSP`).

**Merging sunnypilot master brings (compat checks):** GPS fuser #2112 (`LastGPSPositionLLK` removed; only the dropped map
features used it), modeld_v2 in-place frame reads #2100, hardwared params-lock fix #2109, MADS pause with brake held #2031,
resumable model downloads, opendbc bumps.

## B. sunnypilot features — all agreed Nov 24, Base, keep as settings

MADS and its options; Pause Lateral with Blinker (+ min speed, post-blinker delay; note: if it pauses at turn speeds, Turn
Hold and Turn Lead have nothing to work with); Lane Turn Desire; torque tuning (Enforce, v0/v1, Self-Tune, Custom and
Real-Time Tuning); Lateral Jerk and NNLC (FM feedforward and Release Guard stand down with them); Live Learning Steer
Delay, Software Delay, Camera Offset; lane change and blind-spot settings; cruise (disengage on accelerator, press
increments); Smart Cruise Control (vision/map) and Speed Limit Assist, mapd, navd (kept in code; check CPU cost when off;
the new map feature may reuse them); lane departure warning; display settings. Gas Override Boost and Predictive Boost
become picker options. **Blendful:** hidden from sunnypilot's Models page; only in FM's Lane Keeping selector.
**Screen saver:** check sunnypilot's against AP's; on any conflict AP's version wins.

## C. AP layer (in `dev-next-src`) — all agreed Nov 24, Base, keep

Rivian port (torque lockout, radar, park-assist signals, ACC fault recorder, panda torque rules, handback fix); Enhanced
Rivian MADS; MADS Min Engage Speed; Rivian Resume; Following Time Offset and Stop Distance; AP Lane Centering (one option
in FM's Lane Keeping selector); AP's rear off-tracking (FM's gerrylum copy dropped); JetPilot / Jetlink (FM delta on top);
Opview (he plans to use it); Acceleration Control (picker option "AP Accel Control"); AP's Curve Speed Control toggle
(left in, off; confirm what it does before build); AP's crawl-speed curvature floor 1.0 (Calmer Crawl-Speed raises it to
2.5 when on); screen saver text/logo (wins over sunnypilot's) and Rivian menu (as is). opendbc stays copied in-repo as AP
does.

## D. FM features — decided Nov 24 (all Port unless marked)

### Cruise / longitudinal
- **Accel Boost picker:** Off / **Gas** (openpilot #39015 via sunnypilot) / **Sunny Boost** (sunnypilot #2089 predictive) /
  **EVP Set Speed** (EVPilot) / **AP Accel Control** (AP) / **OP Personality** (openpilot #39430 + #39440, Nov 19–23:
  +0 / 0.05 / 0.15 m/s² by personality, faded at low speed and when braking; not in sunnypilot yet). One at a time; the
  picker turns sunnypilot's `AccelBoostSP` and AP's toggle off.
- **Accel Boost bar:** must show the boost for all five modes (each publishes its amount to `longitudinalPlan.accelBoost`).
- **Radar-matched and Camera-only Lead Confidence, Ignore Stationary Radar Ghosts.**
- **Smooth Lead Handoffs** (toggle, stays in Cruise).
- **Radar Lead Status log** (`fmRadarLead`; feeds the Lead readout and `tools/fm/lead.py`).
- **Dropped:** Lead Shadow Log, Radar Badge.

### Steering page layout (sunnypilot's own sections stay where they are)
1. **Lane Keeping** (Steering > Lane Centering): selector Off / Lane Centering (AP) / Lane Guard / Blendful; Blendful down
   to 0.3 m/s, blue center line only.
2. **FM Custom Torque Settings:** Max Torque slider 385–580 in steps of 15 (description: "385 = stock AP"; the panda's
   limit follows it; off-road only, applies next drive; default 385, his 580 carried over); Release Guard drop-down
   Stock / 75 / 150 (default Stock, his value carried over). Folded: Headroom mode, Faster Ramp, Full Torque Past 90°,
   steerLimitTimer 0.8 + near-zero curvature crash fix (EVPilot).
3. **Feedforward Settings:** Low-Speed Feedforward Left Turns / Right Turns (Off / Gentle 40 / Medium 60 / Firm 80 /
   Full 100; default Off, his values carried over). Master toggle folded.
4. **FM Smoothing Settings**, in order: Smooth Turn Start (toggle); **Low-Speed Correction Strength** (renamed from Calmer
   Low-Speed Correction; slider 40–100 %, 100 % = stock, lower = calmer; default 100, his 70 carried over); Calmer
   Crawl-Speed Steering (toggle); Calmer Straight Pull-Away (toggle).
5. **Model Lateral Smoothing:** Small Models, then Big Models: Model Default / 0.00 s / 0.10 s, each with its note (small =
   smoother and slightly earlier steering; big = smoothing only, about 0.1 s later).
6. **Turns:** Turn Hold Settings, Turn Lead Settings, Pre/Post Turn Behavior, other turn options (below).

### Turns
- **Turn Hold Settings:** Turn Hold, Hold Path Source, Hold Adds Your Steering, Hold Follows the Model's Exit, Stop Before
  Turning Starts Over (toggles). Folded: Turn Nudge, Hold Through Early Correction, Ignore Brief Hand Blips, Hold Starts at
  Turn-In, Hold Only Counts While Steering, Hold Ignores Crawl Spikes (drop the Nov 20 drive example from its description).
- **Turn Lead Settings:** Turn Lead, Turn Lead Min Speed (2–7 mph), Adaptive Look-Ahead, Wait for the Corner + distance,
  Turn Lead Yields to Your Hands, Turn Path Tightest Radius (shared with Hold Path Source; placed here unless he moves it).
- **Pre/Post Turn Behavior:** Queue Guard, Turn Pulse on Pull-Away, Settle After Turn, Settle After Turn Early Right Push
  (toggles).
- **Finish Turn After Signal Cancels** (toggle); Carry Only at Turn-In folded (always on with it).
- **Quiet road-edge block:** folded (always on).
- **Dropped:** Turn Boost, Road Edge Turn Desire.

### Readouts, logs, tools — agreed Nov 24
- **Top Readout:** Torque / CPU / Lead, all three kept, with the torque log fields.
- **Turn Assist and low-speed steering logging** (moving into FM-owned log structs, see the build plan).
- **hardwared stall diagnostics:** kept until the Low Communication Rate warnings are confirmed gone (sunnypilot #2109),
  then dropped.
- **Jetlink extras:** zoompilot's update (API 4, v0.9.0 fixes), Jetlink Status log (`fm_jetlinkstatusd`), icon states.
- **`tools/fm`:** fmlog, timeline, events, turns (with the `fm-dev-max` `d189ccc` timing tweak), lead, settle, sway, dm,
  jetlink, maclog and their tests. `nav.py` and its test dropped.
- **Docs:** fresh `FM_PILOT.md`, `FM_PILOT_HANDOFF.md` and `FM_FOLDED_DEFAULTS.md` for the new branch; earlier branch
  history moves into `FM_PILOT_FINDINGS.md`.
- **Install:** `pmalekian/fm-dev-sync`; old branches stay as fallbacks.

### Dropped (agreed)
Nav Assist (`fm_navassistd`), Download Nav Map, Map Turn Desire, Nav Turn Speed, `sunnypilot/fm_nav/`, the `fmNavTurn`
message and its tests, `tools/fm/nav.py`; Turn Boost; Road Edge Turn Desire; Lead Shadow Log; Radar Badge.
