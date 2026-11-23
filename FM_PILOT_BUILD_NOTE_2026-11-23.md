# FM Pilot build note: three planned builds (Nov 23, 2026)

Decided with Frankie on Nov 23 in the "Settle After Turn speed" chat. Nothing here is built yet. Base: `fm-dev-nav` @
`8f00a93` (repo `pmalekian/fmpilot`, now `pmalekian/FMPilot`; old URL redirects). Same item is in
`FM_PILOT_HANDOFF.md` as 0v.

## Order and how to split the work

1. **Build A: Lead readout** first, by itself, in one chat.
2. Then in parallel, one agent each:
   - **Build B: Lead sensitivity (two sliders, smooth handoffs, shadow mode)**
   - **Build C: Settle After Turn early right push**
3. **Branches:** each parallel agent works on its own branch off `fm-dev-nav` (after Build A is pushed), e.g.
   `fm-dev-nav-lead` and `fm-dev-nav-settle`. Both will touch shared files: `openpilot/cereal/custom.capnp`,
   `openpilot/common/params_keys.h`, the Sunnylink YAML + compiled `settings_ui.json`, `FM_PILOT.md`, the handoff.
   Merge into `fm-dev-nav` one at a time; after the second merge, resolve those files by hand, re-number any capnp
   fields that collide, recompile Sunnylink settings and rerun tests.

Project rules that apply to all three: new settings off by default; one logical change per commit, messages start
`FM Pilot`; author `Frankie Malekian <107580415+pmalekian@users.noreply.github.com>`; credit borrowed code in the commit,
the source file and `FM_PILOT.md`; tests + lint; recompile Sunnylink after YAML changes; update `FM_PILOT.md` and the
handoff; never weaken driver monitoring; panda limits / torque cap changes need Frankie first; anything that changes
driving behavior is replayed offline against the last build before he drives it.

---

## Build A: Lead readout (new Top Readout option)

**What:** a fourth option for `FM_TopReadout` (Visuals; today 0 Off, 1 Torque, 2 CPU Avg, 3 CPU Busiest; code in
`openpilot/selfdrive/ui/sunnypilot/onroad/fm_torque_readout.py`). Label "Lead". Default unchanged (Torque).

**Format (minimal, no labels, single spaces):** `<conf>% <src> E<n> G<n>`

```
18% – E0 G0     gray: below your threshold, no lead
38% C E0 G0     cyan: camera-only lead, early (threshold..50%)
40% R E1 G0     purple: radar-matched lead, early (under 50%)
62% C E1 G0     white: camera-only, 50%+
87% R E1 G0     white: radar-matched, 50%+
74% X E1 G1     X flashes red ~2 s: ghost dropped
```

- **Left number:** the model's live lead confidence, read from `modelV2.leadsV3[0].prob` (or radard's filtered value
  if exposed), NOT `fmRadarLead.leadProb` (that is 0 whenever there is no lead, so it would never show the climb).
- **Colors of the number (Frankie's final choice):**
  - gray: no lead counted (below threshold), and any other case not listed below
  - cyan (light, distinct from purple): camera-only lead with confidence under 50%
  - purple (light lavender, readable in daylight): radar-matched lead with confidence under 50%
  - white: any lead at 50%+ (radar or camera)
  - Low-speed radar path (radar point straight ahead, camera unsure) shows R with a gray number.
- **Source letter:** R radar-matched, C camera-only, X ghost dropped (amber pulse replaced by red flash ~2 s), `–` no
  lead. Not G for ghost (clashes with the G count).
- **E count:** number of times this drive a lead got a radar match while confidence was under 50% (once per event, not
  per frame; count the rising edge of "radar-matched and under 50%", excluding the low-speed radar path). Frankie only
  cares about early radar matches, not matches at stock settings.
- **G count:** `fmRadarLead.ghostDrops` (already counted per drive in radard).
- Both counts reset each drive.
- **Data needed:** source, earlyLead, ghostDropped, ghostDrops, threshold are in `fmRadarLead` already. E needs a new
  per-drive counter in `FmRadarLeadStatus` (`fm_radar.py`) or in the UI with the same rising-edge logic as ghostDrops
  (the UI can miss frames, so radard counting is safer). Live confidence: read modelV2 in the UI.
- **Why the old badge was likely never seen** (not proven broken): needs `FM_RadarBadge` on; ghost only with the guard
  on, camera lead >11 mph and not braking hard, matched to a standing-still radar point (ramps/curves by barriers);
  early ring only with Lead Detection Sensitivity below 50%; not drawn while disengaged. A recent rlog's `ghostDrops` /
  `earlyLead` would tell. Frankie runs 40% with the guard on (handoff 0u). Keep the badge as is unless he says
  otherwise.
- **Checks:** unit tests for the readout text/colors and E counting; can't render raylib off-device, so on-device look
  is unverified.

---

## Build B: Lead sensitivity — two sliders, smooth handoffs, shadow mode

Based on ChatGPT's proposal (Frankie uploaded it Nov 23) plus our changes. Today: one slider
`FM_LeadDetectionThreshold` (25-50%, default 50) gates both radar-matched and camera-only leads in `get_lead()`
(`openpilot/selfdrive/controls/radard.py` ~L158-192; both lead slots ~L263-285; settings/polling in
`openpilot/sunnypilot/selfdrive/controls/lib/fm_radar.py`). Key fact Frankie confirmed he now understands: below the
threshold nothing counts; above it a lead counts as radar-matched if a radar point fits, else camera-only, and the
planner (MPC, min with e2e in Experimental) can brake for either.

### B1. Two sliders (admission)
- **Radar-matched lead confidence** and **Camera-only lead confidence**, each 25-50%, default 50%.
- Try radar matching from the LOWER of the two cutoffs. If a valid radar match exists (and the guard doesn't reject
  it), use it. Otherwise fall back to camera-only only if the camera cutoff is cleared. (ChatGPT's row 4 choice: with
  camera below radar, a lead eligible through the camera cutoff still uses radar numbers if a match exists. Agreed.)
- Same rule for `leadOne` and `leadTwo`; low-speed radar override on `leadOne` unchanged; keep the asymmetric prob
  filter and strict `>` checks so 50/50 is identical to today.
- **Upgrade (our change):** do NOT reset to 50/50. Carry his current value into both sliders (radar = camera = old
  value, i.e. today's behavior), keep the legacy param for rollback. Ask Frankie if he'd rather start at radar = old,
  camera = 50.
- Polling ~1 s as today (changes apply while parked without a restart).

### B2. Smooth handoffs (combined ChatGPT + Claude design), all in radard; planner untouched
- **One continuous estimate per lead**, predicted forward each frame (gap, closing speed, accel, lateral) and corrected
  by whichever sensor reports (radar or camera). No one-frame jumps when the source changes.
- **Same-car check** before blending: the new measurement must fit the predicted state (gap, lateral position, closing
  speed). If not, it's a different car.
- **Cut-in / new car:** fresh estimate at once; the old car's state and hold timer never transfer.
- **Bad news at once, good news eased (every transition):** if the new reading says closer or slower, follow it
  immediately; if farther or faster, ease toward it over a few frames (~0.5-1 s).
- **Hold on radar loss:** keep predicting from the last confirmed state, up to ~0.5 s, only while the camera still sees
  the car (even below its cutoff). End at timeout, when the car leaves the path, or on a ghost-guard rejection. If the
  camera clears its cutoff, blend toward camera numbers instead of dropping.
- **Hysteresis on both cutoffs:** a lead joins at the slider value and stays until ~5 points lower (like Lane Guard's
  0.7 / 0.6 lines).
- **Sticky radar matching:** prefer the same radar track while it still passes the checks (avoids swapping between a
  car and a sign behind it).
- No global acceleration filter (would delay real cut-ins). If replay still shows a brake-release pulse on same-car
  switches, look at that specifically.
- Recommend running with Ignore Stationary Radar Ghosts on. Open gap: wrong radar matches to MOVING objects (car in the
  next lane) aren't caught by the guard.
- Its own toggle(s), off by default (e.g. "Smooth Lead Handoffs").

### B3. Shadow mode (Frankie: "add shadow mode into the mix")
- radard runs the real settings and ALSO computes, without acting, what alternate settings would have chosen:
  at least 30/50 and 30/40, with and without the B2 smoothing. Log per frame for both lead slots: selected source,
  confidence, radar track ID, distance, relative speed, hold/blend state, guard/low-speed override, which cutoff
  admitted it. Off by default (setting "Lead Shadow Log"), alternates fixed or picked in settings.
- Lets him drive at real settings (e.g. 50/50 or his 40) and compare from the log: earlier real leads, added fake
  leads, flicker, jumps, and where the planner would have differed. Limit: can't show how the truck would have felt
  afterwards (the real truck didn't brake that way).
- Add a `tools/fm` report for it.

### B4. Logging and validation
- Extend `fmRadarLead` (keep existing fields/readers working) with both effective cutoffs and per-slot decision data.
- Compare against `longitudinalPlan` source and target accel (lead MPC vs e2e vs cruise) to tell whether a lead
  actually set the slowdown.
- **Regression:** 50/50 identical to today: exact cutoff values, no tracks, rejected matches, radar loss/reacquire, both
  slots, low-speed override, missing/invalid params.
- **Replay** on the same recorded inputs (model + guard fixed; pass thresholds explicitly, `FmRadarSettings(params=None)`
  is baseline): 50/50, 50/40, 30/50, 30/40, smoothing off/on. Report earlier real leads, false leads, dropouts, biggest
  jumps in lead distance/speed and in target accel, and response time to a real braking car (smoothing must not slow
  it). Look at cut-ins, stopped queues, curves with roadside returns. Logs: `000000a8--df5d2d0570--6/--16` (guard on,
  40%) plus whatever he sends.
- Then shadow-log drives, then real drives.

---

## Build C: Settle After Turn — early right push

Current feature: `openpilot/sunnypilot/modeld_v2/fm_turn_settle.py` (`FM_TurnSettle`, off), wired through the Blendful
hook in `modeld.py`, setting under Steering > FM Pilot: Turn Tweaks. Today: 8-30 mph (`MIN_SPEED`, hard-coded; 8 had no
documented reason, Claude picked it in the Nov 20 proposal), starts when the turn desire ends after a 45+ deg turn,
aims at the lane center from 30% lines, curb only a limit (1.4 m), 0.25 m deadband, after a left only pushes right,
cap 0.004 1/m, Blendful's build/release rates, ends at Blendful trust 0.5 / desire / lane change / 30 mph / 6 s / not
steering / hands. Does nothing if no line is seen.

**Frankie's goal:** act as soon as the turn is ending, well below 8 mph. If the lane can't be seen, heading toward the
right side is safer than drifting into oncoming traffic (US right-hand traffic: oncoming is always left).

**Build (all decided):**
1. **Earlier start:** trigger on the truck's turning rate dropping / heading lining up with the new road, not only the
   desire ending. Keep the 45 deg minimum turn.
2. **Speed floor ~2-3 mph** (from 8). Max stays 30 mph.
3. **Low-speed strength:** cap by lateral accel at low speed instead of the fixed 0.004 1/m (which moves the truck only
   ~0.2 m in 10 m); keep the current cap at higher speed. Values from replay.
4. **Capped right push when no lane center is visible:** after both left and right turns, push right, at most ~0.5 m in
   total, tracked from the truck's own motion. This is the blind backstop.
5. **Stop at center:** if a 30%+ line shows the truck already centered or right of center, the push ends (protects good
   turns like 000000af Turns 2 and 4).
6. **Switch to centering:** once a center is visible, use the existing centering (never left after a left turn), hand off
   to Blendful at trust 0.5 as now.
7. **Curb guard inside Settle** (works whatever `FM_LaneKeepMode` is): a sure curb (std <= 0.5 m) stops the push at 1.4
   m; if the planned path brings the body within Lane Guard's margin, steer back left. Borrow the math from
   `openpilot/selfdrive/controls/lib/lane_guard.py` (gerrylum rx-wb `17ea338`, MIT; Rivian half width 1.007 m, edge
   margin 0.15 m) with credit. Lane Guard's own gates (70% lines, Rivian tune guards lines only in curves or >35 mph)
   are too strict here; use Settle's looser curb rule.
8. **Parked-car yield:** stop or hold the push when the model's own plan is clearly steering away from the right (going
   around a car). Based on Blendful's going-around / yield logic (`sunnypilot/modeld_v2/blendful.py`, sunnypilot
   #2073, MIT) with credit.
9. **Hands always win:** any steering cancels at once.
10. **Logging:** push distance used, which limit stopped it, phase (push / centering / handed off), in `modelDataV2SP`.

**Open:** new option under Settle After Turn (off by default) vs replacing the current behavior; exact speed floor, low-
speed cap and push distance (from replay).

**Offline checks before he drives it:** replay on `000000af--565e9cd6b0` (Nov 20, Cheddar), all 8 turns, vs the current
Settle as baseline. Report each turn's exit speed; confirm Turns 2 and 4 aren't pushed toward the curb; count how often
each limit fired (if the 0.5 m cap does most of the stopping, the guard rarely sees anything in time and the cap should
shrink). Out of scope: a too-tight turn-in (Turns 5 and 7) is Turn Lead / Turn Hold's job; a map-based side check via
Nav Assist is a later idea.
