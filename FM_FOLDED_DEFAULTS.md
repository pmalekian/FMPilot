# FM Pilot: behavior folded into defaults (no longer a setting)

On `fm-dev-sync` (decided Nov 24, 2026; built PR by PR from Nov 25, each entry lands with its feature's PR). These used
to be toggles or options. In `fm-dev-sync` they are fixed behavior, so nothing in the settings shows them. Each line says
what happens and where the value lives, so any of them can be tweaked or turned back into a setting later. The repo copy
(`FM_FOLDED_DEFAULTS.md` on `fm-dev-sync`) and the project copy (`claude/FM_FOLDED_DEFAULTS.md`) are kept identical.

**Branch switch:** on first boot of `fm-dev-sync`, a one-time migration copies his current values from the old settings
into the new or reshaped ones (e.g. Release Guard on + 150 -> drop-down 150; Lateral Smoothing on + 0.1 -> 0.10 s;
Max Torque 580 -> 580). Fresh installs start at stock/off.

## FM Custom Torque Settings (Steering)
- **Panda allows higher torque:** toggle removed. The panda's Rivian steering limit always follows the Max Torque slider
  (385 = stock AP, said in the slider's description; slider 385–580 in steps of 15). Confirmed Nov 24. Was
  `FM_PandaHighTorque`.
- **Higher Torque mode:** always Headroom. Was `FM_TorqueMode` (Off / Headroom / Full).
- **Faster Torque Ramp:** always on with Max Torque above 385 (5 per frame below 25 mph; the panda allows it below
  about 28 mph). Was `FM_TorqueFastRamp`.
- **Full Torque Past 90°:** always on with Max Torque above 385 (skips AP's 95% cap past 90° of wheel, so tight turns
  can use the full Max Torque). Was `FM_TorqueFullCapHighAngle`.
- At 385 nothing above applies: panda, car code and torque controller are stock AP, as these all needed Panda Allows
  Higher Torque on before. Code: `opendbc/sunnypilot/car/rivian/values.py`, `opendbc/car/rivian/carcontroller.py`,
  `opendbc/safety/modes/rivian.h`, `sunnypilot/selfdrive/controls/lib/fm_torque_mode.py`.
- **Rivian steerLimitTimer 0.8 + near-zero curvature crash fix** (EVPilot): always on, as before (never a setting).
- Still settings: Max Torque slider (`FM_CustomMaxTorque`; new key, the old `FM_MaxTorque` is left untouched), Release
  Guard drop-down (Stock / 75 / 150).

## Turn Hold (when Turn Hold is on)
- **Turn Nudge:** always on. Was `FM_TurnNudgeEnabled`.
- **Hold Through Early Correction:** always on. Was `FM_TurnHoldEarlyCorrection`.
- **Ignore Brief Hand Blips:** always on. Was `FM_TurnHoldReleaseDebounce`.
- **Hold Starts at Turn-In:** always on. Was `FM_TurnHoldStartsAtTurnIn`.
- **Hold Only Counts While Steering:** always on. Was `FM_TurnHoldSteeringOnly`.
- **Hold Ignores Crawl Spikes:** always on (below 5 mph the hold's peak only learns a request kept up for 0.3 s). Was
  `FM_TurnHoldSteadyPeak`.
- **Hold Adds Your Steering stops at 45°:** fixed behavior (not a setting). Once the truck first passes 45° from the
  start of the turn, that crossing stays remembered for the rest of the turn (even if heading briefly dips back below
  45°): Hold Adds Your Steering stops latching new driver input into the hold (anything added before 45° stays frozen in
  the peak and exits only through normal exit logic), driver pressure into the turn no longer blocks the 0.5 s
  model-exit fade or release (active hold or signal-cancel fade), and Hold Follows the Model's Exit stays eligible using
  that remembered crossing (`fm_turn_assist.py`). Before 45° and Ordinary Turn Hold are unchanged.
- **Turn Hold steps aside for Settle After Turn / Early Right Push:** fixed behavior (not a setting). Once `fmModel`
  shows Settle After Turn / Early Right Push active for the current turn, Turn Hold ends its floor with a 0.5 s fade and
  does not restart for that turn (`fm_turn_assist.py`, `fm_lateral.py`).
- Still settings (Turn Hold Settings): Turn Hold, Hold Path Source, Hold Adds Your Steering, Hold Follows the Model's Exit,
  Stop Before Turning Starts Over.

## Finish Turn After Signal Cancels (when it is on)
- **Carry Only at Turn-In:** always on (no carry when the desire ends after 45° turned or once the truck is straightening,
  curve under 70% of the turn's tightest). Was `FM_TurnCarryTurnInOnly`.
- **Lane keeping waits for the turn carry:** fixed behavior (not a setting). While Finish Turn After Signal Cancels is
  carrying a turn (`fmControls.turnAssist.source == carry`), Blendful (both small-model and Jetlink big-model paths),
  AP Lane Centering and Lane Guard add no new steering: any already-applied correction fades smoothly to zero over about
  0.2 s and stays at zero for the rest of the carry, with internal state reset/advanced so each resumes normally once
  the carry ends (Lane Guard keeps its 1 s lane-line cooldown armed). When the turn desire clears in `modeld_v2`,
  Blendful stays out until a fresh `fmControls` carry status published after that change arrives, and resumes on the
  first fresh not-carrying status when a carry ends (`BlendfulCarryGate`). Disabling the last Turn Assist feature clears
  the published carry and turn state so Blendful is never left suppressed (`modeld_v2/modeld.py`, `modeld_v2/blendful.py`,
  `lane_centering.py`, `controlsd.py`, `fm_lateral.py`, `fm_turn_assist.py`). Settle After Turn's carry handling is
  unchanged.

## Alerts
- **Quiet road-edge block:** always on (small gray message, no sound). `FM_QuietRoadEdgeBlock` is not read.

## Feedforward Settings (Steering)
- **Low-Speed Feedforward master toggle:** removed; always on. The Left and Right drop-downs (Off / Gentle / Medium / Firm /
  Full) decide it, so Off on both sides = feature off. Was `FM_LowSpeedFFEnabled`.

## FM Smoothing Settings (Steering)
- **Calmer Low-Speed Correction toggle:** removed; the setting is renamed **Low-Speed Correction Strength**. The slider
  (40–100 %) alone decides it: the share of openpilot's stock low-speed proportional correction kept below about 16 mph;
  100 % = stock, lower = calmer. Was `FM_LowSpeedGainEnabled` + `FM_LowSpeedGainPercent`.

## Model Lateral Smoothing (Steering)
- **Lateral Smoothing toggles:** removed. Each model size gets one drop-down: Model Default (= old toggle off) / 0.00 s /
  0.10 s (0.20 s dropped). Were `FM_LateralSmoothingEnabled` + `FM_LateralSmoothingSeconds` (small) and
  `FM_BigModelSmoothing` (big).
- Short descriptions must say the difference: **small models**, the value both smooths the steering and is added to the
  model's lateral delay, so a higher value steers slightly earlier as well as calmer; **big models**, the value only
  smooths (not added to the delay), so it calms and answers about 0.1 s later.
