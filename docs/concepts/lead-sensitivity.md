# Custom Lead Sensitivity

Custom Lead Sensitivity adds four lead detection settings behind one master toggle on the Cruise page (device and Sunnylink). With the toggle off, radard behaves exactly as it does without this feature.

## What it does

Custom Lead Sensitivity controls when sunnypilot starts following a car ahead, and how steady that lead stays. When the toggle is on, four sub-settings unlock:

1. **Camera-only Lead Confidence** (25-50%): how sure the camera must be to follow a car with no radar point behind it.
2. **Radar-matched Lead Confidence** (25-50%): how sure the camera must be before radard tries to pair that car with a radar point.
3. **Smooth Lead Handoffs** (on/off): keeps the same car steady when radard switches between radar and camera, or between radar points.
4. **Ignore Stationary Radar Ghosts** (on/off): refuses to pair a moving car with a radar point that is standing still, like a guardrail.

Everything ships off, with both sliders at 50%. All of them apply only with openpilot longitudinal control. The code is in `openpilot/sunnypilot/selfdrive/controls/lib/lead_sensitivity.py` and `get_lead_custom` in `openpilot/selfdrive/controls/radard.py`.

## How stock radard picks a lead

Stock radard uses one fixed 50% camera confidence cutoff for everything. Each model frame (20 times a second) it runs these steps:

1. **Camera check.** The driving model reports a lead car and how sure it is. Below 50%, there is no lead.
2. **Radar pairing.** Above 50%, radard picks the radar point that best fits the camera's car. It keeps the pairing only if the point is within 5 m (or 25%) of the camera's distance and within 10 m/s of its speed. A paired lead uses radar distance and speed, which are more precise.
3. **Camera fallback.** If no radar point fits, radard follows the camera's own estimate of the car.
4. **Low-speed override.** When you are nearly stopped, a radar point straight ahead replaces the lead if it is closer. None of the custom settings touch this step.

Two weak spots follow from this. A real car at 40-50% confidence is ignored even when radar clearly sees it. And a guardrail point at 0 m/s can pass the step 2 checks on a ramp, so the planner brakes for a "stopped car" that isn't there.

## Each setting

| Setting | Range, default | What it changes | Trade-off |
| --- | --- | --- | --- |
| Camera-only Lead Confidence | 25-50%, 50% | Cutoff for following a car the radar can't confirm | Lower picks up cars sooner, but a camera-only false lead can brake you |
| Radar-matched Lead Confidence | 25-50%, 50% | Cutoff for trying to pair the camera's car with a radar point | Lower gets radar distance and speed sooner; radar confirmation limits false leads |
| Smooth Lead Handoffs | On/off, off | Keeps one car steady through source switches and short dropouts | Can hold a lost car up to 0.5 s |
| Ignore Stationary Radar Ghosts | On/off, off | Blocks a standing-still radar point from pairing with a moving car | A real stopped car is still caught when the camera sees it braking |

**Radar-matched should be at or below Camera-only.** Radar pairing starts at the lower of the two. If you set Radar-matched higher, it has no effect.

**Smooth Lead Handoffs details.** It tracks one car and predicts where it should be next frame. A new reading counts as the same car if it is within 1.5 m sideways, 4 m/s in speed, and 4-10 m in distance (10% of the gap). Then:

- **Release margin:** once you are following a car, it stays the lead until confidence drops 5 points below its cutoff.
- **Hold:** if a radar-backed lead drops out but the camera still sees the same car within 2 m of your path, the predicted lead is kept for up to 0.5 s.
- **Blend:** when the source changes (radar to camera, or a new radar point for the same car), or after a hold, good news is eased in over 0.75 s. Good news means the car looks farther, faster or braking less. Bad news (closer, slower, braking harder) applies immediately.
- **Reset:** a ghost block, the low-speed override or any settings change clears the tracked car.

**Ignore Stationary Radar Ghosts details.** A radar point counts as a ghost when it is moving under 1 m/s, the camera's car is moving over 5 m/s (11 mph), and the camera does not see that car braking harder than -1.5 m/s². radard then uses the camera's estimate instead, if it clears the camera-only cutoff. The thresholds come from real drives, where ramp ghosts showed up with the camera's car at 4.4 m/s and faster.

## How they work together

The two sliders decide when a car becomes a lead. The ghost guard protects the cheaper radar path from bad points. Smooth Lead Handoffs keeps the lead steady once you have one. Here is what changes with the recommended loadout (all on, camera 45%, radar 35%):

| Situation | Stock | Recommended loadout |
| --- | --- | --- |
| Far car at 38% camera confidence, radar sees it | Ignored until 50% | Paired with radar and followed |
| Car at 40% confidence, no radar point fits | Ignored | Still ignored: camera alone needs 45% |
| Lead you are following flickers between 32% and 55% | Lead drops in and out | Kept: it stays above 35% minus the 5-point margin |
| Same car's radar point is swapped or drops out briefly | Distance and speed jump between radar and camera estimates | Held up to 0.5 s; good news eased in over 0.75 s |
| On-ramp guardrail at 0 m/s, real car ahead at 25 mph has no radar point | Pairs the guardrail and brakes for a "stopped car" | Guardrail blocked; follows the camera's estimate |
| Car ahead brakes hard | Radar lead | Same: the guard steps aside, and bad news is never delayed |
| Creeping in stop-and-go traffic | Low-speed radar override | Same: untouched |

The gap between the sliders is the point. Between 35% and 45%, a car counts only if radar confirms it. Above 45%, the camera alone is enough. Lowering the radar slider is fairly safe because a stray camera guess also needs a radar point to match, and the ghost guard filters the most common bad match.

## Recommended loadout

| Setting | Value |
| --- | --- |
| Custom Lead Sensitivity | On |
| Camera-only Lead Confidence | 45% |
| Radar-matched Lead Confidence | 35% |
| Smooth Lead Handoffs | On |
| Ignore Stationary Radar Ghosts | On |

- **Camera 45%:** a small step down from stock 50%. Camera-only leads have no second sensor to check them, so this stays close to stock.
- **Radar 35%:** picks up radar-confirmed cars well before stock does, so the planner gets precise distance and speed earlier.
- **Smooth Lead Handoffs on:** the lower radar cutoff means more switching between radar and camera estimates at the edges. Handoffs keep those switches from showing up as small speed surges.
- **Ghosts on:** a lower radar cutoff means more radar pairings, so the guardrail filter matters more.

## What to watch for and turning it off

Lower cutoffs pick up cars sooner, at some risk of false leads. On the road, watch for:

- **Braking for a car that isn't in your lane**, especially parked cars or cars on a curve. This means the radar cutoff is too low.
- **Late pickup of a car ahead** compared with stock. This means a cutoff is too high.
- **A small surge or brake tap when the lead changes**, such as after a lane change ahead of you. Handoffs should prevent this.
- **Braking on ramps or curves with nothing stopped ahead.** The ghost guard should prevent this.

To review a drive, the standard lead data in `radarState` shows whether each lead was radar-backed, its radar point, distance, speed and model confidence.

**Turning it off:** turn off Custom Lead Sensitivity. radard rereads settings once a second and goes straight back to stock behavior. Slider values are kept for next time.

## Credits

The adjustable confidence cutoff is adapted from FrogPilot's Lead Detection Sensitivity (FrogAi, MIT).
