# FM Pilot tools

Log and model analysis scripts used while developing FM Pilot. They run on a PC (not the comma) with Python 3.12.

## Setup

Windows PowerShell (on Linux/macOS: `python3 -m venv venv && . venv/bin/activate`):
```
python -m venv venv
.\venv\Scripts\Activate.ps1
pip install numpy pycapnp zstandard onnx onnxruntime pytest
```
Run from this folder (`cd tools/fm`); the scripts import the fork's own `cereal` from the repo, so no openpilot build is needed.

Get logs from Comma Connect or from the comma. To pull several segments of one drive at your PC's `PS C:\...>` prompt
(keeps the `<route>--<n>` folder names, which the tools read):
```
$r = "000001a3--c20ba54385"
foreach ($n in 3..7) { New-Item -ItemType Directory -Force "$r--$n" | Out-Null; scp -O "comma@192.168.1.246:/data/media/0/realdata/$r--$n/rlog.zst" "$r--$n/" }
```
Each segment is one minute. modeld logs the model file it loaded only in a drive's first segment (`--0`); for later
segments the tools work the model out from the settings saved at the start of the drive and mark it "(from settings)".

### Several segments of one drive

Every log tool takes any number of files, folders or wildcards (wildcards work in PowerShell too):
```
python turns.py 000001a3--c20ba54385--*\rlog.zst
python turns.py .                 # every rlog under this folder
python turns.py a\rlog.zst b\rlog.zst --each   # each file on its own
```
Segments of the same drive (same route name in the path) are put in order and joined on one clock, so a turn or a
straight stretch that crosses a segment boundary is seen whole. Times read `--3 +12.4s`: segment 3, 12.4 s in. A
missing segment is reported (`missing --5`) and never bridged. Different drives are analyzed separately. Files whose
path has no route name (e.g. a bare `rlog.zst`) are taken as one drive.

## Log tools

| Script | What it shows |
|---|---|
| `timeline.py rlog.zst ... [--step 2] [--start s] [--end s]` | Steering over time: model / final / achieved curvature, torque requested and sent, lane lines, hands, blinker, turn desire, faults |
| `events.py rlog.zst ...` | Version, model, FM-related settings, on-screen alerts, onroad events, fault and model-load log messages |
| `turns.py rlog.zst ...` | Every sharp turn: tightest radius asked vs driven, whether the model or the steering was late (see below), where torque was held back (at the limit, still climbing, held, unwinding; see below), hand force: with or against openpilot's torque while it steers, into the turn while you steer |
| `sway.py rlog.zst ...` | Straight-road sway: wheel, model, lane centering's share, dominant period |
| `dm.py rlog.zst ...` | Driver monitoring: % distracted and why (pose / eyes / phone), alert levels |

### Sign conventions (Rivian, this fork)

Checked against `controlsd.py`, `latcontrol_torque.py` and the Rivian car code on Nov 3, 2026.

| Signal | Positive means | Why |
|---|---|---|
| `controlsState.curvature`, `controlsState.desiredCurvature`, `modelV2.action.desiredCurvature`, `torqueState.actualLateralAccel` / `desiredLateralAccel` | **right** | Model outputs are in the calibrated frame (forward, right, down), and controlsd / latcontrol_torque compute curvature as `-VM.calc_curvature(steeringAngleDeg)` |
| `modelV2.laneLines[i].y`, `roadEdges[i].y` | right of the camera | Same frame. `laneLines[1]` is the near left line, `[2]` the near right |
| `carState.steeringAngleDeg` | **left** | opendbc convention; raw `EPAS_InternalSas`. The minus above makes curvature right-positive |
| `carState.steeringTorque` (hands / column) | **left** | Raw `EPAS_TorsionBarTorque`. Same sign as the torque command: `apply_driver_steer_torque_limits` relies on that |
| `carControl.actuators.torque`, `carOutput.actuatorsOutput.torque`, `torqueState.output` | **left** | latcontrol_torque works right-positive internally, then returns and logs `-output_torque` ("left is positive") |

The angle and hand-torque signs come straight from the EPAS, so the code can only assume them; lateral control would not
work at all if the angle were flipped. To confirm in a log: with openpilot off, turn the wheel left by hand and both
should read positive.

"Model asked" in `turns.py`, `timeline.py` and `sway.py` comes from `fmlog.model_requests`: `modelV2.action.desiredCurvature`
while openpilot steers, and the raw `modelDataV2SP.modelCurvature` while it doesn't. Since Nov 8, 2026 modeld seeds the
smoothed action from the truck's own curvature whenever lateral is off (openpilot #39091), so on manual stretches the
action value leans toward your steering. Logs from before the Nov 5 Blendful build have no `modelCurvature` and keep the
action value.

Hand torque is the column's torsion bar, so it also reads the reaction of a loosely held wheel to openpilot's own torque.
That reaction always points against openpilot's torque. So `turns.py` splits the hands by what openpilot was sending:
*with* openpilot's torque is certainly you helping, while *against* it (marked `*`) is you resisting **or** just the
wheel's reaction, and this sensor can't tell those apart. While you steer alone, *into the turn* is your own effort.

### Torque line in `turns.py`

Each turn has a second line, `torque: peak sent 506 of 551 limit | at limit … climbing … held … unwinding …`, as shares of
the time openpilot was steering in that turn (counts, left-positive):

| Share | Means |
|---|---|
| at limit | sent torque within 5 counts of the limit in force, and openpilot asked for at least that much |
| climbing | asked for more than was sent (by over 20 counts) and torque still rising: the ramp (3, or 5 per message with Faster Torque Ramp) is the limit, not the cap |
| held | asked for more than was sent and torque not rising: something else holds it back (hands on the wheel, the driver-torque limiter) |
| unwinding | more torque still being sent than asked for, or the opposite way: the same ramp, on the way down |

The limit is `carStateSP.fmSteerCap` frame by frame, so Max Torque, the 95% cap past 90° and the speed curve all count.
"Asked" is `carControl.actuators.torque` times the base speed curve (`STEER_MAX_LOOKUP`), and "sent" is
`carOutput.actuatorsOutput.torqueOutputCan`. Older logs without them fall back to the speed curve and the logged torque, and the
line says so. Before Nov 11, 2026 this column was `controlsState` torque at or above 95% of the base 385 curve (the
request, not what was sent), which read high once Max Torque was on.

### Timing line in `turns.py`

Each turn also has `timing: truck turned in at --1 +24.7s | model asked +1.1 s, openpilot asked -0.5 s | model late`:
when the model's request, openpilot's request (after Turn Lead / Hold / Boost) and the truck's own curve each first
reached 30% of the turn's tightest driven curve, looking back up to 4 s before the turn. Model and openpilot times are
relative to the truck (minus = before it); "never" means that request never got that tight.

| Verdict | Means |
|---|---|
| model late | the model asked less than 0.3 s before the truck turned in, or never: only Turn Lead / Hold (or you) start the turn on time |
| wheel late | the model asked more than 0.8 s before the truck turned in: the request was there and the steering followed slowly |
| on time | in between |

The 0.3 / 0.8 s lines are working guesses for sorting turns. Drive known corners with Turn Lead off to see which kind of
late they are.

### Release line in `turns.py`

Turns where you pressed the wheel into the turn (hand torque toward it, the truck already turning that way) for at least
0.3 s while openpilot steered get a third line, for Release Guard:
```
release: you pressed into the turn 2.0 s, openpilot against you max 472 avg 188 (guard off) | let go at --0 +34.9s:
  433 still pushing out, 396 at center, wheel  39 deg past center, pushed past center 0.78 s
```
Against you is sent torque out of the turn while you press (counts). After you let go: what was still being sent out of
the turn, what was sent when the wheel crossed center, how far past center it went within 2 s, and how long openpilot's
torque kept pushing it that way after it crossed. With Release Guard on, `(guard on 95% of it, cut up to 8.1 m/s^2)`
from `carControlSP.releaseGuard`. The longest press in the turn is the one checked, so catching an overshoot afterwards
doesn't replace it. A turn that started in a segment you didn't load may not be found as a turn.

Tests: `python -m pytest tests` (small synthetic logs; no device or real logs needed).
