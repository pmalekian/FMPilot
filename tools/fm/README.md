# FM Pilot tools

Log and model analysis scripts used while developing FM Pilot. They run on a PC (not the comma) with Python 3.12.

## Setup

```
python3 -m venv venv && . venv/bin/activate
pip install numpy pycapnp zstandard onnx onnxruntime
```
Run from this folder (`cd tools/fm`); the scripts import the fork's own `cereal` from the repo, so no openpilot build is needed.

Get logs from the comma (`scp -O comma@<ip>:/data/media/0/realdata/<route>--<segment>/rlog.zst .`) or Comma Connect.
Each segment is one minute; the model file modeld loaded is only logged in a drive's first segment (`--0`).

## Log tools

| Script | What it shows |
|---|---|
| `timeline.py rlog.zst [--step 2]` | Steering over time: model / final / achieved curvature, torque requested and sent, lane lines, hands, blinker, turn desire, faults |
| `events.py rlog.zst` | Version, model, FM-related settings, on-screen alerts, onroad events, fault and model-load log messages |
| `turns.py rlog.zst ...` | Every sharp turn: tightest radius asked vs driven, time at the torque limit, hand force and its direction |
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

Hand torque is the column's torsion bar, so it also reads the reaction of a loosely held wheel to openpilot's own torque.

## Model tools

| Script | What it does |
|---|---|
| `splice_model.py rlm.onnx opm16d_on_policy.onnx out.onnx` | Builds the OPM16D-steering + Rebel Legion-speed splice (shared vision and plan, both action networks) |
| `verify_splice.py splice.onnx rlm.onnx opm16d_dir` | Checks the splice on CPU against both originals |

On the comma, `openpilot/sunnypilot/modeld_v2/fm_build_custom_model.sh` compiles a custom ONNX (with the IMAGE=1
miscompile workaround) and `fm_check_model.py` checks it for NaN and, with `--compare`, against its base model. See
`FM_PILOT.md` at the repo root.
