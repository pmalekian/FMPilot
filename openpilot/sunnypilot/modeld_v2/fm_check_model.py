#!/usr/bin/env python3
"""FM Pilot: run compiled driving model files on the comma (offroad) and report outputs that come out NaN.

  PYTHONPATH=/data/openpilot python3 /data/openpilot/openpilot/sunnypilot/modeld_v2/fm_check_model.py [model.pkl ...]

With no arguments it checks the custom (splice) model and the downloaded Rebel Legion model. Each file is run the way
modeld runs it (same loader and adapter) for 20 frames of random camera images. Run it with the car off: modeld must
not be running.
"""
import gc
import glob
import os
import re
import sys

os.environ['GMMU'] = '0'
import numpy as np

from openpilot.common.params import Params

CAMERAS = ((1344, 760), (1928, 1208))
FRAMES = 20


def default_paths() -> list[str]:
  paths = []
  if custom := Params().get("FM_CustomModelPath"):
    paths.append(custom)
  for f in sorted(glob.glob("/data/media/0/models/**/*rebel_legion*tinygrad.pkl*", recursive=True)):
    base = re.sub(r"\.(chunk\d+of\d+|chunkmanifest)$", "", f)
    if base not in paths:
      paths.append(base)
  return paths


def check(path: str) -> None:
  from openpilot.sunnypilot.modeld_v2.modeld import ModelState
  print(f"\n=== {path}")
  os.environ['COMBINED_MODEL_PKL'] = path
  model = None
  for cam_w, cam_h in CAMERAS:
    try:
      model = ModelState(cam_w, cam_h)
      break
    except FileNotFoundError:
      continue
  if model is None:
    print("  could not load (no matching camera warp)")
    return
  print(f"  format: {'native' if model.adapter.is_native else 'legacy'} {model._combined_model_type}, camera {cam_w}x{cam_h}")

  rng = np.random.default_rng(0)
  frames, transforms, inputs = model.adapter.get_dummy_inputs()
  if 'traffic_convention' in inputs:
    inputs['traffic_convention'][:] = 0
    inputs['traffic_convention'].flat[0] = 1
  if 'action_t' in inputs:
    inputs['action_t'][:] = 0.3

  nan_any: dict[str, bool] = {}
  outputs = {}
  for _ in range(FRAMES):
    for f in frames.values():
      f[:] = rng.integers(0, 256, f.shape, dtype=np.uint8)
    outputs = model.run(frames, transforms, inputs)
    for k, v in outputs.items():
      nan_any[k] = nan_any.get(k, False) or bool(np.isnan(np.asarray(v, dtype=np.float32)).any())

  bad = sorted(k for k, v in nan_any.items() if v)
  good = sorted(k for k, v in nan_any.items() if not v)
  print(f"  NaN outputs: {', '.join(bad) if bad else 'none'}")
  print(f"  OK outputs:  {', '.join(good)}")
  del model
  gc.collect()


if __name__ == "__main__":
  for p in (sys.argv[1:] or default_paths()):
    try:
      check(p)
    except Exception as e:  # keep going so one bad file does not hide the others
      print(f"  failed: {type(e).__name__}: {e}")
