#!/usr/bin/env python3
"""FM Pilot: run compiled driving model files on the comma (offroad) and report outputs that come out NaN.

  PYTHONPATH=/data/openpilot python3 /data/openpilot/openpilot/sunnypilot/modeld_v2/fm_check_model.py [model.pkl ...]

With no arguments it checks the custom (splice) model and the downloaded Rebel Legion model. Each file is run the way
modeld runs it (same loader and adapter) for 20 frames of random camera images. Run it with the car off: modeld must
not be running.

  ... fm_check_model.py --compare custom.pkl reference.pkl

runs both on the same frames and prints how far each output differs. A splice should match its base model exactly
everywhere except action (the spliced steering).
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


def check(path: str, results: dict | None = None) -> bool:
  """True when every output is free of NaN. The last frame's outputs go into results."""
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
    return False
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
  if results is not None:
    results.update({k: np.asarray(v, dtype=np.float32).copy() for k, v in outputs.items()})
  del model
  gc.collect()
  return not bad


def compare(path_a: str, path_b: str) -> bool:
  a, b = {}, {}
  check(path_a, a)
  check(path_b, b)
  print(f"\n=== difference, {os.path.basename(path_a)} vs {os.path.basename(path_b)} (last frame)")
  ok = True
  for k in sorted(set(a) & set(b)):
    if a[k].shape != b[k].shape:
      print(f"  {k:28s} shapes differ {a[k].shape} vs {b[k].shape}")
      continue
    diff = float(np.nanmax(np.abs(a[k] - b[k]))) if a[k].size else 0.0
    scale = float(np.nanmax(np.abs(b[k]))) if b[k].size else 0.0
    close = diff <= 0.05 * max(scale, 1e-3) + 1e-3
    expected = k.startswith('action')
    ok = ok and (close or expected)
    print(f"  {k:28s} max diff {diff:10.4f}  (size {scale:9.4f})  {'match' if close else ('differs, expected' if expected else 'DIFFERS')}")
  print(f"  {'all outputs match apart from action' if ok else 'MISMATCH: something besides action differs'}")
  return ok


if __name__ == "__main__":
  if len(sys.argv) == 4 and sys.argv[1] == '--compare':
    sys.exit(0 if compare(sys.argv[2], sys.argv[3]) else 1)
  ok = True
  for p in (sys.argv[1:] or default_paths()):
    try:
      ok = check(p) and ok
    except Exception as e:  # keep going so one bad file does not hide the others
      print(f"  failed: {type(e).__name__}: {e}")
      ok = False
  sys.exit(0 if ok else 1)
