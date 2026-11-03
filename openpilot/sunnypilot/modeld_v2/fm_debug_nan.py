#!/usr/bin/env python3
"""FM Pilot: find where a driving model's outputs turn NaN on the comma's GPU.

Runs an ONNX model with tinygrad directly (no compile step, no JIT) using the same flags modeld's models are compiled
with, then reports which output groups are NaN and the value range of every step in the policy head that feeds lane
lines, meta and pose. Run with the car off:

  cd /data/openpilot && PYTHONPATH=/data/openpilot DEV=QCOM IMAGE=1 FLOAT16=1 NOLOCALS=1 JIT_BATCH_SIZE=0 OPENPILOT_HACKS=1 \\
    python3 openpilot/sunnypilot/modeld_v2/fm_debug_nan.py /data/media/0/fm_models/rlm.onnx
"""
import sys

import numpy as np
from tinygrad import Device, Tensor
from tinygrad.nn.onnx import OnnxRunner

from openpilot.sunnypilot.modeld_v2.compile_modeld import make_metadata_dict

# the head between the vision features ("linear", the hidden state) and the lane/meta/pose outputs ("add_13")
HEAD = ['linear', 'layer_norm', 'linear_1', 'gelu_19', 'linear_2', 'add_12', 'layer_norm_1', 'linear_3', 'gelu_20',
        'linear_4', 'add_13', 'linear_7', 'linear_10', 'mul_13']


def stats(t: Tensor) -> str:
  a = t.float().numpy().astype(np.float32)
  finite = a[np.isfinite(a)]
  rng = f"absmax {np.abs(finite).max():10.4f}" if finite.size else "absmax        n/a"
  return f"{rng}  NaN {int(np.isnan(a).sum()):5d}  inf {int(np.isinf(a).sum()):5d}  of {a.size}"


def main(path: str) -> None:
  print(f"device {Device.DEFAULT}, model {path}")
  meta = make_metadata_dict(path)
  runner = OnnxRunner(path)
  rng = np.random.default_rng(0)
  inputs = {}
  for name, spec in runner.graph_inputs.items():
    shape = tuple(s if isinstance(s, int) else 1 for s in spec.shape)
    if 'img' in name:
      inputs[name] = Tensor(rng.integers(0, 256, shape, dtype=np.uint8))
    else:
      arr = np.zeros(shape, dtype=np.float32)
      if name == 'traffic_convention':
        arr[..., 0] = 1
      elif name == 'action_t':
        arr[:] = 0.3
      elif name == 'features_buffer':
        arr[:] = rng.standard_normal(shape).astype(np.float32) * 0.1
      inputs[name] = Tensor(arr)

  out = next(iter(runner(inputs).values())).float().numpy().reshape(-1)
  print("\noutput groups (whole graph in one go):")
  for key, sl in meta['output_slices'].items():
    if isinstance(sl, slice) and key != 'pad':
      n = int(np.isnan(out[sl]).sum())
      print(f"  {key:28s} {'NaN ' + str(n) if n else 'ok'}")

  print("\npolicy head, step by step:")
  for name in HEAD:
    if name in runner.graph_values:
      print(f"  {name:14s} {stats(runner.graph_values[name])}")


if __name__ == "__main__":
  main(sys.argv[1])
