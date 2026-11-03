#!/usr/bin/env python3
"""FM Pilot: try equivalent rewrites of the point-policy residual add (add_12 = linear_2 + linear) that the comma's GPU
gets wrong with IMAGE=1, and report which rewrites give clean outputs. Run with the car off:

  cd /data/openpilot && PYTHONPATH=/data/openpilot DEV=QCOM IMAGE=1 FLOAT16=1 NOLOCALS=1 JIT_BATCH_SIZE=0 OPENPILOT_HACKS=1 \\
    python3 openpilot/sunnypilot/modeld_v2/fm_debug_variants.py /data/media/0/fm_models/rlm.onnx
"""
import sys
import time

import numpy as np
from tinygrad import Tensor
from tinygrad.nn.onnx import OnnxNode, OnnxRunner

from openpilot.sunnypilot.modeld_v2.compile_modeld import make_metadata_dict

POINT_POLICY_OUTPUTS = ('lane_lines', 'lane_lines_prob', 'road_edges', 'meta', 'desire_pred', 'pose', 'wide_from_device_euler',
                        'road_transform')
FLOAT, FLOAT16 = 1, 10


def rewrite(nodes, variant):
  """Return the node list with add_12 computed the given way."""
  by_out = {n.outputs[0]: n for n in nodes}
  add, lin2 = by_out['add_12'], by_out['linear_2']
  ops = add.opset_id

  def node(op, inputs, output, **opts):
    return OnnxNode(op, ops, tuple(inputs), (output,), opts)

  out = []
  for n in nodes:
    if variant == 'contiguous' and n is add:
      out += [node('FMContiguous', ['linear'], 'fm_linear_c'), node('Add', ['linear_2', 'fm_linear_c'], 'add_12')]
    elif variant == 'gemm_c' and n is lin2:
      out.append(OnnxNode('Gemm', lin2.opset_id, (lin2.inputs[0], lin2.inputs[1], 'linear'), ('add_12',), dict(lin2.opts)))
    elif variant == 'gemm_c' and n is add:
      continue
    elif variant == 'swap' and n is add:
      out.append(node('Add', ['linear', 'linear_2'], 'add_12'))
    elif variant == 'fp32' and n is add:
      out += [node('Cast', ['linear'], 'fm_a', to=FLOAT), node('Cast', ['linear_2'], 'fm_b', to=FLOAT),
              node('Add', ['fm_b', 'fm_a'], 'fm_s'), node('Cast', ['fm_s'], 'add_12', to=FLOAT16)]
    elif variant == 'sub_neg' and n is add:
      out += [node('Neg', ['linear'], 'fm_n'), node('Sub', ['linear_2', 'fm_n'], 'add_12')]
    else:
      out.append(n)
  return tuple(out)


def make_inputs(runner):
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
  return inputs


def main(path, variants):
  slices = make_metadata_dict(path)['output_slices']
  reference = None
  for variant in variants:
    st = time.monotonic()
    try:
      runner = OnnxRunner(path)
      runner.onnx_ops = {**runner.onnx_ops, 'FMContiguous': lambda x: x.contiguous()}
      runner.graph_nodes = rewrite(runner.graph_nodes, variant)
      out = next(iter(runner(make_inputs(runner)).values())).float().numpy().reshape(-1)
    except Exception as e:
      print(f"{variant:11s} failed: {type(e).__name__}: {e}")
      continue
    nan = sum(int(np.isnan(out[slices[k]]).sum()) for k in POINT_POLICY_OUTPUTS)
    pp = np.concatenate([out[slices[k]] for k in POINT_POLICY_OUTPUTS])
    if reference is None and nan == 0:
      reference = pp
    diff = f"max diff vs first clean {np.nanmax(np.abs(pp - reference)):.4f}" if reference is not None and nan == 0 else ""
    print(f"{variant:11s} {'CLEAN' if nan == 0 else f'NaN {nan}':10s} {diff}  ({time.monotonic() - st:.0f} s)", flush=True)


if __name__ == "__main__":
  main(sys.argv[1], sys.argv[2:] or ['baseline', 'contiguous', 'gemm_c', 'swap', 'fp32', 'sub_neg'])
