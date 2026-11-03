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


def rewrite(runner, variant):
  """Set runner.graph_nodes with add_12 (and for *_both also add_13) computed the given way."""
  nodes = runner.graph_nodes
  by_out = {n.outputs[0]: n for n in nodes}
  add, add13, lin = by_out['add_12'], by_out['add_13'], by_out['linear']
  ops = add.opset_id
  dtype = runner.graph_values[lin.inputs[1]].dtype  # dtype of the weights as tinygrad holds them

  def node(op, inputs, output, **opts):
    return OnnxNode(op, ops, tuple(inputs), (output,), opts)

  def eye_sum(a, b, out, tag):
    """out = a + b as Concat([a, b]) @ [[I], [I]], one matmul kernel instead of an elementwise add."""
    eye = f'fm_eye_{tag}'
    runner.graph_values[eye] = Tensor(np.vstack([np.eye(512), np.eye(512)]).astype(np.float32)).cast(dtype).realize()
    return [node('Concat', [a, b], f'fm_cat_{tag}', axis=-1), node('MatMul', [f'fm_cat_{tag}', eye], out)]

  out = []
  for n in nodes:
    if variant == 'contig_l2' and n is add:
      out += [node('FMContiguous', ['linear_2'], 'fm_l2c'), node('Add', ['fm_l2c', 'linear'], 'add_12')]
    elif variant == 'contig_both' and n is add:
      out += [node('FMContiguous', ['linear_2'], 'fm_l2c'), node('FMContiguous', ['linear'], 'fm_lc'),
              node('Add', ['fm_l2c', 'fm_lc'], 'add_12')]
    elif variant == 'dup_linear' and n is add:
      # recompute the hidden state separately for the residual (alpha and beta 2 then x0.5 so tinygrad cannot merge it with "linear")
      out += [OnnxNode('Gemm', lin.opset_id, tuple(lin.inputs), ('fm_lin2x',), {**dict(lin.opts), 'alpha': 2.0, 'beta': 2.0}),
              node('Mul', ['fm_lin2x', 'fm_half'], 'fm_lin_dup'), node('Add', ['linear_2', 'fm_lin_dup'], 'add_12')]
      runner.graph_values['fm_half'] = Tensor([0.5]).cast(dtype).realize()
    elif variant in ('eye_sum', 'eye_sum_both') and n is add:
      out += eye_sum('linear_2', 'linear', 'add_12', 'a')
    elif variant == 'eye_sum_both' and n is add13:
      out += eye_sum('linear_4', 'add_12', 'add_13', 'b')
    else:
      out.append(n)
  runner.graph_nodes = tuple(out)


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
      rewrite(runner, variant)
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
  main(sys.argv[1], sys.argv[2:] or ['contig_l2', 'contig_both', 'dup_linear', 'eye_sum', 'eye_sum_both'])
