"""
FM Pilot: compile-time workaround for a miscompile on the comma's GPU (DEV=QCOM with IMAGE=1).

In the gen 12 supercombo models (Rebel Legion, OPM16D family) the point-policy head adds the hidden state to the
output of a matmul (add_12 = linear_2 + linear). When tinygrad fuses that matmul and add into one kernel on the comma,
the result is garbage, and lane lines, road edges, meta and pose come out NaN (so locationd rejects the model). The
inputs are fine and IMAGE=0 is correct but about 6x too slow. Making the matmul output its own buffer before the add
(a contiguous) gives correct results with the image path kept.

FM_CONTIGUOUS_OUTPUTS names the node outputs to make contiguous (comma separated); names not in the graph are ignored.
"""
import dataclasses
import os

ENV = "FM_CONTIGUOUS_OUTPUTS"
OP = "FMContiguous"


def apply(runner, names=None):
  if names is None:
    names = [n.strip() for n in os.environ.get(ENV, "").split(",") if n.strip()]
  wanted = set(names)
  if not wanted:
    return runner

  from tinygrad.nn.onnx import OnnxNode
  nodes, done = [], []
  for n in runner.graph_nodes:
    hit = [o for o in n.outputs if o in wanted]
    if not hit:
      nodes.append(n)
      continue
    nodes.append(dataclasses.replace(n, outputs=tuple(f"{o}__fm_raw" if o in wanted else o for o in n.outputs)))
    for o in hit:
      nodes.append(OnnxNode(OP, n.opset_id, (f"{o}__fm_raw",), (o,), {}))
      done.append(o)

  if done:
    runner.onnx_ops = {**runner.onnx_ops, OP: lambda x: x.contiguous()}
    runner.graph_nodes = tuple(nodes)
    print(f"FM Pilot: contiguous after {', '.join(done)}")
  return runner
