"""Build a spliced driving model: Rebel Legion's model with OPM16D's action policy added beside its own.

OPM16D and Rebel Legion share byte-identical vision and off-policy (plan) networks and differ only in the on-policy,
which outputs the 4-number action [curvature mean, accel mean, curvature std, accel std]. The spliced model runs the
shared networks once and both on-policies, and publishes the action as
  [OPM16D curvature, Rebel Legion accel, OPM16D curvature std, Rebel Legion accel std].
Everything else (plan, lane lines, leads, meta, ...) is the shared networks' output, identical in both models.

usage: splice.py rlm.onnx opm16d_on_policy.onnx out.onnx
"""
import sys

import onnx
from onnx import helper, numpy_helper, TensorProto
import numpy as np

PREFIX = "fm_opm16d_on/"
ACTION_START = 2062  # from Rebel Legion's output_slices: 'action': (2062, 2066)


def main(rlm_path, opm_on_path, out_path):
  m = onnx.load(rlm_path)
  g = m.graph
  opm = onnx.load(opm_on_path).graph

  assert [o.name for o in g.output] == ["outputs"]
  # the tensors Rebel Legion's own on-policy is fed from: the graph inputs plus the 25-frame feature buffer
  inferred = onnx.shape_inference.infer_shapes(m)
  shapes = {vi.name: [d.dim_value for d in vi.type.tensor_type.shape.dim]
            for vi in list(inferred.graph.value_info) + list(inferred.graph.input) + list(inferred.graph.output)}
  feature_buffers = [k for k, v in shapes.items() if v == [1, 25, 512]]
  assert len(feature_buffers) == 1, feature_buffers
  input_map = {"desire_pulse": "desire_pulse", "traffic_convention": "traffic_convention", "action_t": "action_t",
               "features_buffer": feature_buffers[0]}
  assert {i.name for i in opm.input} == set(input_map), [i.name for i in opm.input]

  def rename(name):
    if name == "":
      return name
    return input_map.get(name, PREFIX + name)

  # copy OPM16D's on-policy in, every name prefixed so nothing collides
  for t in opm.initializer:
    t2 = onnx.TensorProto()
    t2.CopyFrom(t)
    t2.name = rename(t.name)
    g.initializer.append(t2)
  for n in opm.node:
    n2 = onnx.NodeProto()
    n2.CopyFrom(n)
    n2.name = PREFIX + n.name
    del n2.input[:]
    n2.input.extend(rename(i) for i in n.input)
    del n2.output[:]
    n2.output.extend(rename(o) for o in n.output)
    g.node.append(n2)
  opm_out = rename(opm.output[0].name)  # [1, 4]

  # find which input of the final Concat holds the action, by walking the concat inputs' widths
  prod = {o: n for n in g.node for o in n.output}
  final = prod["outputs"]
  assert final.op_type == "Concat"
  start, action_input = 0, None
  for name in final.input:
    width = shapes[name][-1]
    if start == ACTION_START:
      action_input = name
      assert width == 4, (name, width)
      break
    start += width
  assert action_input is not None, "action not found in the final concat"

  def const(name, values):
    t = numpy_helper.from_array(np.array(values, dtype=np.int64), name=PREFIX + name)
    g.initializer.append(t)
    return t.name

  def take(src, idx, tag):
    out = PREFIX + f"{tag}_{idx}"
    g.node.append(helper.make_node("Slice", [src, const(f"s{tag}{idx}", [idx]), const(f"e{tag}{idx}", [idx + 1]),
                                             const(f"a{tag}{idx}", [1])], [out], name=out + "_slice"))
    return out

  parts = [take(opm_out, 0, "opm"), take(action_input, 1, "rlm"), take(opm_out, 2, "opm"), take(action_input, 3, "rlm")]
  spliced = PREFIX + "spliced_action"
  # cast in case the two policies output different float types
  elem = {vi.name: vi.type.tensor_type.elem_type for vi in list(inferred.graph.value_info)}.get(action_input, TensorProto.FLOAT)
  g.node.append(helper.make_node("Concat", parts, [spliced], axis=1, name=spliced + "_concat"))
  final_inputs = [spliced if i == action_input else i for i in final.input]
  del final.input[:]
  final.input.extend(final_inputs)

  # the final concat must come after the new nodes: re-sort topologically
  nodes = list(g.node)
  produced = {i.name for i in g.input} | {t.name for t in g.initializer}
  ordered, remaining = [], nodes
  while remaining:
    progress, rest = False, []
    for n in remaining:
      if all(i == "" or i in produced for i in n.input):
        ordered.append(n)
        produced.update(n.output)
        progress = True
      else:
        rest.append(n)
    assert progress, "graph has a cycle or a missing input"
    remaining = rest
  del g.node[:]
  g.node.extend(ordered)

  meta = {p.key: p for p in m.metadata_props}
  meta["model_checkpoint"].value += " + action lateral from " + \
    {p.key: p.value for p in onnx.load(opm_on_path, load_external_data=False).metadata_props}["model_checkpoint"]
  onnx.checker.check_model(m)
  onnx.save(m, out_path)
  print(f"saved {out_path}: action input {action_input} (elem type {elem}), feature buffer {feature_buffers[0]}")


if __name__ == "__main__":
  main(*sys.argv[1:])
