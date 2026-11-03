import numpy as np
import onnx
from onnx import helper, TensorProto, numpy_helper
import pytest

tinygrad = pytest.importorskip("tinygrad")
from tinygrad import Tensor
from tinygrad.nn.onnx import OnnxRunner

from openpilot.sunnypilot.modeld_v2 import fm_compile_fixes


def make_model(path):
  """out = (x @ W^T) + x, the shape of the point-policy residual (linear_2 + linear)."""
  rng = np.random.default_rng(0)
  w = numpy_helper.from_array(rng.standard_normal((4, 4)).astype(np.float32), "W")
  nodes = [helper.make_node("Gemm", ["x", "W"], ["linear_2"], transB=1), helper.make_node("Add", ["linear_2", "x"], ["out"])]
  graph = helper.make_graph(nodes, "g", [helper.make_tensor_value_info("x", TensorProto.FLOAT, [1, 4])],
                            [helper.make_tensor_value_info("out", TensorProto.FLOAT, [1, 4])], [w])
  onnx.save(helper.make_model(graph, opset_imports=[helper.make_opsetid("", 17)]), path)


def run(runner):
  x = Tensor(np.arange(4, dtype=np.float32).reshape(1, 4))
  return next(iter(runner({"x": x}).values())).numpy()


def test_contiguous_inserted_and_output_unchanged(tmp_path):
  path = str(tmp_path / "m.onnx")
  make_model(path)
  ref = run(OnnxRunner(path))
  patched = fm_compile_fixes.apply(OnnxRunner(path), ["linear_2"])
  assert [n.op for n in patched.graph_nodes] == ["Gemm", fm_compile_fixes.OP, "Add"]
  np.testing.assert_allclose(run(patched), ref, rtol=1e-6)


def test_missing_name_and_empty_env_do_nothing(tmp_path, monkeypatch):
  path = str(tmp_path / "m.onnx")
  make_model(path)
  r = OnnxRunner(path)
  before = r.graph_nodes
  assert fm_compile_fixes.apply(r, ["not_in_graph"]).graph_nodes == before
  monkeypatch.delenv(fm_compile_fixes.ENV, raising=False)
  assert fm_compile_fixes.apply(r).graph_nodes == before
