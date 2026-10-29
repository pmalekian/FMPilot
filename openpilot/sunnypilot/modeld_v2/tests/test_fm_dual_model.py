from types import SimpleNamespace

import numpy as np

from openpilot.cereal import custom, log
from openpilot.sunnypilot.modeld_v2 import fm_dual_model as fm
from openpilot.sunnypilot.modeld_v2.constants import ModelConstants, Plan

T = np.array(ModelConstants.T_IDXS)
N = ModelConstants.IDX_N
PLAN_WIDTH = 15


def make_plan(speed, curvature, lateral_offset=0.0):
  """A constant-speed, constant-curvature path, shape (1, N, PLAN_WIDTH)."""
  plan = np.zeros((1, N, PLAN_WIDTH), dtype=np.float32)
  x = speed * T
  plan[0, :, Plan.POSITION.start] = x
  plan[0, :, Plan.POSITION.start + 1] = curvature * x ** 2 / 2 + lateral_offset
  plan[0, :, Plan.VELOCITY.start] = speed
  plan[0, :, Plan.T_FROM_CURRENT_EULER.start + 2] = curvature * x
  plan[0, :, Plan.ORIENTATION_RATE.start + 2] = curvature * speed
  return plan


def make_output(speed, curvature, tag):
  out = {k: np.full(s, tag, dtype=np.float32) for k, s in {
    'plan_stds': (1, N, PLAN_WIDTH), 'lane_lines': (1, 4, N, 2), 'lane_lines_stds': (1, 4, N, 2), 'lane_lines_prob': (1, 8),
    'road_edges': (1, 2, N, 2), 'road_edges_stds': (1, 2, N, 2), 'lead': (1, 3, 6, 4), 'lead_stds': (1, 3, 6, 4),
    'lead_prob': (1, 3), 'desire_state': (1, 8), 'desire_pred': (1, 32), 'meta': (1, 55), 'pose': (1, 6),
    'desired_curvature': (1, 1)}.items()}
  out['plan'] = make_plan(speed, curvature)
  return out


class FakeParams:
  def __init__(self, values):
    self.values = values

  def get(self, key):
    return self.values.get(key)


def bundle(ref, name):
  return SimpleNamespace(ref=ref, displayName=name)


class TestResolveSlots:
  def test_empty_slots_use_selected(self):
    selected = bundle("sel", "Selected")
    lat, long = fm.resolve_slot_bundles(FakeParams({}), selected, [bundle("a", "A")])
    assert lat is selected and long is selected

  def test_slots_resolve_by_ref(self):
    a, b = bundle("a", "A"), bundle("b", "B")
    lat, long = fm.resolve_slot_bundles(FakeParams({fm.PARAM_LAT_REF: "a", fm.PARAM_LONG_REF: "b"}), bundle("sel", "S"), [a, b])
    assert lat is a and long is b

  def test_unknown_ref_falls_back_to_selected(self):
    selected = bundle("sel", "S")
    lat, _ = fm.resolve_slot_bundles(FakeParams({fm.PARAM_LAT_REF: "gone"}), selected, [bundle("a", "A")])
    assert lat is selected


class TestCompatibility:
  def test_same_shapes_ok(self):
    assert fm.check_compatible(make_output(20, 0, 1), make_output(20, 0, 2)) is None

  def test_shape_mismatch(self):
    other = make_output(20, 0, 2)
    other['lead'] = np.zeros((1, 2, 6, 4))
    assert "lead" in fm.check_compatible(make_output(20, 0, 1), other)

  def test_missing_key(self):
    other = make_output(20, 0, 2)
    del other['meta']
    assert "meta" in fm.check_compatible(make_output(20, 0, 1), other)


class TestMerge:
  def test_field_ownership(self):
    lat, long = make_output(20, 0.01, 1.0), make_output(15, 0.0, 2.0)
    merged = fm.merge_outputs(lat, long, fm.CURVE_SOURCE_LATERAL)
    for key in ('lane_lines', 'road_edges', 'desire_state', 'desire_pred', 'pose', 'desired_curvature'):
      assert merged[key] is lat[key], key
    for key in ('lead', 'lead_prob', 'meta'):
      assert merged[key] is long[key], key
    plan = merged['plan'][0]
    # distance and speed from the longitudinal model
    np.testing.assert_allclose(plan[:, Plan.POSITION.start], long['plan'][0, :, Plan.POSITION.start])
    np.testing.assert_allclose(plan[:, Plan.VELOCITY.start], 15)

  def test_path_geometry_follows_lateral_model(self):
    k = 0.01
    lat, long = make_output(20, k, 1.0), make_output(15, 0.0, 2.0)
    plan = fm.merge_outputs(lat, long, fm.CURVE_SOURCE_LATERAL)['plan'][0]
    x = plan[:, Plan.POSITION.start]
    inside = x <= 20 * T[-1]  # within the lateral path's length
    # lateral offset at each distance matches the lateral model's path at that distance
    lat_x, lat_y = lat['plan'][0, :, Plan.POSITION.start], lat['plan'][0, :, Plan.POSITION.start + 1]
    np.testing.assert_allclose(plan[:, Plan.POSITION.start + 1], np.interp(x, lat_x, lat_y), rtol=1e-5)
    # and stays close to the true curve (linear interpolation between the model's 33 points)
    np.testing.assert_allclose(plan[inside, Plan.POSITION.start + 1], k * x[inside] ** 2 / 2, atol=0.15)
    # curve speed (lateral source): turn rate = lateral curvature * longitudinal speed
    np.testing.assert_allclose(plan[:, Plan.ORIENTATION_RATE.start + 2], k * 15, rtol=1e-4)

  def test_longitudinal_curve_source(self):
    lat, long = make_output(20, 0.01, 1.0), make_output(15, 0.002, 2.0)
    plan = fm.merge_outputs(lat, long, fm.CURVE_SOURCE_LONGITUDINAL)['plan'][0]
    np.testing.assert_allclose(plan[:, Plan.ORIENTATION_RATE.start + 2], 0.002 * 15, rtol=1e-4)

  def test_standstill_uses_lateral_path_as_is(self):
    lat, long = make_output(0, 0.0, 1.0), make_output(0, 0.0, 2.0)
    lat['plan'][0, :, Plan.POSITION.start + 1] = 0.3
    plan = fm.merge_outputs(lat, long, fm.CURVE_SOURCE_LATERAL)['plan'][0]
    np.testing.assert_allclose(plan[:, Plan.POSITION.start + 1], 0.3)

  def test_inputs_not_modified(self):
    lat, long = make_output(20, 0.01, 1.0), make_output(15, 0.0, 2.0)
    before = long['plan'].copy()
    fm.merge_outputs(lat, long, fm.CURVE_SOURCE_LATERAL)
    np.testing.assert_array_equal(long['plan'], before)

  def test_merge_actions(self):
    lat = log.ModelDataV2.Action(desiredCurvature=0.02, desiredAcceleration=1.0, shouldStop=False)
    long = log.ModelDataV2.Action(desiredCurvature=-0.01, desiredAcceleration=-1.5, shouldStop=True)
    action = fm.merge_actions(lat, long)
    assert abs(action.desiredCurvature - 0.02) < 1e-6
    assert abs(action.desiredAcceleration - -1.5) < 1e-6
    assert action.shouldStop


class TestFeedback:
  def test_long_model_gets_lateral_curvature(self):
    buf = np.zeros((1, 100, 1), dtype=np.float32)
    long_model = SimpleNamespace(numpy_inputs={'prev_desired_curv': buf}, mlsim=False)
    fm.feed_back_curvature(long_model, {'desired_curvature': np.array([[0.03]])})
    assert abs(buf[0, -1, 0] - 0.03) < 1e-6

  def test_mlsim_untouched(self):
    buf = np.zeros((1, 100, 1), dtype=np.float32)
    long_model = SimpleNamespace(numpy_inputs={'prev_desired_curv': buf}, mlsim=True)
    fm.feed_back_curvature(long_model, {'desired_curvature': np.array([[0.03]])})
    assert buf[0, -1, 0] == 0


class TestStatus:
  def test_slow_pair_falls_back(self):
    status = fm.DualModelStatus()
    status.active = True
    assert not any(status.update(0.030) for _ in range(500))
    assert any(status.update(0.060) for _ in range(500))

  def test_notice_times_out(self):
    status = fm.DualModelStatus()
    status.fall_back("too slow")
    msg = custom.ModelDataV2SP.new_message()
    status.fill(msg)
    assert msg.fmDualModelFallback and not msg.fmDualModelActive
    for _ in range(int(fm.FALLBACK_NOTICE_TIME / 0.05) + 1):
      status.update(0.0)
    status.fill(msg)
    assert not msg.fmDualModelFallback


def test_short_name():
  assert fm.short_name("Terrible Super Fantastic Do Over Model (August 15, 2026)") == "TSFDO"
  assert fm.short_name("OP Model 16 Deep (June 03, 2026)") == "OP 16 Deep"
  assert fm.short_name("Rebellious Hope (May 1, 2026)") == "Rebellious Hope"
  assert fm.short_name("") == ""


def test_slot_options(tmp_path):
  def make(ref, name, index, file_name):
    return SimpleNamespace(ref=ref, displayName=name, index=index, models=[SimpleNamespace(artifact=SimpleNamespace(fileName=file_name))])
  (tmp_path / "a.pkl").touch()
  bundles = [make("a", "Alpha", 3, "a.pkl"), make("c", "Gamma", 5, "c.pkl"), make("d", "Delta", 1, "d.pkl")]
  options = fm.build_slot_options("a;c", bundles, str(tmp_path), ("d", ""))
  # default first; downloaded favorite kept; favorite not downloaded dropped; a slot's current value always kept
  assert [o["value"] for o in options] == ["", "a", "d"]
  assert options[0]["label"] == fm.SAME_AS_SELECTED_LABEL
