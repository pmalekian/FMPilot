"""Check the spliced model against the originals on random inputs (ONNX runtime, CPU).

1. Everything except action[0] and action[2] must equal Rebel Legion's output exactly.
2. action[0] and action[2] must equal OPM16D's own pipeline (its vision -> 25-frame features -> its on-policy).
3. Sanity: OPM16D's split vision + off-policy must reproduce Rebel Legion's plan, proving the shared networks match
   and that the feature buffer is assembled the same way.
"""
import pickle
import codecs
import sys

import numpy as np
import onnx
import onnxruntime as ort

# usage: verify_splice.py splice.onnx rlm.onnx opm16d_models_dir
# opm16d_models_dir holds OPM16D's split files: opm16d_vision.onnx, opm16d_off_policy.onnx, opm16d_on_policy.onnx
SPLICE, RLM, D = sys.argv[1], sys.argv[2], sys.argv[3]


def session(path):
  so = ort.SessionOptions()
  so.log_severity_level = 3
  return ort.InferenceSession(path, so, providers=["CPUExecutionProvider"])


def slices(path):
  meta = {p.key: p.value for p in onnx.load(path, load_external_data=False).metadata_props}
  return pickle.loads(codecs.decode(meta["output_slices"].encode(), "base64"))


def np_type(t):
  return {"tensor(float16)": np.float16, "tensor(float)": np.float32, "tensor(uint8)": np.uint8}[t]


import os
rng = np.random.default_rng(int(os.environ.get('SEED', '0')))
s_splice, s_rlm = session(SPLICE), session(RLM)
feeds = {}
for i in s_rlm.get_inputs():
  shape = i.shape
  if i.name in ("img", "big_img"):
    feeds[i.name] = rng.integers(0, 255, size=shape).astype(np_type(i.type))
  elif i.name == "traffic_convention":
    feeds[i.name] = np.array([[1, 0]], dtype=np_type(i.type))
  elif i.name == "desire_pulse":
    feeds[i.name] = np.zeros(shape, dtype=np_type(i.type))
    if os.environ.get('DESIRE'):
      feeds[i.name][0, -1, int(os.environ['DESIRE'])] = 1  # e.g. 2 = turn right
  elif i.name == "action_t":
    feeds[i.name] = np.array([[0.3, 0.4]], dtype=np_type(i.type))
  else:  # features_buffer
    feeds[i.name] = (rng.standard_normal(size=shape) * 0.5).astype(np_type(i.type))

out_s = s_splice.run(None, feeds)[0].astype(np.float32)
out_r = s_rlm.run(None, feeds)[0].astype(np.float32)
sl = slices(RLM)
a0 = sl["action"].start
mask = np.ones(out_r.shape[1], dtype=bool)
mask[[a0, a0 + 2]] = False
print("1. non-action outputs identical to Rebel Legion:", bool(np.array_equal(out_s[0, mask], out_r[0, mask])))
print("   accel mean/std taken from Rebel Legion:", out_s[0, a0 + 1] == out_r[0, a0 + 1], out_s[0, a0 + 3] == out_r[0, a0 + 3])

# OPM16D's own pipeline
s_vis, s_off, s_on = session(f"{D}/opm16d_vision.onnx"), session(f"{D}/opm16d_off_policy.onnx"), session(f"{D}/opm16d_on_policy.onnx")
vis = s_vis.run(None, {"img": feeds["img"], "big_img": feeds["big_img"]})[0].astype(np.float32)
vsl = slices(f"{D}/opm16d_vision.onnx")
hidden = vis[0, vsl["hidden_state"]]
fb_type = np_type(s_on.get_inputs()[3].type) if s_on.get_inputs()[3].name == "features_buffer" else np.float16
fb25 = np.concatenate([feeds["features_buffer"].astype(np.float32), hidden[None, None, :]], axis=1).astype(fb_type)
pol_feeds = {}
for i in s_on.get_inputs():
  pol_feeds[i.name] = fb25 if i.name == "features_buffer" else feeds[i.name].astype(np_type(i.type))
on = s_on.run(None, pol_feeds)[0].astype(np.float32)
print(f"2. curvature from OPM16D's on-policy: spliced {out_s[0, a0]:.5f} vs OPM16D {on[0, 0]:.5f}; std {out_s[0, a0 + 2]:.5f} vs {on[0, 2]:.5f}")
print(f"   (Rebel Legion's own curvature here: {out_r[0, a0]:.5f})")

off_feeds = {i.name: (fb25 if i.name == "features_buffer" else feeds[i.name].astype(np_type(i.type))) for i in s_off.get_inputs()}
off = s_off.run(None, off_feeds)[0].astype(np.float32)
osl = slices(f"{D}/opm16d_off_policy.onnx")
plan_opm = off[0, osl["plan"]]
plan_rlm = out_r[0, sl["plan"]]
print(f"3. plan: OPM16D split pipeline vs Rebel Legion, max abs difference {np.abs(plan_opm - plan_rlm).max():.5f} "
      f"(plan values span {np.abs(plan_rlm).max():.2f})")
