#!/usr/bin/env python3
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from collections.abc import Callable
import os
os.environ['GMMU'] = '0'
import numpy as np
import threading
import time
from setproctitle import setproctitle

import openpilot.cereal.messaging as messaging
from openpilot.common.hardware import COMMA_HARDWARE
from openpilot.selfdrive.modeld.helpers import chestnut_present
from openpilot.cereal import log
from opendbc.car.structs import car
from openpilot.cereal.services import SERVICE_LIST
from openpilot.cereal.messaging import PubMaster, SubMaster
from openpilot.cereal.visionipc import VisionStreamType
from msgq.visionipc import VisionIpcClient, VisionBuf
from opendbc.car.car_helpers import get_demo_car_params
from openpilot.common.file_chunker import open_file_chunked
from openpilot.common.swaglog import cloudlog
from openpilot.common.params import Params
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import config_realtime_process, DT_MDL
from openpilot.common.transformations.camera import DEVICE_CAMERAS
from openpilot.common.transformations.model import get_warp_matrix
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper
from openpilot.selfdrive.controls.lib.drive_helpers import get_accel_from_plan, smooth_value
from openpilot.selfdrive.modeld.modeld import ChestnutGpuState

from openpilot.sunnypilot.modeld_v2.fill_model_msg import fill_model_msg, fill_pose_msg, PublishState, get_curvature_from_output
from openpilot.sunnypilot.modeld_v2.parse_model_outputs import Parser
from openpilot.sunnypilot.modeld_v2.constants import ModelConstants, Plan
from openpilot.sunnypilot.modeld_v2.meta_helper import load_meta_constants
from openpilot.sunnypilot.modeld_v2.camera_offset_helper import CameraOffsetHelper
from openpilot.sunnypilot.livedelay.helpers import get_lat_delay
from openpilot.sunnypilot.modeld_v2.modeld_base import ModelStateBase
from openpilot.sunnypilot.modeld_v2.helpers import load_oob
from openpilot.sunnypilot.modeld_v2.model_adapters import get_model_adapter
from openpilot.sunnypilot.models.helpers import get_active_bundle
from openpilot.sunnypilot.models.fetcher import get_cached_bundles
from openpilot.sunnypilot.modeld_v2 import fm_dual_model as fm_dual
from openpilot.sunnypilot.selfdrive.controls.lib.relc import RoadEdgeLaneChangeController

PROCESS_NAME = "openpilot.selfdrive.modeld.modeld_tinygrad"
BIG_MODEL_TIMEOUT = 60


def _pkl_exists(path):
  from openpilot.common.file_chunker import get_manifest_path
  return os.path.exists(path) or os.path.exists(get_manifest_path(path))


def _find_driving_pkl(bundle):
  if (override := os.environ.get('COMBINED_MODEL_PKL')) and _pkl_exists(override):
    return override
  if bundle is None or not bundle.models:
    return None
  from openpilot.common.hardware.hw import Paths
  model_root = Paths.model_root()

  pkl_name = bundle.models[0].artifact.fileName
  pkl_path = os.path.join(model_root, pkl_name)
  if _pkl_exists(pkl_path):
    return pkl_path
  return None


class FrameMeta:
  frame_id: int = 0
  timestamp_sof: int = 0
  timestamp_eof: int = 0

  def __init__(self, vipc=None):
    if vipc is not None:
      self.frame_id, self.timestamp_sof, self.timestamp_eof = vipc.frame_id, vipc.timestamp_sof, vipc.timestamp_eof


class ModelState(ModelStateBase):
  inputs: dict[str, np.ndarray]
  prev_desire: np.ndarray

  def __init__(self, cam_w: int, cam_h: int, chestnut: bool = False, bundle=None):
    ModelStateBase.__init__(self)

    env_pkl = os.environ.get('COMBINED_MODEL_PKL')
    if bundle is not None:  # FM Pilot: a specific bundle for a dual model slot
      model_bundle = bundle
    elif env_pkl and os.path.exists(env_pkl):
      model_bundle = None
    else:
      model_bundle = get_active_bundle(chestnut=chestnut)
    self.bundle = model_bundle
    self.generation = model_bundle.generation if model_bundle is not None else None
    overrides = {override.key: override.value for override in model_bundle.overrides} if model_bundle else {}

    self.LAT_SMOOTH_SECONDS = float(overrides.get('lat', ".0"))
    self.LONG_SMOOTH_SECONDS = float(overrides.get('long', ".0"))
    self.MIN_LAT_CONTROL_SPEED = 0.3
    self.PLANPLUS_CONTROL: float = 1.0
    self.chestnut = chestnut

    pkl_path = _find_driving_pkl(model_bundle)
    assert pkl_path is not None, f"No driving pkl found for {'chestnut' if chestnut else 'small model'} — all models must be compiled with compile_modeld.py"
    self._init_combined(pkl_path, cam_w, cam_h, model_bundle)

  def _init_combined(self, pkl_path, cam_w, cam_h, bundle):
    cloudlog.warning(f"loading combined pkl: {pkl_path}")
    jits = load_oob(open_file_chunked(pkl_path))

    metadata = jits['metadata']
    self.WARP_DEV = metadata.get('warp_dev', 'QCOM') if COMMA_HARDWARE else 'CPU'
    self.DEV = ('AMD' if self.chestnut else 'QCOM') if COMMA_HARDWARE else 'CPU'
    self.QUEUE_DEV = self.DEV
    self.adapter = get_model_adapter(jits, cam_w, cam_h, self.DEV, self.QUEUE_DEV, self.WARP_DEV, self.chestnut)
    self.vision_output_slices = self.adapter.vision_output_slices
    self.policy_output_slices = self.adapter.policy_output_slices
    self._policy_slices_list = self.adapter._policy_slices_list
    self._combined_model_type = self.adapter._combined_model_type
    self._vision_input_names = self.adapter._vision_input_names
    self.numpy_inputs = self.adapter.numpy_inputs
    self._policy_keys = self.adapter._policy_keys
    self._has_on_policy = self.adapter._has_on_policy
    self._desire_key = self.adapter._desire_key
    self._road_key = self.adapter._road_key
    self._wide_key = self.adapter._wide_key
    self.frame_buf_params = self.adapter.frame_buf_params

    is_20hz = bundle.is20hz if bundle else self._combined_model_type in ('split', 'multi_policy')
    if is_20hz:
      from openpilot.sunnypilot.models.split_model_constants import SplitModelConstants
      self.constants = SplitModelConstants()
    else:
      self.constants = ModelConstants()

    self.parser = Parser()
    self.prev_desire = np.zeros(self.constants.DESIRE_LEN, dtype=np.float32)

  def warmup(self) -> dict[str, np.ndarray] | None:
    dummy_frames, transforms, dummy_inputs = self.adapter.get_dummy_inputs()
    outputs = self.run(dummy_frames, transforms, dummy_inputs)
    self.adapter.reset_warmup_buffers()
    self.prev_desire[:] = 0
    return outputs

  @property
  def mlsim(self) -> bool:
    return bool(self.generation is not None and self.generation >= 11)

  @property
  def vision_input_names(self) -> list[str]:
    return self._vision_input_names

  @property
  def desire_key(self) -> str:
    return self._desire_key

  def run(self, bufs: dict[str, VisionBuf], transforms: dict[str, np.ndarray],
          inputs: dict[str, np.ndarray],
          after_enqueue: Callable[[], None] | None = None) -> dict[str, np.ndarray] | None:
    self.adapter.copy_frames(bufs)

    desire_key = self.desire_key
    inputs[desire_key][0] = 0
    (self.numpy_inputs[desire_key].flat if self.adapter.is_native else self.numpy_inputs[desire_key])[:] = \
      np.where(inputs[desire_key] - self.prev_desire > .99, inputs[desire_key], 0)
    self.prev_desire[:] = inputs[desire_key]

    for key in ('traffic_convention', 'lateral_control_params', 'action_t'):
      if key in self.numpy_inputs and key in inputs:
        self.numpy_inputs[key][:] = inputs[key]

    if self.adapter.is_native:
      for i, key in enumerate(self._vision_input_names):
        if key in transforms:
          self.numpy_inputs['tfm'][i] = transforms[key].reshape(3, 3)
    else:
      self.numpy_inputs['tfm'][:, :] = transforms[self._road_key].reshape(3, 3)
      self.numpy_inputs['big_tfm'][:, :] = transforms[self._wide_key].reshape(3, 3)

    raw_outputs = self.adapter.run()

    if after_enqueue is not None:
      after_enqueue()

    if self._combined_model_type == 'supercombo':
      model_output = raw_outputs.numpy().flatten()
      if self.chestnut and not np.all(np.isfinite(model_output)):
        raise RuntimeError("model output not finite")
      sliced = {k: model_output[np.newaxis, v] for k, v in self.vision_output_slices.items()}
      outputs = self.parser.parse_outputs(sliced)
      if 'prev_feat' in self.numpy_inputs and 'hidden_state' in self.vision_output_slices:
        (self.numpy_inputs['prev_feat'].flat if self.adapter.is_native else self.numpy_inputs['prev_feat'])[:] = \
          model_output[self.vision_output_slices['hidden_state']]
    else:
      vision_output = raw_outputs[0].numpy().flatten()
      vision_sliced = {k: vision_output[np.newaxis, v] for k, v in self.vision_output_slices.items()}
      outputs = self.parser.parse_vision_outputs(vision_sliced)

      if 'prev_feat' in self.numpy_inputs and 'hidden_state' in self.vision_output_slices:
        (self.numpy_inputs['prev_feat'].flat if self.adapter.is_native else self.numpy_inputs['prev_feat'])[:] = \
          vision_output[self.vision_output_slices['hidden_state']]

      for i, policy_slices in enumerate(self._policy_slices_list):
        policy_output = raw_outputs[i + 1].numpy().flatten()
        policy_sliced = {k: policy_output[np.newaxis, v] for k, v in policy_slices.items()}
        parsed = self.parser.parse_policy_outputs(policy_sliced)
        if 'off' in self._policy_keys[i] and self._has_on_policy:
          for key in ('plan', 'planplus', 'action'):
            if any(key in self._policy_slices_list[j] for j, k in enumerate(self._policy_keys) if 'on' in k.lower()):
              parsed.pop(key, None)

        outputs.update(parsed)

      if 'planplus' in outputs and 'plan' in outputs:
        outputs['plan'] = outputs['plan'] + outputs['planplus']

    if 'desired_curvature' in outputs and 'prev_desired_curv' in self.numpy_inputs:
      buf = self.numpy_inputs['prev_desired_curv']
      buf[0, :-1] = buf[0, 1:]
      buf[0, -1, :] = outputs['desired_curvature'][0, :] if not self.mlsim else 0

    return outputs

  def get_action_from_model(self, model_output: dict[str, np.ndarray], prev_action: log.ModelDataV2.Action,
                            lat_action_t: float, long_action_t: float, v_ego: float) -> log.ModelDataV2.Action:
    if 'action' not in model_output:
      plan = model_output['plan'][0]
      desired_accel = get_accel_from_plan(plan[:, Plan.VELOCITY][:, 0], plan[:, Plan.ACCELERATION][:, 0], self.constants.T_IDXS,
                                          action_t=long_action_t)

      curvature_plan = (plan + (self.PLANPLUS_CONTROL - 1.0) * model_output['planplus'][0]
                        if 'planplus' in model_output and self.PLANPLUS_CONTROL != 1.0 else plan)
      desired_curvature = get_curvature_from_output(model_output, curvature_plan, v_ego, lat_action_t, self.mlsim)
    else:
      desired_accel = model_output['action'][0, 1]
      desired_curvature = model_output['action'][0, 0] / (max(1.0, v_ego))**2

    stop = v_ego < 0.3 and desired_accel < 0.1
    desired_accel = smooth_value(desired_accel, prev_action.desiredAcceleration, self.LONG_SMOOTH_SECONDS)

    if self.generation is not None and self.generation >= 10: # smooth curvature for post FOF models
      if v_ego > self.MIN_LAT_CONTROL_SPEED:
        desired_curvature = smooth_value(desired_curvature, prev_action.desiredCurvature, self.LAT_SMOOTH_SECONDS)
      else:
        desired_curvature = prev_action.desiredCurvature

    return log.ModelDataV2.Action(desiredCurvature=float(desired_curvature), desiredAcceleration=float(desired_accel), shouldStop=bool(stop))


def build_model_inputs(m: ModelState, desire: int, traffic_convention: np.ndarray, v_ego: float, lat_delay: float,
                       lat_action_t: float, long_action_t: float) -> dict[str, np.ndarray]:
  vec_desire = np.zeros(m.constants.DESIRE_LEN, dtype=np.float32)
  if desire >= 0 and desire < m.constants.DESIRE_LEN:
    vec_desire[desire] = 1

  inputs: dict[str, np.ndarray] = {
    m.desire_key: vec_desire,
    'traffic_convention': traffic_convention,
  }
  if 'lateral_control_params' in m.numpy_inputs:
    inputs['lateral_control_params'] = np.array([v_ego, lat_delay], dtype=np.float32)
  if 'action_t' in m.numpy_inputs:
    inputs['action_t'] = np.array([lat_action_t, long_action_t], dtype=np.float32)
  return inputs


def setup_dual_models(params: Params, model: ModelState, cam_w: int, cam_h: int,
                      status: fm_dual.DualModelStatus) -> tuple[ModelState, ModelState | None]:
  """FM Pilot. Returns (primary model, longitudinal model). The longitudinal model is None unless dual mode runs;
  the primary model is the lateral slot's model in dual mode, otherwise the model to run alone."""
  selected = model.bundle
  status.lat_name = status.long_name = selected.displayName if selected is not None else ""
  status.requested = params.get_bool(fm_dual.PARAM_ENABLED)
  if not status.requested:
    return model, None
  if model.chestnut:
    status.fall_back("big model active")
    return model, None

  try:
    lat_bundle, long_bundle = fm_dual.resolve_slot_bundles(params, selected, get_cached_bundles(params, "qcom"))
    status.lat_name = lat_bundle.displayName if lat_bundle is not None else status.lat_name
    status.long_name = long_bundle.displayName if long_bundle is not None else status.long_name

    def load(bundle):
      if bundle is None or fm_dual._bundle_ref(bundle) == fm_dual._bundle_ref(selected):
        return model
      return ModelState(cam_w=cam_w, cam_h=cam_h, chestnut=False, bundle=bundle)

    lat_model = load(lat_bundle)
    if fm_dual._bundle_ref(lat_bundle) == fm_dual._bundle_ref(long_bundle):
      cloudlog.warning(f"fm dual model: both slots are {status.lat_name}, running it alone")
      return lat_model, None
    long_model = load(long_bundle)

    lat_output, long_output = lat_model.warmup(), long_model.warmup()
    reason = fm_dual.check_compatible(lat_output, long_output) if lat_output and long_output else "no warmup output"
    if reason is None and type(lat_model.constants) is not type(long_model.constants):
      reason = "different run rate"
    if reason is None:
      st = time.perf_counter()
      lat_model.warmup()
      long_model.warmup()
      pair_time = time.perf_counter() - st
      if pair_time > fm_dual.MAX_DUAL_EXECUTION_TIME:
        reason = f"too slow ({pair_time * 1000:.0f} ms)"
  except Exception:
    cloudlog.exception("fm dual model: load failed")
    reason, lat_model, long_model = "load failed", model, None

  if reason is not None:
    cloudlog.warning(f"fm dual model: falling back to the selected model: {reason}")
    status.lat_name = status.long_name = selected.displayName if selected is not None else ""
    status.fall_back(reason)
    return model, None

  cloudlog.warning(f"fm dual model: lateral {status.lat_name}, longitudinal {status.long_name}")
  status.active = True
  return lat_model, long_model


def main(demo=False):
  cloudlog.warning("modeld init")

  cloudlog.bind(daemon=PROCESS_NAME)
  setproctitle(PROCESS_NAME)
  config_realtime_process(7, 54)

  CHESTNUT = chestnut_present()
  if CHESTNUT:
    os.environ['HCQDEV_WAIT_TIMEOUT_MS'] = '3000'

  params = Params()
  params.put_bool("ChestnutLoading", CHESTNUT)
  params.remove("ChestnutActive")

  # visionipc clients
  while True:
    available_streams = VisionIpcClient.available_streams("camerad", block=False)
    if available_streams:
      use_extra_client = VisionStreamType.VISION_STREAM_WIDE_ROAD in available_streams and VisionStreamType.VISION_STREAM_NARROW_ROAD in available_streams
      main_wide_camera = VisionStreamType.VISION_STREAM_NARROW_ROAD not in available_streams
      break
    time.sleep(.1)

  vipc_client_main_stream = VisionStreamType.VISION_STREAM_WIDE_ROAD if main_wide_camera else VisionStreamType.VISION_STREAM_NARROW_ROAD
  vipc_client_main = VisionIpcClient("camerad", vipc_client_main_stream, True)
  vipc_client_extra = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_WIDE_ROAD, False)
  cloudlog.warning(f"vision stream set up, main_wide_camera: {main_wide_camera}, use_extra_client: {use_extra_client}")

  while not vipc_client_main.connect(False):
    time.sleep(0.1)
  while use_extra_client and not vipc_client_extra.connect(False):
    time.sleep(0.1)

  cloudlog.warning(f"connected main cam with buffer size: {vipc_client_main.buffer_len} ({vipc_client_main.width} x {vipc_client_main.height})")
  if use_extra_client:
    cloudlog.warning(f"connected extra cam with buffer size: {vipc_client_extra.buffer_len} ({vipc_client_extra.width} x {vipc_client_extra.height})")

  cloudlog.warning("loading model")
  st = time.monotonic()

  model = None
  if CHESTNUT:
    big_model = None
    def load_big():
      nonlocal big_model
      try:
        m = ModelState(cam_w=vipc_client_main.width, cam_h=vipc_client_main.height, chestnut=True)
        m.warmup()
        big_model = m
      except Exception:
        cloudlog.exception("chestnut load failed")
    loader = threading.Thread(target=load_big, daemon=True)
    loader.start()
    loader.join(BIG_MODEL_TIMEOUT)
    model = big_model
    params.put_bool("ChestnutActive", model is not None)

  small_model = ModelState(cam_w=vipc_client_main.width, cam_h=vipc_client_main.height, chestnut=False) if model is None or CHESTNUT else None
  if model is None:
    model = small_model
  params.put_bool("ChestnutLoading", False)
  assert model is not None

  # FM Pilot: dual model mode, the lateral model becomes `model` and the longitudinal model runs beside it
  dual_status = fm_dual.DualModelStatus()
  model, long_model = setup_dual_models(params, model, vipc_client_main.width, vipc_client_main.height, dual_status)
  if not CHESTNUT:
    small_model = model
  curve_source = fm_dual.CURVE_SOURCE_LATERAL
  cloudlog.warning(f"models loaded in {time.monotonic() - st:.1f}s, modeld starting")

  # messaging
  pub_socks = ["modelV2", "drivingModelData", "cameraOdometry", "modelDataV2SP"] + (["chestnutGpuState"] if CHESTNUT else [])
  pm = PubMaster(pub_socks)
  sm = SubMaster(["deviceState", "carState", "narrowRoadCameraState", "extrinsicsCalibration", "driverMonitoringState", "carControl", "lateralDelay"])

  publish_state = PublishState()
  chestnut_state = ChestnutGpuState(pm, model.chestnut) if CHESTNUT else None

  # setup filter to track dropped frames
  frame_dropped_filter = FirstOrderFilter(0., 10., 1. / model.constants.MODEL_FREQ)
  frame_id = 0
  last_vipc_frame_id = 0
  run_count = 0

  model_transform_main = np.zeros((3, 3), dtype=np.float32)
  model_transform_extra = np.zeros((3, 3), dtype=np.float32)
  live_calib_seen = False
  buf_main, buf_extra = None, None
  meta_main = FrameMeta()
  meta_extra = FrameMeta()
  camera_offset_helper = CameraOffsetHelper()


  if demo:
    CP = get_demo_car_params()
  else:
    CP = messaging.log_from_bytes(params.get("CarParams", block=True), car.CarParams)
  cloudlog.info("modeld got CarParams: %s", CP.brand)

  # TODO Move smooth seconds to action function
  long_delay = CP.longitudinalActuatorDelay + (long_model or model).LONG_SMOOTH_SECONDS
  prev_action = log.ModelDataV2.Action()

  DH = DesireHelper()
  meta_constants = load_meta_constants()
  RELC = RoadEdgeLaneChangeController()

  while True:
    # Keep receiving frames until we are at least 1 frame ahead of previous extra frame
    while meta_main.timestamp_sof < meta_extra.timestamp_sof + 25000000:
      buf_main = vipc_client_main.recv()
      meta_main = FrameMeta(vipc_client_main)
      if buf_main is None:
        break

    if buf_main is None:
      cloudlog.debug("vipc_client_main no frame")
      continue

    if use_extra_client:
      # Keep receiving extra frames until frame id matches main camera
      while True:
        buf_extra = vipc_client_extra.recv()
        meta_extra = FrameMeta(vipc_client_extra)
        if buf_extra is None or meta_main.timestamp_sof < meta_extra.timestamp_sof + 25000000:
          break

      if buf_extra is None:
        cloudlog.debug("vipc_client_extra no frame")
        continue

      if abs(meta_main.timestamp_sof - meta_extra.timestamp_sof) > 10000000:
        cloudlog.error(f"frames out of sync! main: {meta_main.frame_id} ({meta_main.timestamp_sof / 1e9:.5f}),\
                       extra: {meta_extra.frame_id} ({meta_extra.timestamp_sof / 1e9:.5f})")

    else:
      # Use single camera
      buf_extra = buf_main
      meta_extra = meta_main

    sm.update(0)
    desire = DH.desire
    is_rhd = sm["driverMonitoringState"].isRHD
    frame_id = sm["narrowRoadCameraState"].frameId
    v_ego = max(sm["carState"].vEgo, 0.)
    if sm.frame % 60 == 0:
      model.lat_delay = get_lat_delay(params, sm["lateralDelay"].lateralDelay)
      model.PLANPLUS_CONTROL = params.get("PlanplusControl", return_default=True)
      camera_offset_helper.set_offset(params.get("CameraOffset", return_default=True))
      if long_model is not None:
        long_model.lat_delay = model.lat_delay
        curve_source = params.get(fm_dual.PARAM_CURVE_SOURCE, return_default=True) or fm_dual.CURVE_SOURCE_LATERAL
    lat_delay = model.lat_delay + model.LAT_SMOOTH_SECONDS
    if sm.updated["extrinsicsCalibration"] and sm.seen['narrowRoadCameraState'] and sm.seen['deviceState']:
      device_from_calib_euler = np.array(sm["extrinsicsCalibration"].rpyCalib, dtype=np.float32)
      dc = DEVICE_CAMERAS[(str(sm['deviceState'].deviceType), str(sm['narrowRoadCameraState'].sensor))]
      main_intrinsics = dc.wide_road.intrinsics if main_wide_camera else dc.narrow_road.intrinsics
      model_transform_main = get_warp_matrix(device_from_calib_euler, main_intrinsics, False).astype(np.float32)
      model_transform_extra = get_warp_matrix(device_from_calib_euler, dc.wide_road.intrinsics, True).astype(np.float32)
      model_transform_main, model_transform_extra = camera_offset_helper.update(model_transform_main, model_transform_extra, sm, main_wide_camera)
      live_calib_seen = True

    traffic_convention = np.zeros(2)
    traffic_convention[int(is_rhd)] = 1

    # tracked dropped frames
    vipc_dropped_frames = max(0, meta_main.frame_id - last_vipc_frame_id - 1)
    frames_dropped = frame_dropped_filter.update(min(vipc_dropped_frames, 10))
    if run_count < 10: # let frame drops warm up
      frame_dropped_filter.x = 0.
      frames_dropped = 0.
    run_count = run_count + 1

    frame_drop_ratio = frames_dropped / (1 + frames_dropped)

    bufs = {name: buf_extra if 'big' in name else buf_main for name in model.vision_input_names}
    transforms = {name: model_transform_extra if 'big' in name else model_transform_main for name in model.vision_input_names}

    frame_delay = DT_MDL # compensate for time passed since the frame was captured: current_time - timestamp_eof is 50ms on average
    action_delay = DT_MDL / 2 # middle of the interval between model output (current state) and next frame (expected state)
    lat_action_t = lat_delay + frame_delay + action_delay
    long_action_t = long_delay + frame_delay + action_delay

    inputs = build_model_inputs(model, desire, traffic_convention, v_ego, lat_delay, lat_action_t, long_action_t)

    mt1 = time.perf_counter()
    long_output = None
    try:
      send_chestnut = (chestnut_state is not None and
                       run_count % round(model.constants.MODEL_FREQ / SERVICE_LIST['chestnutGpuState'].frequency) == 0)
      model_output = model.run(bufs, transforms, inputs, chestnut_state.send if send_chestnut else None)
      if long_model is not None:
        long_bufs = {name: buf_extra if 'big' in name else buf_main for name in long_model.vision_input_names}
        long_transforms = {name: model_transform_extra if 'big' in name else model_transform_main for name in long_model.vision_input_names}
        long_inputs = build_model_inputs(long_model, desire, traffic_convention, v_ego, lat_delay, lat_action_t, long_action_t)
        long_output = long_model.run(long_bufs, long_transforms, long_inputs)
    except Exception:
      if not params.get_bool("ChestnutActive"):
        raise
      cloudlog.exception("big model failed, fall back to small")
      params.put_bool("ChestnutActive", False)
      assert small_model is not None
      model = small_model
      if chestnut_state is not None:
        chestnut_state.big = False
      run_count = 0
      model_output = None
    mt2 = time.perf_counter()
    model_execution_time = mt2 - mt1
    if dual_status.update(model_execution_time):
      cloudlog.warning("fm dual model: over the time budget, continuing with the lateral model alone")
      dual_status.fall_back("too slow")
      dual_status.long_name = dual_status.lat_name
      long_model, long_output = None, None
      long_delay = CP.longitudinalActuatorDelay + model.LONG_SMOOTH_SECONDS

    if model_output is not None:
      modelv2_send = messaging.new_message('modelV2')
      drivingdata_send = messaging.new_message('drivingModelData')
      posenet_send = messaging.new_message('cameraOdometry')
      mdv2sp_send = messaging.new_message('modelDataV2SP')

      if long_output is not None:
        lat_action = model.get_action_from_model(model_output, prev_action, lat_action_t, long_action_t, v_ego)
        long_action = long_model.get_action_from_model(long_output, prev_action, lat_action_t, long_action_t, v_ego)
        action = fm_dual.merge_actions(lat_action, long_action)
        fm_dual.feed_back_curvature(long_model, model_output)
        model_output = fm_dual.merge_outputs(model_output, long_output, curve_source)
      else:
        action = model.get_action_from_model(model_output, prev_action, lat_action_t, long_action_t, v_ego)
      prev_action = action
      fill_model_msg(drivingdata_send, modelv2_send, model_output, action,
                     publish_state, meta_main.frame_id, meta_extra.frame_id, frame_id,
                     frame_drop_ratio, meta_main.timestamp_eof, model_execution_time, live_calib_seen, meta_constants)
      modelv2_send.modelV2.big = model.chestnut

      desire_state = modelv2_send.modelV2.meta.desireState
      l_lane_change_prob = desire_state[log.Desire.laneChangeLeft]
      r_lane_change_prob = desire_state[log.Desire.laneChangeRight]
      lane_change_prob = l_lane_change_prob + r_lane_change_prob
      left_edge, right_edge = RELC.update_and_fill(modelv2_send.modelV2, mdv2sp_send.modelDataV2SP, v_ego)
      DH.update(sm['carState'], sm['carControl'].latActive, lane_change_prob, left_edge, right_edge)
      modelv2_send.modelV2.meta.laneChangeState = DH.lane_change_state
      modelv2_send.modelV2.meta.laneChangeDirection = DH.lane_change_direction
      mdv2sp_send.valid = modelv2_send.valid
      mdv2sp_send.modelDataV2SP.laneTurnDirection = DH.lane_turn_direction
      dual_status.fill(mdv2sp_send.modelDataV2SP)
      drivingdata_send.drivingModelData.meta.laneChangeState = DH.lane_change_state
      drivingdata_send.drivingModelData.meta.laneChangeDirection = DH.lane_change_direction

      fill_pose_msg(posenet_send, model_output, meta_main.frame_id, vipc_dropped_frames, meta_main.timestamp_eof, live_calib_seen)
      pm.send('modelV2', modelv2_send)
      pm.send('drivingModelData', drivingdata_send)
      pm.send('cameraOdometry', posenet_send)
      pm.send('modelDataV2SP', mdv2sp_send)
    last_vipc_frame_id = meta_main.frame_id

if __name__ == "__main__":
  try:
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--demo', action='store_true', help='A boolean for demo mode.')
    args = parser.parse_args()
    main(demo=args.demo)
  except KeyboardInterrupt:
    cloudlog.warning(f"child {PROCESS_NAME} got SIGINT")
  except Exception:
    cloudlog.exception("modeld exception")
    raise
