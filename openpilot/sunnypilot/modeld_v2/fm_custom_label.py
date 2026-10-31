"""FM Pilot: the small on-screen line shown while a locally compiled custom model (e.g. the splice) is driving."""
import os

PARAM_ACTIVE = "FM_CustomModelActive"


def custom_model_name(pkl_path: str) -> str:
  """fm_opm16d_rlm_splice_tinygrad.pkl -> fm_opm16d_rlm_splice"""
  name = os.path.basename(pkl_path)
  for suffix in (".pkl", "_tinygrad"):
    if name.endswith(suffix):
      name = name[:-len(suffix)]
  return name


def label_text(active_name: str | None, model_execution_time: float) -> str:
  """ASCII only (the UI font has no middle dot). Empty when no custom model is loaded."""
  if not active_name:
    return ""
  text = f"Custom model: {active_name}"
  if model_execution_time > 0:
    text += f" | {model_execution_time * 1000:.0f} ms"
  return text
