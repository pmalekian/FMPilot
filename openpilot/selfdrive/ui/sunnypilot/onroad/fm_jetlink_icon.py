"""FM Pilot: the road view's model icon for Jetlink.

The icon next to the steering wheel shows the chestnut's big model, and only while a chestnut is fitted. With Jetlink
the big model runs on the computer on the USB-C port instead, so nothing showed which model drives. This picks the
same icon for Jetlink: pulsing while the big model is being prepared or is joining, green for as long as it drives
(modelV2.big). None leaves the chestnut's own icon logic in charge.
"""

ACTIVE = "active"
LOADING = "loading"


def jetlink_icon_state(chestnut_present: bool, jetlink, model_alive: bool, model_big: bool) -> str | None:
  """jetlink: ui_state.jetlink (jetlink's Status, refreshed in the params pass), or None."""
  if chestnut_present or jetlink is None or not jetlink.enabled:
    return None
  if model_alive and model_big:
    return ACTIVE
  progress = jetlink.progress
  if progress and str(progress.get("stage", "")) not in ("", "ready"):
    return LOADING
  return None
