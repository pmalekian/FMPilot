"""FM Pilot: Jetlink drives the comma's model icon, on the home screen and the road view.

The icon (chestnut icons) shows the chestnut's big model. With Jetlink the big model runs on the computer on the
USB-C port instead, so the icon follows Jetlink, as zoompilot does (zoompilot develop 92e1ecb, ui_state_zp.py
`_jetlink_state` and mici home `_set_chestnut_visibility`, Zeph Leggett, MIT; jetlink docs/using-jetlink.md):

  pulsing      the big model is downloading or being prepared (parked), or joining (driving)
  green        ready (parked) or driving (onroad)
  dimmed green ready, waiting for a switch: disengage fully
  orange       a problem: read the home screen's alert

The state comes from jetlink's own Status.icon(). Onroad it needs the joining model's state; zoompilot publishes
that in its own message, FM reads the same value from jetlink's runtime record (Status.running).
FM change: on the road view the green and dimmed green stay on (the chestnut's own green fades after 2.5 s).
"""

DIMMED = 0.5  # zoompilot's opacity for the waiting icon


def icon_state(view, started: bool, model_seen: bool, running_big: bool) -> str:
  """A ChestnutState value for the link. view: jetlink's Status (ui_state.jetlink_view)."""
  return view.icon(started, model_seen, running_big, view.running or 'none')


def stays_on(state: str, from_jetlink: bool) -> bool:
  """Road view: Jetlink's green and dimmed green stay as long as they hold, instead of fading."""
  return from_jetlink and state in ('active', 'waiting')
