#!/usr/bin/env python3
"""
FM Pilot Jetlink Status process: publishes fmJetlinkStatus, which model the Jetlink link is running and why.

Reads Jetlink's records on the comma (/dev/shm/jetlink) and the comma's params (see status.py) and publishes them at 1 Hz
and at once on a change, so a log shows the model picked and the swap as they happen, not only what the params said when
the drive started. One log line per change (the model driving, the picks). Read only: it sends nothing to the car, the
model or Jetlink. Runs only while driving with the Jetlink link on, on the small cores at low priority.
"""
import os
import time

import openpilot.cereal.messaging as messaging
from openpilot.common.hardware.hw import Paths
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.fm_jetlink.status import ChangeLog, Reader, fill_message

POLL_S = 0.25  # a change is on the road within this; the message itself goes out at PUBLISH_PERIOD otherwise
PUBLISH_PERIOD = 1.0
SMALL_CORES = (0, 1, 2, 3)


def main() -> None:
  try:
    os.sched_setaffinity(0, SMALL_CORES)
    os.nice(15)
  except (AttributeError, OSError):
    pass

  pm = messaging.PubMaster(["fmJetlinkStatus"])
  reader = Reader(Params(), index_path=os.path.join(Paths.model_root(), "jetlink", "index.json"))
  changes = ChangeLog()
  last = None
  last_publish = 0.0
  failures = 0

  while True:
    now = time.monotonic()
    try:
      d = reader.read()
      for line in changes.lines(d):
        cloudlog.warning(line)
      key = {k: v for k, v in d.items() if k != "recordAge"}
      if key != last or now - last_publish >= PUBLISH_PERIOD - 0.01:
        msg = messaging.new_message("fmJetlinkStatus")
        fill_message(msg.fmJetlinkStatus, d)
        pm.send("fmJetlinkStatus", msg)
        last, last_publish = key, now
    except Exception:  # a bad record must not end the process or take the message's rate with it
      failures += 1
      if failures <= 3:
        cloudlog.exception("fm_jetlinkstatusd: read failed")
    time.sleep(POLL_S)


if __name__ == "__main__":
  main()
