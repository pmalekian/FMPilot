#!/usr/bin/env python3
"""Driver monitoring summary: how much of a drive counted as distracted, and why (head pose, eyes, phone).

usage: dm.py rlog.zst [rlog.zst ...] [--each]
  Segments of one drive are joined into one summary; --each gives one line per file.
"""
import numpy as np

from fmlog import load_drives, log_args


def main(drive):
  path = drive.label()
  dm = [e.driverMonitoringState for e in drive.evs if e.which() == "driverMonitoringState"]
  if not dm:
    print(f"{path}: no driver monitoring data")
    return
  v = [d.visionPolicyState for d in dm]
  pct = lambda f: np.mean([f(x) for x in v]) * 100  # noqa: E731
  print(f"{path}: distracted {pct(lambda x: x.isDistracted):.1f}% (pose {pct(lambda x: x.distractedTypes.pose):.1f}, eyes "
        f"{pct(lambda x: x.distractedTypes.eye):.1f}, phone {pct(lambda x: x.distractedTypes.phone):.1f}) | lowest awareness "
        f"{min(x.awarenessPercent for x in v)}% | alerts {sorted(set(str(d.alertLevel) for d in dm))}")


if __name__ == "__main__":
  args = log_args(__doc__).parse_args()
  for d in load_drives(args.rlog, args.each):
    main(d)
