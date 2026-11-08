#!/usr/bin/env python3
"""FM Pilot: download a driving model's .onnx from openpilot's model storage onto the comma, so
fm_build_custom_model.sh can compile it here.

Why: sunnypilot only publishes new models (e.g. Cheddar) in its newer model list (driving_models_v25+), which needs
a newer tinygrad and model file format than FM Pilot runs. Compiling the .onnx on the comma with FM Pilot's own
tinygrad sidesteps that. Run over SSH with the car off:

  PYTHONPATH=/data/openpilot python3 /data/openpilot/openpilot/sunnypilot/modeld_v2/fm_fetch_openpilot_onnx.py cheddar

It saves /data/media/0/fm_models/<name>.onnx and checks its size and sha256 against the openpilot commit it came
from. Other models: --oid <sha256> --size <bytes> --name <name> (from the model file's pointer in an openpilot commit).

openpilot keeps model files in Git LFS on Hugging Face (commaai/openpilot-lfs, since openpilot #39206); this asks that
store's public LFS batch API for a download link, the same thing `git lfs pull` does.
"""
import argparse
import hashlib
import os
import sys

LFS_BATCH_URL = "https://huggingface.co/commaai/openpilot-lfs.git/info/lfs/objects/batch"
LFS_HEADERS = {"Accept": "application/vnd.git-lfs+json", "Content-Type": "application/vnd.git-lfs+json"}
OUT_DIR = "/data/media/0/fm_models"

# openpilot's driving_supercombo.onnx pointers. Cheddar is openpilot PR #39255 (YassineYousfi, Nov 5, 2026; open, not
# merged when added here). "cheddar" is the commit sunnypilot published as "Cheddar Model (November 05, 2026)"
# (catalog ref 875a6b7); "cheddar-20480" is the PR's later commit 10e305d, a further-trained checkpoint.
KNOWN = {
  "cheddar": {"oid": "5520d198bbbe2736b7fbbc432c2d7a97e306db74214aa1677fed70521b9b9eb4", "size": 82558205,
              "source": "commaai/openpilot 875a6b7 (PR #39255)"},
  "cheddar-20480": {"oid": "18719f77a7781a50b0983309c936cd99717eb8c982b60c057f544ab8450133c2", "size": 82502228,
                    "source": "commaai/openpilot 10e305d (PR #39255)"},
}


def batch_body(oid: str, size: int) -> dict:
  return {"operation": "download", "transfers": ["basic"], "objects": [{"oid": oid, "size": size}]}


def parse_batch_response(resp: dict, oid: str) -> tuple[str, dict]:
  """Returns (download url, headers) for oid, or raises ValueError with the store's reason."""
  for obj in resp.get("objects", []):
    if obj.get("oid") != oid:
      continue
    if "error" in obj:
      raise ValueError(f"model store refused {oid[:12]}: {obj['error'].get('message', obj['error'])}")
    action = obj.get("actions", {}).get("download")
    if not action or not action.get("href"):
      raise ValueError(f"model store gave no download link for {oid[:12]}")
    return action["href"], dict(action.get("header", {}))
  raise ValueError(f"model store did not answer for {oid[:12]}")


def file_matches(path: str, oid: str, size: int) -> bool:
  if not os.path.isfile(path) or os.path.getsize(path) != size:
    return False
  h = hashlib.sha256()
  with open(path, "rb") as f:
    for block in iter(lambda: f.read(1 << 20), b""):
      h.update(block)
  return h.hexdigest() == oid


def fetch(name: str, oid: str, size: int, out_dir: str = OUT_DIR, session=None) -> str:
  import requests
  session = session or requests.Session()
  os.makedirs(out_dir, exist_ok=True)
  out = os.path.join(out_dir, f"{name}.onnx")
  if file_matches(out, oid, size):
    print(f"already have {out} (checksum ok)")
    return out

  r = session.post(LFS_BATCH_URL, json=batch_body(oid, size), headers=LFS_HEADERS, timeout=30)
  r.raise_for_status()
  href, headers = parse_batch_response(r.json(), oid)

  part = out + ".part"
  print(f"downloading {size / 1e6:.0f} MB -> {out}")
  with session.get(href, headers=headers, stream=True, timeout=60) as d:
    d.raise_for_status()
    with open(part, "wb") as f:
      for chunk in d.iter_content(1 << 20):
        f.write(chunk)
  if not file_matches(part, oid, size):
    os.remove(part)
    raise ValueError("downloaded file failed the size/sha256 check; nothing saved")
  os.replace(part, out)
  print(f"saved {out} (checksum ok)")
  return out


def main(argv=None) -> int:
  ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
  ap.add_argument("model", nargs="?", choices=sorted(KNOWN), help="a known model")
  ap.add_argument("--oid", help="sha256 from the .onnx LFS pointer")
  ap.add_argument("--size", type=int, help="size from the .onnx LFS pointer")
  ap.add_argument("--name", help="file name to save as (without .onnx)")
  ap.add_argument("--out-dir", default=OUT_DIR)
  args = ap.parse_args(argv)

  if args.model:
    m = KNOWN[args.model]
    name, oid, size = args.model, m["oid"], m["size"]
    print(f"{name}: {m['source']}")
  elif args.oid and args.size and args.name:
    name, oid, size = args.name, args.oid.lower(), args.size
  else:
    ap.error("give a known model, or --oid, --size and --name")

  try:
    fetch(name, oid, size, args.out_dir)
  except Exception as e:
    print(f"failed: {e}", file=sys.stderr)
    return 1
  return 0


if __name__ == "__main__":
  sys.exit(main())
