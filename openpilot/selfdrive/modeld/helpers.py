import sys
import time
from pathlib import Path

from openpilot.common.hardware import AGNOS
from openpilot.common.hardware.usb import CHESTNUT_USB_PRODUCT, USB_DEVICES_PATH, is_chestnut_usb_id

MODELS_DIR = Path(__file__).resolve().parent / 'models'


def modeld_pkl_path(chestnut: bool):
  prefix = 'big_' if chestnut else ''
  return MODELS_DIR / f'{prefix}driving_tinygrad.pkl'

def join_chunks(path) -> None:
  """FM Pilot: FM's release builds split model files over 95 MB into chunks (tools/release, file_chunker.py), since FM
  can't upload to sunnypilot's LFS. tinygrad's loader needs one file, so put a chunked file back together once."""
  from openpilot.common.file_chunker import get_chunk_name, get_manifest_path
  path, manifest = Path(path), Path(get_manifest_path(path))
  if path.exists() or not manifest.is_file():
    return
  num_chunks = int(manifest.read_text().strip())
  tmp = path.with_name(path.name + ".joining")
  with open(tmp, "wb") as out:
    for i in range(num_chunks):
      with open(get_chunk_name(str(path), i, num_chunks), "rb") as f:
        while block := f.read(1 << 24):
          out.write(block)
  tmp.replace(path)

def load_oob(path, chestnut=False):
  join_chunks(path)
  from tinygrad import Context
  device = 'USB+AMD:LLVM' if chestnut else 'QCOM' if AGNOS else 'METAL' if sys.platform == 'darwin' else 'CPU:LLVM'
  with Context(DEV=device):
    from tinygrad_repo.examples.openpilot.helpers import load_pickle
    return load_pickle(path, out_of_band=True)

def chestnut_present() -> bool:
  for d in USB_DEVICES_PATH.glob("*"):
    try:
      usb_id = (int((d / "idVendor").read_text(), 16), int((d / "idProduct").read_text(), 16))
      product = (d / "product").read_text().strip()
      if is_chestnut_usb_id(*usb_id) and product == CHESTNUT_USB_PRODUCT:
        return True
    except Exception:
      pass
  return False

def wait_for_chestnut(timeout: float = 10.) -> None:
  # the cable is detected before chestnut enumerates
  st = time.monotonic()
  while not chestnut_present():
    if time.monotonic() - st > timeout:
      raise TimeoutError("chestnut did not enumerate")
    time.sleep(0.1)

def chestnut_compiled() -> bool:
  path = modeld_pkl_path(chestnut=True)
  return path.is_file() and all(
    (MODELS_DIR / f'big_driving_warp_{size}_tinygrad.pkl').is_file() for size in ('1344x760', '1928x1208'))
