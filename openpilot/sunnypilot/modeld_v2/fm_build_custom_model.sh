#!/usr/bin/env bash
# FM Pilot: compile a custom small driving model (e.g. the OPM16D + Rebel Legion splice) on the comma itself and
# point modeld at it. Run over SSH while the car is off (offroad), from anywhere:
#
#   bash /data/openpilot/openpilot/sunnypilot/modeld_v2/fm_build_custom_model.sh /data/media/0/fm_models/model.onnx <base model ref>
#
# <base model ref> is the catalog ref of the model the custom file is built on; its settings (generation, 20 Hz
# timing, smoothing) are used, and the custom file only loads while that model is the selected one.
# To go back to the stock file:  bash fm_build_custom_model.sh --off
set -euo pipefail

ROOT=/data/openpilot
OUT_DIR=/data/media/0/fm_models
export PYTHONPATH="$ROOT"
cd "$ROOT"

put_param() {
  python3 -c "import sys; from openpilot.common.params import Params; p = Params(); p.remove(sys.argv[1]) if sys.argv[2] == '' else p.put(sys.argv[1], sys.argv[2])" "$1" "$2"
}

if [ "${1:-}" = "--off" ]; then
  put_param FM_CustomModelPath ""
  put_param FM_CustomModelBaseRef ""
  echo "custom model off: modeld loads the selected model's own file again (next drive)"
  exit 0
fi

ONNX="${1:?usage: fm_build_custom_model.sh model.onnx base_ref}"
BASE_REF="${2:?usage: fm_build_custom_model.sh model.onnx base_ref}"
mkdir -p "$OUT_DIR"
OUT="$OUT_DIR/$(basename "${ONNX%.onnx}")_tinygrad.pkl"

echo "compiling $ONNX -> $OUT (this takes several minutes)"
# same tinygrad flags the openpilot build uses for the comma's GPU
DEV=QCOM IMAGE=1 FLOAT16=1 NOLOCALS=1 JIT_BATCH_SIZE=0 OPENPILOT_HACKS=1 \
  python3 openpilot/sunnypilot/modeld_v2/compile_modeld.py \
    --model-type supercombo --model-size 512x256 --camera-resolutions 1928x1208 1344x760 \
    --supercombo-onnx "$ONNX" --output "$OUT"

put_param FM_CustomModelPath "$OUT"
put_param FM_CustomModelBaseRef "$BASE_REF"
echo "done: $OUT will load on the next drive while the model with ref $BASE_REF is selected"
