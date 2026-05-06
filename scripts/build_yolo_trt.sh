#!/bin/bash
# Build TensorRT engines from yolov8n_urc.onnx for both Jetson tiers.
# Run on the actual target Jetson — TRT engines are device + version specific.
#
# Outputs:
#   ~/deimos_data/yolov8n_urc_fp16.engine  (NX, DLA-compatible)
#   ~/deimos_data/yolov8n_urc_int8.engine  (Nano, GPU)
#
# Requires:
#   - TensorRT 8.5+ (ships with JetPack 5.x and 6.x)
#   - calibration data folder for INT8: ~/deimos_data/yolo_urc/calib_images/

set -euo pipefail

ONNX="${HOME}/deimos_data/yolov8n_urc.onnx"
OUT_DIR="${HOME}/deimos_data"
CALIB_DIR="${HOME}/deimos_data/yolo_urc/calib_images"

if [ ! -f "$ONNX" ]; then
    echo "ERROR: ONNX not found at $ONNX. Run train_yolo_urc.py first."
    exit 1
fi

mkdir -p "$OUT_DIR"

echo "=== Building FP16 engine (NX / DLA-eligible) ==="
trtexec \
    --onnx="$ONNX" \
    --saveEngine="${OUT_DIR}/yolov8n_urc_fp16.engine" \
    --fp16 \
    --useDLACore=0 \
    --allowGPUFallback \
    --workspace=2048 \
    --verbose

echo "=== Building INT8 engine (Nano / GPU) ==="
if [ -d "$CALIB_DIR" ] && [ "$(ls -A "$CALIB_DIR" 2>/dev/null)" ]; then
    trtexec \
        --onnx="$ONNX" \
        --saveEngine="${OUT_DIR}/yolov8n_urc_int8.engine" \
        --int8 \
        --calib="${OUT_DIR}/yolo_calib.cache" \
        --workspace=2048 \
        --verbose
else
    echo "WARN: No calibration data; falling back to FP16 INT8 (effectively FP16)."
    trtexec \
        --onnx="$ONNX" \
        --saveEngine="${OUT_DIR}/yolov8n_urc_int8.engine" \
        --best \
        --workspace=2048 \
        --verbose
fi

echo
echo "Engines built:"
ls -la "${OUT_DIR}"/yolov8n_urc_*.engine
