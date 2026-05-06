"""Fine-tune YOLOv8n for URC autonomy mission objects on DGX Spark.

Trains on a mix of:
  - Synthetic renders from Isaac Sim Replicator (script in this folder)
  - Public bottle/tool subset from Open Images / COCO
  - Photographed samples on Mars-desert backgrounds (collect manually)

Output: yolov8n_urc.pt + ONNX export for TRT compile.

URC autonomy mission target classes:
  0 orange_rubber_mallet
  1 rock_pick
  2 water_bottle
  3 aruco_post           (square fiducial standoff)
  4 traversable_obstacle (small rocks, vegetation)
  5 nontraversable_rock  (large boulders, drops)

Usage on DGX Spark:
    python train_yolo_urc.py --data ~/deimos_data/yolo_urc/dataset.yaml --epochs 80
"""

import argparse
import os
import shutil
from pathlib import Path

from ultralytics import YOLO


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--data', type=str, required=True,
                   help='Path to dataset.yaml (Ultralytics format)')
    p.add_argument('--base', type=str, default='yolov8n.pt',
                   help='Base checkpoint to fine-tune')
    p.add_argument('--epochs', type=int, default=80)
    p.add_argument('--imgsz', type=int, default=640)
    p.add_argument('--batch', type=int, default=64)
    p.add_argument('--device', type=str, default='0')
    p.add_argument('--project', type=str, default='~/deimos_data/yolo_urc/runs')
    p.add_argument('--name', type=str, default='urc_v1')
    p.add_argument('--export-onnx', action='store_true', default=True)
    p.add_argument('--onnx-out', type=str, default='~/deimos_data/yolov8n_urc.onnx')
    args = p.parse_args()

    args.project = os.path.expanduser(args.project)
    args.onnx_out = os.path.expanduser(args.onnx_out)

    model = YOLO(args.base)
    results = model.train(
        data=args.data,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        project=args.project,
        name=args.name,
        amp=True,
        cache=True,
        patience=20,
        plots=True,
    )

    best = Path(args.project) / args.name / 'weights' / 'best.pt'
    print(f'Best weights: {best}')

    if args.export_onnx:
        m = YOLO(str(best))
        out = m.export(
            format='onnx',
            imgsz=args.imgsz,
            opset=17,
            simplify=True,
            dynamic=False,
            half=False,
        )
        target = args.onnx_out
        os.makedirs(os.path.dirname(target), exist_ok=True)
        shutil.copy(out, target)
        print(f'ONNX exported to {target}')


if __name__ == '__main__':
    main()
