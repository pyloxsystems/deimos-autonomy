# YOLOv8n Inference Samples

Annotated detections from the deimos-autonomy YOLOv8n model on held-out test
images from the WVU URC 2024 dataset (universe.roboflow.com/wvu-urc-2024,
CC BY 4.0).

Model: `yolov8n_urc.pt` fine-tuned from `yolov8n.pt`. Trained on 1,382 images,
validated on 311 (mAP50 0.98, mAP50-95 0.846, P 0.98, R 0.97).

These 10 samples show real detections from the model — bounding boxes,
class label, confidence score — on test images the model never saw during
training. Confidence threshold 0.45 for display.

## Class taxonomy

| ID | Name | Source |
|----|------|--------|
| 0 | orange_rubber_mallet | URC autonomy mission target (rubber sledgehammer-shaped) |
| 1 | rock_pick | URC autonomy mission target (geological hammer, pointed end) |
| 2 | water_bottle | URC autonomy mission target |
| 3 | aruco_post | URC GPS waypoint fiducial standoff |
| 4 | traversable_obstacle | small rocks, vegetation (no public training data, future fine-tune) |
| 5 | nontraversable_rock | large boulders, drops (future fine-tune) |
| 6 | keyboard | URC Equipment Servicing mission target |

The current model has training data for classes 0, 2, 3, 6 (from WVU + Monash
public datasets). Class 1 (rock_pick) is reserved for a future fine-tune
once Monash dataset re-export succeeds. Classes 4 and 5 are placeholders
for a synthetic-data fine-tune pass.

## Detected objects in this sample set

- 4× keyboard (0.93-0.95)
- 3× aruco_post (0.84-0.88)
- 2× water_bottle (0.47, 0.91)
- 1× orange_rubber_mallet (0.97)

## Reproducing

```bash
# On a machine with the trained weights and Ultralytics installed:
python3 -c "
from ultralytics import YOLO
model = YOLO('yolov8n_urc.pt')
model.predict('your_test_image.jpg', conf=0.45, save=True)
"
```

Or use the TensorRT INT8 engine on Jetson via `src/deimos_autonomy/deimos_autonomy/yolo_trt_node.py`.
