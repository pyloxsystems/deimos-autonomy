"""YOLOv8 TensorRT inference node for URC autonomy mission objects.

Loads a TRT engine compiled for either:
  - Orin NX:   yolov8n_urc_fp16.engine (DLA-eligible)
  - Orin Nano: yolov8n_urc_int8.engine (GPU)

URC autonomy mission target classes (per official rules):
  0: orange_rubber_mallet
  1: rock_pick
  2: water_bottle
  3: aruco_post
  4: traversable_obstacle
  5: nontraversable_rock

Subscribes:
  image_topic (sensor_msgs/Image, RGB8) — default /zed/zed_node/rgb/image_rect_color

Publishes:
  detection_topic (vision_msgs/Detection2DArray)
  /yolo/annotated (sensor_msgs/Image) — debug overlay (optional, set publish_debug=True)
"""

import os
import time
from typing import Optional, List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import Header
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
    BoundingBox2D,
)

# TensorRT + pycuda imports are deferred so the package can build/lint
# without TRT being present on the dev machine.
try:
    import tensorrt as trt
    import pycuda.driver as cuda
    import pycuda.autoinit  # noqa: F401
    TRT_AVAILABLE = True
except ImportError:
    TRT_AVAILABLE = False
    trt = None
    cuda = None


URC_CLASSES = [
    'orange_rubber_mallet',
    'rock_pick',
    'water_bottle',
    'aruco_post',
    'traversable_obstacle',
    'nontraversable_rock',
]


class TRTInfer:
    """Minimal TRT engine wrapper for a single-input/single-output YOLOv8 graph."""

    def __init__(self, engine_path: str, logger):
        if not TRT_AVAILABLE:
            raise RuntimeError(
                'tensorrt + pycuda not installed; cannot load engine'
            )

        self.logger = logger
        self.trt_logger = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, 'rb') as f, trt.Runtime(self.trt_logger) as runtime:
            self.engine = runtime.deserialize_cuda_engine(f.read())
        self.context = self.engine.create_execution_context()

        self.input_idx = 0
        self.output_idx = self.engine.num_bindings - 1

        in_shape = self.engine.get_binding_shape(self.input_idx)
        out_shape = self.engine.get_binding_shape(self.output_idx)
        self.input_shape = tuple(in_shape)
        self.output_shape = tuple(out_shape)

        self.input_h = self.input_shape[2]
        self.input_w = self.input_shape[3]

        self.d_input = cuda.mem_alloc(int(np.prod(in_shape) * 4))
        self.d_output = cuda.mem_alloc(int(np.prod(out_shape) * 4))
        self.bindings = [int(self.d_input), int(self.d_output)]
        self.h_output = np.empty(out_shape, dtype=np.float32)
        self.stream = cuda.Stream()

    def infer(self, blob: np.ndarray) -> np.ndarray:
        cuda.memcpy_htod_async(self.d_input, blob, self.stream)
        self.context.execute_async_v2(
            bindings=self.bindings, stream_handle=self.stream.handle
        )
        cuda.memcpy_dtoh_async(self.h_output, self.d_output, self.stream)
        self.stream.synchronize()
        return self.h_output


def preprocess_image(rgb: np.ndarray, target_h: int, target_w: int) -> Tuple[np.ndarray, float, float]:
    src_h, src_w = rgb.shape[:2]
    scale = min(target_w / src_w, target_h / src_h)
    new_w, new_h = int(src_w * scale), int(src_h * scale)

    import cv2
    resized = cv2.resize(rgb, (new_w, new_h))
    canvas = np.zeros((target_h, target_w, 3), dtype=np.uint8)
    canvas[:new_h, :new_w] = resized

    blob = canvas.astype(np.float32) / 255.0
    blob = blob.transpose(2, 0, 1)[None, ...]
    blob = np.ascontiguousarray(blob)
    return blob, scale, scale


def postprocess_yolov8(
    raw: np.ndarray, scale_x: float, scale_y: float,
    conf_threshold: float, iou_threshold: float,
) -> List[dict]:
    """Decode YOLOv8 export format: (1, 84, 8400) for COCO80.
    For our 6-class custom model: (1, 10, 8400) where rows are
    [cx, cy, w, h, cls0, cls1, ..., cls5].
    """
    import cv2

    if raw.ndim == 3:
        raw = raw[0]
    if raw.shape[0] < raw.shape[1]:
        raw = raw.T  # -> (8400, 4+nc)

    n_classes = raw.shape[1] - 4
    boxes_xywh = raw[:, :4]
    class_scores = raw[:, 4 : 4 + n_classes]

    class_ids = class_scores.argmax(axis=1)
    confidences = class_scores.max(axis=1)

    keep = confidences >= conf_threshold
    boxes_xywh = boxes_xywh[keep]
    class_ids = class_ids[keep]
    confidences = confidences[keep]

    if len(boxes_xywh) == 0:
        return []

    cx, cy, w, h = boxes_xywh.T
    x1 = (cx - w / 2) / scale_x
    y1 = (cy - h / 2) / scale_y
    boxes_xyxy = np.stack([x1, y1, w / scale_x, h / scale_y], axis=1)

    indices = cv2.dnn.NMSBoxes(
        boxes_xyxy.tolist(),
        confidences.tolist(),
        conf_threshold,
        iou_threshold,
    )
    detections = []
    if len(indices) == 0:
        return detections
    for idx in np.array(indices).flatten():
        x, y, ww, hh = boxes_xyxy[idx]
        detections.append({
            'class_id': int(class_ids[idx]),
            'confidence': float(confidences[idx]),
            'x': float(x),
            'y': float(y),
            'w': float(ww),
            'h': float(hh),
        })
    return detections


class YoloTrtNode(Node):
    def __init__(self):
        super().__init__('yolo_trt_node')

        self.declare_parameter('compute_profile', 'nano_8gb')
        self.declare_parameter(
            'engine_path_nx',
            os.path.expanduser('~/deimos_data/yolov8n_urc_fp16.engine'),
        )
        self.declare_parameter(
            'engine_path_nano',
            os.path.expanduser('~/deimos_data/yolov8n_urc_int8.engine'),
        )
        self.declare_parameter(
            'image_topic', '/zed/zed_node/rgb/image_rect_color'
        )
        self.declare_parameter('detection_topic', '/yolo/detections')
        self.declare_parameter('confidence_threshold', 0.45)
        self.declare_parameter('iou_threshold', 0.5)
        self.declare_parameter('publish_debug', False)
        self.declare_parameter('infer_every_n', 1)

        profile = self.get_parameter('compute_profile').value
        engine_path = (
            self.get_parameter('engine_path_nx').value
            if profile == 'nx_16gb'
            else self.get_parameter('engine_path_nano').value
        )

        self.engine: Optional[TRTInfer] = None
        if not TRT_AVAILABLE:
            self.get_logger().error(
                'TensorRT + pycuda not available. Node up but will not infer.'
            )
        elif not os.path.exists(engine_path):
            self.get_logger().error(
                f'Engine file not found at {engine_path}. '
                f'Build with scripts/build_yolo_trt.sh first.'
            )
        else:
            try:
                self.engine = TRTInfer(engine_path, self.get_logger())
                self.get_logger().info(
                    f'Loaded TRT engine: {engine_path} '
                    f'(input {self.engine.input_shape}, output {self.engine.output_shape})'
                )
            except Exception as exc:
                self.get_logger().error(f'Failed to load engine: {exc}')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
        )

        self.image_sub = self.create_subscription(
            Image,
            self.get_parameter('image_topic').value,
            self._image_cb,
            qos,
        )
        self.det_pub = self.create_publisher(
            Detection2DArray,
            self.get_parameter('detection_topic').value,
            10,
        )
        self.debug_pub = (
            self.create_publisher(Image, '/yolo/annotated', 5)
            if self.get_parameter('publish_debug').value
            else None
        )

        self._frame_counter = 0
        self._last_log_t = time.time()
        self._frames_since_log = 0

    def _image_cb(self, msg: Image):
        self._frame_counter += 1
        if self._frame_counter % int(self.get_parameter('infer_every_n').value) != 0:
            return

        if self.engine is None:
            return

        if msg.encoding not in ('rgb8', 'bgr8'):
            self.get_logger().warn(
                f'Unsupported encoding {msg.encoding}; expected rgb8/bgr8.'
            )
            return

        rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3
        )
        if msg.encoding == 'bgr8':
            rgb = rgb[:, :, ::-1]

        blob, sx, sy = preprocess_image(
            rgb, self.engine.input_h, self.engine.input_w
        )
        raw = self.engine.infer(blob)
        detections = postprocess_yolov8(
            raw, sx, sy,
            self.get_parameter('confidence_threshold').value,
            self.get_parameter('iou_threshold').value,
        )

        out = Detection2DArray()
        out.header = msg.header

        for det in detections:
            d = Detection2D()
            d.header = msg.header
            bbox = BoundingBox2D()
            bbox.center.position.x = det['x'] + det['w'] / 2
            bbox.center.position.y = det['y'] + det['h'] / 2
            bbox.size_x = det['w']
            bbox.size_y = det['h']
            d.bbox = bbox
            hyp = ObjectHypothesisWithPose()
            cls_id = det['class_id']
            hyp.hypothesis.class_id = (
                URC_CLASSES[cls_id] if cls_id < len(URC_CLASSES) else str(cls_id)
            )
            hyp.hypothesis.score = det['confidence']
            d.results.append(hyp)
            out.detections.append(d)

        self.det_pub.publish(out)

        self._frames_since_log += 1
        now = time.time()
        if now - self._last_log_t > 5.0:
            fps = self._frames_since_log / (now - self._last_log_t)
            self.get_logger().info(
                f'YOLO inference: {fps:.1f} FPS | last frame {len(detections)} dets'
            )
            self._last_log_t = now
            self._frames_since_log = 0


def main(args=None):
    rclpy.init(args=args)
    node = YoloTrtNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
