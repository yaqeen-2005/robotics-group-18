"""Detect people in camera images with a small pretrained YOLO model.

This module only converts an image into person bounding boxes and confidence
scores. Webots image capture, world-coordinate conversion, map updates, and
event recording belong to their respective integration components.
"""

from math import atan, pi, tan


class HumanDetection:
    def __init__(self, box: tuple[float, float, float, float], confidence: float):
        self.box = box
        self.confidence = confidence

    def to_lidar_query(
        self,
        image_width: int,
        horizontal_fov_radians: float,
        timestamp_seconds: float,
    ) -> dict:
        """Return the angle and metadata needed to match a LiDAR measurement.

        Bearing zero is straight ahead, positive is left, and negative is right.
        """
        if image_width <= 0:
            raise ValueError("image_width must be positive")
        if not 0.0 < horizontal_fov_radians < pi:
            raise ValueError("horizontal_fov_radians must be between 0 and pi")

        x1, y1, x2, y2 = self.box
        center_x = (x1 + x2) / 2.0
        center_y = (y1 + y2) / 2.0
        focal_length = image_width / (2.0 * tan(horizontal_fov_radians / 2.0))
        bearing = atan((image_width / 2.0 - center_x) / focal_length)
        return {
            "type": "person",
            "timestamp_seconds": float(timestamp_seconds),
            "confidence": self.confidence,
            "box_xyxy": self.box,
            "center_pixel": (center_x, center_y),
            "bearing_radians": bearing,
        }


class HumanDetector:
    def __init__(self, model_name: str = "yolo26n.pt", confidence: float = 0.5, model=None):
        if not 0.0 <= confidence <= 1.0:
            raise ValueError("confidence must be between 0.0 and 1.0")
        if model is None:
            from ultralytics import YOLO

            model = YOLO(model_name)
        self.model = model
        self.confidence = confidence

    def detect(self, image) -> list[HumanDetection]:
        """Return COCO person detections for one image."""
        result = self.model.predict(
            source=image, classes=[0], conf=self.confidence, verbose=False
        )[0]
        return [
            HumanDetection(tuple(box), float(score))
            for box, score in zip(result.boxes.xyxy.tolist(), result.boxes.conf.tolist())
        ]
