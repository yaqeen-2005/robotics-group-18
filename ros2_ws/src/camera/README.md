# Camera

The camera component uses the pretrained Ultralytics YOLO26 nano detection model (`yolo26n.pt`) and keeps only the COCO `person` class.

Install its dependency:

```bash
pip install -r requirements.txt
```

Basic use:

```python
from models.human_detection import HumanDetector

detector = HumanDetector()
detections = detector.detect(image)

lidar_query = detections[0].to_lidar_query(
    image_width=640,
    horizontal_fov_radians=1.047,
    timestamp_seconds=webots_time,
)
```

`bearing_radians` is zero straight ahead, positive to the left, and negative to the right. The LiDAR component should match a scan near `timestamp_seconds` and obtain the range at that bearing.

The model weights download automatically on first use. Webots camera input, LiDAR matching, and conversion to map coordinates are not connected yet.
