import sys
import unittest
from math import pi
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "models"))

from human_detection import HumanDetector


class Values:
    def __init__(self, values):
        self.values = values

    def tolist(self):
        return self.values


class FakeModel:
    def predict(self, **kwargs):
        boxes = type("Boxes", (), {"xyxy": Values([[10, 20, 30, 40]]), "conf": Values([0.9])})()
        return [type("Result", (), {"boxes": boxes})()]


class HumanDetectorTest(unittest.TestCase):
    def test_returns_person_box_and_confidence(self):
        detection = HumanDetector(model=FakeModel()).detect("test-image")[0]
        self.assertEqual(detection.box, (10, 20, 30, 40))
        self.assertEqual(detection.confidence, 0.9)

        output = detection.to_lidar_query(40, pi / 2, 12.5)
        self.assertEqual(output["center_pixel"], (20.0, 30.0))
        self.assertEqual(output["bearing_radians"], 0.0)
        self.assertEqual(output["timestamp_seconds"], 12.5)


if __name__ == "__main__":
    unittest.main()
