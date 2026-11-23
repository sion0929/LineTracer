from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

import cv2
import numpy as np

from experiments.dl.evaluate import evaluate


class ObstacleAuditTest(unittest.TestCase):
    def test_only_obstacle_free_frames_count_as_frame_false_alarms(self):
        with TemporaryDirectory() as temp:
            root = Path(temp)
            (root / "image").mkdir()
            (root / "mask").mkdir()
            image = np.full((480, 640, 3), 230, dtype=np.uint8)
            labels = np.zeros(image.shape[:2], dtype=np.uint8)
            obstacles = np.zeros_like(labels, dtype=bool)
            obstacles[:5, :7] = True

            def segment(_):
                return np.zeros_like(obstacles), obstacles

            names = ["clean.png", "with_obstacle.png"]
            self.assertTrue(cv2.imwrite(str(root / "image" / names[0]), image))
            self.assertTrue(cv2.imwrite(str(root / "image" / names[1]), image))
            self.assertTrue(cv2.imwrite(str(root / "mask" / names[0]), labels))
            labels[-1, -1] = 2
            self.assertTrue(cv2.imwrite(str(root / "mask" / names[1]), labels))
            records = [{"file": name} for name in names]
            results, _ = evaluate(segment, [(root, {"map": "example"}, records)], root)
            _, _, audit, _ = results["example"]
            self.assertEqual(audit, {
                "obstacle_free_frames": 1,
                "false_obstacle_frames": 1,
                "max_false_obstacle_pixels": 35,
            })


if __name__ == "__main__":
    unittest.main()
