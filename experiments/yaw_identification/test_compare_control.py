import csv
import tempfile
import unittest
from pathlib import Path

from compare_control import complete_trials, metrics


class CompareControlTests(unittest.TestCase):
    def test_complete_trial_and_metrics(self):
        names = ["/yaw_experiment/state", "/yaw_experiment/completed",
                 "/yaw_experiment/time_s", "/yaw_experiment/yaw_target_offset",
                 "/yaw_experiment/yaw_error", "/gimbal/yaw/control_torque",
                 "/yaw_experiment/pitch_world", "/gimbal/pitch/temperature",
                 "/gimbal/yaw/temperature"]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"feedback.csv"
            with path.open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=names)
                writer.writeheader()
                for i in range(100):
                    writer.writerow(dict(zip(names, [3, 0, i*.01, .05, .01, .2, .08, 40, 35])))
                writer.writerow(dict(zip(names, [2, 1, 1.0, 0, 0, 0, .08, 40, 35])))
            trials, rejected = complete_trials(path)
        self.assertEqual(len(trials), 1)
        self.assertFalse(rejected)
        result = metrics(trials, 4.5)
        self.assertAlmostEqual(result["angle_rmse_deg"], 0.5729578, places=5)
        self.assertAlmostEqual(result["torque_rms_Nm"], .2)
        self.assertEqual(result["max_pitch_temp_C"], 40)


if __name__ == "__main__":
    unittest.main()
