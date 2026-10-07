import csv
import tempfile
import unittest
from pathlib import Path

import numpy as np
import yaml

from compare_control import check_matched, complete_trials, metrics


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

    def test_matched_targets_allow_sampling_jitter_but_reject_other_trajectory(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            files = [root / mode / "feedback.csv" for mode in ("base", "tuned")]
            for path in files:
                path.parent.mkdir()
                (path.parent / "config.yaml").write_text(yaml.safe_dump({
                    "yaw_experiment": {"ros__parameters": {
                        "tracking_test": True, "tracking_amplitude_deg": 5.0,
                        "tracking_frequency_hz": 0.25, "duration": 20.0,
                        "yaw_torque_limit": 3.6,
                    }}
                }))
            times = np.linspace(0, 20, 2001)
            target = np.deg2rad(5) * np.sin(2 * np.pi * 0.25 * times)
            baseline = [{"time": times, "target": target}]
            slight_phase_shift = [{"time": times, "target": np.deg2rad(5) *
                                   np.sin(2 * np.pi * 0.25 * (times + 0.005))}]
            match = check_matched(*files, baseline, slight_phase_shift)
            self.assertLess(match["max_sampled_target_difference_deg"], 0.1)
            wrong_target = [{"time": times, "target": target * 0.8}]
            with self.assertRaisesRegex(ValueError, "target trajectories differ"):
                check_matched(*files, baseline, wrong_target)

    def test_three_wide_frequencies_must_be_logged_in_order(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            files = [root / mode / "feedback.csv" for mode in ("base", "tuned")]
            for path in files:
                path.parent.mkdir()
                (path.parent / "config.yaml").write_text(yaml.safe_dump({
                    "yaw_experiment": {"ros__parameters": {
                        "tracking_test": True, "tracking_amplitude_deg": 15.0,
                        "tracking_frequency_hz": 0.15,
                        "tracking_frequencies_hz": [0.1, 0.15, 0.25],
                        "duration": 20.0, "yaw_torque_limit": 3.6,
                    }}
                }))
            times = np.linspace(0, 20, 2001)
            def trial(frequency, shift=0):
                return {"time": times, "target": np.deg2rad(15) *
                        np.sin(2 * np.pi * frequency * (times + shift)),
                        "frequency_hz": frequency}
            baseline = [trial(frequency) for frequency in (0.1, 0.15, 0.25)]
            tuned = [trial(frequency, 0.005) for frequency in (0.1, 0.15, 0.25)]
            self.assertEqual(check_matched(*files, baseline, tuned)["trial_frequencies_hz"],
                             [0.1, 0.15, 0.25])
            tuned[1]["frequency_hz"] = 0.1
            with self.assertRaisesRegex(ValueError, "logged frequency"):
                check_matched(*files, baseline, tuned)


if __name__ == "__main__":
    unittest.main()
