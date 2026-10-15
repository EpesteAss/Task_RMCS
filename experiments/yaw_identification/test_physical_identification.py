"""Check physical parameter recovery and truly free-run simulation."""
import tempfile
import unittest
from pathlib import Path

import numpy as np

import physical_identification as physical


class PhysicalIdentificationTests(unittest.TestCase):
    def synthetic(self, friction):
        parameters = {"J_kg_m2": .4, "B_Nm_s_rad": .9,
                      "Fc_Nm": .12 if friction else 0., "bias_Nm": .025}
        group = []
        t = np.arange(0, 6, .005)
        for number, frequency in enumerate((.3, .5, .8), 1):
            torque = np.sin(2*np.pi*frequency*t) + .3*np.sin(2*np.pi*1.7*t)
            trial = {"number": number, "profile": number-1, "t": t,
                     "theta": np.zeros_like(t), "omega": np.zeros_like(t), "u": torque}
            simulated = physical.simulate(trial, parameters)
            trial["theta"], trial["omega"] = simulated[:, 0], simulated[:, 1]
            group.append(trial)
        return parameters, group

    def test_recovers_inertia_damping_and_bias(self):
        expected, group = self.synthetic(False)
        result = physical.estimate(physical.integral_blocks(group, False), False)
        parameters = result["parameters_conditional_on_torque_calibration"]
        for key in ("J_kg_m2", "B_Nm_s_rad", "bias_Nm"):
            self.assertAlmostEqual(parameters[key]/expected[key], 1., delta=.02)
        metrics, _ = physical.evaluate(group, parameters)
        self.assertGreater(metrics["velocity_fit_percent"], 98)

    def test_recovers_opposing_friction(self):
        expected, group = self.synthetic(True)
        result = physical.estimate(physical.integral_blocks(group, True), True)
        parameters = result["parameters_conditional_on_torque_calibration"]
        for key in expected:
            self.assertAlmostEqual(parameters[key]/expected[key], 1., delta=.04)

    def test_future_measured_states_never_correct_prediction(self):
        parameters, group = self.synthetic(True)
        original = physical.simulate(group[0], parameters)
        changed = {key: value.copy() if isinstance(value, np.ndarray) else value
                   for key, value in group[0].items()}
        changed["theta"][1:] = 500
        changed["omega"][1:] = -500
        np.testing.assert_array_equal(physical.simulate(changed, parameters), original)

    def test_unexcited_data_is_not_a_physical_parameter_result(self):
        t = np.arange(0, 6, .01)
        trial = {"t": t, "theta": t*0, "omega": t*0, "u": t*0}
        with self.assertRaisesRegex(ValueError, "excitation"):
            physical.estimate(physical.integral_blocks([trial], False), False)

    def test_audit_includes_holding_and_rejects_legacy(self):
        data = {"/yaw_experiment/physical_identification": np.ones(20),
                "/yaw_experiment/yaw_pid_torque": np.zeros(20),
                "/yaw_experiment/yaw_feedforward": np.zeros(20)}
        self.assertTrue(physical.capture_is_pid_free(data))
        data["/yaw_experiment/yaw_pid_torque"][0] = .01
        self.assertFalse(physical.capture_is_pid_free(data))
        self.assertFalse(physical.capture_is_pid_free({}))
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/"legacy.csv"
            path.write_text("index\n0\n")
            with self.assertRaisesRegex(ValueError, "PID-free"):
                physical.analyze(path, 2, Path(folder)/"report")
            self.assertFalse((Path(folder)/"report").exists())


if __name__ == "__main__":
    unittest.main()
