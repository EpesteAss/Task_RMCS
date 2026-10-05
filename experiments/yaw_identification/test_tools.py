import tempfile
import unittest
from pathlib import Path
import numpy as np
import identify


class IdentificationTests(unittest.TestCase):
    def test_known_plant_and_free_run(self):
        t = np.arange(0, 20, .01)
        u = np.sin(2*np.pi*(.2*t+.05*t*t))
        v = np.zeros_like(t)
        theta = np.zeros_like(t)
        for i in range(len(t)-1):
            v[i+1] = np.exp(-3*.01)*v[i] + (1-np.exp(-3*.01))/3*(2*u[i]+.1)
            theta[i+1] = theta[i] + .01*(v[i]+v[i+1])/2
        trial = dict(t=t, u=u, omega=v, theta=theta, number=1)
        coeff, _, _ = identify.fit([trial], False)
        np.testing.assert_allclose(coeff, [-3, 2, .1], rtol=.02, atol=.005)
        metrics, _ = identify.evaluate([trial], coeff)
        self.assertFalse(metrics["simulation_diverged"])
        self.assertGreater(metrics["velocity_fit_percent"], 98)

    def test_truncated_final_record(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"sample.csv"
            path.write_text("a,b\n1,2\n3,")
            data = identify.read_log(path)
            self.assertEqual(list(data["a"]), [1])
            path.write_text("a,b\n1,2\n3,\n4,5\n")
            with self.assertRaises(ValueError):
                identify.read_log(path)

    def test_delayed_second_order_arx(self):
        t = np.arange(0, 20, .01)
        u = np.sin(2*np.pi*(.2*t+.05*t*t))
        theta = np.zeros_like(t)
        expected = np.array([1.7, -.72, .002, .0001])
        delay = 3
        for k in range(delay, len(t)):
            theta[k] = (expected[0]*theta[k-1] + expected[1]*theta[k-2]
                        + expected[2]*u[k-delay] + expected[3])
        trial = dict(t=t, u=u, omega=np.gradient(theta, t), theta=theta, number=1)
        coeff, _, _ = identify.fit_delayed_arx2([trial], delay)
        np.testing.assert_allclose(coeff, expected, rtol=1e-7, atol=1e-9)
        metrics, _ = identify.evaluate_delayed_arx2([trial], coeff, delay)
        self.assertGreater(metrics["velocity_fit_percent"], 99.9)
        self.assertGreater(metrics["angle_fit_percent"], 99.9)

    def test_incomplete_sweep_is_excluded(self):
        n = 600
        states = np.full(n, 3.)
        states[200:210] = 2
        states[410:420] = 4
        completed = np.zeros(n)
        completed[200:210] = 1
        data = {"/yaw_experiment/state": states,
                "/yaw_experiment/completed": completed,
                "/yaw_experiment/time_s": np.arange(n)*.01,
                "/gimbal/yaw/angle": np.arange(n)*.0001,
                "/gimbal/yaw/velocity_imu": np.zeros(n),
                "/gimbal/yaw/control_torque": np.ones(n)}
        accepted, rejected = identify.trials(data)
        self.assertEqual(len(accepted), 1)
        self.assertEqual(len(rejected), 2)


if __name__ == "__main__":
    unittest.main()
