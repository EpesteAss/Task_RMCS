"""Exercise orchestration without ROS or motors, using a virtual clock."""
import contextlib
import io
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest.mock import patch
import session


class SessionTests(unittest.TestCase):
    def run_session(self, fault=False, hot=False, trials=None):
        clock = [0.0]
        state = [0, 0.0, 0]
        commands = []
        callback = [None]

        class Node:
            def __init__(self, name): pass
            def create_subscription(self, cls, topic, fn, depth): callback[0] = fn
            def create_publisher(self, *args): return types.SimpleNamespace(publish=lambda msg: None)
            def create_timer(self, *args): pass
            def create_client(self, cls, topic):
                name = topic.rsplit("/", 1)[-1]
                def call(request):
                    commands.append(name)
                    state[:] = [dict(arm=1, sweep=3, off=0)[name], clock[0], 0]
                    return types.SimpleNamespace(done=lambda: True,
                        result=lambda: types.SimpleNamespace(success=True, message="Queued"))
                return types.SimpleNamespace(service_is_ready=lambda: True, call_async=call)
            def destroy_node(self): pass

        def spin(node, timeout_sec):
            clock[0] += .05
            if state[0] == 1 and clock[0]-state[1] > .2:
                state[0] = 2
            if state[0] == 3 and clock[0]-state[1] > 1:
                state[0], state[2] = (4, 0) if fault else (2, 1)
            temp = 56 if hot and "sweep" in commands else 30
            torque = 0 if state[0] in (0, 4) else -3
            message = (f"state={state[0]} fault={5 if state[0] == 4 else 0} completed={state[2]} "
                       f"heartbeat_age_s=0.05 "
                       f"pitch_temp={temp} yaw_temp=30 pitch_world_deg=5 pitch_cmd_Nm={torque} yaw_cmd_Nm=0")
            callback[0](types.SimpleNamespace(data=message))

        fake = {
            "rclpy": types.SimpleNamespace(init=lambda: None, shutdown=lambda: None, spin_once=spin),
            "rclpy.node": types.SimpleNamespace(Node=Node),
            "std_msgs.msg": types.SimpleNamespace(Empty=object, String=object),
            "std_srvs.srv": types.SimpleNamespace(Trigger=types.SimpleNamespace(Request=object)),
            "operator_lock": types.SimpleNamespace(acquire=lambda: types.SimpleNamespace(close=lambda: None)),
        }
        with tempfile.TemporaryDirectory() as directory:
            (Path(directory)/"identification.yaml").write_text("yaw_experiment:\n  ros__parameters:\n    duration: 1\n")
            argv = ["session.py", "--minutes", ".2", "--rest", "1"]
            if trials is not None:
                argv += ["--trials", str(trials)]
            with patch.dict(sys.modules, fake), patch.object(session, "__file__", str(Path(directory)/"session.py")), \
                 patch.object(sys, "argv", argv), \
                 patch.object(session.time, "monotonic", lambda: clock[0]), \
                 patch.object(session.signal, "signal"), contextlib.redirect_stdout(io.StringIO()):
                result = session.main()
        return result, commands

    def test_complete_and_off(self):
        result, commands = self.run_session()
        self.assertEqual(result, 0)
        self.assertGreater(commands.count("sweep"), 1)
        self.assertEqual(commands[-1], "off")

    def test_fault_does_not_retry(self):
        result, commands = self.run_session(fault=True)
        self.assertEqual(result, 1)
        self.assertEqual(commands.count("sweep"), 1)
        self.assertEqual(commands[-1], "off")

    def test_heat_stops(self):
        result, commands = self.run_session(hot=True)
        self.assertEqual(result, 1)
        self.assertEqual(commands.count("sweep"), 1)
        self.assertEqual(commands[-1], "off")

    def test_exact_trial_count(self):
        result, commands = self.run_session(trials=3)
        self.assertEqual(result, 0)
        self.assertEqual(commands.count("sweep"), 3)
        self.assertEqual(commands[-1], "off")


if __name__ == "__main__":
    unittest.main()
