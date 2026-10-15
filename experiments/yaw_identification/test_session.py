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
    def run_session(self, fault=False, hot=False, trials=None, physical=False, running_physical=True,
                    off_ack=True, off_pitch_world_deg=5, required_off_pitch_world_deg=None,
                    passive=False, running_passive=True, controlled_parking=False,
                    running_controlled_parking=True, pitch_cycle=False,
                    required_off_slew=None, running_off_slew=None, silent=False,
                    fixed_pitch=False, running_fixed_pitch=True, operator_monitored=False):
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
                    next_state = dict(arm=1, sweep=3, off=0)[name]
                    if name == "off" and not off_ack:
                        next_state = 4
                    state[:] = [next_state, clock[0], 0]
                    return types.SimpleNamespace(done=lambda: True,
                        result=lambda: types.SimpleNamespace(success=True, message="Queued"))
                return types.SimpleNamespace(service_is_ready=lambda: True, call_async=call)
            def destroy_node(self): pass

        def spin(node, timeout_sec):
            clock[0] += .05
            if silent:
                return
            if state[0] == 1 and clock[0]-state[1] > .2:
                state[0] = 2
            if state[0] == 3 and clock[0]-state[1] > 1:
                state[0], state[2] = (4, 0) if fault else (2, 1)
            temp = 56 if hot and "sweep" in commands else (0 if fixed_pitch else 30)
            torque = 0 if passive or state[0] in (0, 4) else -3
            message = (f"state={state[0]} fault={5 if state[0] == 4 else 0} completed={state[2]} "
                       f"heartbeat_age_s=0.05 "
                       f"pitch_temp={temp} yaw_temp={125 if fixed_pitch else 30} pitch_world_deg={off_pitch_world_deg} "
                       f"pitch_cmd_Nm={torque} yaw_cmd_Nm=0 "
                       f"physical_identification={int(running_physical)} passive_pitch={int(running_passive)} "
                       f"mechanically_fixed_pitch={int(running_fixed_pitch)} "
                       f"controlled_pitch_parking={int(running_controlled_parking)} "
                       f"pitch_off_slew_Nm_s={running_off_slew if running_off_slew is not None else 1.0} "
                       f"yaw_pid_Nm=0 physical_guard_stop=0")
            callback[0](types.SimpleNamespace(data=message))

        fake = {
            "rclpy": types.SimpleNamespace(init=lambda: None, shutdown=lambda: None, spin_once=spin),
            "rclpy.node": types.SimpleNamespace(Node=Node),
            "std_msgs.msg": types.SimpleNamespace(Empty=object, String=object),
            "std_srvs.srv": types.SimpleNamespace(Trigger=types.SimpleNamespace(Request=object)),
            "operator_lock": types.SimpleNamespace(acquire=lambda: types.SimpleNamespace(close=lambda: None)),
        }
        with tempfile.TemporaryDirectory() as directory:
            (Path(directory)/"identification.yaml").write_text(
                "yaw_experiment:\n  ros__parameters:\n    duration: 1\n"
                + f"    physical_identification: {str(physical).lower()}\n"
                + f"    passive_pitch: {str(passive).lower()}\n"
                + f"    mechanically_fixed_pitch: {str(fixed_pitch).lower()}\n"
                + f"    controlled_pitch_parking: {str(controlled_parking).lower()}\n"
                + f"    pitch_off_slew_Nm_s: {required_off_slew if required_off_slew is not None else 1.0}\n"
                + f"    allow_arm: true\n"
                + f"    allow_sweep: {str(not pitch_cycle).lower()}\n"
                + f"    yaw_temperature_trip_c: {130 if fixed_pitch else 65}\n")
            argv = ["session.py", "--minutes", ".2", "--rest", "1"]
            if trials is not None:
                argv += ["--trials", str(trials)]
            if required_off_pitch_world_deg is not None:
                argv += ["--require-off-pitch-world-deg", str(required_off_pitch_world_deg)]
            if passive:
                argv += ["--require-passive-pitch"]
            if fixed_pitch:
                argv += ["--require-mechanically-fixed-pitch", "--yaw-max-start-temp", "126",
                         "--yaw-stop-temp", "130"]
                argv += (["--operator-monitored-yaw-shell-temperature"] if operator_monitored
                         else ["--measured-yaw-shell-temp-c", "30"])
            if controlled_parking:
                argv += ["--require-controlled-pitch-parking"]
            if pitch_cycle:
                argv += ["--pitch-cycle-only"]
            if required_off_slew is not None:
                argv += ["--require-off-slew-Nm-s", str(required_off_slew)]
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

    def test_physical_session_refuses_old_running_controller_before_arm(self):
        result, commands = self.run_session(physical=True, running_physical=False, trials=1)
        self.assertEqual(result, 1)
        self.assertNotIn("arm", commands)
        self.assertNotIn("sweep", commands)
        self.assertEqual(commands[-1], "off")

    def test_physical_session_completes_with_mode_telemetry(self):
        result, commands = self.run_session(physical=True, trials=1)
        self.assertEqual(result, 0)
        self.assertEqual(commands.count("sweep"), 1)
        self.assertEqual(commands[-1], "off")

    def test_completed_sweep_without_off_ack_reports_failure(self):
        result, commands = self.run_session(physical=True, trials=1, off_ack=False)
        self.assertEqual(result, 1)
        self.assertEqual(commands.count("sweep"), 1)
        self.assertEqual(commands[-1], "off")

    def test_off_pitch_support_check_refuses_arm_at_low_position(self):
        result, commands = self.run_session(physical=True, trials=1, off_pitch_world_deg=-8,
                                            required_off_pitch_world_deg=5)
        self.assertEqual(result, 1)
        self.assertNotIn("arm", commands)
        self.assertNotIn("sweep", commands)
        self.assertEqual(commands[-1], "off")

    def test_off_pitch_support_check_accepts_held_position(self):
        result, commands = self.run_session(physical=True, trials=1,
                                            required_off_pitch_world_deg=5)
        self.assertEqual(result, 0)
        self.assertEqual(commands.count("sweep"), 1)

    def test_passive_mode_refuses_mismatched_controller_before_arm(self):
        result, commands = self.run_session(physical=True, passive=True, running_passive=False,
                                            off_pitch_world_deg=-8, trials=1,
                                            required_off_pitch_world_deg=-8)
        self.assertEqual(result, 1)
        self.assertNotIn("arm", commands)
        self.assertEqual(commands[-1], "off")

    def test_passive_mode_one_trial(self):
        result, commands = self.run_session(physical=True, passive=True, off_pitch_world_deg=-8,
                                            trials=1, required_off_pitch_world_deg=-8)
        self.assertEqual(result, 0)
        self.assertEqual(commands.count("sweep"), 1)

    def test_fixed_pitch_without_motor_feedback_one_trial(self):
        result, commands = self.run_session(physical=True, passive=True, fixed_pitch=True,
                                            trials=1, off_pitch_world_deg=-8)
        self.assertEqual(result, 0)
        self.assertEqual(commands.count("sweep"), 1)
        self.assertEqual(commands[-1], "off")

    def test_fixed_pitch_with_operator_temperature_monitoring(self):
        result, commands = self.run_session(physical=True, passive=True, fixed_pitch=True,
                                            operator_monitored=True, trials=1)
        self.assertEqual(result, 0)
        self.assertEqual(commands.count("sweep"), 1)
        self.assertEqual(commands[-1], "off")

    def test_fixed_pitch_refuses_wrong_running_mode(self):
        result, commands = self.run_session(physical=True, passive=True, fixed_pitch=True,
                                            running_fixed_pitch=False, trials=1)
        self.assertEqual(result, 1)
        self.assertNotIn("arm", commands)

    def test_raised_pitch_cycle_without_yaw_sweep(self):
        result, commands = self.run_session(physical=True, controlled_parking=True,
                                            pitch_cycle=True, trials=0,
                                            off_pitch_world_deg=-8,
                                            required_off_pitch_world_deg=-8)
        self.assertEqual(result, 0)
        self.assertIn("arm", commands)
        self.assertNotIn("sweep", commands)
        self.assertEqual(commands[-1], "off")

    def test_raised_pitch_cycle_refuses_wrong_controller(self):
        result, commands = self.run_session(physical=True, controlled_parking=True,
                                            running_controlled_parking=False,
                                            pitch_cycle=True, trials=0)
        self.assertEqual(result, 1)
        self.assertNotIn("arm", commands)

    def test_slow_release_cycle_without_yaw_sweep(self):
        result, commands = self.run_session(physical=True, pitch_cycle=True, trials=0,
                                            required_off_slew=.3, running_off_slew=.3,
                                            off_pitch_world_deg=-8,
                                            required_off_pitch_world_deg=-8)
        self.assertEqual(result, 0)
        self.assertIn("arm", commands)
        self.assertNotIn("sweep", commands)
        self.assertEqual(commands[-1], "off")

    def test_slow_release_cycle_refuses_wrong_running_slew(self):
        result, commands = self.run_session(physical=True, pitch_cycle=True, trials=0,
                                            required_off_slew=.3, running_off_slew=1.0)
        self.assertEqual(result, 1)
        self.assertNotIn("arm", commands)

    def test_no_experiment_controller_sends_no_command(self):
        result, commands = self.run_session(physical=True, pitch_cycle=True,
                                            trials=0, required_off_slew=.3,
                                            silent=True)
        self.assertEqual(result, 1)
        self.assertEqual(commands, [])


if __name__ == "__main__":
    unittest.main()
