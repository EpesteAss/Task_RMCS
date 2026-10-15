#!/usr/bin/env python3
"""Supervised repeated yaw sweeps. Starts motion, then sends OFF on every exit."""
import argparse
import json
import math
import signal
import time
from datetime import datetime
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--minutes", type=float, default=30.0)
    parser.add_argument("--trials", type=int, help="run exactly this many complete trials")
    parser.add_argument("--config", default="identification.yaml",
                        help="config file used by terminal A (duration is read from it)")
    parser.add_argument("--rest", type=float, default=5.0, help="holding seconds between trials")
    parser.add_argument("--stop-temp", type=float, default=55.0, help="early stop temperature C")
    parser.add_argument("--max-start-temp", type=float, default=55.0,
                        help="refuse to arm when either motor starts at or above this temperature")
    parser.add_argument("--yaw-stop-temp", type=float,
                        help="Yaw-only early stop; explicit physical-mode override")
    parser.add_argument("--yaw-max-start-temp", type=float,
                        help="Yaw-only start limit; explicit physical-mode override")
    parser.add_argument("--require-off-pitch-world-deg", type=float,
                        help="require zero-torque Pitch support at this world angle before Arm")
    parser.add_argument("--off-pitch-tolerance-deg", type=float, default=1.5)
    parser.add_argument("--require-passive-pitch", action="store_true",
                        help="refuse Arm unless controller confirms zero-torque passive Pitch mode")
    parser.add_argument("--require-mechanically-fixed-pitch", action="store_true",
                        help="require externally clamped Pitch and a matching zero-torque controller mode")
    parser.add_argument("--measured-yaw-shell-temp-c", type=float,
                        help="external Yaw shell temperature, required when Pitch is mechanically fixed")
    parser.add_argument("--operator-monitored-yaw-shell-temperature", action="store_true",
                        help="operator watches Yaw shell temperature during a fixed-Pitch trial")
    parser.add_argument("--require-controlled-pitch-parking", action="store_true",
                        help="require the raised Pitch mode to return under control before OFF")
    parser.add_argument("--require-off-slew-Nm-s", type=float,
                        help="require the running Pitch OFF torque slew to match this value")
    parser.add_argument("--pitch-cycle-only", action="store_true",
                        help="raise and return Pitch once without a Yaw sweep; requires --trials 0")
    args = parser.parse_args()
    yaw_stop_temp = args.yaw_stop_temp if args.yaw_stop_temp is not None else args.stop_temp
    yaw_max_start_temp = (args.yaw_max_start_temp if args.yaw_max_start_temp is not None
                          else args.max_start_temp)
    if not (0 < args.minutes <= 120 and 1 <= args.rest <= 60 and 30 <= args.stop_temp <= 64
            and 30 <= args.max_start_temp <= args.stop_temp
            and 30 <= yaw_max_start_temp <= yaw_stop_temp <= 130
            and (args.trials is None or 1 <= args.trials <= 30
                 or args.pitch_cycle_only and args.trials == 0)):
        parser.error("minutes: (0,120], rest: [1,60], pitch temperatures <=64, yaw temperatures <=130")
    if args.require_off_pitch_world_deg is not None and (
            not math.isfinite(args.require_off_pitch_world_deg)
            or not math.isfinite(args.off_pitch_tolerance_deg)
            or args.off_pitch_tolerance_deg <= 0):
        parser.error("OFF Pitch support target/tolerance must be finite and tolerance positive")
    if args.require_off_slew_Nm_s is not None and (
            not math.isfinite(args.require_off_slew_Nm_s)
            or not 0 < args.require_off_slew_Nm_s <= 1):
        parser.error("OFF Pitch slew must be in (0, 1] Nm/s")
    if args.require_mechanically_fixed_pitch and (
            not args.require_passive_pitch
            or (args.measured_yaw_shell_temp_c is None
                and not args.operator_monitored_yaw_shell_temperature)
            or (args.measured_yaw_shell_temp_c is not None
                and (not math.isfinite(args.measured_yaw_shell_temp_c)
                     or not 0 < args.measured_yaw_shell_temp_c < 55))):
        parser.error("Fixed Pitch requires passive mode and either a measured Yaw shell temperature below 55 C or explicit operator monitoring")
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Empty, String
    from std_srvs.srv import Trigger
    from operator_lock import acquire
    operator_lock = acquire()
    rclpy.init()
    node = Node("yaw_identification_session")
    status = {}
    last_status = 0.0
    last_print = 0.0
    completed = 0
    started = False
    physical = False
    events = []
    outcome = "interrupted"
    folder = Path(__file__).resolve().parent / "sessions"
    folder.mkdir(exist_ok=True)
    report = folder / (datetime.now().strftime("%Y%m%d_%H%M%S_%f") + ".json")
    begin = time.monotonic()

    def log(message):
        print(message, flush=True)
        events.append({"elapsed_s": time.monotonic() - begin, "message": message})
        report.write_text(json.dumps({"outcome": outcome, "completed_trials": completed,
                                     "events": events}, indent=2) + "\n")

    def receive(message):
        nonlocal last_status, last_print
        fields = {}
        try:
            for item in message.data.split():
                key, value = item.split("=", 1)
                fields[key] = float(value)
        except ValueError:
            return
        status.clear()
        status.update(fields)
        last_status = time.monotonic()
        if last_status - last_print >= 5:
            last_print = last_status
            print(message.data, flush=True)

    node.create_subscription(String, "/yaw_experiment/status", receive, 1)
    heartbeat = node.create_publisher(Empty, "/yaw_experiment/heartbeat", 1)
    node.create_timer(0.2, lambda: heartbeat.publish(Empty()))
    clients = {name: node.create_client(Trigger, "/yaw_experiment/" + name)
               for name in ("arm", "sweep", "off")}

    def spin(check=True):
        rclpy.spin_once(node, timeout_sec=0.05)
        if not check or not started:
            return
        if time.monotonic() - last_status > 2:
            raise RuntimeError("Status lost; stopping")
        if status.get("state") in (0, 4):
            raise RuntimeError(f"Controller stopped: state={status.get('state')} fault={status.get('fault')}")
        if any(not math.isfinite(status.get(key, float('nan')))
               for key in ("pitch_temp", "yaw_temp", "pitch_world_deg")):
            raise RuntimeError("Missing/nonfinite telemetry; rebuild the addon")
        if not args.require_mechanically_fixed_pitch and status["pitch_temp"] >= args.stop_temp:
            raise RuntimeError(f"Pitch temperature reached {args.stop_temp} C; cool before another run")
        if status["yaw_temp"] >= yaw_stop_temp:
            raise RuntimeError(f"Yaw temperature reached {yaw_stop_temp} C; cool before another run")
        if physical and (status.get("physical_identification") != 1 or status.get("yaw_pid_Nm") != 0):
            raise RuntimeError("Physical mode/PID-free telemetry mismatch; stopping")
        if args.require_passive_pitch and (status.get("passive_pitch") != 1
                                            or status.get("pitch_cmd_Nm") != 0):
            raise RuntimeError("Passive Pitch mode/zero torque telemetry mismatch; stopping")
        if args.require_mechanically_fixed_pitch and status.get("mechanically_fixed_pitch") != 1:
            raise RuntimeError("Mechanically fixed Pitch mode telemetry mismatch; stopping")
        if args.require_controlled_pitch_parking and status.get("controlled_pitch_parking") != 1:
            raise RuntimeError("Controlled Pitch parking telemetry mismatch; stopping")
        if args.require_off_slew_Nm_s is not None and (
                not math.isfinite(status.get("pitch_off_slew_Nm_s", float("nan")))
                or abs(status["pitch_off_slew_Nm_s"] - args.require_off_slew_Nm_s) > 1e-6):
            raise RuntimeError("Pitch OFF torque slew telemetry mismatch; stopping")
        if physical and status.get("physical_guard_stop", 0) != 0:
            raise RuntimeError("Physical trial aborted near yaw travel/speed guard; no automatic retry")

    def wait_for(condition, timeout, check=True):
        deadline = time.monotonic() + timeout
        while not condition():
            if time.monotonic() >= deadline:
                raise RuntimeError("Timed out waiting for controller state/service")
            spin(check)

    def send(name, check=True):
        client = clients[name]
        wait_for(client.service_is_ready, 5, check)
        future = client.call_async(Trigger.Request())
        wait_for(future.done, 3, check)
        response = future.result()
        if not response.success:
            raise RuntimeError(f"{name} rejected: {response.message}")
        log(f"{name}: {response.message}")

    def interrupted(signum, frame):
        raise KeyboardInterrupt

    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, interrupted)
    try:
        log("Keep this SSH terminal open. Motion starts now; Ctrl+C sends OFF. Keep Pitch at the required pose.")
        try:
            wait_for(lambda: bool(status), 10, False)
        except RuntimeError as error:
            raise RuntimeError(
                "No /yaw_experiment/status from terminal A. Check that its experiment "
                "controller started; another rmcs_executor may have blocked it. "
                "No Arm or Sweep was sent.") from error
        import yaml
        config_path = Path(__file__).resolve().parent / args.config
        config = yaml.safe_load(config_path.read_text())
        parameters = config["yaw_experiment"]["ros__parameters"]
        physical = parameters.get("physical_identification", False)
        if args.pitch_cycle_only and (args.trials != 0 or parameters.get("allow_sweep", True)):
            raise RuntimeError("Pitch cycle requires --trials 0 and a Yaw Sweep-disabled config")
        if args.require_controlled_pitch_parking and not (
                physical and parameters.get("controlled_pitch_parking") is True):
            raise RuntimeError("Selected configuration does not enable controlled Pitch parking")
        if args.require_off_slew_Nm_s is not None and (
                not physical or parameters.get("controlled_pitch_parking", False)
                or abs(float(parameters.get("pitch_off_slew_Nm_s", 0))
                       - args.require_off_slew_Nm_s) > 1e-6):
            raise RuntimeError("Selected configuration does not match the required OFF torque slew")
        fixed_tracking = (parameters.get("tracking_test") is True
                          and parameters.get("mechanically_fixed_pitch") is True)
        if args.require_passive_pitch and not (
                (physical or fixed_tracking) and parameters.get("passive_pitch") is True):
            raise RuntimeError("Selected configuration does not enable passive Pitch")
        if args.require_mechanically_fixed_pitch and not (
                (physical or fixed_tracking) and parameters.get("mechanically_fixed_pitch") is True
                and parameters.get("allow_arm") is True
                and parameters.get("allow_sweep") is True):
            raise RuntimeError("Selected configuration is not the enabled fixed-Pitch Yaw trial")
        if yaw_stop_temp > 64 or yaw_max_start_temp > 64:
            if (not (physical or fixed_tracking)
                    or physical and args.trials != (0 if args.pitch_cycle_only else 1)
                    or parameters.get("yaw_temperature_trip_c") != 130):
                raise RuntimeError("Yaw temperature override requires fixed-Pitch test mode and explicit 130 C controller trip")
        if physical and (status.get("physical_identification") != 1
                         or status.get("yaw_pid_Nm") != 0):
            raise RuntimeError("Terminal A is not running the new torque-only physical mode; rebuild and restart it")
        if args.require_passive_pitch and (status.get("passive_pitch") != 1
                                            or status.get("pitch_cmd_Nm") != 0):
            raise RuntimeError("Terminal A is not running passive Pitch with zero torque; rebuild and restart it")
        if args.require_mechanically_fixed_pitch and status.get("mechanically_fixed_pitch") != 1:
            raise RuntimeError("Terminal A is not running mechanically fixed Pitch mode; rebuild and restart it")
        if args.require_controlled_pitch_parking and status.get("controlled_pitch_parking") != 1:
            raise RuntimeError("Terminal A is not running controlled Pitch parking; rebuild and restart it")
        if args.require_off_slew_Nm_s is not None and (
                not math.isfinite(status.get("pitch_off_slew_Nm_s", float("nan")))
                or abs(status["pitch_off_slew_Nm_s"] - args.require_off_slew_Nm_s) > 1e-6):
            raise RuntimeError("Terminal A is not running the requested Pitch OFF torque slew")
        if status.get("state") != 0:
            raise RuntimeError("Controller must be OFF before session start; use console.py off")
        if args.require_off_pitch_world_deg is not None:
            if not physical:
                raise RuntimeError("OFF Pitch support check requires physical mode")
            support_deadline = time.monotonic() + 2.0
            while time.monotonic() < support_deadline:
                spin(False)
                actual = status.get("pitch_world_deg", float("nan"))
                if (time.monotonic() - last_status > 1 or not math.isfinite(actual)
                        or abs(actual - args.require_off_pitch_world_deg)
                        > args.off_pitch_tolerance_deg):
                    raise RuntimeError(
                        f"Pitch is not supported at {args.require_off_pitch_world_deg:g} deg "
                        f"under zero torque (observed {actual:g} deg); refusing Arm")
        if not args.require_mechanically_fixed_pitch and status.get("pitch_temp", 100) >= args.max_start_temp:
            raise RuntimeError(f"Pitch motor temperature must be below {args.max_start_temp:g} C before starting")
        if args.require_mechanically_fixed_pitch:
            if args.measured_yaw_shell_temp_c is not None:
                log(f"Operator measured Yaw shell temperature: {args.measured_yaw_shell_temp_c:g} C; Pitch is mechanically clamped")
            else:
                log("Operator is monitoring Yaw shell temperature; no new numerical measurement recorded; Pitch is mechanically clamped")
        if status.get("yaw_temp", 100) >= yaw_max_start_temp:
            raise RuntimeError(f"Yaw motor temperature must be below {yaw_max_start_temp:g} C before starting")
        # Discovery of the status subscriber and the heartbeat subscriber is
        # independent.  A queued arm before the first heartbeat is discarded
        # by the controller while the service still reports successful queueing.
        try:
            wait_for(lambda: 0 <= status.get("heartbeat_age_s", float("inf")) < 1.0,
                     8, False)
        except RuntimeError as error:
            raise RuntimeError("Heartbeat did not reach the controller; check terminal A logs") from error
        send("arm", False)
        try:
            wait_for(lambda: status.get("state") != 0, 3, False)
        except RuntimeError as error:
            raise RuntimeError("Arm was queued but controller stayed OFF; check terminal A for 'Arm ignored'") from error
        started = True
        wait_for(lambda: status.get("state") == 2, 15)
        if args.pitch_cycle_only:
            log("Pitch reached Ready; holding 1 s before controlled return")
            ready_until = time.monotonic() + 1.0
            while time.monotonic() < ready_until:
                spin()
        # Read the actual configured trial duration; do not lengthen the chirp to 30 minutes.
        duration = float(parameters["duration"])
        finish = time.monotonic() + args.minutes * 60
        def another_trial():
            return (completed < args.trials if args.trials is not None
                    else time.monotonic() + duration + 3 < finish)
        while another_trial():
            send("sweep")
            wait_for(lambda: status.get("state") == 3, 3)
            wait_for(lambda: status.get("state") != 3, duration + 5)
            if status.get("state") != 2 or status.get("completed") != 1:
                raise RuntimeError("Sweep interrupted; check pitch, clearance and fault before restarting")
            completed += 1
            log(f"Completed trial {completed}; holding for {args.rest:g} s")
            end_rest = (time.monotonic() + args.rest if args.trials is not None
                        else min(finish, time.monotonic() + args.rest))
            while time.monotonic() < end_rest:
                spin()
        outcome = "complete"
        log(f"Session complete: {completed} full sweeps")
    except KeyboardInterrupt:
        outcome = "operator_stop"
        log("Operator interrupted")
    except Exception as error:
        outcome = "stopped"
        log(str(error))
    finally:
        # Ignore repeated terminal signals while attempting a bounded OFF handshake.
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            signal.signal(sig, signal.SIG_IGN)
        if not status:
            log("No experiment status was received; no Arm or Sweep was sent")
        else:
            try:
                send("off", False)
                wait_for(lambda: status.get("state") == 0 and status.get("pitch_cmd_Nm") == 0
                         and status.get("yaw_cmd_Nm") == 0,
                         25 if args.require_off_slew_Nm_s is not None
                         else 16 if args.require_controlled_pitch_parking else 8, False)
                log("Confirmed OFF, both torque commands zero")
            except Exception as error:
                outcome = "stopped"
                log(f"OFF acknowledgement unavailable: {error}; heartbeat stops now")
        node.destroy_node()
        rclpy.shutdown()
        operator_lock.close()
        print(f"Session report: {report}", flush=True)
    return 0 if outcome == "complete" else 1


if __name__ == "__main__":
    raise SystemExit(main())
