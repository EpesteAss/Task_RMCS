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
    args = parser.parse_args()
    if not (0 < args.minutes <= 120 and 1 <= args.rest <= 60 and 30 <= args.stop_temp <= 64
            and 30 <= args.max_start_temp <= args.stop_temp
            and (args.trials is None or 1 <= args.trials <= 30)):
        parser.error("minutes: (0,120], rest: [1,60], temperatures: 30 <= start <= stop <= 64")
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
        if max(status["pitch_temp"], status["yaw_temp"]) >= args.stop_temp:
            raise RuntimeError(f"Temperature reached {args.stop_temp} C; cool before another run")

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
        log("Keep this SSH terminal open. Motion starts now; Ctrl+C sends OFF. Support pitch against dropping.")
        wait_for(lambda: bool(status), 10, False)
        if status.get("state") != 0:
            raise RuntimeError("Controller must be OFF before session start; use console.py off")
        if max(status.get("pitch_temp", 100), status.get("yaw_temp", 100)) >= args.max_start_temp:
            raise RuntimeError(f"Motor already warm; cool below {args.max_start_temp:g} C before starting")
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
        # Read the actual configured trial duration; do not lengthen the chirp to 30 minutes.
        import yaml
        config_path = Path(__file__).resolve().parent / args.config
        config = yaml.safe_load(config_path.read_text())
        duration = float(config["yaw_experiment"]["ros__parameters"]["duration"])
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
        try:
            send("off", False)
            wait_for(lambda: status.get("state") == 0 and status.get("pitch_cmd_Nm") == 0
                     and status.get("yaw_cmd_Nm") == 0, 8, False)
            log("Confirmed OFF, both torque commands zero")
        except Exception as error:
            log(f"OFF acknowledgement unavailable: {error}; heartbeat stops now")
        node.destroy_node()
        rclpy.shutdown()
        operator_lock.close()
        print(f"Session report: {report}", flush=True)
    return 0 if outcome == "complete" else 1


if __name__ == "__main__":
    raise SystemExit(main())
