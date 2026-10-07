#!/usr/bin/env python3
"""Request experiment OFF and verify both commanded torques reach zero."""
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger


def main():
    rclpy.init()
    node = Node("yaw_experiment_safe_off")
    status = {}

    def receive(message):
        fields = {}
        try:
            for item in message.data.split():
                key, value = item.split("=", 1)
                fields[key] = float(value)
        except ValueError:
            return
        status.clear()
        status.update(fields)

    node.create_subscription(String, "/yaw_experiment/status", receive, 1)
    heartbeat = node.create_publisher(Empty, "/yaw_experiment/heartbeat", 1)
    node.create_timer(0.2, lambda: heartbeat.publish(Empty()))
    client = node.create_client(Trigger, "/yaw_experiment/off")

    deadline = time.monotonic() + 5.0
    while not client.service_is_ready() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not client.service_is_ready():
        raise SystemExit("OFF service unavailable: cut motor power; do not rely on pkill")

    future = client.call_async(Trigger.Request())
    deadline = time.monotonic() + 3.0
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not future.done() or not future.result().success:
        raise SystemExit("OFF request failed: cut motor power")

    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if (status.get("state") == 0
                and math.isclose(status.get("yaw_cmd_Nm", math.inf), 0.0, abs_tol=1e-9)
                and math.isclose(status.get("pitch_cmd_Nm", math.inf), 0.0, abs_tol=1e-9)):
            print("Confirmed OFF: yaw_cmd_Nm=0 and pitch_cmd_Nm=0")
            node.destroy_node()
            rclpy.shutdown()
            return
    raise SystemExit(f"OFF not confirmed within 10 s; last status: {status}. Cut motor power")


if __name__ == "__main__":
    main()
