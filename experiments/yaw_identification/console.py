#!/usr/bin/env python3
"""Run in a second SSH terminal on the robot; heartbeat stops when this exits."""
import select
import sys
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger


def main():
    from operator_lock import acquire
    operator_lock = acquire()
    rclpy.init()
    node = Node("yaw_experiment_console")
    publisher = node.create_publisher(Empty, "/yaw_experiment/heartbeat", 1)
    node.create_timer(0.2, lambda: publisher.publish(Empty()))
    node.create_subscription(String, "/yaw_experiment/status", lambda m: print(m.data), 1)
    clients = {name: node.create_client(Trigger, "/yaw_experiment/" + name)
               for name in ("arm", "sweep", "hold", "off")}

    def send(name):
        if not clients[name].service_is_ready():
            print("Service unavailable; verify the controller is running")
            return
        future = clients[name].call_async(Trigger.Request())
        future.add_done_callback(lambda f: print(f.result().message))
        return future

    print("arm=raise/hold pitch; sweep=start yaw sweep; hold=cancel sweep/keep holding; off=zero both torques; quit=off and exit")
    print("States: 0 OFF, 1 RAISING, 2 READY, 3 SWEEP, 4 FAULT. Heartbeat loss zeros both torques after 1.5 seconds.")
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if select.select([sys.stdin], [], [], 0)[0]:
                line = sys.stdin.readline()
                if not line or line.strip() == "quit":
                    break
                command = line.strip()
                if command in clients:
                    send(command)
                else:
                    print("Use arm / sweep / hold / off / quit")
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            future = send("off")
            deadline = time.monotonic() + 0.5
            while future is not None and not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
