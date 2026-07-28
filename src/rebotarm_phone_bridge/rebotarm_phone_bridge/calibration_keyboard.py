from __future__ import annotations

import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


def _read_key() -> str:
    descriptor = sys.stdin.fileno()
    previous = termios.tcgetattr(descriptor)
    try:
        tty.setcbreak(descriptor)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(descriptor, termios.TCSADRAIN, previous)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Node("phone_calibration_keyboard")
    client = node.create_client(Trigger, "/phone_tracking/calibrate")
    try:
        if not sys.stdin.isatty():
            raise RuntimeError("keyboard client requires an interactive terminal")
        print("Press c to calibrate, q to quit.", flush=True)
        while rclpy.ok():
            key = _read_key().lower()
            if key == "q":
                break
            if key != "c":
                continue
            if not client.wait_for_service(timeout_sec=1.0):
                print("Calibration service is not available.", flush=True)
                continue
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, future)
            response = future.result()
            if response is None:
                print("Calibration request failed.", flush=True)
            else:
                print(response.message, flush=True)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
