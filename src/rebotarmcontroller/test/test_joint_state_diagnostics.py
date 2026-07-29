import threading
import time
import unittest
from types import SimpleNamespace

import numpy as np
from builtin_interfaces.msg import Time

from rebotarmcontroller.ros_publishers import JointStatePublisher


class _Diagnostics:
    active = True

    def __init__(self) -> None:
        self.events = []
        self.lock = threading.Lock()

    def record(self, event: str, **values) -> None:
        with self.lock:
            self.events.append({"event": event, **values})


class _Publisher:
    def publish(self, _message) -> None:
        pass


class JointStateDiagnosticsTest(unittest.TestCase):
    def test_records_overlapping_publish_callbacks(self) -> None:
        barrier = threading.Barrier(2)

        def get_joint_state():
            barrier.wait(timeout=1.0)
            time.sleep(0.01)
            return np.array([0.1]), np.array([0.2]), np.array([0.3])

        hardware = SimpleNamespace(
            joint_names=["joint1"],
            get_joint_state=get_joint_state,
            get_joint_status_codes=lambda: [1],
        )
        clock = SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=1, nanosec=2))
        )
        node = SimpleNamespace(
            get_clock=lambda: clock,
            get_logger=lambda: SimpleNamespace(warn=lambda _message: None),
        )
        diagnostics = _Diagnostics()
        publisher = JointStatePublisher.__new__(JointStatePublisher)
        publisher._node = node
        publisher._hardware = hardware
        publisher._diagnostics = diagnostics
        publisher._publish_state_lock = threading.Lock()
        publisher._active_publishes = 0
        publisher._publish_sequence = 0
        publisher._joint_state_publishers = {"joint1": _Publisher()}
        publisher._gripper_state_publisher = None
        publisher._publisher = _Publisher()

        threads = [
            threading.Thread(target=publisher.publish),
            threading.Thread(target=publisher.publish),
        ]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(timeout=1.0)

        self.assertTrue(all(not thread.is_alive() for thread in threads))
        self.assertEqual(len(diagnostics.events), 2)
        self.assertEqual(
            max(event["concurrent_callbacks"] for event in diagnostics.events),
            2,
        )
        self.assertTrue(
            all(event["outcome"] == "published" for event in diagnostics.events)
        )
        self.assertTrue(
            all(event["position"] == [0.1] for event in diagnostics.events)
        )


if __name__ == "__main__":
    unittest.main()
