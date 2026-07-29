import json
import tempfile
import time
import unittest
from pathlib import Path

from rebotarmcontroller.streaming_diagnostics import StreamingDiagnostics


class _Logger:
    def __init__(self) -> None:
        self.messages = []

    def info(self, message: str) -> None:
        self.messages.append(message)


class StreamingDiagnosticsTest(unittest.TestCase):
    def test_records_sorted_jsonl_session(self) -> None:
        logger = _Logger()
        with tempfile.TemporaryDirectory() as directory:
            diagnostics = StreamingDiagnostics(logger, Path(directory))
            path = diagnostics.start(control_rate_hz=50.0)
            late_ns = time.monotonic_ns()
            diagnostics.record("late", monotonic_ns=late_ns, values=[1.0, 2.0])
            diagnostics.record("early", monotonic_ns=late_ns - 1)
            saved_path = diagnostics.stop("test_complete")

            self.assertEqual(saved_path, path)
            self.assertFalse(diagnostics.active)
            with path.open(encoding="utf-8") as stream:
                events = [json.loads(line) for line in stream]

        monotonic_ns = [event["monotonic_ns"] for event in events]
        self.assertEqual(monotonic_ns, sorted(monotonic_ns))
        self.assertEqual(events[0]["event"], "session_start")
        self.assertEqual(events[1]["event"], "early")
        self.assertEqual(events[2]["event"], "late")
        self.assertEqual(events[-1]["event"], "session_stop")
        self.assertEqual(events[-1]["reason"], "test_complete")
        self.assertTrue(any("saved" in message for message in logger.messages))

    def test_record_is_ignored_outside_session(self) -> None:
        diagnostics = StreamingDiagnostics(_Logger())

        diagnostics.record("ignored")

        self.assertIsNone(diagnostics.stop("not_active"))


if __name__ == "__main__":
    unittest.main()
