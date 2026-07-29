from __future__ import annotations

import json
import threading
import time
from pathlib import Path


class StreamingDiagnostics:
    """Collect controller-side EEF streaming events and flush after motion."""

    def __init__(self, logger, output_dir: Path = Path("/tmp")) -> None:
        self._logger = logger
        self._output_dir = output_dir
        self._lock = threading.Lock()
        self._events: list[dict[str, object]] | None = None
        self._path: Path | None = None

    @property
    def active(self) -> bool:
        return self._events is not None

    def start(self, **metadata) -> Path:
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        suffix = time.time_ns() % 1_000_000_000
        path = self._output_dir / (
            f"rebotarm_controller_eef_streaming_{timestamp}_{suffix:09d}.jsonl"
        )
        event = {
            "event": "session_start",
            "monotonic_ns": time.monotonic_ns(),
            "recorded_monotonic_ns": time.monotonic_ns(),
            "thread_id": threading.get_ident(),
            "wall_time_ns": time.time_ns(),
            **metadata,
        }
        with self._lock:
            if self._events is not None:
                raise RuntimeError("EEF streaming diagnostics are already active")
            self._events = [event]
            self._path = path
        self._logger.info(f"controller diagnostic recording started: {path}")
        return path

    def record(
        self,
        event: str,
        *,
        monotonic_ns: int | None = None,
        **values,
    ) -> None:
        if self._events is None:
            return
        record = {
            "event": event,
            "monotonic_ns": (
                time.monotonic_ns() if monotonic_ns is None else monotonic_ns
            ),
            "recorded_monotonic_ns": time.monotonic_ns(),
            "thread_id": threading.get_ident(),
            **values,
        }
        with self._lock:
            if self._events is not None:
                self._events.append(record)

    def stop(self, reason: str) -> Path | None:
        with self._lock:
            if self._events is None or self._path is None:
                return None
            self._events.append(
                {
                    "event": "session_stop",
                    "monotonic_ns": time.monotonic_ns(),
                    "recorded_monotonic_ns": time.monotonic_ns(),
                    "thread_id": threading.get_ident(),
                    "wall_time_ns": time.time_ns(),
                    "reason": reason,
                }
            )
            events = self._events
            path = self._path
            self._events = None
            self._path = None

        events.sort(key=lambda item: int(item["monotonic_ns"]))
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", encoding="utf-8") as stream:
            for item in events:
                stream.write(
                    json.dumps(item, separators=(",", ":"), default=_json_default)
                    + "\n"
                )
        self._logger.info(f"controller diagnostic log saved to {path}")
        return path


def _json_default(value):
    if hasattr(value, "tolist"):
        return value.tolist()
    raise TypeError(f"cannot serialize {type(value).__name__}")
