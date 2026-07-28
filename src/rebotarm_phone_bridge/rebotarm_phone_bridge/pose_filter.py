from __future__ import annotations

import math
from typing import Iterable

import numpy as np

from .calibration import normalize_quaternion_xyzw


class PoseLowPassFilter:
    def __init__(self, time_constant: float) -> None:
        self._time_constant = float(time_constant)
        self.reset()

    def reset(self) -> None:
        self._position: np.ndarray | None = None
        self._quaternion: np.ndarray | None = None
        self._timestamp: float | None = None

    def update(
        self,
        position: Iterable[float],
        quaternion: Iterable[float],
        timestamp: float,
    ) -> tuple[np.ndarray, np.ndarray]:
        current_position = np.asarray(tuple(position), dtype=np.float64)
        current_quaternion = normalize_quaternion_xyzw(quaternion)
        if self._timestamp is None:
            self._position = current_position.copy()
            self._quaternion = current_quaternion.copy()
        else:
            alpha = -math.expm1(
                -max(float(timestamp) - self._timestamp, 0.0)
                / self._time_constant
            )
            self._position += alpha * (current_position - self._position)
            self._quaternion = _slerp(
                self._quaternion,
                current_quaternion,
                alpha,
            )
        self._timestamp = float(timestamp)
        return self._position.copy(), self._quaternion.copy()


def _slerp(start: np.ndarray, end: np.ndarray, ratio: float) -> np.ndarray:
    target = end.copy()
    dot = float(start @ target)
    if dot < 0.0:
        target *= -1.0
        dot *= -1.0
    dot = float(np.clip(dot, -1.0, 1.0))
    if dot > 0.9995:
        return normalize_quaternion_xyzw(
            start + ratio * (target - start)
        )

    angle = math.acos(dot)
    scale = math.sin(angle)
    result = (
        math.sin((1.0 - ratio) * angle) / scale * start
        + math.sin(ratio * angle) / scale * target
    )
    return normalize_quaternion_xyzw(result)
