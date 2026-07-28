from __future__ import annotations

from collections.abc import Sequence


def map_relative_position(
    initial_phone: Sequence[float],
    current_phone: Sequence[float],
    initial_eef: Sequence[float],
    scale: float,
) -> tuple[float, float, float]:
    return tuple(
        float(initial_eef[index])
        + float(scale)
        * (float(current_phone[index]) - float(initial_phone[index]))
        for index in range(3)
    )
