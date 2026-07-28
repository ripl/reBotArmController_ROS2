from __future__ import annotations

from collections.abc import Sequence
import math


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


def normalize_quaternion(
    quaternion: Sequence[float],
) -> tuple[float, float, float, float]:
    values = tuple(float(quaternion[index]) for index in range(4))
    norm = math.sqrt(sum(value * value for value in values))
    if not all(math.isfinite(value) for value in values) or norm < 1e-6:
        raise ValueError("quaternion must be finite and non-zero")
    return tuple(value / norm for value in values)


def multiply_quaternions(
    left: Sequence[float],
    right: Sequence[float],
) -> tuple[float, float, float, float]:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def map_relative_orientation(
    initial_phone: Sequence[float],
    current_phone: Sequence[float],
    initial_eef: Sequence[float],
) -> tuple[float, float, float, float]:
    phone_initial = normalize_quaternion(initial_phone)
    phone_current = normalize_quaternion(current_phone)
    eef_initial = normalize_quaternion(initial_eef)
    if sum(a * b for a, b in zip(phone_initial, phone_current)) < 0.0:
        phone_current = tuple(-value for value in phone_current)
    phone_initial_inverse = (
        -phone_initial[0],
        -phone_initial[1],
        -phone_initial[2],
        phone_initial[3],
    )
    relative_phone = multiply_quaternions(
        phone_current,
        phone_initial_inverse,
    )
    target = normalize_quaternion(
        multiply_quaternions(relative_phone, eef_initial)
    )
    if sum(a * b for a, b in zip(target, eef_initial)) < 0.0:
        target = tuple(-value for value in target)
    return target
