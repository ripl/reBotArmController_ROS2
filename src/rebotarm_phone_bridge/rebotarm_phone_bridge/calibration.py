from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import numpy as np


R_BC_KNOWN = np.array(
    [
        [0.0, -1.0, 0.0],
        [-1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
    ],
    dtype=np.float64,
)


def normalize_quaternion_xyzw(quaternion: Iterable[float]) -> np.ndarray:
    values = np.asarray(tuple(quaternion), dtype=np.float64)
    if values.shape != (4,) or not np.all(np.isfinite(values)):
        raise ValueError("quaternion must contain four finite values")
    norm = float(np.linalg.norm(values))
    if norm < 1e-12:
        raise ValueError("quaternion norm is zero")
    return values / norm


def quaternion_xyzw_to_matrix(quaternion: Iterable[float]) -> np.ndarray:
    x, y, z, w = normalize_quaternion_xyzw(quaternion)
    return np.array(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ],
        dtype=np.float64,
    )


def matrix_to_quaternion_xyzw(rotation: np.ndarray) -> np.ndarray:
    matrix = np.asarray(rotation, dtype=np.float64)
    _validate_rotation(matrix)
    davenport = np.array(
        [
            [
                matrix[0, 0] - matrix[1, 1] - matrix[2, 2],
                matrix[0, 1] + matrix[1, 0],
                matrix[0, 2] + matrix[2, 0],
                matrix[2, 1] - matrix[1, 2],
            ],
            [
                matrix[0, 1] + matrix[1, 0],
                matrix[1, 1] - matrix[0, 0] - matrix[2, 2],
                matrix[1, 2] + matrix[2, 1],
                matrix[0, 2] - matrix[2, 0],
            ],
            [
                matrix[0, 2] + matrix[2, 0],
                matrix[1, 2] + matrix[2, 1],
                matrix[2, 2] - matrix[0, 0] - matrix[1, 1],
                matrix[1, 0] - matrix[0, 1],
            ],
            [
                matrix[2, 1] - matrix[1, 2],
                matrix[0, 2] - matrix[2, 0],
                matrix[1, 0] - matrix[0, 1],
                matrix[0, 0] + matrix[1, 1] + matrix[2, 2],
            ],
        ],
        dtype=np.float64,
    )
    eigenvalues, eigenvectors = np.linalg.eigh(davenport)
    quaternion = eigenvectors[:, int(np.argmax(eigenvalues))]
    if quaternion[3] < 0.0:
        quaternion = -quaternion
    return normalize_quaternion_xyzw(quaternion)


def _average_quaternions(quaternions: np.ndarray) -> np.ndarray:
    if quaternions.ndim != 2 or quaternions.shape[1] != 4 or len(quaternions) == 0:
        raise ValueError("at least one shape-(4,) quaternion is required")
    normalized = np.stack(
        [normalize_quaternion_xyzw(quaternion) for quaternion in quaternions]
    )
    reference = normalized[0]
    aligned = np.stack(
        [
            quaternion if float(quaternion @ reference) >= 0.0 else -quaternion
            for quaternion in normalized
        ]
    )
    accumulator = aligned.T @ aligned
    eigenvalues, eigenvectors = np.linalg.eigh(accumulator)
    average = eigenvectors[:, int(np.argmax(eigenvalues))]
    if float(average @ reference) < 0.0:
        average = -average
    return normalize_quaternion_xyzw(average)


def _validate_rotation(rotation: np.ndarray) -> None:
    if rotation.shape != (3, 3) or not np.all(np.isfinite(rotation)):
        raise ValueError("rotation must be a finite 3x3 matrix")
    orthogonality_error = float(
        np.linalg.norm(rotation.T @ rotation - np.eye(3), ord="fro")
    )
    determinant = float(np.linalg.det(rotation))
    if orthogonality_error > 1e-6 or abs(determinant - 1.0) > 1e-6:
        raise ValueError("matrix is not a proper rotation")


def _rotation_angle(first: np.ndarray, second: np.ndarray) -> float:
    cosine = (float(np.trace(first.T @ second)) - 1.0) / 2.0
    return float(np.arccos(np.clip(cosine, -1.0, 1.0)))


@dataclass(frozen=True)
class CalibrationResult:
    R_BW: np.ndarray
    inlier_count: int
    outlier_count: int
    reconstruction_error_rad: float


def calibrate_world_to_base(
    quaternion_samples: Iterable[Iterable[float]],
    *,
    outlier_threshold_rad: float,
    min_inlier_samples: int,
) -> CalibrationResult:
    samples = np.stack(
        [normalize_quaternion_xyzw(sample) for sample in quaternion_samples]
    )
    if not np.isfinite(outlier_threshold_rad) or outlier_threshold_rad <= 0.0:
        raise ValueError("outlier threshold must be positive")
    if min_inlier_samples <= 0:
        raise ValueError("min_inlier_samples must be positive")

    initial = _average_quaternions(samples)
    dots = np.clip(np.abs(samples @ initial), 0.0, 1.0)
    errors = 2.0 * np.arccos(dots)
    inlier_mask = errors <= outlier_threshold_rad
    inlier_count = int(np.count_nonzero(inlier_mask))
    if inlier_count < min_inlier_samples:
        raise ValueError(
            f"only {inlier_count} calibration inliers; need {min_inlier_samples}"
        )

    average = _average_quaternions(samples[inlier_mask])
    R_WC_average = quaternion_xyzw_to_matrix(average)
    R_BW = R_BC_KNOWN @ R_WC_average.T
    _validate_rotation(R_BW)
    reconstruction_error = _rotation_angle(R_BW @ R_WC_average, R_BC_KNOWN)
    return CalibrationResult(
        R_BW=R_BW,
        inlier_count=inlier_count,
        outlier_count=int(len(samples) - inlier_count),
        reconstruction_error_rad=reconstruction_error,
    )


def transform_pose_world_to_base(
    R_BW: np.ndarray,
    position_WC: Iterable[float],
    quaternion_WC_xyzw: Iterable[float],
) -> tuple[np.ndarray, np.ndarray]:
    rotation = np.asarray(R_BW, dtype=np.float64)
    position = np.asarray(tuple(position_WC), dtype=np.float64)
    _validate_rotation(rotation)
    if position.shape != (3,) or not np.all(np.isfinite(position)):
        raise ValueError("position must contain three finite values")
    R_BC = rotation @ quaternion_xyzw_to_matrix(quaternion_WC_xyzw)
    return rotation @ position, matrix_to_quaternion_xyzw(R_BC)
