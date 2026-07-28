import unittest

import numpy as np

from rebotarm_phone_bridge.calibration import (
    calibrate_world_to_base,
    matrix_to_quaternion_xyzw,
    R_BC_KNOWN,
    transform_pose_world_to_base,
)


class CalibrationTests(unittest.TestCase):
    def test_known_pose_produces_identity_world_to_base(self):
        quaternion = matrix_to_quaternion_xyzw(R_BC_KNOWN)
        result = calibrate_world_to_base(
            [quaternion] * 20,
            outlier_threshold_rad=np.deg2rad(3.0),
            min_inlier_samples=10,
        )

        np.testing.assert_allclose(result.R_BW, np.eye(3), atol=1e-12)
        self.assertEqual(result.inlier_count, 20)
        self.assertEqual(result.outlier_count, 0)

    def test_outlier_is_rejected(self):
        quaternion = matrix_to_quaternion_xyzw(R_BC_KNOWN)
        outlier = np.array([0.0, 0.0, np.sin(0.3), np.cos(0.3)])
        result = calibrate_world_to_base(
            [quaternion] * 20 + [outlier],
            outlier_threshold_rad=np.deg2rad(3.0),
            min_inlier_samples=10,
        )

        self.assertEqual(result.inlier_count, 20)
        self.assertEqual(result.outlier_count, 1)

    def test_transform_rotates_position_and_orientation(self):
        angle = np.pi / 2.0
        R_BW = np.array(
            [
                [np.cos(angle), -np.sin(angle), 0.0],
                [np.sin(angle), np.cos(angle), 0.0],
                [0.0, 0.0, 1.0],
            ]
        )
        position, quaternion = transform_pose_world_to_base(
            R_BW,
            [1.0, 0.0, 2.0],
            [0.0, 0.0, 0.0, 1.0],
        )

        np.testing.assert_allclose(position, [0.0, 1.0, 2.0], atol=1e-12)
        self.assertAlmostEqual(abs(float(quaternion[2])), np.sqrt(0.5))
        self.assertAlmostEqual(abs(float(quaternion[3])), np.sqrt(0.5))


if __name__ == "__main__":
    unittest.main()
