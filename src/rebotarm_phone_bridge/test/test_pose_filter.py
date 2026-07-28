import math
import unittest

import numpy as np

from rebotarm_phone_bridge.pose_filter import PoseLowPassFilter


class PoseLowPassFilterTests(unittest.TestCase):
    def test_first_pose_passes_through_and_position_step_is_smoothed(self):
        pose_filter = PoseLowPassFilter(time_constant=1.0)
        position, quaternion = pose_filter.update(
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            0.0,
        )

        np.testing.assert_allclose(position, [0.0, 0.0, 0.0])
        np.testing.assert_allclose(quaternion, [0.0, 0.0, 0.0, 1.0])

        position, _ = pose_filter.update(
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            1.0,
        )
        self.assertAlmostEqual(position[0], 1.0 - math.exp(-1.0))

    def test_orientation_uses_slerp_and_reset_clears_history(self):
        pose_filter = PoseLowPassFilter(time_constant=1.0)
        pose_filter.update(
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            0.0,
        )
        _, quaternion = pose_filter.update(
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            math.log(2.0),
        )

        np.testing.assert_allclose(
            quaternion,
            [0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)],
            atol=1e-12,
        )

        pose_filter.reset()
        position, _ = pose_filter.update(
            [2.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            10.0,
        )
        np.testing.assert_allclose(position, [2.0, 0.0, 0.0])


if __name__ == "__main__":
    unittest.main()
