import unittest

from rebotarm_phone_teleop.mapping import (
    map_relative_orientation,
    map_relative_position,
)


class MappingTest(unittest.TestCase):
    def test_zero_phone_motion_preserves_initial_eef_position(self):
        result = map_relative_position(
            (0.4, -0.2, 0.1),
            (0.4, -0.2, 0.1),
            (0.3, 0.0, 0.25),
            0.3,
        )

        self.assertEqual(result, (0.3, 0.0, 0.25))

    def test_phone_delta_is_scaled_in_base_frame(self):
        result = map_relative_position(
            (1.0, 2.0, 3.0),
            (1.2, 1.5, 3.1),
            (0.3, 0.1, 0.2),
            0.5,
        )

        self.assertAlmostEqual(result[0], 0.4)
        self.assertAlmostEqual(result[1], -0.15)
        self.assertAlmostEqual(result[2], 0.25)

    def test_zero_phone_rotation_preserves_initial_eef_orientation(self):
        result = map_relative_orientation(
            (0.0, 0.0, 0.0, 1.0),
            (0.0, 0.0, 0.0, -1.0),
            (0.0, 2**-0.5, 0.0, 2**-0.5),
        )

        self.assertAlmostEqual(result[0], 0.0)
        self.assertAlmostEqual(result[1], 2**-0.5)
        self.assertAlmostEqual(result[2], 0.0)
        self.assertAlmostEqual(result[3], 2**-0.5)

    def test_phone_rotation_is_applied_in_base_frame(self):
        result = map_relative_orientation(
            (0.0, 0.0, 0.0, 1.0),
            (0.0, 0.0, 2**-0.5, 2**-0.5),
            (2**-0.5, 0.0, 0.0, 2**-0.5),
        )

        for value in result:
            self.assertAlmostEqual(value, 0.5)


if __name__ == "__main__":
    unittest.main()
