import math
import unittest

from rebotarmcontroller.examples.eef_streaming_test import profile_progress


class EefStreamingProfileTest(unittest.TestCase):
    def test_linear_profile(self) -> None:
        self.assertEqual(profile_progress("linear", 0.0), 0.0)
        self.assertEqual(profile_progress("linear", 0.25), 0.25)
        self.assertEqual(profile_progress("linear", 1.0), 1.0)

    def test_sine_profile(self) -> None:
        self.assertEqual(profile_progress("sine", 0.0), 0.0)
        self.assertAlmostEqual(profile_progress("sine", 0.5), 0.5)
        self.assertEqual(profile_progress("sine", 1.0), 1.0)

    def test_sine_profile_is_monotonic(self) -> None:
        values = [
            profile_progress("sine", index / 100.0)
            for index in range(101)
        ]
        self.assertTrue(
            all(left <= right for left, right in zip(values, values[1:]))
        )
        start_slope = values[1] - values[0]
        middle_slope = values[51] - values[50]
        self.assertLess(start_slope, middle_slope)
        self.assertAlmostEqual(
            start_slope,
            0.5 - 0.5 * math.cos(math.pi / 100.0),
        )

    def test_profiles_clamp_normalized_time(self) -> None:
        self.assertEqual(profile_progress("linear", -1.0), 0.0)
        self.assertEqual(profile_progress("sine", 2.0), 1.0)


if __name__ == "__main__":
    unittest.main()
