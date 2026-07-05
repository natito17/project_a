"""
Standalone unit tests for the Follow the Gap algorithm.

Runs WITHOUT ROS2 by mocking all ROS2 / message-type dependencies.
The algorithm logic (preprocess, bubble, gap find, best point, speed) is pure
NumPy and can be fully validated on any machine with Python 3 + NumPy.

Run from the workspace root:
    python3 gap_follow/test/test_reactive_node.py

Or from the gap_follow directory:
    python3 test/test_reactive_node.py
"""

import sys
import os
import unittest
from unittest.mock import MagicMock

import numpy as np

# ── Stub out every ROS2 / message import ──────────────────────────────────────
# This must happen BEFORE importing reactive_node.
#
# IMPORTANT — Python 3.14 MagicMock base-class caveat:
#   Using a raw MagicMock() as a base class (e.g. sys.modules['rclpy.node'] =
#   MagicMock()) breaks in Python 3.14:
#     1. Only the FIRST instantiation of the subclass works; subsequent calls
#        raise StopIteration from the mock's internal iterator.
#     2. The mock's __getattribute__ intercepts method calls on the subclass,
#        returning child mocks instead of the real methods.
#
#   Fix: provide a real Python class (_FakeNode) as the Node substitute.
#   This is safe for multiple instantiations and lets the subclass's own
#   methods resolve normally through the MRO.


class _FakeNode:
    """
    Minimal substitute for rclpy.node.Node.

    Provides real Python implementations of the ROS2 Node API methods used
    by ReactiveFollowGap.__init__, so the class can be instantiated any number
    of times without mock exhaustion.  Each create_publisher() call returns a
    fresh MagicMock so the publisher's .publish() call history is tracked per
    node instance.
    """
    def __init__(self, *args, **kwargs):
        self._params = {}

    def declare_parameter(self, name, default_value=None):
        self._params[name] = default_value
        return MagicMock(value=default_value)

    def get_parameter(self, name):
        return MagicMock(value=self._params[name])

    def create_subscription(self, *args, **kwargs):
        return MagicMock()

    def create_publisher(self, *args, **kwargs):
        return MagicMock()

    def get_logger(self):
        return MagicMock()


# Mock the rclpy.node module, but replace .Node with our real _FakeNode class.
_rclpy_node_mod = MagicMock()
_rclpy_node_mod.Node = _FakeNode

sys.modules['rclpy']              = MagicMock()
sys.modules['rclpy.node']         = _rclpy_node_mod
sys.modules['sensor_msgs']        = MagicMock()
sys.modules['sensor_msgs.msg']    = MagicMock()
sys.modules['ackermann_msgs']     = MagicMock()
sys.modules['ackermann_msgs.msg'] = MagicMock()

# Add the scripts/ directory to the path so we can import the node directly.
_scripts_dir = os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..', 'scripts')
)
sys.path.insert(0, _scripts_dir)

# Now import is safe — Node.__init__ is a MagicMock and will not raise.
from reactive_node import (  # noqa: E402
    ReactiveFollowGap,
    BUBBLE_RADIUS,
    MAX_RANGE_CLIP,
    SMOOTH_WINDOW,
    FAST_SPEED,
    MID_SPEED,
    SLOW_SPEED,
    FAST_THRESHOLD,
    MID_THRESHOLD,
)


def _make_node():
    """Instantiate ReactiveFollowGap with all ROS2 internals mocked out."""
    return ReactiveFollowGap()


# ── Helper ────────────────────────────────────────────────────────────────────

class MockScan:
    """
    Minimal stand-in for sensor_msgs/LaserScan used in integration tests.

    Replicates the fields that lidar_callback reads:
        angle_min, angle_max, angle_increment, ranges

    Default FOV matches the Hokuyo UST-10LX used on the RoboRacer (270°).
    """
    def __init__(self, ranges_list, fov_deg=270):
        fov_rad = np.deg2rad(fov_deg)
        n = len(ranges_list)
        self.angle_min       = -fov_rad / 2          # ~ -2.356 rad
        self.angle_max       =  fov_rad / 2           # ~  2.356 rad
        self.angle_increment = fov_rad / n            # ~ 0.00436 rad per step
        self.ranges          = ranges_list


# ── Test suites ───────────────────────────────────────────────────────────────

class TestPreprocessLidar(unittest.TestCase):
    """Validate sanitization and smoothing of raw LiDAR data."""

    def setUp(self):
        self.node = _make_node()

    def test_inf_replaced(self):
        """All inf inputs must become finite after preprocessing."""
        raw = [float('inf')] * 20
        result = self.node.preprocess_lidar(raw)
        self.assertTrue(np.all(np.isfinite(result)),
                        "inf values survived preprocessing")

    def test_nan_replaced(self):
        """All nan inputs must become finite after preprocessing."""
        raw = [float('nan')] * 20
        result = self.node.preprocess_lidar(raw)
        self.assertTrue(np.all(np.isfinite(result)),
                        "nan values survived preprocessing")

    def test_values_clipped_to_max_range(self):
        """Values above MAX_RANGE_CLIP must be clipped down."""
        raw = [MAX_RANGE_CLIP + 5.0] * 20
        result = self.node.preprocess_lidar(raw)
        self.assertTrue(np.all(result <= MAX_RANGE_CLIP),
                        "Values above MAX_RANGE_CLIP survived clipping")

    def test_values_clipped_to_zero(self):
        """Negative values (sensor glitch) must be clipped to 0."""
        raw = [-1.0, -0.5, 0.0, 1.0] + [1.0] * 16
        result = self.node.preprocess_lidar(raw)
        self.assertTrue(np.all(result >= 0.0),
                        "Negative values survived clipping")

    def test_uniform_array_unchanged_by_smoothing(self):
        """
        A constant array must not be changed by the windowed mean — including
        at the boundaries.  This validates edge-padding (not zero-padding) is
        used in preprocess_lidar; zero-padding would reduce the first/last
        elements and cause the safety bubble to over-expand in real scans.
        """
        raw = [1.5] * 50
        result = self.node.preprocess_lidar(raw)
        np.testing.assert_allclose(result, 1.5, atol=0.01,
                                   err_msg="Uniform array altered by smoothing")

    def test_output_length_preserved(self):
        """Preprocessing must not change the number of elements."""
        raw = [1.0] * 1080
        result = self.node.preprocess_lidar(raw)
        self.assertEqual(len(result), 1080,
                         "Preprocessing changed array length")

    def test_mixed_inf_and_valid(self):
        """Mixed scan: valid values survive, inf values are replaced."""
        raw = [float('inf'), 1.0, float('inf'), 2.0, float('inf')]
        result = self.node.preprocess_lidar(raw)
        self.assertTrue(np.all(np.isfinite(result)))
        self.assertTrue(np.all(result <= MAX_RANGE_CLIP))


class TestFindMaxGap(unittest.TestCase):
    """Validate the max-gap detection against known arrays."""

    def setUp(self):
        self.node = _make_node()

    def test_two_runs_returns_longer(self):
        # Run 1: indices 1–3 (length 3)
        # Run 2: indices 6–7 (length 2)
        # Expect run 1 to win.
        arr = np.array([0, 1, 2, 3, 0, 0, 1, 2, 0], dtype=float)
        start, end = self.node.find_max_gap(arr)
        self.assertEqual(start, 1)
        self.assertEqual(end,   3)

    def test_single_nonzero_element(self):
        arr = np.array([0, 0, 5, 0, 0], dtype=float)
        start, end = self.node.find_max_gap(arr)
        self.assertEqual(start, 2)
        self.assertEqual(end,   2)

    def test_all_free_space(self):
        """Entire array non-zero → gap spans the whole array."""
        arr = np.ones(10, dtype=float) * 2.0
        start, end = self.node.find_max_gap(arr)
        self.assertEqual(start, 0)
        self.assertEqual(end,   9)

    def test_gap_at_end_of_array(self):
        # Larger run at the tail.
        arr = np.array([1, 0, 0, 0, 1, 2, 3, 4, 5], dtype=float)
        start, end = self.node.find_max_gap(arr)
        self.assertEqual(start, 4)
        self.assertEqual(end,   8)

    def test_gap_at_start_of_array(self):
        arr = np.array([3, 2, 1, 0, 0, 1, 0], dtype=float)
        start, end = self.node.find_max_gap(arr)
        self.assertEqual(start, 0)
        self.assertEqual(end,   2)

    def test_equal_length_gaps_first_wins(self):
        # [1,1,1, 0, 2,2,2] — both length 3; first should win.
        arr = np.array([1, 1, 1, 0, 2, 2, 2], dtype=float)
        start, end = self.node.find_max_gap(arr)
        self.assertEqual(start, 0)
        self.assertEqual(end,   2)

    def test_all_zeros_does_not_crash(self):
        """No free space must not raise an exception."""
        arr = np.zeros(50, dtype=float)
        result = self.node.find_max_gap(arr)
        self.assertIsNotNone(result)
        self.assertEqual(len(result), 2,
                         "find_max_gap must return a 2-tuple")

    def test_large_realistic_array(self):
        """Sanity check on a 1080-element array (real LiDAR size)."""
        arr = np.ones(1080, dtype=float)
        arr[200:300] = 0.0   # zeroed bubble region
        start, end = self.node.find_max_gap(arr)
        # Longest run is either [0,199] (200 elements) or [300,1079] (780).
        self.assertEqual(start, 300)
        self.assertEqual(end,   1079)


class TestFindBestPoint(unittest.TestCase):
    """Validate that best point returns the center of the gap."""

    def setUp(self):
        self.node = _make_node()

    def test_center_even_width(self):
        # Gap 2..8 → center = (2+8)//2 = 5
        idx = self.node.find_best_point(2, 8, np.ones(10))
        self.assertEqual(idx, 5)

    def test_center_odd_width(self):
        # Gap 1..5 → center = (1+5)//2 = 3
        idx = self.node.find_best_point(1, 5, np.ones(10))
        self.assertEqual(idx, 3)

    def test_single_element_gap(self):
        idx = self.node.find_best_point(4, 4, np.ones(10))
        self.assertEqual(idx, 4)

    def test_full_array_gap(self):
        n = 1080
        idx = self.node.find_best_point(0, n - 1, np.ones(n))
        self.assertEqual(idx, (n - 1) // 2)

    def test_result_within_bounds(self):
        """Best point index must always fall within [start_i, end_i]."""
        for start, end in [(0, 9), (5, 5), (100, 999), (0, 1079)]:
            idx = self.node.find_best_point(start, end, np.ones(1080))
            self.assertGreaterEqual(idx, start,
                f"best_idx {idx} below start {start}")
            self.assertLessEqual(idx, end,
                f"best_idx {idx} above end {end}")


class TestSpeedChoice(unittest.TestCase):
    """Validate piecewise speed selection."""

    def setUp(self):
        self.node = _make_node()

    def test_far_obstacle_fast_speed(self):
        speed = self.node._choose_speed(FAST_THRESHOLD + 0.5)
        self.assertAlmostEqual(speed, FAST_SPEED)

    def test_medium_distance_mid_speed(self):
        speed = self.node._choose_speed((FAST_THRESHOLD + MID_THRESHOLD) / 2)
        self.assertAlmostEqual(speed, MID_SPEED)

    def test_close_obstacle_slow_speed(self):
        speed = self.node._choose_speed(MID_THRESHOLD - 0.1)
        self.assertAlmostEqual(speed, SLOW_SPEED)

    def test_boundary_fast_threshold(self):
        """Exactly at FAST_THRESHOLD should NOT give fast speed (not >)."""
        speed = self.node._choose_speed(FAST_THRESHOLD)
        self.assertLess(speed, FAST_SPEED,
                        "Speed at exact threshold should not be FAST_SPEED")

    def test_boundary_mid_threshold(self):
        speed = self.node._choose_speed(MID_THRESHOLD)
        self.assertLess(speed, MID_SPEED,
                        "Speed at exact threshold should not be MID_SPEED")


class TestIntegration(unittest.TestCase):
    """
    End-to-end tests: call lidar_callback() with a synthetic scan and inspect
    the AckermannDriveStamped message passed to publisher.publish().

    Because AckermannDriveStamped is mocked, drive_msg is a MagicMock object.
    We can write attributes onto it (drive_msg.drive.steering_angle = val) and
    read them back — MagicMock fully supports dynamic attribute assignment.

    LiDAR angle convention (F1TENTH):
        index 0           → angle_min (rightmost beam, ~-135°)
        index n//2        → ~0 rad    (straight ahead)
        index n-1         → angle_max (leftmost beam, ~+135°)
        steering_angle > 0 → left turn
        steering_angle < 0 → right turn
    """

    def setUp(self):
        self.node = _make_node()

    def _last_drive(self):
        """Return the drive message from the most recent publish() call."""
        self.assertTrue(self.node.publisher.publish.called,
                        "publisher.publish() was never called")
        return self.node.publisher.publish.call_args[0][0]

    def test_obstacle_on_right_steers_left(self):
        """
        Obstacle at 0.5 m on the right side (low indices) → max gap on the
        left → steering angle should be positive (left turn).

        0.5 m is realistic (car is ~0.31 m wide so anything closer means the
        car body is already making contact).  Using 0.05 m would make the
        safety bubble larger than the entire scan, zeroing all free space.
        """
        n = 1080
        ranges = [3.0] * n
        for i in range(n // 4):
            ranges[i] = 0.5
        scan = MockScan(ranges)
        self.node.lidar_callback(scan)

        msg = self._last_drive()
        self.assertGreater(msg.drive.steering_angle, 0.0,
            f"Expected left turn (positive), got {msg.drive.steering_angle:.3f}")

    def test_obstacle_on_left_steers_right(self):
        """
        Obstacle at 0.5 m on the left side (high indices) → max gap on the
        right → steering angle should be negative (right turn).
        """
        n = 1080
        ranges = [3.0] * n
        for i in range(3 * n // 4, n):
            ranges[i] = 0.5
        scan = MockScan(ranges)
        self.node.lidar_callback(scan)

        msg = self._last_drive()
        self.assertLess(msg.drive.steering_angle, 0.0,
            f"Expected right turn (negative), got {msg.drive.steering_angle:.3f}")

    def test_obstacles_both_sides_steers_forward(self):
        """
        Obstacles at 0.5 m on both far-left and far-right quarters.
        Only the center of the scan is open → steering should be near 0.
        Allow ±0.3 rad tolerance (gap center shifts with bubble placement).
        """
        n = 1080
        ranges = [3.0] * n
        for i in list(range(n // 4)) + list(range(3 * n // 4, n)):
            ranges[i] = 0.5
        scan = MockScan(ranges)
        self.node.lidar_callback(scan)

        msg = self._last_drive()
        self.assertAlmostEqual(msg.drive.steering_angle, 0.0, delta=0.3,
            msg=f"Both-sides obstacle scan produced steering {msg.drive.steering_angle:.3f}")

    def test_open_field_drives_fast(self):
        """All obstacles at max range → FAST_SPEED."""
        scan = MockScan([MAX_RANGE_CLIP] * 1080)
        self.node.lidar_callback(scan)
        self.assertAlmostEqual(self._last_drive().drive.speed,
                               FAST_SPEED, places=2)

    def test_close_obstacles_drives_slow(self):
        """All obstacles very close → SLOW_SPEED."""
        scan = MockScan([0.2] * 1080)
        self.node.lidar_callback(scan)
        self.assertAlmostEqual(self._last_drive().drive.speed,
                               SLOW_SPEED, places=2)

    def test_publish_called_every_scan(self):
        """A drive command must be published for every scan received."""
        scan = MockScan([1.5] * 1080)
        for _ in range(5):
            self.node.lidar_callback(scan)
        self.assertEqual(self.node.publisher.publish.call_count, 5,
                         "Expected one publish per scan")

    def test_no_crash_on_all_inf(self):
        """All-inf scan (real LiDAR edge case at startup) must not crash."""
        scan = MockScan([float('inf')] * 1080)
        try:
            self.node.lidar_callback(scan)
        except Exception as exc:
            self.fail(f"lidar_callback raised {type(exc).__name__} on "
                      f"all-inf scan: {exc}")

    def test_no_crash_on_all_zeros(self):
        """All-zero scan (sensor error / total occlusion) must not crash."""
        scan = MockScan([0.0] * 1080)
        try:
            self.node.lidar_callback(scan)
        except Exception as exc:
            self.fail(f"lidar_callback raised {type(exc).__name__} on "
                      f"all-zero scan: {exc}")

    def test_empty_gap_fallback_steers_straight(self):
        """
        When the safety bubble consumes all free space (extreme proximity to
        obstacles in every direction), the node must publish a straight,
        slow command rather than crashing or producing a garbage angle.
        """
        # Very close obstacles everywhere — bubble will zero out the whole scan.
        n = 1080
        scan = MockScan([0.01] * n)
        self.node.lidar_callback(scan)

        msg = self._last_drive()
        self.assertAlmostEqual(msg.drive.steering_angle, 0.0, places=5,
            msg="Emergency straight-ahead not triggered")
        self.assertAlmostEqual(msg.drive.speed, SLOW_SPEED, places=2,
            msg="Emergency slow speed not applied")

    def test_steering_within_lidar_fov(self):
        """
        Steering output must stay within ±angle_max (the LiDAR's FOV half-angle).
        Any valid best-point index produces an angle inside the scan's range.
        """
        n = 1080
        ranges = [0.05] * n
        # Clear gap on the left third of the scan.
        for i in range(2 * n // 3, n):
            ranges[i] = 3.0
        scan = MockScan(ranges)
        self.node.lidar_callback(scan)

        fov_half = np.deg2rad(270 / 2)  # 2.356 rad — full extent of the scan
        steering = abs(self._last_drive().drive.steering_angle)
        self.assertLessEqual(steering, fov_half + 0.01,
            f"Steering {steering:.3f} rad is outside LiDAR FOV ±{fov_half:.3f} rad")


if __name__ == '__main__':
    unittest.main(verbosity=2)
