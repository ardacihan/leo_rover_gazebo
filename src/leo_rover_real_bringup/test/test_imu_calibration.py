#!/usr/bin/env python3

"""Decision tests for gyro bias estimation."""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

from imu_calibration import GyroBiasEstimator

MEASURED_BIAS = -0.00715  # rad/s, Rover 4, 2941 stationary samples


class GyroBiasEstimatorTest(unittest.TestCase):
    def test_learns_the_measured_rover_bias(self):
        estimator = GyroBiasEstimator(settle_samples=10)
        for _ in range(10):
            estimator.update(MEASURED_BIAS, 0.0, 0.0)
        self.assertTrue(estimator.ready)
        self.assertAlmostEqual(estimator.bias, MEASURED_BIAS, places=6)
        self.assertAlmostEqual(estimator.correct(MEASURED_BIAS), 0.0, places=9)

    def test_passes_raw_rate_through_before_settling(self):
        estimator = GyroBiasEstimator(settle_samples=10)
        estimator.update(MEASURED_BIAS, 0.0, 0.0)
        self.assertFalse(estimator.ready)
        self.assertAlmostEqual(estimator.correct(0.5), 0.5)

    def test_ignores_samples_while_moving(self):
        """Real rotation must never be mistaken for bias."""
        estimator = GyroBiasEstimator(settle_samples=5)
        for _ in range(20):
            estimator.update(0.4, 0.08, 0.3)
        self.assertEqual(estimator.samples, 0)
        self.assertEqual(estimator.bias, 0.0)

    def test_rejects_an_outlier_once_settled(self):
        """A knock while the wheels report stillness must not poison bias."""
        estimator = GyroBiasEstimator(settle_samples=5, max_rate_deviation=0.05)
        for _ in range(5):
            estimator.update(MEASURED_BIAS, 0.0, 0.0)
        before = estimator.bias
        estimator.update(2.0, 0.0, 0.0)
        self.assertAlmostEqual(estimator.bias, before, places=9)

    def test_corrected_rate_preserves_real_rotation(self):
        estimator = GyroBiasEstimator(settle_samples=5)
        for _ in range(5):
            estimator.update(MEASURED_BIAS, 0.0, 0.0)
        self.assertAlmostEqual(
            estimator.correct(MEASURED_BIAS + 0.30), 0.30, places=9
        )

    def test_stationary_detection_thresholds(self):
        estimator = GyroBiasEstimator()
        self.assertTrue(estimator.is_stationary(0.0, 0.0))
        self.assertFalse(estimator.is_stationary(0.5, 0.0))
        self.assertFalse(estimator.is_stationary(0.0, 0.5))


if __name__ == "__main__":
    unittest.main()
