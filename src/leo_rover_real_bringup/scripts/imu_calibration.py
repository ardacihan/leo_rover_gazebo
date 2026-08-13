#!/usr/bin/env python3

"""Pure gyro-bias estimation for the rover IMU. No ROS imports.

Rover 4's gyro carries a large constant offset: measured over 2941 stationary
samples, gyro_z averaged -0.00715 rad/s, which integrates to -24.6 deg/min. The
same class of offset is what made an earlier IMU-integrated yaw drift by 7 deg
while Rover 1 sat still.

robot_localization's EKF does not estimate IMU bias, so feeding it the raw
signal would carry that drift straight into the fused yaw. Removing the bias
first is what makes the gyro usable: with the offset subtracted, the measured
yaw random walk over 30 s was 0.00 deg.

Bias is only observable while the rover is genuinely still, so this estimator
updates exclusively during zero-velocity intervals and holds its last value
whenever the rover moves.
"""


class GyroBiasEstimator:
    """Track a gyro axis offset using zero-velocity updates."""

    def __init__(
        self,
        settle_samples=50,
        linear_threshold=0.01,
        angular_threshold=0.02,
        max_rate_deviation=0.05,
    ):
        self.settle_samples = int(settle_samples)
        self.linear_threshold = float(linear_threshold)
        self.angular_threshold = float(angular_threshold)
        self.max_rate_deviation = float(max_rate_deviation)
        self.bias = 0.0
        self.samples = 0
        self._sum = 0.0

    @property
    def ready(self):
        """True once enough stationary samples have been averaged."""
        return self.samples >= self.settle_samples

    def is_stationary(self, linear_speed, angular_speed):
        return (abs(float(linear_speed)) <= self.linear_threshold
                and abs(float(angular_speed)) <= self.angular_threshold)

    def update(self, rate, linear_speed, angular_speed):
        """Fold one sample in and return the current bias estimate.

        A reading far from the running estimate is rejected even when the
        wheels report stillness: a knock or someone lifting the rover would
        otherwise poison the offset.
        """
        if not self.is_stationary(linear_speed, angular_speed):
            return self.bias
        rate = float(rate)
        if self.ready and abs(rate - self.bias) > self.max_rate_deviation:
            return self.bias
        self._sum += rate
        self.samples += 1
        self.bias = self._sum / self.samples
        return self.bias

    def correct(self, rate):
        """Return the bias-corrected rate, or the raw rate before settling."""
        if not self.ready:
            return float(rate)
        return float(rate) - self.bias
