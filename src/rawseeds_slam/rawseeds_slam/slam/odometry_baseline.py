"""Runnable baseline that maps with wheel odometry only."""
import numpy as np


class PoseEstimator:
    def update(self, points_laser, odom_base, base_to_laser, timestamp):
        return np.asarray(odom_base).copy()
