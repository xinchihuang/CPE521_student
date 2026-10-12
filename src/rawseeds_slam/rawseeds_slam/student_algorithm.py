"""Student extension point.

Edit this file to implement your SLAM pose estimator. The starter returns
odometry unchanged so the package runs before your algorithm is finished.
"""
import numpy as np


class PoseEstimator:
    def update(self, points_laser, odom_base, base_to_laser, timestamp):
        """Return [x,y,yaw] of base_link in map (meters, radians).

        points_laser: Nx2 finite scan endpoints in the laser frame.
        odom_base:    [x,y,yaw] of base_link in odom at this scan's time.
        base_to_laser: fixed planar extrinsic; apply before matching.
        timestamp: original data time in seconds.

        TODO:
        - initialize your map frame
        - predict from incremental odometry
        - match the current scan against previous scans or a local map
        - return your corrected base_link pose in map
        """
        return np.asarray(odom_base).copy()
