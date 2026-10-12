"""Small 2D scan-matching demo for the SLAM interface.

This is intentionally compact rather than benchmark-grade. It keeps the previous
laser scan as a local target, predicts motion from odometry, then refines the
pose with a few point-to-point ICP iterations.
"""
import math
import numpy as np

from rawseeds_slam.mapping import compose, inverse


def transform_points(points, pose):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    rot = np.array([[c, -s], [s, c]])
    return points @ rot.T + pose[:2]


def best_fit_transform(source, target):
    src_mean = source.mean(axis=0)
    tgt_mean = target.mean(axis=0)
    src = source - src_mean
    tgt = target - tgt_mean
    u, _, vt = np.linalg.svd(src.T @ tgt)
    rot = vt.T @ u.T
    if np.linalg.det(rot) < 0:
        vt[-1] *= -1
        rot = vt.T @ u.T
    theta = math.atan2(rot[1, 0], rot[0, 0])
    trans = tgt_mean - src_mean @ rot.T
    return np.array([trans[0], trans[1], theta])


class PoseEstimator:
    def __init__(self):
        self.last_odom = None
        self.pose = None
        self.target_points = None

    def update(self, points_laser, odom_base, base_to_laser, timestamp):
        odom_base = np.asarray(odom_base, dtype=float)
        base_to_laser = np.asarray(base_to_laser, dtype=float)
        if self.pose is None or len(points_laser) < 20:
            self.pose = odom_base.copy()
            self.last_odom = odom_base.copy()
            self.target_points = transform_points(points_laser, compose(self.pose, base_to_laser))
            return self.pose.copy()

        odom_delta = compose(inverse(self.last_odom), odom_base)
        pose = compose(self.pose, odom_delta)
        target = self.target_points
        sample = points_laser[:: max(1, len(points_laser) // 180)]

        for _ in range(6):
            source = transform_points(sample, compose(pose, base_to_laser))
            distances = np.sum((source[:, None, :] - target[None, :, :]) ** 2, axis=2)
            nearest = target[np.argmin(distances, axis=1)]
            mask = np.min(distances, axis=1) < 0.8 ** 2
            if np.count_nonzero(mask) < 12:
                break
            correction = best_fit_transform(source[mask], nearest[mask])
            if np.linalg.norm(correction[:2]) > 0.8 or abs(correction[2]) > 0.4:
                break
            pose = compose(correction, pose)
            if np.linalg.norm(correction[:2]) < 0.01 and abs(correction[2]) < 0.002:
                break

        self.pose = pose
        self.last_odom = odom_base.copy()
        current = transform_points(points_laser, compose(self.pose, base_to_laser))
        self.target_points = np.vstack((target[::2], current[:: max(1, len(current) // 240)]))
        if len(self.target_points) > 600:
            self.target_points = self.target_points[-600:]
        return self.pose.copy()
