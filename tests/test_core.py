import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'src/rawseeds_slam')]

from rawseeds_slam.mapping import Occupancy, compose, inverse
from rawseeds_slam.slam.odometry_baseline import PoseEstimator as OdomEstimator
from rawseeds_slam.slam.scan_matcher import PoseEstimator as DemoEstimator


def test_frame_composition():
    pose = np.array([3., 2., 1.2])
    assert np.allclose(compose(pose, inverse(pose)), 0)


def test_raycast_and_nan():
    grid = Occupancy(1., 10, (0., 0.))
    grid.integrate(np.array([1.5, 1.5, 0.]), [3., float('nan')], 0., 1.)
    values = grid.values()
    assert values[1, 2] < 50
    assert values[1, 4] > 50
    assert values[9, 9] == -1


def test_odometry_baseline_interface():
    estimator = OdomEstimator()
    odom = np.array([1., 2., .3])
    result = estimator.update(np.zeros((0, 2)), odom, np.zeros(3), 0.)
    assert np.allclose(result, odom)
    assert result is not odom


def test_demo_estimator_interface():
    angles = np.linspace(-1., 1., 80)
    points = np.column_stack((3.0 * np.cos(angles), 3.0 * np.sin(angles)))
    estimator = DemoEstimator()
    first = estimator.update(points, np.array([0., 0., 0.]), np.zeros(3), 0.)
    second = estimator.update(points, np.array([.1, 0., 0.]), np.zeros(3), .2)
    assert first.shape == (3,)
    assert second.shape == (3,)
    assert np.isfinite(second).all()
