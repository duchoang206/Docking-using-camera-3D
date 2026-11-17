"""Task 8 — core/kalman.py (vật đứng yên trong odom)"""
import math

import numpy as np

from camera3d_docking.core.kalman import StaticPoseKF

R_MEAS = np.diag([0.01**2, 0.01**2, math.radians(0.5) ** 2])


def test_kf_converges_under_noise():
    rng = np.random.default_rng(0)
    truth = np.array([1.0, 0.5, 0.3])
    kf = StaticPoseKF()
    for k in range(50):
        z = truth + rng.normal(0, [0.01, 0.01, math.radians(0.5)])
        assert kf.update(z, R_MEAS, stamp=0.1 * k)
    assert np.hypot(*(kf.x[:2] - truth[:2])) < 0.004
    assert abs(kf.x[2] - truth[2]) < math.radians(0.2)


def test_kf_rejects_outlier():
    kf = StaticPoseKF()
    for k in range(10):
        kf.update([1.0, 0.0, 0.0], R_MEAS, stamp=0.1 * k)
    before = kf.x.copy()
    assert not kf.update([2.0, 0.0, 0.0], R_MEAS, stamp=1.0)
    assert np.allclose(kf.x, before)


def test_kf_yaw_wraparound():
    kf = StaticPoseKF()
    for k in range(20):
        yaw = math.pi - 0.01 if k % 2 == 0 else -math.pi + 0.01
        assert kf.update([0.0, 0.0, yaw], R_MEAS, stamp=0.1 * k)
    assert abs(math.remainder(kf.x[2] - math.pi, 2 * math.pi)) < 0.02


def test_kf_reinitializes_after_consistent_outliers():
    kf = StaticPoseKF(reset_after=5)
    for k in range(10):
        kf.update([1.0, 0.0, 0.0], R_MEAS, stamp=0.1 * k)
    accepted = [kf.update([1.0, 1.0, 0.2], R_MEAS, stamp=1.0 + 0.1 * k) for k in range(5)]
    assert accepted[:4] == [False] * 4 and accepted[4]
    assert np.allclose(kf.x, [1.0, 1.0, 0.2], atol=1e-6)


def test_kf_predict_grows_covariance():
    kf = StaticPoseKF()
    kf.update([0.0, 0.0, 0.0], R_MEAS, stamp=0.0)
    p0 = np.trace(kf.P)
    kf.predict(stamp=10.0)
    assert np.trace(kf.P) > p0
    assert kf.last_update == 0.0
