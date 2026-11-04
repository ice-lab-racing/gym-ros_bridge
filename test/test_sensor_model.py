import numpy as np

from f1tenth_gym_ros.sensor_model import measured_speed, noisy_scan, odom_step

L = 0.33


def drive(v, steer, seconds, dt=0.01):
    pose = [0.0, 0.0, 0.0]
    for _ in range(round(seconds / dt)):
        pose, _ = odom_step(pose, v, steer, dt, L)
    return pose


def test_straight():
    assert np.allclose(drive(2.0, 0.0, 1.0), [2.0, 0.0, 0.0])


def test_quarter_circle():
    steer, v = 0.3, 2.0
    radius = L / np.tan(steer)
    seconds = (np.pi / 2) * radius / v
    pose = drive(v, steer, seconds, dt=seconds / 100)
    assert np.allclose(pose, [radius, radius, np.pi / 2], atol=1e-3)


def test_yaw_rate():
    _, omega = odom_step([0, 0, 0], 2.0, 0.3, 0.01, L)
    assert np.isclose(omega, 2.0 * np.tan(0.3) / L)


def test_speed_zero_when_stopped():
    rng = np.random.default_rng(0)
    assert measured_speed(0.0, rng, 0.02, 0.05) == 0.0


def test_speed_scale_error():
    rng = np.random.default_rng(0)
    assert np.isclose(measured_speed(5.0, rng, 0.02, 0.0), 5.1)


def test_odometry_drift_repeatable():
    def run(seed):
        rng = np.random.default_rng(seed)
        pose = [0.0, 0.0, 0.0]
        for _ in range(1000):
            pose, _ = odom_step(pose, measured_speed(3.0, rng, 0.02, 0.05), 0.2 + 0.01, 0.01, L)
        return pose
    assert np.allclose(run(1), run(1))
    assert not np.allclose(run(1), drive(3.0, 0.2, 10.0), atol=0.1)


def test_scan():
    rng = np.random.default_rng(0)
    ranges = np.full(10000, 5.0)
    ranges[:100] = 20.0
    r = noisy_scan(ranges, rng, 0.03, 10.0, 0.01)
    assert np.isinf(r[:100]).all()
    finite = r[np.isfinite(r)]
    assert 0.005 < 1 - finite.size / 9900 < 0.015            # ~1% dropped
    assert abs(finite.std() - np.sqrt(0.03 ** 2 - 0.01 ** 2)) < 0.002
    assert (r >= 0).all()


def test_scan_no_noise():
    rng = np.random.default_rng(0)
    assert np.array_equal(noisy_scan([1.0, 2.0], rng, 0.0, 10.0, 0.0), [1.0, 2.0])
