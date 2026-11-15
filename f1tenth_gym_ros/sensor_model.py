"""Sensor models of the real car: VESC odometry and LiDAR. Pure functions, no ROS."""
import numpy as np

GYM_SCAN_STD = 0.01     # m, noise the gym already adds to every beam


def odom_step(pose, v, steer, dt, wheelbase):
    """One step of the kinematic bicycle the VESC driver integrates. Returns the new pose and the yaw rate.

    The VESC only knows the wheel speed and the servo command: it does not see the slip,
    so in fast corners and hard braking the pose drifts away from the truth.
    """
    x, y, theta = pose
    omega = v * np.tan(steer) / wheelbase
    mid = theta + 0.5 * omega * dt          # heading at the middle of the step
    x += v * np.cos(mid) * dt
    y += v * np.sin(mid) * dt
    theta += omega * dt
    theta = np.arctan2(np.sin(theta), np.cos(theta))
    return [x, y, theta], omega


def measured_speed(v, rng, scale_error, noise):
    """Wheel speed as the VESC reads it: wrong ERPM gain + noise proportional to speed (exact 0 when stopped)."""
    return v * (1.0 + scale_error + rng.normal(0.0, noise))


def noisy_scan(ranges, rng, std, range_max, dropout):
    """The gym's scan with the sensor's error: total noise std, no return past range_max or on dropped beams (inf)."""
    r = np.array(ranges, dtype=float)
    extra = np.sqrt(max(std ** 2 - GYM_SCAN_STD ** 2, 0.0))
    if extra > 0.0:
        r = np.maximum(r + rng.normal(0.0, extra, r.size), 0.0)
    r[(r > range_max) | (rng.random(r.size) < dropout)] = np.inf
    return r
